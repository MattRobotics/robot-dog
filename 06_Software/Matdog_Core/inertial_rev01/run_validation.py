#!/usr/bin/env python3
"""Build deterministic offline evidence in new directories only; never use hardware."""
from __future__ import annotations

import argparse
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
from model import ROOT, URDF_REL, at_pose, body_dict, fk, c4e, load_baseline, missing, project_support, resolve_budget, sensitivity
from sources import source_audit, sha
from contract import PR50_SHA, HEADER_REL, candidate_payload, cpp_probe

HERE = Path(__file__).resolve().parent
BASE_SHA = "fdd04345c39b50137cc7cdc50fb08cd592b8c2af"
C4C_REL = "09_Logs/Validation_Reports/C4_rest_to_stand_trajectory/2026-07-08_190405_C4C_contact_locked_rest_to_stand_trajectory.json"
C4E_REL = "09_Logs/Validation_Reports/C4_static_stability_support_polygon/2026-07-08_191840_C4E_static_stability_support_polygon.json"
REPORT_REL = "09_Logs/Validation_Reports/REV01_Inertial_2026-10-14"


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)+"\n")


def run_cmd(args, cwd=ROOT):
    result = subprocess.run(args, cwd=cwd, text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError(f"Offline command failed: {args[0]}\n{result.stdout}\n{result.stderr}")
    return result


def compile_contract(candidate, compiler):
    """Read pinned Git objects locally. No fetch/download in the offline runner."""
    with tempfile.TemporaryDirectory(prefix="matdog-inertial-contract-") as tmp:
        checkout=Path(tmp)
        paths=run_cmd(["git","ls-tree","-r","--name-only",PR50_SHA,"--","06_Software/Matdog_Core",
                       "05_Firmware/MATDOG_Controller/src/motion"]).stdout.splitlines()
        headers=[p for p in paths if p.endswith(".h")]
        if HEADER_REL not in headers: raise ValueError("Pinned PR50 header not available locally")
        for relative in headers:
            content=subprocess.run(["git","show",f"{PR50_SHA}:{relative}"],cwd=ROOT,check=True,capture_output=True).stdout
            output=checkout/relative
            output.parent.mkdir(parents=True,exist_ok=True)
            output.write_bytes(content)
        source=checkout/"contract_probe.cpp"
        source.write_text(cpp_probe(candidate))
        exe=checkout/"contract_probe"
        include_dirs=sorted({str((checkout/p).parent) for p in headers})
        include_args=[part for directory in include_dirs for part in ("-I",directory)]
        run_cmd([compiler,"-std=c++17","-Wall","-Wextra","-Wpedantic","-Werror",
                 *include_args,str(source),"-o",str(exe)])
        run_cmd([str(exe)])
        header=checkout/HEADER_REL
        return {"status":"OFFLINE VALIDATED","compiler":Path(compiler).name,
                "mode":"C++17 host fixture, compile and execute; no pipeline/runtime integration",
                "pinned_pr50_commit":PR50_SHA,"header_sha256":sha(header),
                "header_path":HEADER_REL,"link_capacity":17,
                "string_capacities_bytes":{"linkName":32,"assemblyRevision":32,"provenance":96,"sourceSha256":65},
                "exported_readiness":candidate["payload"]["readiness"],
                "missing_dependency_preserved":True}


def replay(links, mass_evidence="HISTORICAL_REV00"):
    trajectory=json.loads((ROOT/C4C_REL).read_text())
    old=json.loads((ROOT/C4E_REL).read_text())
    frames=[]
    for frame in trajectory["frames"]:
        pose={n:q for leg in frame["legs"].values() for n,q in leg["joint_positions_rad"].items()}
        total=at_pose(links,pose)
        body=frame["body_pose"]
        translation=body["translation_world_m"]
        rpy=(body["roll_rad"],body["pitch_rad"],body["yaw_rad"])
        contacts=[frame["legs"][leg]["achieved_contact_reference_world_m"] for leg in ("lf","rf","rh","lh")]
        projection=project_support(total.com,translation,rpy,contacts,(0.,0.,0.),(0.,0.,1.),(0.,0.,-1.),1e-5)
        proxy=c4e._evaluate_frame(frame=frame,com_proxy_world_xy_m=(0.,0.),
                                 com_uncertainty_xy_m=.020,min_support_margin_m=0.)
        actual_box=c4e._evaluate_frame(frame=frame,
                                      com_proxy_world_xy_m=tuple(projection["projected_com_world_m"][:2]),
                                      com_uncertainty_xy_m=.020,min_support_margin_m=0.)
        expected=old["frames"][frame["trajectory_index"]]["evaluation"]["min_support_margin_m"]
        if abs(expected-proxy["min_support_margin_m"])>1e-12:
            raise AssertionError("Historical C4-E regression mismatch")
        frames.append({"trajectory_index":frame["trajectory_index"],"body_pose":body,
                       "joint_positions_rad":pose,"total_base":body_dict(total),
                       "mass_evidence":mass_evidence,
                       "projection":projection,
                       "old_base_link_proxy_box_margin_m":proxy["min_support_margin_m"],
                       "computed_com_centered_same_20mm_box_margin_m":actual_box["min_support_margin_m"]})
    return {"status":"OFFLINE VALIDATED","mass_evidence":mass_evidence,
            "actual_REV01_assembly_verified":False,"samples":len(frames),
            "all_com_centers_geometrically_inside":all(f["projection"]["geometrically_inside"] for f in frames),
            "worst_center_margin_m":min(f["projection"]["signed_margin_m"] for f in frames),
            "worst_same_20mm_box_margin_m":min(f["computed_com_centered_same_20mm_box_margin_m"] for f in frames),
            "assumptions":["C4-C nominal four-contact horizontal support geometry, not physical contact evidence",
                           "Plane tolerance 10 micrometres from historical C4-C numerical envelope",
                           "20 mm box reproduced only for comparison with C4-E, not measured REV01 uncertainty"],
            "command_eligible":False,"hardware_verified":False,"frames":frames}


def accessory_sensitivities(budget_data, resolved_links, pose):
    rows=[]
    for r in budget_data["components"]:
        row={"component_id":r["id"],"disposition":r["disposition"],
             "parent_link":r["parent_link"],"mass_kg":r["mass_kg"],
             "status":"PENDING_MEASUREMENT","sensitivity":None}
        if resolved_links is not None and r["disposition"]=="add":
            total=at_pose(resolved_links,pose)
            transform=fk.forward_kinematics(ROOT/URDF_REL,"base_link",r["parent_link"],pose).tip_transform
            local=r["com_in_parent_m"]
            point=tuple(sum(transform[i][j]*local[j] for j in range(3))+transform[i][3] for i in range(3))
            mass=r["mass_kg"]
            rest_mass=total.mass-mass
            rest_com=tuple((total.mass*total.com[i]-mass*point[i])/rest_mass for i in range(3))
            row.update(status="ASSUMED",accessory_com_base_m=point,
                       sensitivity=sensitivity(rest_mass,rest_com,mass,point),
                       reference="complete scenario excluding this disjoint addition; same explicit pose")
        elif r["disposition"] in {"absent","included_in_link"}:
            row.update(status="NO_SEPARATE_ADDITION",reference="No separate increment; evidence of coverage is required")
        elif r["disposition"]=="replace_link":
            row.update(status="COMPARE_COMPLETE_CONFIGURATIONS",reference="Use budget delta, not additive sensitivity")
        rows.append(row)
    return {"readiness":"REV01_PENDING","hardware_verified":False,"command_eligible":False,
            "pose_semantic_rad":pose,"components":rows,
            "uncertainty_propagation":"dC = (r-C)/M_total * dm + (m/M_total) * dr; actual dm,dr bounds PENDING_MEASUREMENT",
            "multiple_additions":"Use one aggregate denominator, not a sum of individual exact shifts"}


def report_markdown(audit, baseline, budget, support, oracle, unit, compatibility):
    c=baseline.com
    if budget["complete"]:
        budget_summary = ("Il budget è numericamente completo per lo scenario offline dichiarato. "
                          f"Massa candidata q=0: {budget['result']['mass_kg']:.9f} kg; "
                          f"CoM candidato in base: {budget['result']['com_m']} m. "
                          "Le proprietà complete sono in mass_budget_result.json. La revisione resta REV01_PENDING, "
                          "HARDWARE UNVERIFIED: i valori storici trattenuti e la copertura dell’inventario "
                          "restano assunzioni offline, senza promozione a VERIFIED.")
    else:
        budget_summary = (f"Il template ha 10 gruppi di componenti e 17 revisioni di link. Sono aperti {len(budget['pending_fields'])} campi obbligatori. "
                          "Massa totale REV01, CoM REV01 e inerzia REV01 sono **PENDING_MEASUREMENT**; "
                          "i 2,480 kg sono esclusivamente il riferimento storico, non un subtotal verificato né un limite inferiore.")
    rows=["# MATDOG REV01 — Mass budget e CoM offline", "",
          f"Base `main` verificata: `{BASE_SHA}`. Branch separata `feat/matdog-rev01-inertial-model`.","",
          "| Stato | Significato e risultato |","|---|---|",
          "| IMPLEMENTED | Estrazione, verifica SI/tensori, budget parametrico, aggregazione, proiezione C4-E e mapping candidato PR #50. |",
          f"| OFFLINE VALIDATED | {unit['tests_run']} test deterministici; oracle NumPy indipendente su {oracle['poses']} pose; replay di {support['samples']} frame C4-C; probe C++17 sul contratto reale PR #50. |",
          "| ASSUMED | Configurazione CAD REV00 storica; simmetrie locali dichiarate nel workbook; accessori rigidamente solidali al parent solo dopo indicazione esplicita. |",
          "| PENDING MEASUREMENT | REV01, configurazione reale, inventario, masse/CoM/inerzie degli accessori e riconciliazione per-link. |",
          "| HARDWARE UNVERIFIED | Nessun test fisico, seriale, flash o movimento. Ogni uscita resta command_eligible=false. |", "",
          "## Risultato e fonti", "",
          "URDF ed Excel coincidono sui 17 link: 5 report CAD diretti (base e LF), 12 derivazioni locali per simmetria/copia. Tutti i tensori sono simmetrici, definiti positivi e rispettano le disuguaglianze triangolari dei momenti principali.","",
          "Il modello storico contiene 2,480 kg. L’Excel registra anche 2,3894 kg da distinta pesi e 2,297 kg da una pesata fisica storica. Il divario CAD–pesata è 183 g (7,379% della massa CAD); configurazione, data, strumento e allocazione per-link della pesata non sono disponibili. La pesata resta un’evidenza storica riportata, non una verifica REV01. Non è stata applicata alcuna scala globale alle inerzie.","",
          audit["documentation_conflict"],"",
          "Gli originali `physprop__*.txt` e `weight_breakdown_summary.txt` sono nominati dal workbook ma non presenti nel repository. La provenienza è una trascrizione storica. `CAD_Source_Data` non specifica l’unità delle inerzie globali arrotondate: questi campi non alimentano il calcolo. Il segno dei prodotti CAD va riconfermato nel nuovo export; i termini URDF sono già elementi del tensore.","",
          f"Posa numerica esplicita q=0 (non una stance di contatto): massa {baseline.mass:.9f} kg; CoM base [{c[0]*1000:.9f}, {c[1]*1000:.9f}, {c[2]*1000:.9f}] mm.","",
          "Inerzia equivalente al CoM, assi base, kg·m²:","","```text",
          *["["+", ".join(f"{v:.12g}" for v in row)+"]" for row in baseline.inertia],"```","",
          "L’Excel `Physical_Property_Method` conserva il confronto storico del CoM ricostruito [9,571; −0,336; 5,970] mm con il CAD completo [9,480; −0,710; 5,910] mm. La differenza massima di circa 0,374 mm è una verifica interna CAD storica, non una misura dell’assemblaggio attuale. Il CoM aggregato cambia con la posa dei joint.","",
          "## Estratto dei 17 link", "",
          "Masse in kg; CoM in m nel frame locale. I sei termini del tensore e gli autovalori completi sono in `historical_source_audit.json`, senza arrotondamento editoriale.","",
          "| Link | Massa kg | CoM locale [m] | Evidenza storica |", "|---|---:|---|---|",
          *[f"| {r['link_name']} | {r['mass_kg']:.6f} | {r['com_in_link_m']} | {r['historical_evidence']} |" for r in audit["links"]],"",
          "## Budget REV01 e misure mancanti", "",
          budget_summary,"",
          "Compilare il [template operativo](../../../06_Software/Matdog_Core/inertial_rev01/MEASUREMENTS.md) e la copia del [budget parametrico](../../../06_Software/Matdog_Core/inertial_rev01/data/mass_budget_rev01.json). `missing_measurements.json` separa i 116 campi del budget iniziale dai campi di acquisizione CAD/robot e provenienza del template.","",
          "Per elettronica, BNO085, cablaggi, supporto antenna posteriore, antenna/cavo, portafusibile, contatti di ricarica, configurazione batteria/alimentazione, assieme neck yaw e altri componenti occorrono disposizione, parent, massa, CoM, tensore al CoM, orientamento ed evidenza di copertura. Nessuna massa, densità o posizione è stata attribuita agli accessori.","",
          "Ogni riga deve diventare `absent`, `included_in_link`, `add` o `replace_link`, con evidenza. `add` è ammesso solo per contenuti disgiunti dal link. Per rimuovere/sostituire componenti già inglobati nel CAD si misura e sostituisce il link completo; niente masse negative o sottrazioni da proprietà ignote. Un accessorio articolato non è rappresentabile come addizione rigida durante il suo movimento: serve una futura revisione di topologia e del limite 17 link.","",
          "Le decisioni per tutti i link (`retain_historical_offline` oppure `replace_complete_link`) e la revisione dell’inventario impediscono di trasformare il completamento dei soli accessori in una verifica dell’intero robot. Anche un budget interamente numerico resta `REV01_PENDING`.","",
          "## Sensibilità del CoM", "",
          "Per un accessorio disgiunto di massa m al punto r, con baseline M,C nella stessa posa/frame:","",
          "```text", "C_new = C + m/(M+m) * (r-C)",
          "dC_new/dm = M/(M+m)^2 * (r-C)", "dC_new/dr = m/(M+m) * E", "```", "",
          "Finché r e m restano PENDING_MEASUREMENT non è possibile un ranking fisico né una previsione in mm. `accessory_sensitivity.json` riporta lo stato per ogni riga e, con budget completo, lo spostamento incrementale e le derivate rispetto allo scenario privo della singola addizione. Sul solo riferimento storico M=2,480 kg, il coefficiente infinitesimo per unità di braccio è 1/M = 0,403225806 kg⁻¹. Un incremento di massa al CoM non sposta C; la posizione lungo l’asse posteriore produce uno spostamento solo dopo che il suo braccio è misurato. Per più accessori si sommano i momenti di massa con il denominatore totale, non gli spostamenti individuali. Le incertezze di massa/posizione restano da fornire per propagare un’incertezza del CoM.","",
          "| Rapporto parametrico μ=m/M (nessun accessorio reale) | Frazione del braccio che diventa spostamento μ/(1+μ) |",
          "|---:|---:|", *[f"| {mu:.3f} | {mu/(1+mu):.9f} |" for mu in (.001,.01,.05,.10)],"",
          "Per un link sostituito il confronto è la differenza fra le due aggregazioni complete alla stessa posa. Non si usa la formula di sola addizione. Il modulo `sensitivity` e i test a differenze finite verificano le derivate; i numeri inventati nei test sono fixture matematiche, separati dai dati robot.","",
          "## CoM e poligono C4-E", "",
          f"Riuso in sola lettura di `_convex_hull`, `_polygon_signed_margin` e `_evaluate_frame` originali. Le 51 pose C4-C usano i joint semantici in radianti e il CoM dei 17 link trasformato in world. Il peggiore margine storico al centro è {support['worst_center_margin_m']*1000:.6f} mm; riproducendo la vecchia scatola ±20 mm attorno al nuovo CoM storico è {support['worst_same_20mm_box_margin_m']*1000:.6f} mm. Il replay riproduce anche tutti i margini originali C4-E entro 1e-12 m.","",
          "La scatola ±20 mm è soltanto il confronto storico, non l’incertezza misurata REV01. La proiezione generale è lungo la gravità world sul piano esplicito, non lungo la normale. Contatti insufficienti/collineari, non coplanari o gravità parallela al piano sono rifiutati. La routine valuta geometria offline: non osserva carico, aderenza, freshness né contatti reali confermati. Non produce un’approvazione di stabilità dinamica. La valutazione del CoM effettivo REV01 si abilita nel runner solo con budget numerico completo e continua a dichiarare HARDWARE UNVERIFIED.","",
          "## Compatibilità futura PR #50", "",
          f"Contratto letto al commit `{PR50_SHA}`; header SHA-256 `{compatibility['header_sha256']}`. Probe C++17 compilato ed eseguito contro l’header autentico e i suoi include transitivi, estratti in una directory temporanea da oggetti Git locali. Nessuna modifica a G5-B.","",
          "Il mapping conserva 17 link: addizioni rigide vengono aggregate al parent; inerzie per-link al CoM negli assi del link; inerzia totale al CoM negli assi base. Le capacità stringa includono il terminatore NUL e sono controllate in byte UTF-8. Il payload storico usa `HISTORICAL_REV00`, quello numerico candidato `REV01_PENDING`. Il template incompleto non viene serializzato in numeri C++ zero. `measuredAtS=0` significa indisponibile e non è un timestamp di misura fresco.","",
          "Il campo aggregato dipende dalla posa, che il contratto C++ non trasporta: il companion JSON conserva tutti i joint e la preimmagine SHA-256 canonica del payload senza hash. Un futuro consumer dovrà rigenerare l’aggregato per la posa, validare provenienza/configurazione/timestamp e implementare i gate di contatto e freshness. Il contratto richiede DEPENDENCY_MISSING finché readiness != REV01_VERIFIED; il probe preserva tale default. Non è stata implementata un’integrazione runtime né una qualifica REV01.","",
          "## Riproduzione e isolamento", "",
          "```bash", "python3 -B -m unittest discover -s 06_Software/Matdog_Core/inertial_rev01/tests -v",
          "python3 -B 06_Software/Matdog_Core/inertial_rev01/run_validation.py", "```", "",
          "Il runner necessita di NumPy per l’oracle indipendente e di g++ per il probe host, già disponibili nell’ambiente usato. Non installa dipendenze e non scarica niente. Per il probe deve esistere localmente l’oggetto Git PR50 fissato sopra. I calcoli core e l’estrattore OOXML usano solo la libreria standard. Gli output deterministici possono essere rigenerati in una directory temporanea con `--report-dir`; non contengono timestamp di esecuzione.","",
          "Modifiche limitate a nuove directory `06_Software/Matdog_Core/inertial_rev01` e `09_Logs/Validation_Reports/REV01_Inertial_2026-10-14`. URDF/STL/Excel REV00, C4-E, firmware, calibrazione, Motion, G5-B e PR #47/#49/#50 sono preservati. Nessun merge. Hash delle fonti, estratto workbook con celle/formule/cache, risultato dei test e lista precisa dei campi mancanti sono negli artefatti JSON."]
    return "\n".join(rows)+"\n"


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report-dir",type=Path,default=ROOT/REPORT_REL)
    parser.add_argument("--budget",type=Path,default=HERE/"data/mass_budget_rev01.json")
    parser.add_argument("--oracle-python",default=sys.executable)
    parser.add_argument("--compiler",default="g++")
    args=parser.parse_args()
    # Do not emit misleading success evidence if protected tracked files changed.
    allowed=("06_Software/Matdog_Core/inertial_rev01/",REPORT_REL+"/")
    changed=run_cmd(["git","diff","--name-only",BASE_SHA]).stdout.splitlines()
    changed+=run_cmd(["git","ls-files","--others","--exclude-standard"]).stdout.splitlines()
    protected_changes=[p for p in changed if not p.startswith(allowed)]
    if protected_changes: raise ValueError("Protected sources changed: "+str(protected_changes))
    stream=io.StringIO()
    suite=unittest.defaultTestLoader.discover(str(HERE/"tests"))
    result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    if not result.wasSuccessful(): raise AssertionError(stream.getvalue())
    unit={"status":"OFFLINE VALIDATED","tests_run":result.testsRun,"failures":len(result.failures),"errors":len(result.errors)}
    oracle=json.loads(run_cmd([args.oracle_python,"-B",str(HERE/"oracle_numpy.py")]).stdout)
    audit=source_audit()
    links=load_baseline()
    zero={n:0. for n,j in fk.load_urdf_joints(ROOT/URDF_REL).items() if j.joint_type!="fixed"}
    historical=at_pose(links,zero)
    budget_data=json.loads(args.budget.read_text())
    budget=resolve_budget(links,budget_data)
    resolved_links=budget.pop("links")
    if resolved_links is not None:
        budget["result"]=body_dict(at_pose(resolved_links,zero))
        budget["result_pose_semantic_rad"]=zero
        budget["delta_from_historical_q0"]={
            "mass_kg":budget["result"]["mass_kg"]-historical.mass,
            "com_m":[a-b for a,b in zip(budget["result"]["com_m"],historical.com)],
            "inertia_at_com_kg_m2":[[budget["result"]["inertia_at_com_kg_m2"][i][j]-historical.inertia[i][j]
                                    for j in range(3)] for i in range(3)]}
    source_hashes={audit[k]["path"]:audit[k]["sha256"] for k in ("urdf","workbook")}
    candidate=candidate_payload(links,zero,"REV00-historical","URDF/Excel historical CAD; offline only","HISTORICAL_REV00",source_hashes)
    compatibility=compile_contract(candidate,args.compiler)
    pending_candidate=None
    pending_compatibility=None
    if resolved_links is not None:
        pending_candidate=candidate_payload(resolved_links,zero,budget_data["assembly_revision"],
                                           "Offline REV01 candidate, assembly unverified","REV01_PENDING",
                                           {**source_hashes,"rev01_budget_input":sha(args.budget)})
        pending_compatibility=compile_contract(pending_candidate,args.compiler)
    support=replay(links)
    pending_support=replay(resolved_links,"REV01_PENDING") if resolved_links is not None else None
    sensitivities=accessory_sensitivities(budget_data,resolved_links,zero)
    markdown=report_markdown(audit,historical,budget,support,oracle,unit,compatibility)
    output=args.report_dir
    output.mkdir(parents=True,exist_ok=True)
    # Never leave stale candidate numbers when a subsequent input becomes incomplete.
    for name in ("rev01_pending_c4e_com_replay.json","pr50_candidate_rev01_pending.json"):
        (output/name).unlink(missing_ok=True)
    write_json(output/"historical_source_audit.json",audit)
    write_json(output/"mass_budget_result.json",budget)
    write_json(output/"accessory_sensitivity.json",sensitivities)
    write_json(output/"missing_measurements.json",{"status":"PENDING_MEASUREMENT","budget_fields":budget["pending_fields"],
                                                 "intake_fields":missing(json.loads((HERE/"data/measurement_intake_template.json").read_text())),
                                                 "intake_template":"06_Software/Matdog_Core/inertial_rev01/data/measurement_intake_template.json"})
    write_json(output/"historical_c4e_com_replay.json",support)
    write_json(output/"pr50_candidate_historical.json",candidate)
    write_json(output/"validation.json",{"status":"OFFLINE VALIDATED","unit":unit,"numpy_oracle":oracle,
                                       "pr50_contract":compatibility,"command_eligible":False,"hardware_verified":False,
                                       "pr50_pending_contract":pending_compatibility,
                                       "protected_files_changed":[],"base_main_commit":BASE_SHA,
                                       "budget_input_sha256":sha(args.budget),
                                       "source_hashes":{rel:sha(ROOT/rel) for rel in (URDF_REL,C4C_REL,C4E_REL,
                                         "06_Software/Matdog_Core/kinematics/matdog_offline_static_stability_support_polygon.py",
                                         "06_Software/Matdog_Core/kinematics/matdog_urdf_fk.py")}})
    output.joinpath("REPORT.md").write_text(markdown)
    if resolved_links is not None:
        write_json(output/"rev01_pending_c4e_com_replay.json",pending_support)
        write_json(output/"pr50_candidate_rev01_pending.json",pending_candidate)
    print(json.dumps({"offline_validated":True,"unit_tests":result.testsRun,"oracle_poses":oracle["poses"],
                      "rev01_readiness":budget["readiness"],"pending_fields":len(budget["pending_fields"]),
                      "hardware_verified":False,"report":str(output/"REPORT.md")},sort_keys=True))


if __name__=="__main__":
    main()
