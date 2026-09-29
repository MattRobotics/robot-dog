# G3.5 resume evidence

Started at accepted G3 `87c3e914e068260170e7c19c5d664f727f5b03ce`.
No tracked changes, staged changes or G3.5 commits existed at the interruptions.
Untracked `model.py`, `ik.py`, `contact_bridge.cpp` were retained and continued.
The preliminary model/IK adapter built and ran; no full G3.5 test gate had run.

XGO checkout remains on unrelated research HEAD `72234e03739727356470834ab55daee721ed1864`
with pre-existing untracked analysis/, references/intake/, tools/. Never modified.
The read-only evidence view is a Git archive of origin/main
`a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298` at /tmp/matdog-g35-xgo.
This matched `git ls-remote origin refs/heads/main` at initial inspection.

Initial systematic search: front LF and rear RH independently, 5 hip values
[-0.52,0.52], 35 values across each upper/lower URDF limit = 6125 per geometry.
279 front / 95 rear configurations passed per-leg mesh-ground and nonadjacent
self collision. Initial h=0 ground-contact sweep found four front solutions and
no rear solutions in its 2 mm x grid; this is not a proof of global absence.
Preliminary combined body-only candidate was reproduced exactly before adding
solid containment checks; it also passes those checks. Further families and
transition work remain necessary.

Initial scratch outputs retained under /tmp: g35-leg0.json, g35-leg2.json,
g35-feet-{lf,rf,rh,lh}.json, g35-rest.json. These lack full provenance and are
exploratory inputs only; final reproducible search replaces their evidentiary role.
Base mesh minimum local Z=-1.0999563076780766e-16 m; derived height is its negative.
Initial rest CAD/URDF mass=2.48 kg. These findings were revalidated on resume.

Evidence read before interruption: canonical URDF/contact/C4 policies; XGO main
README, AGENTS, state/plan/known-facts/unresolved/handoff documents; H2 Lite action
catalogue, source manifests, URDF identity and joint inventory; source-pinned
xgolib and CM5 preset notebook; parser, command-state and whole-program Ghidra
exports. New headless export runs only on a disposable copy of the exact G2
Ghidra project referenced by the canonical artifact-integrity manifest, using
-noanalysis -readOnly. Vendor code and firmware are never executed.
