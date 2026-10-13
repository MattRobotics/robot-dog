#!/usr/bin/env python3
"""Independent RC guard and mutation checks. No devices or persistent writes.

These checks pin the deliberately disabled physical entry points and the
semantic/actuator separation, in addition to the pre-existing static audit.
They do not certify hardware, collision paths or stability.
"""
from pathlib import Path
import re

SRC = Path(__file__).resolve().parents[2] / "src"
REQUIRED = {
    "config/BuildConfig.h": ("constexpr bool kOperationalMotionAuthorized = false;",),
    "core/Controller.h": ("bool reviewed_motion_bound_=false;", "execution::LocalTrialEvidence local_trial_evidence_{};",),
    "execution/MotionSession.cpp": ("if(!image)", "WALK_DYNAMICS_REQUIRED", "FIXTURE_SUPPORT_REQUIRED", "PRIME_REQUIRES_OFF_AND_LIMITS", "policy_->plan(", "runtime_->execute(",),
    "core/Controller.cpp": (
        "ready.hardware_approved=scoped", "ready.support_observable=reviewed_motion_bound_",
        "build::kOperationalMotionAuthorized", "r.policy_epoch!=actuator_policy_.epoch()",
        "motion::pairedProneMatch(local_motion_observation_", "e.scope!=reviewed_motion_.scope",
        "motion_service_.begin(&motion_executor_, &motion_coordinator_);",
        "service_.bindMotionService(&motion_service_);",
        "motion_coordinator_.update(operational_facts_,operational_feedback_,now_ms);",
        "motion_executor_.update(operational_facts_,operational_feedback_,now_ms);",
        "motion_service_.refresh(ready,now_ms);",
        "servo_bus_.safeOff(joint->bus_id)", "motion_safe_off_mask_==0x0fff",
    ),
    "execution/MotionCoordinator.cpp": (
        "arm_generation_==executor_->status().arm_generation",
        "submitted_sequence_=0;assessment_={};frame_=motion::FeedbackFrame{};walking_=false;",
        "feedback_policy_.mode!=motion::FeedbackMode::PHYSICAL_REVIEW||!frame_.physicalReadiness",
        "o.acknowledgedSequence=executor_->status().completed_sequence",
    ),
    "execution/MotionExecutor.cpp": (
        "if(!f.hardware_qualified||!f.sequential_bus_qualified)",
        "if(!f.operator_permit||!f.permit_generation", "arbiter_->inhibited()",
        "SESSION_CHANGED", "SEGMENT_UNVALIDATED", "HOLD_TRACKING_ERROR",
        "TRACKING_DIVERGENCE", "TRACKING_TIMEOUT", "BUS_BATCH_TIMEOUT",
        "policy_->plan(cmd,f.lease,f.mode,&tx)", "runtime_->execute(&tx,bus,nullptr)",
    ),
    "core/MotionService.cpp": (
        "if(!readiness_.operational_permit)", "if(!readiness_.support_observable)",
        "else if(!a->implemented)why=\"ACTION_NOT_VALIDATED\"",
        "active_arm_generation_!=executor_->status().arm_generation",
        "if(active_intent_==motion::Intent::STOP)", "SAFE_OFF_VERIFIED",
    ),
    "network/HttpTransport.cpp": (
        "!socketFacts(req,r) || !self->admitMotion(r)", "req->content_len>1024",
        "r.motion_command.owner!=motion::ClientOwner::DASHBOARD",
        "req.peer,req.cookie,req.csrf))",
        "service_->motionCommand(req.motion_command,now_ms)",
        "char[kMotionCapabilitiesBytes]", "memcpy(large_text,motion_capabilities_,response->text_len+1)",
        'strncmp(uri.uri, "/wifi/", 6) == 0',
    ),
    "core/CommandRouterPersistence.cpp": (
        "persistenceQuietViolation()", "CONFIRM_CURRENT_INSTALLATION",
        "for(uint8_t i=0;i<12;i++)", "state.torque_enable!=0",
        "request.torque_off_confirmed=off", "modules_.runtime_restore->restore(",
    ),
}


def violations(files):
    bad = []
    for path, tokens in REQUIRED.items():
        code = files.get(path, "")
        for token in tokens:
            if token not in code:
                bad.append(f"{path}: missing {token}")
    controller = files["core/Controller.cpp"]
    if re.search(r"motion_executor_\.arm\s*\(", controller):
        bad.append("Controller must arm only through the scoped local MotionSession")
    ordering = [controller.find(token) for token in (
        "motion_coordinator_.update(operational_facts_,operational_feedback_,now_ms);",
        "motion_executor_.update(operational_facts_,operational_feedback_,now_ms);",
        "motion_service_.refresh(ready,now_ms);")]
    if any(index < 0 for index in ordering) or ordering != sorted(ordering):
        bad.append("Support/pose preflight must precede actuator update and terminal service refresh")
    network = files["network/HttpTransport.cpp"]
    if re.search(r"(?:ServoBus|GoalPosition|WritePos|EnableTorque|->acquire\(|->grant\()", network):
        bad.append("Web must never name a bus primitive or obtain actuator authority")
    restore = files["core/CommandRouterPersistence.cpp"].split("void CommandRouter::handleRuntimeRestoreCommand", 1)[1].split("CommandRouter::PersistenceSchemaIdentity", 1)[0]
    if re.search(r"(?:->grant\(|->acquire\(|safeOff\(|writeTorque|writePosition|->save\(|writeEeprom)", restore):
        bad.append("RESTORE only reads torque and restores RAM")
    return bad


def main():
    files = {name: (SRC / name).read_text() for name in REQUIRED}
    failures = violations(files)
    checks = 1
    for name, tokens in REQUIRED.items():
        for token in tokens:
            changed = dict(files)
            changed[name] = files[name].replace(token, "MUTATED_GUARD")
            checks += 1
            if not violations(changed):
                failures.append(f"undetected guard mutation: {name}: {token}")
    for name, mutation in (
        ("core/Controller.cpp", "motion_executor_.arm(facts,envelope,feedback,now);"),
        ("network/HttpTransport.cpp", "service_->acquire();"),
    ):
        changed = dict(files)
        changed[name] += mutation
        checks += 1
        if not violations(changed):
            failures.append(f"undetected ownership mutation: {mutation}")
    changed = dict(files)
    planner = "motion_coordinator_.update(operational_facts_,operational_feedback_,now_ms);"
    executor = "motion_executor_.update(operational_facts_,operational_feedback_,now_ms);"
    changed["core/Controller.cpp"] = files["core/Controller.cpp"].replace(planner, "SWAP_PLACEHOLDER").replace(executor, planner).replace("SWAP_PLACEHOLDER", executor)
    checks += 1
    if not violations(changed):
        failures.append("undetected actuator-before-support ordering mutation")
    for failure in failures:
        print("FAIL:", failure)
    print(f"MOTION_SAFETY_BOUNDARIES: {checks} checks, {len(failures)} failures; hardware qualification=false")
    return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
