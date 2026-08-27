# MATDOG — Phase 2A G2 Final Call-Graph Correction
## Resolving the last normative contradiction — 2026-08-13

## Status

```text
P2A-G0 = CLOSED / PASS
P2A-G1 = CLOSED / PASS
P2A-G2 = OPEN — AG-1..AG-5 ACCEPTED; this is the last consistency correction
P2A-G3 = NOT AUTHORIZED
```

Not an architecture reopen. **Unchanged:** W1..W23; the twelve engine operations; the
LF traces, constants and runtime modes; contact semantics; provenance; global safety;
G3 scope. No motion semantics change.

Applies to `MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md` at revision 2.2a
(`sha256 01fc11144619c350f8e9d78dbefd4ccb16fa63c9197ae8250fdc8b0c8dd1b581`,
HEAD `4c952cb70d13476ba960d3717a1a22658d84cd6c`).

---

## The contradiction

Revision 2.2a specified two incompatible call graphs.

```text
§8.3 SINK-2 said
    ONE private function constructs the RamRegister::GoalPosition envelope, and
    the successors of set_motor_goal_verified and set_startup_home_goal_verified
    BOTH CALL IT.
    => the direct callers of the raw constructor are those two writers.

M-27s said
    "the writing-helper set is exactly the two immutable sinks and
     the caller set is exactly the twelve §8.2 operations"
    => the direct callers of the raw constructor are the twelve operations.
```

Both cannot hold. If the two writers call the constructor, the twelve operations are
*indirect* callers, two levels up. A CI gate written against the second reading would
fail against an implementation built to the first.

Root cause: §8.3 introduced a third layer (the unique raw constructor) in revision 2.2,
but M-27s kept revision 2.1's two-layer phrasing, in which the "sinks" were themselves
the lowest boundary.

---

## Ruling — the frozen G3 GoalPosition call graph

```text
TWELVE reviewed engine motion operations                          §8.2
        |
        v   each calls exactly ONE of two private POLICY WRITERS
        |
   +----+---------------------------+
   |                                |
A. startup-home policy writer   B. normal armed-goal policy writer
   semantics inherited from        semantics inherited from
   set_startup_home_goal_verified  set_motor_goal_verified
   used ONLY for W1, W2, W3        used for W4 .. W23
   |                                |
   +----+---------------------------+
        |
        v
   ONE private raw GoalPosition constructor / emitter
        |
        v
   RamRegister::GoalPosition
```

### Structural facts

```text
CG-1  Exactly ONE production location in the MATDOG implementation scope constructs the
      RamRegister::GoalPosition write envelope.
CG-2  The DIRECT caller set of that raw constructor is exactly TWO private policy
      writers: the startup-home writer and the normal armed-goal writer.
CG-3  The startup-home policy writer is reachable only from the reviewed startup
      operations covering W1/W2/W3.
CG-4  The normal armed-goal policy writer is reachable only from the remaining reviewed
      engine operations covering W4..W23.
CG-5  Across both policy writers the reviewed engine caller set is exactly the TWELVE
      §8.2 engine operations.
CG-6  No other production helper, direct constructor, third writer or caller exists.
CG-7  The two policy writers RETAIN their different historical authorization/gating
      semantics. The startup-home gate is NOT collapsed into the normal armed-goal gate
      merely to simplify the call graph.
CG-8  Terminal verified global torque OFF, hard abort and operator stop remain OUTSIDE
      this GoalPosition graph and continue to write TorqueEnable only.
```

### Why CG-7 matters

The two historical paths gate on **different motor sets**, verified in source:

```text
set_startup_home_goal_verified :3792  ->  write_startup_home_ram_verified :3830
    :3837  if !MATDOG_MOTOR_IDS.contains(&motor_id)          -> Err
           i.e. admits ANY of the twelve canonical motors
    :3840  plus a local ram_write_allowed_for_profile(..) pre-check

set_motor_goal_verified :4371         ->  write_motor_ram_verified :4436
    :4443  if !self.profile.allowed_motor_ids.contains(&motor_id) -> Err
           i.e. admits ONLY the armed profile's allowlist
```

This is not a stylistic difference. W1 normalizes **all twelve** motors and W2/W3 recover
home-only joints that are **not** participants of the armed profile. Routing them through
the normal armed-goal writer would reject those motors outright and break Full-mode
startup and legacy restart-safe entry. Conversely, routing W4..W23 through the startup
writer would drop the armed-profile allowlist from normal motion.

Collapsing the two writers is therefore a behavior change in both directions, and is
forbidden.

---

## Required corrections

### §8.3 — terminology and layering

Distinguish the immutable source from the G3 target, and stop calling all three
functions "sinks".

```text
IMMUTABLE V25 SOURCE TODAY
    two lowest historical GoalPosition sinks (:4371, :3792), each building its own
    write envelope.

G3 TARGET
    two private POLICY-WRITER successors of those sinks
        -> ONE private RAW CONSTRUCTOR / EMITTER
        -> RamRegister::GoalPosition

TERMINOLOGY
    raw constructor / emitter  the unique lowest construction boundary
    policy writers             the two reviewed semantic/gating wrappers
    engine operations          the twelve authority-owning callers
```

### M-27s — mechanically checkable against the frozen graph

```text
A. enumerate every RamRegister::GoalPosition occurrence in MATDOG SCOPE A;
   the PRODUCTION occurrence count must be exactly ONE
   (deliberate test literals are handled by the already-defined matdog_test.rs
   literal-scan exclusion).
B. identify the ONE raw constructor / emitter.
C. enumerate its DIRECT production callers; expected set = exactly the TWO policy writers.
D. enumerate callers of those two policy writers; their union must equal exactly the
   TWELVE §8.2 operations, with no other production caller.
E. verify startup-writer usage corresponds only to W1/W2/W3.
F. verify normal-writer usage corresponds only to W4..W23.
G. any direct GoalPosition constructor, third policy writer, third raw caller or
   unreviewed policy-writer caller FAILS the gate.

M-27s must NOT claim that the twelve operations directly call the raw constructor.
```

### §16.0b — namespace-escape inventory is recursive

The inventory must be **recursive** over
`software/drivers/st3215/src/auto_calibrate/` for `*.rs`, so a future nested non-matdog
path cannot escape because only top-level files were listed. AG-4 is otherwise unchanged.

### §22 — version label

The header says revision 2.2a while §22 still says `REVISION 2.2 COMPLETE`. Update the
executor-position and directly related revision-record wording to **2.2b**. No technical
content changes for the rename.

---

## Self-check required before commit

```text
GoalPosition production construction sites = 1
raw constructor direct callers             = 2 policy writers
engine operation callers                   = 12 reviewed operations
startup writer W coverage                  = W1..W3 only
normal writer W coverage                   = W4..W23 only
unreviewed production callers              = 0

W1..W23 semantically unchanged
twelve §8.2 operations unchanged
no motion behavior changed
namespace-escape .rs inventory recursive
contract version says Revision 2.2b consistently
no code / runtime / test / workflow file changed
```

---

## Scope

Authorized:

```text
create this record
correct §8.3, M-27s, §16.0b recursion and the §22 version label
one local documentation-only commit
```

Not authorized:

```text
architecture reopen              any .rs / .py / test / workflow change
Station / robot-dog              historical RF worktree
push / PR                        hardware / serial / EEPROM
G3
```

Documents that must remain byte-identical:

```text
37646f2eaf0f56af558b2450ada7908c44b8b1fdb394f3095fb99c25c651bc17  G0/G1 report
89df3920461d347d372f30fb44aa0eefb016543f45b4f5615d8b7309fd51433d  G1 gate decision
c51fb5d74c1752302e886febacde213c6748b70078ecc898ccd97efe5b9fbf6d  Codex review #1
7a8590437cefacae49f7e02c918602642de6a69caca29152669b374aea2670b2  G2 review #1 decision
e2760ad4413b526427f73fd5020ba1720a36bf28b657a569042599652947d533  Codex review #2
4f1c279fb0f91aaa512194aca8c2f9aaaefa6fa548b0cf8210d33b2ad5c5752c  G2 review #2 decision
77829ec1606835ca8d0ee0f86e986bb15d5971fe8e5e8f394e148f0c71cc7951  Codex review #3
475267d86fab6777fc89c2dfd7a48c6512e3d0092ed54551734d673792f82e80  G2 review #3 decision
cb0e938b42266b398df58fac50007d2e00d59dbd087f6da230369755c220ae2a  Codex final review
4afa6cbbe17ada225ebb31b013815aa3f7f71127b12fdd84458e9240ee5e0af7  G2 final review decision
9439d6ace8c561f160028921dd1e70199009ed6b333547f41a234d9f8edcf862  architecture-gate corrections
```
