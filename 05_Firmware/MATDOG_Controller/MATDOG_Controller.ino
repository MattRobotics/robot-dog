#include "src/core/Controller.h"

matdog::core::Controller controller;

// Loop-task stack (2026-10-16): the default 8 KB is below the measured worst
// Controller-thread call chains (STAND command via the HTTP mailbox -> pre-arm
// entry planning; STAND route start; WALK IK), 8.5-10 KB by objdump frame sums.
// 16 KB keeps ~6 KB of margin; the multi-KB temporaries were also moved to
// static storage. A stack overflow corrupted the heap (boot loop, Issue #52 flash).
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

void setup() {
  controller.begin();
}

void loop() {
  controller.update(millis());
}
