#include "src/core/Controller.h"

matdog::core::Controller controller;

// One Controller/UART owner; fixed stack, no browser work in Motion.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

void setup() {
  controller.begin();
}

void loop() {
  controller.update(millis());
  delay(1); // FreeRTOS idle and HTTP tasks; <=1 ms release polling granularity.
}
