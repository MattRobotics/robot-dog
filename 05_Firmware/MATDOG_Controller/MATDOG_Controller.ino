#include "src/core/Controller.h"

matdog::core::Controller controller;

void setup() {
  controller.begin();
}

void loop() {
  controller.update(millis());
}
