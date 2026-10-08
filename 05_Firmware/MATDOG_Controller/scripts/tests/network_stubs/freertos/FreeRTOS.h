#pragma once
#include <cstddef>
#include <cstdint>
using BaseType_t = int;
using TickType_t = uint32_t;
#define pdTRUE 1
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
void vTaskDelay(TickType_t);
