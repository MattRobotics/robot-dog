#pragma once
#include "FreeRTOS.h"
using SemaphoreHandle_t = void *;
SemaphoreHandle_t xSemaphoreCreateMutex();
void vSemaphoreDelete(SemaphoreHandle_t);
BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t);
BaseType_t xSemaphoreGive(SemaphoreHandle_t);
