#pragma once
#include "FreeRTOS.h"
using QueueHandle_t = void *;
QueueHandle_t xQueueCreate(unsigned, size_t);
void vQueueDelete(QueueHandle_t);
BaseType_t xQueueSend(QueueHandle_t, const void *, TickType_t);
BaseType_t xQueueReceive(QueueHandle_t, void *, TickType_t);
