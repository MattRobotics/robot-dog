#pragma once
#include "FreeRTOS.h"
BaseType_t xTaskCreatePinnedToCore(void (*)(void *), const char *, uint32_t,
                                   void *, unsigned, void *, int);
unsigned uxTaskGetStackHighWaterMark(void *);
