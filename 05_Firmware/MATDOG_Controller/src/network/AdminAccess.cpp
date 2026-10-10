#include "AdminAccess.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
namespace matdog { namespace network {
bool AdminAccess::begin() { mutex_ = xSemaphoreCreateMutex(); return mutex_ != nullptr; }
bool AdminAccess::lock() {
  return mutex_ && xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), 0) == pdTRUE;
}
void AdminAccess::unlock() { xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_)); }
} }
