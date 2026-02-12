#include "DisplaySync.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace {

SemaphoreHandle_t g_display_mutex = nullptr;

SemaphoreHandle_t get_mutex()
{
    if (!g_display_mutex) {
        g_display_mutex = xSemaphoreCreateRecursiveMutex();
    }
    return g_display_mutex;
}

}  // namespace

void display_lock()
{
    auto m = get_mutex();
    if (m) {
        xSemaphoreTakeRecursive(m, portMAX_DELAY);
    }
}

void display_unlock()
{
    auto m = get_mutex();
    if (m) {
        xSemaphoreGiveRecursive(m);
    }
}

