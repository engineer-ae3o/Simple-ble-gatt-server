#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble.hpp"

#include "esp_log.h"
#include "esp_err.h"


extern "C" {
    void app_main() {
        ESP_ERROR_CHECK(ble::init());
        ESP_ERROR_CHECK(ble::start());
        while (1) {
            ESP_LOGI("MAIN", "Logging every 10s to stay alive");
            vTaskDelay(pdMS_TO_TICKS(10000U));
        }
        ESP_ERROR_CHECK(ble::stop());
        ESP_ERROR_CHECK(ble::deinit());
    }
} // extern "C"
