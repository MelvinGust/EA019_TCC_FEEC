/**
 * @file main.c
 * @author Melvin Gustavo Maradiaga Elvir
 * @date 24-08-2026
 * 
 * @brief 
 * Entry point of our application.
 * To initialize either plant or the controller, select the appropriate config on the menuconfig.
 * 
 * @note Current version only establishes intercommunication between modules. MQTT is to be implemented in a future update.
 */

/* Preprocessor directives */
// Headers
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"

#if CONFIG_PLANT_MODULE_ENABLE
    #include "plant.h"
#else
    #include "controller.h"
#endif

#define ESP_LOG_TAG "MAIN"

/* Initialization */
void app_main(void)
{
    ESP_LOGI("MAIN", "Booting unified firmware...");

    #if CONFIG_PLANT_MODULE_ENABLE
        ESP_LOGI(ESP_LOG_TAG, "Initializing PLANT target...");
        ESP_ERROR_CHECK(plant_init());
    #else
        ESP_LOGI(ESP_LOG_TAG, "Target detected: CONTROLLER");
        ESP_ERROR_CHECK(controller_init());
    #endif
}
