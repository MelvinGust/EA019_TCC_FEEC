/**
 * @file main.c
 * @author Melvin Gustavo Maradiaga Elvir
 * @date 29-09-2026
 * 
 * @brief 
 * My program's entrypoint. Depending on the value of the PLANT_MODULE_ENABLE KConfig variable, 
 * links either the plant_module or the controller module's binaries.
 * 
 * @details
 * These two modules communicate with each other via an UART "package" formatted as follows:
 * [HEADER][COUNTER][PAYLOAD][CRC]
 * Since UART is a serial communication protocol, data needs (to be decoded carefully. As such, both a HEADER 
 * (for identifying where the package starts) and a simple CRC (for identifying whether our data got corrupted) were 
 * implemented.
 *
 * Depending on the reference (or GOAL) sent by the user on their MQTT interface, the controller regulates its output as to
 * guarantee the plant tracks it. The potentiometers on the board adjust the plant's parameters dynamically, and can be used
 * to test the controller's robustness.
 *
 * @note
 * For extra information, check out controller.c, plant.c, utils.h and utils.c.
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
        ESP_LOGI(ESP_LOG_TAG, "Target detected: PLANT");
        ESP_ERROR_CHECK(plant_init());
    #else
        ESP_LOGI(ESP_LOG_TAG, "Target detected: CONTROLLER");
        ESP_ERROR_CHECK(controller_init());
    #endif
}
