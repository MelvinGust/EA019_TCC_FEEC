/**
 * @file plant.h
 * @author Melvin Gustavo Maradiaga Elvir
 * @date 24-08-2026
 *
 * @brief
 * Header file for the medicine absorption system in state-space representation.
 *
 */

#include "esp_err.h"

/**
 * @brief Initializes the plant module.
 * @return ESP_OK if successful, otherwise an error code.
 */
esp_err_t plant_init(void);