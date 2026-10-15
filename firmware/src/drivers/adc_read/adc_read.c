/** 
 * @file adc_read.c
 * @author Melvin Gustavo Maradiaga Elvir
 * @date 15-10-2026
 *
 * @brief 
 * Driver file for the ADC Peripheral on the ESP32.
 * 
 * This source file is heavily based upon the one available in
 * https://github.com/espressif/esp-idf/blob/v6.1/examples/peripherals/adc/continuous_read/main/continuous_read_main.c
 * 
 * @details
 * 
 */

/* Preprocessor directives */

#include "adc_read.h"

#include <stdio.h>
#include <string.h>

#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "soc/soc_caps.h"

// For ESP_LOG
#define TAG "ADC_READ"

// For ADC configuration: Note only type 1 output is supported by the ESP32
#define ADC_READ_ADC_UNIT        ADC_UNIT_1
#define ADC_READ_ADC_ATTEN       ADC_ATTEN_DB_12
#define ADC_READ_ADC_CONV_MODE   ADC_CONV_SINGLE_UNIT_1
#define ADC_READ_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_READ_ADC_BIT_WIDTH   SOC_ADC_DIGI_MAX_BITWIDTH

// Convenient macros
#define CONVERSION_RESULTS_PER_FRAME 128
#define MAX_STORAGE_BUFFER_SIZE 2U*CONVERSION_RESULTS_PER_FRAME*SOC_ADC_DIGI_DATA_BYTES_PER_CONV

/* Data structures */
typedef struct {
    // For ADC configuration
    adc_continuous_handle_t adc1_handle;
    adc_channel_t channel_list[3];

    // For FreeRTOS or ESPIDF
    SemaphoreHandle_t read_semaphore;
    TaskHandle_t adc_read_task_handle;

    // For ADC processing
    uint32_t adc_output_chan4;
    uint32_t adc_output_chan6;
    uint32_t adc_output_chan7;
} adc_data_t;

/* Variables */
static adc_data_t local_data;

/* Initialization */
static void adc_read_task(void *pvParameters);
static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);

/* ========== CALLBACKS ========== */


/**
 * @brief
 * Notify our main processing task that the ADC continuous driver has done a sufficient amount of data conversions
 *
 * @param handle
 * @param edata
 * @param user_data
 * @return true
 * @return false
 */
static bool IRAM_ATTR conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(local_data.adc_read_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

/* ========== PUBLIC FUNCTIONS ========== */

esp_err_t adc_read_init(void)
{
    esp_err_t esp_err = ESP_OK;
    BaseType_t ret_code = pdPASS;

    #ifdef CONFIG_ADC_READ_DEBUG_LOG_ENABLED
        esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    #endif

    /* Local data initialization */

    // Init ADC config structure
    local_data.channel_list[0] = ADC_CHANNEL_4;
    local_data.channel_list[1] = ADC_CHANNEL_6;
    local_data.channel_list[2] = ADC_CHANNEL_7;

    // Init freeRTOS entities
    local_data.read_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(local_data.read_semaphore);
    if (!local_data.read_semaphore){
        esp_err = ESP_FAIL;
        ESP_LOGE(TAG, "Failed to init adc_read binary semaphore");
        return esp_err;
    }

    /* Task Initialization */

    // Creating main ADC driver task
    ret_code = xTaskCreatePinnedToCore(adc_read_task, "adc_driver", CONFIG_ADC_READ_STACK_SIZE, NULL,
                            CONFIG_ADC_READ_TASK_PRIORITY, &local_data.adc_read_task_handle, tskNO_AFFINITY);
    if (ret_code != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ADC READ task");
        return ESP_FAIL;
    }

    return esp_err;
}

// Used for returning ADC data onto other tasks.
float adc_chan4_read(void)
{  
    float result = 0;
    if (xSemaphoreTake(local_data.read_semaphore, portMAX_DELAY) == pdTRUE)
    {
        result = (float)local_data.adc_output_chan4;
        xSemaphoreGive(local_data.read_semaphore);
    }
    return result;
}

float adc_chan6_read(void)
{  
    float result = 0;
    if (xSemaphoreTake(local_data.read_semaphore, portMAX_DELAY) == pdTRUE)
    {
        result = (float)local_data.adc_output_chan6;
        xSemaphoreGive(local_data.read_semaphore);
    }
    return result;
}

float adc_chan7_read(void)
{  
    float result = 0;
    if (xSemaphoreTake(local_data.read_semaphore, portMAX_DELAY) == pdTRUE)
    {
        result = (float)local_data.adc_output_chan7;
        xSemaphoreGive(local_data.read_semaphore);
    }
    return result;
}

/* ========== PRIVATE FUNCTIONS ========== */

/**
 * @brief
 *
 * @param pvParameters
 */
static void adc_read_task(void *pvParameters)
{
    esp_err_t esp_err = ESP_OK;

    //* ADC Conversion Variables
    uint8_t adc_data_buffer[800];
    uint32_t rxLen = 0; // Stores real length of conversion results read via adc_continuous_read().

    //* ADC Average Variables
    uint32_t read_raw[ADC_CHANNEL_10 + 1] = {0}; // Getting "ADC_CHANNEL_10 (because its an enum) + 1" elements
    uint32_t read_count[ADC_CHANNEL_10 + 1] = {0};
    uint32_t avg_read_raw[ADC_CHANNEL_10 + 1] = {0};

    /* ADC Continuous Init */
    continuous_adc_init(local_data.channel_list, sizeof(local_data.channel_list) / sizeof(adc_channel_t), &local_data.adc1_handle);
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = conv_done_cb,
    };
    
    esp_err = adc_continuous_register_event_callbacks(local_data.adc1_handle, &cbs, NULL);
    if (esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register event callbacks");
    }

   esp_err = adc_continuous_start(local_data.adc1_handle);
   if(esp_err != ESP_OK){
        ESP_LOGE(TAG, "Failed to start ADC");
   }

    ESP_LOGD(TAG, "Initialized Angle Sensor.");

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait until a conversion frame is ready

        esp_err = adc_continuous_read(local_data.adc1_handle, adc_data_buffer, MAX_STORAGE_BUFFER_SIZE, &rxLen, 0);
        memset(read_raw, 0, sizeof(read_raw));
        memset(read_count, 0, sizeof(read_count));
        memset(avg_read_raw, 0, sizeof(avg_read_raw));
        // ESP_LOGD(TAG, "ret is [%x], rxLEN is [%" PRIu32"]", ret, rxLen);

        if (esp_err == ESP_OK)
        {
            adc_continuous_data_t parsed_data_buffer[rxLen  / SOC_ADC_DIGI_RESULT_BYTES];
            uint32_t num_parsed_samples = 0;

            // Parse our received data
            esp_err_t parse_ret = adc_continuous_parse_data(local_data.adc1_handle, adc_data_buffer, rxLen, parsed_data_buffer, &num_parsed_samples);
            if(parse_ret == ESP_OK){
                for (int i = 0; i < num_parsed_samples; i++)
                {
                    if(parsed_data_buffer[i].valid){
                        // ESP_LOGI(TAG, "ADC%d, Channel: %d, Value: %"PRIu32, parsed_data[i].unit + 1, parsed_data[i].channel, parsed_data[i].raw_data);
                        adc_channel_t adc_chan = parsed_data_buffer[i].channel;
                        read_raw[adc_chan] += parsed_data_buffer[i].raw_data;
                        read_count[adc_chan] += 1;
                    } else {
                    ESP_LOGW(TAG, "Invalid data [ADC%d_Ch%d_%"PRIu32"]", parsed_data_buffer[i].unit + 1, parsed_data_buffer[i].channel, parsed_data_buffer[i].raw_data);
                    }
                }
            } else {
                ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
            }
        }

        // Read value 
        for(adc_channel_t it = ADC_CHANNEL_0; it < ADC_CHANNEL_10; it++)
        {
            switch(it){
                case ADC_CHANNEL_4:
                    avg_read_raw[ADC_CHANNEL_4] = read_raw[ADC_CHANNEL_4] / read_count[ADC_CHANNEL_4];
                    if (xSemaphoreTake(local_data.read_semaphore, portMAX_DELAY) == pdTRUE)
                    {
                        local_data.adc_output_chan4 = avg_read_raw[ADC_CHANNEL_4];
                        xSemaphoreGive(local_data.read_semaphore);
                    }
                    break;
                case ADC_CHANNEL_6:
                    avg_read_raw[ADC_CHANNEL_6] = read_raw[ADC_CHANNEL_6] / read_count[ADC_CHANNEL_6];
                    if (xSemaphoreTake(local_data.read_semaphore, portMAX_DELAY) == pdTRUE)
                    {
                        local_data.adc_output_chan6 = avg_read_raw[ADC_CHANNEL_6];
                        xSemaphoreGive(local_data.read_semaphore);
                    }
                    break;
                case ADC_CHANNEL_7:
                    avg_read_raw[ADC_CHANNEL_7] = read_raw[ADC_CHANNEL_7] / read_count[ADC_CHANNEL_7];
                    if (xSemaphoreTake(local_data.read_semaphore, portMAX_DELAY) == pdTRUE)
                    {
                        local_data.adc_output_chan7 = avg_read_raw[ADC_CHANNEL_7];
                        xSemaphoreGive(local_data.read_semaphore);
                    }
                    break;
                default:
                    break;
            }
        }
    }

    adc_continuous_stop(local_data.adc1_handle);

    // Deinitialize semaphores
    vSemaphoreDelete(local_data.read_semaphore);
    vTaskDelete(NULL);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_handle_cfg = {
        .max_store_buf_size = MAX_STORAGE_BUFFER_SIZE, // The driver can stores 256 conv results.
        .conv_frame_size = CONVERSION_RESULTS_PER_FRAME * SOC_ADC_DIGI_DATA_BYTES_PER_CONV, // Conv frame stores 128 conv results
    };
    adc_continuous_new_handle(&adc_handle_cfg, &handle);

    adc_continuous_config_t adc_cfg = {
        .pattern_num = channel_num,
        .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_HIGH,
        .conv_mode = ADC_READ_ADC_CONV_MODE,
        .format = ADC_READ_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_READ_ADC_ATTEN;
        adc_pattern[i].channel = channel[i];
        adc_pattern[i].unit = ADC_READ_ADC_UNIT;
        adc_pattern[i].bit_width = ADC_READ_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    adc_cfg.adc_pattern = adc_pattern;
    adc_continuous_config(handle, &adc_cfg);

    *out_handle = handle;
}