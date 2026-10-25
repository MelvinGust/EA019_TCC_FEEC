/** 
 * @file plant.c
 * @author Melvin Gustavo Maradiaga Elvir
 * @date 29-09-2026
 *
 * @brief 
 * Source file for the medicine absorption system in state-space representation.
 * The plant receives the controller's output (float u) and attempts to simulate how medicine will be absorved by the body.
 * It returns as output (float y) the medicine's organ absorption rate, and sends it back to the controller, closing
 * the control loop.
 * 
 * @details
 * 
 */

/* Preprocessor directives */

#include "sdkconfig.h"
#include "plant.h"
#include "utils.h"
#include "adc_read.h"
#include "mqtt_client.h"

#include "protocol_examples_common.h"
#include "nvs_flash.h"

#include "esp_err.h"
#include "esp_timer.h"
#include <esp_log.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

// FOR ESP_LOG
#define TAG "PLANT"

// FOR UART_DRIVER
#define BUF_SIZE (1024)
#define UART_RX_PKT_HEADER CONTROLLER_PKT_HEADER
#define UART_TX_PKT_HEADER PLANT_PKT_HEADER
#define UART_RX_HEADER_START ((uint8_t)(UART_RX_PKT_HEADER >> 8))
#define UART_RX_HEADER_END ((uint8_t)(UART_RX_PKT_HEADER & 0xFF))

/* Constants */
const float y_mb = 0.095;
const float y_bm = 0.03;
const float y_bo = 0.095;
// const float y_ob = 0.04;
// const float u_m = 0.15;
const float u_b = 0.2;
// const float u_o = 0.12;
const float D_m = 0.01;
const float c_m = 0.0;
const float c_b = 0.0;
const float c_o = 1.0;

/* Data structures */
typedef controller_pkt_t received_pkt_t;
typedef plant_pkt_t sent_pkt_t;

typedef struct {
    // Data for UART reception
    uint8_t rx_buffer[512]; // Parser buffer
    size_t rx_len; // Current position in rx_buffer
    QueueHandle_t uart_event_queue;
    QueueHandle_t internal_data_queue;

    // For FreeRTOS or ESPIDF
    SemaphoreHandle_t mqtt_mutex;
    SemaphoreHandle_t internals_mutex;
    esp_timer_handle_t mqtt_timer_handle;
    esp_timer_handle_t plant_internals_timer_handle;

    // For MQTT processing
    esp_mqtt_client_handle_t mqtt_client;

    // For UART processing
    state_vector_t x;
    state_vector_t x_fut;
    uint64_t internal_counter; // To be used for the message we are sending
    uint64_t msg_counter; // To interpret the message we are receiving
    uint64_t last_msg_counter;

    // For internal state variation
    float y_ob;
    float u_m;
    float u_o;
} system_data_t;

typedef struct {
    float y;
    state_vector_t x;
} mqtt_plant_t;

/* Variables */
static system_data_t local_data;
static mqtt_plant_t mqtt_data = {0};

/* Initialization */
static void mqtt_timer_callback(void *arg);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static esp_err_t mqtt_app_init(void);
static esp_err_t uart_init(void);
static void uart_rx_task(void *pvParameters);
static void uart_process_task(void *pvParameters);

/* ========== CALLBACKS ========== */

/**
 * @brief 
 * MQTT Internal State Publishing
 */
static void mqtt_timer_callback(void *arg)
{
    char mqtt_msg[128];
    char internals_msg[128];
    int len;

    // Publish data onto the state topic
    if(xSemaphoreTake(local_data.mqtt_mutex, portMAX_DELAY) == pdTRUE)
    {
        len = snprintf(mqtt_msg, sizeof(mqtt_msg),
            "{\"y\":%.3f,\"x_b\":%.3f,\"x_m\":%.3f,\"x_o\":%.3f}",
            mqtt_data.y, mqtt_data.x.b, mqtt_data.x.m, mqtt_data.x.o);
        xSemaphoreGive(local_data.mqtt_mutex);

        esp_mqtt_client_publish(local_data.mqtt_client, "plant/state", mqtt_msg, len, 1, 0);
    }

    // Publish data onto the internals topic
    if(xSemaphoreTake(local_data.internals_mutex, portMAX_DELAY) == pdTRUE)
    {
        len = snprintf(internals_msg, sizeof(internals_msg),
            "{\"y_ob\":%.3f,\"u_m\":%.3f,\"u_o\":%.3f}",
            local_data.y_ob, local_data.u_m, local_data.u_o);
        xSemaphoreGive(local_data.internals_mutex);

        esp_mqtt_client_publish(local_data.mqtt_client, "plant/internals", internals_msg, len, 1, 0);
    }
}

static void plant_internals_callback(void *arg){
    // Update the internal plant state according to the voltage values read by the potentiometer
    if(xSemaphoreTake(local_data.internals_mutex, portMAX_DELAY) == pdTRUE)
    {
        local_data.y_ob = (float)(0.003 / 4095.0) * adc_chan4_read() + (float)(0.037);
        local_data.u_m = (float)(0.1 / 4095.0) * adc_chan6_read() + (float)(0.1);
        local_data.u_o = (float)(0.05 / 4095.0) * adc_chan7_read() + (float)(0.1);
        xSemaphoreGive(local_data.internals_mutex);
    }
    
}


/**
 * @brief 
 * MQTT event handler callback. Currently, serves as a debug event handler.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);

    switch ((esp_mqtt_event_id_t)event_id) 
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d, return code=0x%02x ", event->msg_id, (uint8_t)*event->data);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;

        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

/* ========== PUBLIC FUNCTIONS ========== */

/**
 * @brief Initializes the plant module.
 * @return ESP_OK if successful, otherwise an error code.
 * 
 * @details
 * 
 */
esp_err_t plant_init(void)
{
    esp_err_t esp_err = ESP_OK;
    BaseType_t ret_code = pdPASS;

    #ifdef CONFIG_PLANT_DEBUG_LOG_ENABLED
       esp_log_level_set(TAG, ESP_LOG_ERROR);
    #endif

    /* Local data initialization */

    // Initilize with dummy data
    local_data.y_ob = 0.04;
    local_data.u_m = 0.15;
    local_data.u_o = 0.12;

    // Initialize buffer
    local_data.rx_len = 0;

    // Initialize time
    local_data.internal_counter = 0;
    local_data.msg_counter = 0;
    local_data.last_msg_counter = 0;

    // Initialize freeRTOS entities
    local_data.internal_data_queue = xQueueCreate(5, sizeof(received_pkt_t));
    local_data.mqtt_mutex = xSemaphoreCreateMutex();
    local_data.internals_mutex = xSemaphoreCreateMutex();

    /* Peripheral Initialization */

    // Init UART (RX/TX) communication
    esp_err = uart_init();
    if (esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UART");
        return ret_code;
    }

    // Init MQTT 
    esp_err = mqtt_app_init();
    if(esp_err != ESP_OK) { 
        ESP_LOGE(TAG, "Failed to initialize MQTT plugin");
    }

    //  Init ADC
    esp_err = adc_read_init();
    if(esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize the ADC peripheral");
    }

    // Create MQTT publisher timer
    const esp_timer_create_args_t timer_args = {.callback = &mqtt_timer_callback, .name = "mqtt_publisher_timer"};
    esp_err = esp_timer_create(&timer_args, &local_data.mqtt_timer_handle);
    if (esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config mqtt timer");
        return esp_err;
    }

    // Create plant internals timer
    const esp_timer_create_args_t internals_timer_args = {.callback = &plant_internals_callback, .name = "plant_internals_timer"};
    esp_err = esp_timer_create(&internals_timer_args, &local_data.plant_internals_timer_handle);
    if (esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config plant internals timer");
        return esp_err;
    }

    /* Task Initialization */

    // Create the MAIN task, which will receive data from the UART, process it, and send the response back to the controller.
    ret_code = xTaskCreate(uart_process_task, "uart_process_task", CONFIG_PLANT_UART_PROCESS_STACK_SIZE, NULL, CONFIG_PLANT_UART_PROCESS_TASK_PRIORITY, NULL);
    if (ret_code != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART PROCESS task");
        return ESP_FAIL;
    }

    // Create the UART event task, which will create events based on the UARTs FIFO interrupt, and notify the process task.
    ret_code = xTaskCreate(uart_rx_task, "uart_rx_task", CONFIG_PLANT_UART_RX_TASK_STACK_SIZE, NULL, CONFIG_PLANT_UART_RX_TASK_PRIORITY, NULL);
    if (ret_code != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART RX task");
        return ESP_FAIL;
    }

    // Start the MQTT publisher timer. Which every tick will publish data onto the MQTT broker.
    esp_err = esp_timer_start_periodic(local_data.mqtt_timer_handle, HZ_TO_US(CONFIG_PUBLISH_FREQUENCY));
    if (esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT publisher timer");
        return esp_err;
    }

    // Start the Plant internals update timer. Every tick updates the variable plant parameters according to the values read from the ADC.
        esp_err = esp_timer_start_periodic(local_data.plant_internals_timer_handle, HZ_TO_US(10));
    if (esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start plant internals update timer");
        return esp_err;
    }

    ESP_LOGI(TAG, "Plant Module init!");
    return esp_err;
}

/* ========== PRIVATE FUNCTIONS ========== */

/**
 * @brief Initializes the UART driver for communication with the controller.
 * @return ESP_OK if successful, otherwise an error code.
 * 
 * @details
 * By default, this function configures the ESP32's UART as follows:
 * - Baud rate: 115200
 * - Data bits: 8
 * - Parity: None
 * - Stop bits: 1
 * - Flow control: None (RTS/CTS disabled)
 * - Source clock: Default (APB)
 * Since this UART is used for both data transmission and data reception, note both TX and RX have a reasonable buffer size.
 */
static esp_err_t uart_init(void)
{
    esp_err_t esp_err = ESP_OK;

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = CONFIG_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    // Install UART driver with RX and TX buffers
    esp_err = uart_driver_install(CONFIG_UART_PORT_NUM, CONFIG_UART_BUF_SIZE * 2, CONFIG_UART_BUF_SIZE, CONFIG_UART_EVENT_QUEUE_SIZE, &local_data.uart_event_queue, 0);
    if( esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver");
        return esp_err;
    }

    // Internally configure our UART parameters
    esp_err = uart_param_config(CONFIG_UART_PORT_NUM, &uart_config);
    if( esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters");
        return esp_err;
    }

    // Configure UART_DATA event to happen with 100 bytes on the RX buffer.
    esp_err = uart_set_rx_full_threshold(CONFIG_UART_PORT_NUM, 100);
    if( esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set rx full threshold");
        return esp_err;
    }

    // Initialize UART pins (TX and RX)
    esp_err = uart_set_pin(CONFIG_UART_PORT_NUM, CONFIG_UART_TXD_PIN, CONFIG_UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if( esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UART pins");
        return esp_err;
    }

    ESP_LOGI(TAG, "UART initialized successfully");

    return esp_err;    
}


/**
 * @brief Initializes the ESP32s WIFI driver and connects to MQTT.
 * @return ESP_OK if successful, otherwise an error code.
 * 
 * @details
 * This function initializes the peripherals needed to use the ESP32s WIFI.
 * As currently configured, it connects onto my laptop's hotspot and then establishes a connection with
 * the MOSQUITTO broker that's running in the background of my pc. Any other device that connects to it can
 * communicate with my ESP32.
 * 
 * MQTT events are handled by the mqtt_event_handler function.
 */
static esp_err_t mqtt_app_init(void)
{   
    esp_err_t esp_err = ESP_OK;

    esp_err = nvs_flash_init();
    if( esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS memory");
        return esp_err;
    }

    esp_err = esp_netif_init();
    if( esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP NETIF");
        return esp_err;
    }

    esp_err = esp_event_loop_create_default();
    if( esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP Event Loop Library");
        return esp_err;
    }

    // TODO Implement an actual connect to WIFI algorithm.
    // ! example_connect is a BANDAID.
    esp_err = example_connect();
    if( esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        return esp_err;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://10.42.0.1:1883",
        .credentials = {
            .username = "melvingust",
            .authentication = {
                .password = "chihuahuaselvagem",
                },
        },
    };
    local_data.mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    esp_err = esp_mqtt_client_register_event(local_data.mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if( esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register event handler");
        return esp_err;
    }

    esp_err = esp_mqtt_client_start(local_data.mqtt_client);
    if( esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client");
        return esp_err;
    }

    return esp_err;
}

/**
 * @brief 
 * Task that handles UART events, before sending them to the main data processing task.
 * Only valid packets will be processed.
 * 
 * @param pvParameters Pointer to task parameters (not used).
 * 
 * @details
 * Basically, the ESP32s UART has a built-in 128 bytes FIFO which receives data from the RX pin.
 * Upon reaching 120 bytes, the FIFO signals for an interrupt to be fired, moving its data to the RX_BUFFER.
 * The UART can also fire a timeout, which will then signal for data to be written to the RX_BUFFER too.
 * Both these methods are valid for data reception, and they both post a UART_DATA event to event queue.
 * 
 * If the packet is valid, we send it onto the main processing task via a queue.
 */
static void uart_rx_task(void *pvParameters)
{
    uart_event_t event;
    while(true) 
    {
        esp_err_t esp_err = ESP_OK;

        // Lock until any UART event occurs

        if (pdTRUE == xQueueReceive(local_data.uart_event_queue, (void *)&event, pdMS_TO_TICKS(1000))) {
            
            // Received UART data
            ESP_LOGV(TAG, "Received UART data, with event type: %d", event.type);

            // Depending on the event received, perform a different action.
            switch(event.type){
                case UART_DATA:
                    {
                        // * INITIAL DATA RECEPTION
                        // Read everything we have in our UART
                        uint8_t temp[100]; // Temporary buffer to hold the incoming data.
                        int len = uart_read_bytes(CONFIG_UART_PORT_NUM, temp, sizeof(temp), pdMS_TO_TICKS(100));

                        // Check if uart_read_bytes ran properly.
                        if(len == 0){
                            ESP_LOGE(TAG, "Timed out on UART read.");
                            esp_err = ESP_ERR_TIMEOUT;
                            break;
                        } else if (len < 0){
                            ESP_LOGE(TAG, "Failed to read UART data");
                            esp_err = ESP_ERR_INVALID_SIZE;
                            break;
                        }

                        memcpy(&local_data.rx_buffer[local_data.rx_len], temp, len); // Store in internal parser buffer
                        local_data.rx_len += len; // Increase buffer pointer

                        // * INTERNAL DATA PARSING
                        int i = 0;
                        while(i + 1 < local_data.rx_len)
                        {

                            // Check for a valid header in our message

                            // The ESP32 is little_endian, so the data I sent via UART is INVERTED upon arrival here
                            // 0xFF66 is stored in memory as [0x66][0xFF]
                            if((local_data.rx_buffer[i] == UART_RX_HEADER_END) && (local_data.rx_buffer[i+1] == UART_RX_HEADER_START))
                            {
                                // Check if we have enough data left in out buffer for a full packet               
                                if(local_data.rx_len - i >= sizeof(received_pkt_t))
                                {

                                    // Store a local copy
                                    received_pkt_t pkt;
                                    memcpy(&pkt, &local_data.rx_buffer[i], sizeof(received_pkt_t));
                                    

                                    // Check every element of the packet, along with the checksum explicitly
                                    if(xor_checksum_verify((uint8_t *)&pkt, sizeof(received_pkt_t) - sizeof(uint8_t), pkt.checksum))
                                    {
                                        // * SEND TO DATA PROCESSING

                                        // If we reach this point, we have a valid packet. Send it to the processing task.
                                        xQueueSend(local_data.internal_data_queue, &pkt, portMAX_DELAY);
                                    } else {
                                        // If the packet isnt valid, log and error and discard it.
                                        ESP_LOGE(TAG, "Invalid checksum for received packet");
                                    } 

                                    size_t packet_offset = (size_t)i + sizeof(received_pkt_t);
                                    size_t remaining = local_data.rx_len - packet_offset;

                                    // Move the remaining data to the start of the buffer
                                    memmove(local_data.rx_buffer, &local_data.rx_buffer[i + sizeof(received_pkt_t)], local_data.rx_len - (i + sizeof(received_pkt_t)));
                                    local_data.rx_len = remaining;
                                    i = 0; // Reset index to start checking from the beginning of the buffer

                                } else {
                                    // Not enough data for a full packet, wait for more data to arrive
                                    ESP_LOGI(TAG, "Not enough data for a full packet, waiting for more data...");
                                }
                            }

                            i++;
                            vTaskDelay(pdMS_TO_TICKS(1)); // Added for safety.
                        }

                        break;
                    }
                case UART_BREAK:
                    ESP_LOGE(TAG, "HW BREAK - Received 0x01");
                    uart_flush_input(CONFIG_UART_PORT_NUM);
                    xQueueReset(local_data.uart_event_queue);
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGE(TAG, "HW FIFO OVERFLOW");
                    uart_flush_input(CONFIG_UART_PORT_NUM);
                    xQueueReset(local_data.uart_event_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGE(TAG, "RING BUFFER FULL");
                    uart_flush_input(CONFIG_UART_PORT_NUM);
                    xQueueReset(local_data.uart_event_queue);
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown event type: %d", event.type);
                    break;
            }
        }

        if(esp_err != ESP_OK) {
            ESP_LOGE(TAG, "UART RX task error: %s", esp_err_to_name(esp_err));
        }
    }
}

/**
 * @brief 
 * Task that processes the packets received from the plant.
 * 
 * @param pvParameters Pointer to task parameters (not used).
 * 
 * @details
 * This function reads the output (float y) and the counter the plant module sent, and updates our local_data structure.
 */
static void uart_process_task(void *pvParameters)
{
    while(true) 
    {
        received_pkt_t pkt;

        // Wait for data from the UART RX task
        if (xQueueReceive(local_data.internal_data_queue, &pkt, pdMS_TO_TICKS(1000)) == pdTRUE){
            
            // Only process if the received data is new
            ESP_LOGV(TAG, "Received data from controller: counter=%llu, u=%f", pkt.counter, pkt.u);
            local_data.msg_counter = pkt.counter;

            if(local_data.msg_counter != local_data.last_msg_counter)
            {
                // Update message counter for comparison
                local_data.last_msg_counter = local_data.msg_counter;

                // Process the received data (x = A*x_prev + B*u and y = C*x)
                // Get x[k+1]
                local_data.x_fut.m = (1.0-y_mb-local_data.u_m)*local_data.x.m + (y_bm)*local_data.x.b + (D_m)*pkt.u;
                local_data.x_fut.b = (y_mb)*local_data.x.m + (1.0-y_bo-y_bm-u_b)*local_data.x.b + (local_data.y_ob)*local_data.x.o;
                local_data.x_fut.o = y_bo*local_data.x.b + (1.0-local_data.y_ob-local_data.u_o)*local_data.x.o;

                // Get y[k] (Since the other values are zeroed, DONT calculate them)
                float y;
                float temp_y;
                temp_y = c_o*local_data.x.o;
                if(temp_y > 0){
                    y = temp_y;
                } else {
                    y = 0.0;
                }

                // Update our internal state variables
                local_data.x = local_data.x_fut;


                if(xSemaphoreTake(local_data.mqtt_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    mqtt_data.x.b = local_data.x.b;
                    mqtt_data.x.m = local_data.x.m;
                    mqtt_data.x.o = local_data.x.o;
                    mqtt_data.y = y;
                    xSemaphoreGive(local_data.mqtt_mutex);
                }

                // Send the output data back to the controller via UART
                int len;
                sent_pkt_t plant_msg;
                local_data.internal_counter++;
                plant_msg.header = UART_TX_PKT_HEADER;
                plant_msg.counter = local_data.internal_counter;
                plant_msg.y = y;
                plant_msg.checksum = xor_checksum((uint8_t *)&plant_msg, sizeof(sent_pkt_t) - sizeof(uint8_t));
                len = uart_write_bytes(CONFIG_UART_PORT_NUM, &plant_msg, sizeof(sent_pkt_t));
                if (len < 0) {  
                    ESP_LOGE(TAG, "Failed to send data to controller");
                }

            } else {
                ESP_LOGW(TAG, "Received old data, ignoring...");
            }
        }
       vTaskDelay(pdMS_TO_TICKS(1)); // Added so FreeRTOS' idle tasks can run, and reset their internal watchdogs.
    }
} 