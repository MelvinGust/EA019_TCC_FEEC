/** 
 * @file plant.c
 * @author Melvin Gustavo Maradiaga Elvir
 * @date 24-08-2026
 *
 * @brief 
 * Source file for the medicine absorption system in state-space representation.
 * 
 * @details
 * The controller module sends signals via UART to the plant module, which simulates the medicine absorption system.
 * 
 * @note
 * TODO: Check whether I should increase my internal UART data buffer.
 * TODO: Check whether my buffer sizes can handle the data reception rate from the controller. 
 * TODO: Check whether my stack sizes need to be changed, because of performance issues and memory loss on the system.
 */

/* Preprocessor directives */

#include "sdkconfig.h"
#include "plant.h"
#include "utils.h"

#include "esp_err.h"
#include <esp_log.h>

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
const float y_ob = 0.04;
const float u_m = 0.15;
const float u_b = 0.2;
const float u_o = 0.12;
const float D_m = 1.0;
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

    // For UART processing
    state_vector_t x;
    state_vector_t x_fut;
    uint64_t internal_counter; // To be used for the message we are sending
    uint64_t msg_counter; // To interpret the message we are receiving
    uint64_t last_msg_counter;
} system_data_t;

/* Variables */
static system_data_t local_data;

/* Initialization */
static esp_err_t uart_init(void);
static void uart_rx_task(void *pvParameters);
static void uart_process_task(void *pvParameters);

/* ----- PUBLIC FUNCTIONS ----- */

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

    #ifndef CONFIG_DEBUG_LOG_ENABLED
       //esp_log_level_set(TAG, ESP_LOG_ERROR);
    #endif

    // ! PUT INSIDE OF IFDEF
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    /* Local data initialization */
    ESP_LOGI(TAG, "INIT 1");

    // Initialize buffer
    local_data.rx_len = 0;

    // Initialize time
    local_data.internal_counter = 0;
    local_data.msg_counter = 0;
    local_data.last_msg_counter = 0;

    // Initialize freeRTOS entities
    local_data.internal_data_queue = xQueueCreate(5, sizeof(received_pkt_t));

    /* Peripheral Initialization */
    // Init UART (RX/TX) communication
    esp_err = uart_init();
    if (esp_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UART");
        return ret_code;
    }
    
    ESP_LOGI(TAG, "INIT 2");

    /* Task Initialization */
    // Create the UART event task, which will create events based on the UARTs FIFO interrupt, and notify the process task.
    ret_code = xTaskCreate(uart_rx_task, "uart_event_task", CONFIG_PLANT_UART_RX_TASK_STACK_SIZE, NULL, CONFIG_PLANT_UART_RX_TASK_PRIORITY, NULL);
    if (ret_code != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART RX task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "INIT 3");

    // Create the MAIN task, which will receive data from the UART, process it, and send the response back to the controller.
    ret_code = xTaskCreate(uart_process_task, "uart_process_task", CONFIG_PLANT_UART_PROCESS_STACK_SIZE, NULL, CONFIG_PLANT_UART_PROCESS_TASK_PRIORITY, NULL);
    if (ret_code != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART PROCESS task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "INIT 4");

    ESP_LOGI(TAG, "Plant Module init!");
    return esp_err;
}

/* ----- PRIVATE FUNCTIONS ----- */

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
    // ! CHANGE HOW UART DATA RECEPTION WORKS
    // ! look for the rxfifo_full_tresh parameter.
    uart_config_t uart_config = {
        .baud_rate = CONFIG_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_param_config(CONFIG_UART_PORT_NUM, &uart_config);

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

        if (xQueueReceive(local_data.uart_event_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            // Received UART data
            ESP_LOGI(TAG, "Received UART data, with event type: %d", event.type);

            // Depending on the event received, perform a different action.
            switch(event.type){
                case UART_DATA:
                    {
                        // ESP_LOGI(TAG, "UART 1");

                        // ! INITIAL DATA RECEPTION
                        // Read everything we have in our UART
                        uint8_t temp[128]; // Temporary buffer to hold the incoming data.
                        // ! This might be blocking our code until 128 bytes of data have arrived. Is it? Why does it run so slow.
                        int len = uart_read_bytes(CONFIG_UART_PORT_NUM, temp, sizeof(temp), portMAX_DELAY);

                        // ESP_LOGI(TAG, "UART 2");

                        // Check if uart_read_bytes() ran properly
                        if(len < 0){
                            ESP_LOGE(TAG, "Failed to read UART data");
                            esp_err = ESP_ERR_INVALID_SIZE;
                            break;
                        }

                        // ESP_LOGI(TAG, "UART 3");

                        memcpy(&local_data.rx_buffer[local_data.rx_len], temp, len); // Store in internal parser buffer
                        local_data.rx_len += len; // Increase buffer pointer

                        // ESP_LOGI(TAG, "UART 4");

                        // ! INTERNAL DATA PARSING
                        for(int i = 0; i < local_data.rx_len - 1; i++)
                        {

                            // Check for a valid header in our message

                            // ESP_LOGI(TAG, "UART 5: %02X", local_data.rx_buffer[i]);

                            // The ESP32 is little_endian, so the data I sent via UART is INVERTED upon arrival here
                            // 0xFF66 is stored in memory as [0x66][0xFF]
                            if((local_data.rx_buffer[i] == UART_RX_HEADER_END) && (local_data.rx_buffer[i + 1] == UART_RX_HEADER_START))
                            {
                                // Check if we have enough data for a full packet
                                if(i >= sizeof(received_pkt_t))
                                {

                                    ESP_LOGI(TAG, "UART 6");

                                    // Store a local copy
                                    received_pkt_t pkt;
                                    memcpy(&pkt, &local_data.rx_buffer[i], sizeof(received_pkt_t));
                                    
                                    // Reverse the packet to match the original data sent by the controller
                                    // reverse_packet(&pkt);

                                    // ESP_LOGI(TAG, "UART 7");

                                    // Check every element of the packet, along with the checksum explicitly
                                    if(xor_checksum_verify((uint8_t *)&pkt, sizeof(received_pkt_t) - sizeof(uint8_t), pkt.checksum))
                                    {
                                        // ! SEND TO DATA PROCESSING
                                        // If we reach this point, we have a valid packet. Send it to the processing task.
                                        // ESP_LOGI(TAG, "Received valid packet: counter=%llu, u.m=%f, u.b=%f, u.o=%f", pkt.counter, pkt.u.m, pkt.u.b, pkt.u.o);
                                        xQueueSend(local_data.internal_data_queue, &pkt, portMAX_DELAY);
                                        ESP_LOGI(TAG, "Received valid packet: counter=%llu, u.m=%f, u.b=%f, u.o=%f", pkt.counter, pkt.u.m, pkt.u.b, pkt.u.o);
                                    } else {
                                        // If the packet isnt valid, log and error and discard it.
                                        ESP_LOGE(TAG, "Invalid checksum for received packet");
                                    } 

                                    // Move the remaining data to the start of the buffer
                                    memmove(local_data.rx_buffer, &local_data.rx_buffer[i + sizeof(received_pkt_t)], local_data.rx_len - (i + sizeof(received_pkt_t)));
                                    local_data.rx_len -= (i + sizeof(received_pkt_t));
                                    i = -1; // Reset index to start checking from the beginning of the buffer
                                } else {
                                    // Not enough data for a full packet, wait for more data to arrive
                                    ESP_LOGI(TAG, "Not enough data for a full packet, waiting for more data...");
                                }
                            }
                        }
                        break;
                    }
                case UART_BREAK:
                    ESP_LOGV(TAG, "HW BREAK - Received 0x00");
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

static void uart_process_task(void *pvParameters)
{
    while(true) 
    {
        received_pkt_t pkt;

        // Wait for data from the UART RX task
        if (xQueueReceive(local_data.internal_data_queue, &pkt, portMAX_DELAY)){
            
            // Only process if the received data is new
            ESP_LOGI(TAG, "Received data from controller: counter=%llu, u.m=%f, u.b=%f, u.o=%f", pkt.counter, pkt.u.m, pkt.u.b, pkt.u.o);

            // ! MAY NEED TO CHANGE THIS TO ACTUALLY CHECK IF I RESET ANY ESP32
            local_data.msg_counter = pkt.counter;
            if(local_data.msg_counter != local_data.last_msg_counter)
            {
                // Update message counter for comparison
                // Update internal counter for the message we are sending
                local_data.internal_counter++;
                local_data.last_msg_counter = local_data.msg_counter;

                // Process the received data (x = A*x_prev + B*u and y = C*x)
                // Get x[k+1]
                local_data.x_fut.m = (1.0-y_mb-u_m)*local_data.x.m + (y_bm)*local_data.x.b + (D_m)*pkt.u.m;
                local_data.x_fut.b = (y_mb)*local_data.x.m + (1.0-y_bo-y_bm-u_b)*local_data.x.b + (y_ob)*local_data.x.o;
                local_data.x_fut.o = y_bo*local_data.x.b + (1.0-y_ob-u_o)*local_data.x.o;

                // Get y[k]
                state_vector_t y;
                y.m = c_m*local_data.x.m;
                y.b = c_b*local_data.x.b;
                y.o = c_o*local_data.x.o;

                // Update our internal state variables
                local_data.x = local_data.x_fut;

                // Send the output data back to the controller via UART
                int len;
                sent_pkt_t plant_msg;
                plant_msg.header = UART_TX_PKT_HEADER;
                plant_msg.counter = local_data.internal_counter;
                plant_msg.y = y;
                plant_msg.checksum = xor_checksum((uint8_t *)&plant_msg, sizeof(sent_pkt_t) - sizeof(uint8_t));
                len = uart_write_bytes(CONFIG_UART_PORT_NUM, &plant_msg, sizeof(sent_pkt_t));
                if (len < 0) {  
                    ESP_LOGE(TAG, "Failed to send data to controller");
                }

                // Log the data we are sending to the controller
                ESP_LOGI(TAG, "Sent data to controller: counter=%llu, y.m=%f, y.b=%f, y.o=%f", plant_msg.counter, plant_msg.y.m, plant_msg.y.b, plant_msg.y.o);

            } else {
                ESP_LOGW(TAG, "Received old data, ignoring...");
            }
        }
    }
} 