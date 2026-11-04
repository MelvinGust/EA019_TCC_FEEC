#include "utils.h"
#include <esp_log.h>

static void reverse_byte_field(void *packet, size_t len);

/**
 * @brief Calculate the XOR checksum for a data buffer
 * @param data Pointer to the data buffer
 * @param len Length of the data buffer
 * @return The XOR checksum
 */
uint8_t xor_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief Verify the XOR checksum for a data buffer
 * @return 1 if valid, 0 is invalid
 */
bool xor_checksum_verify(const uint8_t *data, size_t len, uint8_t checksum)
{
    uint8_t calc_checksum = xor_checksum(data, len);
    bool is_valid = !((bool)((calc_checksum)^(checksum))); // ! Is this correct?
    return is_valid;
}

/**
 * @brief
 * Because of the ESP32s endianness, we need to reverse the order
 * of the data inside of our packet, as to work with it more easily in code.
 * 
 * @note
 * This code was made for this specific packet.
 * Any changes to it will imply in changes that need
 * to be made to this function.
 */
void reverse_packet(void *packet)
{
    uint8_t *pkt_ptr = packet;
    
    reverse_byte_field(pkt_ptr, sizeof(uint16_t)); // Reverse header
    reverse_byte_field(pkt_ptr + sizeof(uint16_t), sizeof(uint64_t)); // Reverse counter
    reverse_byte_field(pkt_ptr + sizeof(uint16_t) + sizeof(uint64_t), sizeof(state_vector_t)); // Reverse state vector
    // The checksum is a single byte, so it doesn't need to be reversed
}

static void reverse_byte_field(void *packet, size_t len)
{
    uint8_t *byte_ptr = packet;
    for (size_t i = 0; i < len / 2; ++i) {
        uint8_t temp = byte_ptr[i];
        byte_ptr[i] = byte_ptr[len - 1 - i];
        byte_ptr[len - 1 - i] = temp;
    }
}

/* ======== DEBUG FUNCTIONS ======== */

void dump_bytes(const char *label, const uint8_t *buf, size_t len)
{
    const size_t max_bytes = 32;
    size_t print_len = (len < max_bytes) ? len : max_bytes;

    char line[256];
    size_t used = 0;

    if (buf == NULL || len == 0) {
        ESP_LOGI("DEBUG", "%s: <empty>", label);
        return;
    }

    for (size_t i = 0; i < print_len; ++i) {
        int n = snprintf(&line[used], sizeof(line) - used, "%02X ", buf[i]);
        if (n < 0) {
            break;
        }
        used += (size_t)n;
    }

    ESP_LOGI("DEBUG", "%s (%zu bytes): %s%s",
             label,
             len,
             line,
             (len > max_bytes) ? "..." : "");
}
