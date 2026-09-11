#include "utils.h"

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
    bool is_valid = !((bool)((calc_checksum)^(checksum)));
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