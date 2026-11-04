/**
 * @file utils.h
 * @author Melvin Gustavo Maradiaga Elvir
 * @date 24-08-2026
 *
 * @brief 
 * Utility functions and data structures for the medicine absorption system.
 */

#include <stdio.h>

/* Constants */
#define PLANT_PKT_HEADER 0xAA55
#define CONTROLLER_PKT_HEADER 0xFF66

/* Macros */
#define HZ_TO_US(freq_hz) (1000000UL / (freq_hz))

/* Data Structures */
// Stores the state variables 
typedef struct __attribute__((packed)){
    float m; // Muscle
    float b; // Blood
    float o; // Organ
} state_vector_t;

/** 
 * We format our UART packages as follows
 * [HEADER][LEN][CRC][PAYLOAD][STOP]
 */

 // Data packet sent via UART by the controller
typedef struct __attribute__((packed)){
    uint16_t header;
    uint64_t counter;
    float u; // Control output; input to the plant
    uint8_t checksum;
} controller_pkt_t;

// Data packet sent via UART by the plant
typedef struct __attribute__((packed)){
    uint16_t header;
    uint64_t counter;
    float y; // Output of the plant
    uint8_t checksum;
} plant_pkt_t;

void reverse_packet(void *packet);
uint8_t xor_checksum(const uint8_t *data, size_t len);
bool xor_checksum_verify(const uint8_t *data, size_t len, uint8_t checksum);

void dump_bytes(const char *label, const uint8_t *buf, size_t len);