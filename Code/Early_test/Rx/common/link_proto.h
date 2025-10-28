// common/link_proto.h
#pragma once
#include <stddef.h>
#include <stdint.h>

#define LINK_HDR       0xA5
#define LINK_VER       0x01

// HID-mouse payload: buttons, dx, dy, wheel (4 bytes)
// For now we'll always send 3 or 4 bytes; wheel can be 0 if unused.
typedef struct __attribute__((packed)) {
    uint8_t hdr;     // 0xA5
    uint8_t ver;     // 0x01
    uint16_t seq;    // rolling sequence
    uint8_t len;     // payload length in bytes (3 or 4)
    uint8_t payload[4]; // [btn, dx, dy, wheel]
    uint16_t crc;    // CRC16-CCITT over [hdr .. payload]
} link_mouse_frame_t;

// CRC16-CCITT (0x1021, init 0xFFFF, no xorout)
static inline uint16_t link_crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}
