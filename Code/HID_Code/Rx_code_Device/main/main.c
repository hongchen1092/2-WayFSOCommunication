//-------------------------------------------------------------------------
// The University of Western Australia
// Faculty of Engineering and Mathematical Sciences
// ELEC5550 - Design Project
// 2-Way Free Space Optical Communications System
//-------------------------------------------------------------------------
// Team 27
// Author(s):
// 23171349 - Ralph Pilapil
// 24349076 - Karma Norbu
// 24159891 - Rigzing Sherpa
// 23280761 - Hongyuan Chen
// 24639919 - Udaya Kristhbuge
// 24577152 - Kaushika Hewa Panvila
//-------------------------------------------------------------------------
// File Name: main.c
// Description: HID Device side code
// Identify the custom UART FRAME for mouse and keyboard data and enumarate it to PC as a Standard Composite USB Device
// Dynamically Identify the connected device and initiates the correct framing
// Project Partners: UWA, ANFF
//-------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h" // Includes hid.h which defines the default structs

#define TAG "COMPOSITE_RX_DEVICE"

/* ---------- UART (must match TX settings) ---------- */
#define UART_NUM       UART_NUM_1
#define UART_RX_GPIO   (18)
#define UART_BAUD      (115200)
#define UART_BUF_SIZE  (2048)

/* ---------- Protocol (must match TX settings) ---------- */
#define FRAME_HDR       (0xA5)
#define TYPE_MOUSE      (0x01)
#define TYPE_KEYBOARD   (0x02)

// --- FIX: REMOVED the redefinition of hid_mouse_report_t ---
// We will use the definition from TinyUSB's hid.h and map data manually.

// The keyboard struct from TinyUSB's hid.h is already correct (8 bytes)
// typedef struct { uint8_t modifier; uint8_t reserved; uint8_t keycode[6]; } hid_keyboard_report_t;


/************* TinyUSB Descriptors ****************/
// Report IDs, Descriptors, Callbacks remain the same as before...
#define REPORT_ID_KEYBOARD  1
#define REPORT_ID_MOUSE     2
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

const uint8_t hid_report_descriptor[] = { /* ... */
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(REPORT_ID_MOUSE))
};
const char* hid_string_descriptor[5] = { /* ... */
    (char[]){0x09, 0x04}, "Espressif", "Composite HID Link", "123456", "HID Interface",
};
static const uint8_t hid_configuration_descriptor[] = { /* ... */
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) { (void)instance; return hid_report_descriptor; }
uint16_t tud_hid_get_report_cb(uint8_t i, uint8_t r_id, hid_report_type_t rt, uint8_t* b, uint16_t rl) { (void)i; (void)r_id; (void)rt; (void)b; (void)rl; return 0; }
void tud_hid_set_report_cb(uint8_t i, uint8_t r_id, hid_report_type_t rt, uint8_t const* b, uint16_t bs) { (void)i; (void)r_id; (void)rt; (void)b; (void)bs; }

/* ---------- Application Code ---------- */
static void uart_init(void) {
    const uart_config_t cfg = {
        .baud_rate = UART_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void uart_to_hid_task(void *arg) {
    // Buffer for UART data (Header + Type + Max Payload)
    uint8_t uart_data[1 + 1 + sizeof(hid_keyboard_report_t)]; // Max size is for keyboard report (8 bytes payload)

    while (1) {
        // 1. Read header byte
        int bytes_read = uart_read_bytes(UART_NUM, uart_data, 1, portMAX_DELAY);
        if (bytes_read <= 0) continue;

        // 2. Check header
        if (uart_data[0] != FRAME_HDR) {
            ESP_LOGW(TAG, "Sync Error: Expected HDR 0xA5, Got 0x%02X", uart_data[0]);
            uart_flush_input(UART_NUM);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // 3. Read type byte
        bytes_read = uart_read_bytes(UART_NUM, &uart_data[1], 1, pdMS_TO_TICKS(10));
        if (bytes_read != 1) {
            ESP_LOGW(TAG, "Timeout waiting for packet type");
            uart_flush_input(UART_NUM);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // 4. Determine expected payload length
        uint8_t type = uart_data[1];
        size_t payload_len = 0;
        if (type == TYPE_MOUSE) {
            payload_len = 4; // TX sends 4 bytes for mouse
        } else if (type == TYPE_KEYBOARD) {
            payload_len = sizeof(hid_keyboard_report_t); // TX sends 8 bytes for keyboard
        } else {
            ESP_LOGE(TAG, "Unknown Packet Type: 0x%02X", type);
            uart_flush_input(UART_NUM);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // 5. Read payload into the correct offset in the buffer
        bytes_read = uart_read_bytes(UART_NUM, &uart_data[2], payload_len, pdMS_TO_TICKS(20));
        if (bytes_read != payload_len) {
             ESP_LOGW(TAG, "Incomplete Payload: Type 0x%02X, Expected %d, Got %d", type, payload_len, bytes_read);
             uart_flush_input(UART_NUM);
             vTaskDelay(pdMS_TO_TICKS(1));
             continue;
        }

        // 6. Process complete packet
        if (type == TYPE_MOUSE) {
            // --- START OF FIX ---
            // Manually map the 4 received bytes (uart_data[2] to uart_data[5])
            // into the fields expected by tud_hid_mouse_report.
            uint8_t tx_buttons = uart_data[2];
            int8_t  tx_dx      = (int8_t)uart_data[3];
            int8_t  tx_dy      = (int8_t)uart_data[4];
            int8_t  tx_wheel   = (int8_t)uart_data[5];

            ESP_LOGI(TAG, "RX Mouse: B:0x%02X, dx:%d, dy:%d, w:%d", tx_buttons, tx_dx, tx_dy, tx_wheel);
            if (tud_hid_ready()) {
                // Use the mapped values in the correct argument positions
                tud_hid_mouse_report(REPORT_ID_MOUSE, tx_buttons, tx_dx, tx_dy, tx_wheel, 0); // 0 for pan
            }
            // --- END OF FIX ---
        } else { // TYPE_KEYBOARD
            // Cast directly as the keyboard struct matches
            hid_keyboard_report_t *report = (hid_keyboard_report_t*)&uart_data[2];
            ESP_LOGI(TAG, "RX Kbd: M:0x%02X, K:[%02X %02X %02X]", report->modifier, report->keycode[0], report->keycode[1], report->keycode[2]);
            if (tud_hid_ready()) {
                tud_hid_keyboard_report(REPORT_ID_KEYBOARD, report->modifier, report->keycode);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Keep yield for stability

    } // end while(1)
}


void app_main(void) {
    uart_init();
    ESP_LOGI(TAG, "USB composite device initialization");

    const tinyusb_config_t tusb_cfg = { /* ... */
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    xTaskCreate(uart_to_hid_task, "uart_to_hid", 4096, NULL, 10, NULL);
}
