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
// Description: Combined Final Code - Host Side
// Runs the state machine as per the connected USB Device and hosts it dynamically
// Includes msc_tx.c's uart_bridge_task for 2-way MSC communication.
// Correctly starts/stops uart_bridge_task on MSC connect/disconnect.
// Correctly calls hid_host_device_close() on HID disconnect.
// Correctly calls msc_host_uninstall_device() on MSC disconnect.
// Project Partners: UWA, ANFF
//-------------------------------------------------------------------------


#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <assert.h>
#include <endian.h> // <-- FIX: Added missing include from msc_tx.c

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/msc_host.h"

#ifndef PACKED
#define PACKED __attribute__((packed))
#endif

#define TAG "UNIFIED_HOST_FINAL"

/* ---------- UART Configuration & Protocol ---------- */
#define UART_NUM       UART_NUM_1
#define UART_TX_GPIO   (17)
#define UART_RX_GPIO   (18)
#define UART_BAUD      (460800)
#define UART_BUF_SIZE  (4096)

#define FRAME_HDR             (0xA5)
#define TYPE_ANNOUNCE_HID     (0x01)
#define TYPE_ANNOUNCE_MSC     (0x02)
#define TYPE_MOUSE            (0x03)
#define TYPE_KEYBOARD         (0x04)

/* --- MSC Bridge Protocol Codes (Copied from msc_tx.c) --- */
#define CMD_TEST_UNIT_READY   (0x00)
#define CMD_REQUEST_SENSE     (0x03)
#define CMD_READ_CAPACITY     (0x25)
#define CMD_READ10            (0x28)
#define CMD_WRITE10           (0x2A)
#define RSP_SUCCESS           (0x01)
#define RSP_FAILED            (0x02)

/* ---------- HID/MSC Structures ---------- */
typedef struct PACKED {
    uint8_t buttons;
    int8_t  dx;
    int8_t  dy;
    int8_t  wheel;
} hid_mouse_report_t;

typedef struct PACKED {
    uint8_t modifier;
    uint8_t reserved;
    uint8_t keycode[6];
} hid_keyboard_report_t;

// --- Copied from msc_tx.c ---
typedef struct { uint8_t key; uint8_t asc; uint8_t ascq; } scsi_sense_data_t;

/* ---------- App Events ---------- */
typedef enum {
    APP_EVENT_HID_CONNECT,
    APP_EVENT_HID_IF_DISCONNECT,
    APP_EVENT_MSC_CONNECT,
    APP_EVENT_MSC_DISCONNECT,
} app_event_id_t;

typedef struct {
    app_event_id_t event_id;
    union {
        hid_host_device_handle_t hid_hdl;
        msc_host_device_handle_t msc_hdl;
        uint8_t msc_addr;
    } data;
} app_event_msg_t;

static QueueHandle_t app_queue = NULL;

/* ---------- Global State ---------- */
typedef enum { MODE_NONE, MODE_HID, MODE_MSC } device_mode_t;
static volatile device_mode_t   g_current_mode = MODE_NONE;
static SemaphoreHandle_t        g_state_mutex = NULL;
static hid_host_device_handle_t g_hid_driver_handle = NULL;
static msc_host_device_handle_t g_msc_driver_handle = NULL;

/* --- MSC Bridge globals --- */
static msc_host_device_info_t   g_msc_info;
static uint8_t                 *g_msc_data_buffer = NULL;
static uint32_t                 g_msc_buffer_size = 512;
static SemaphoreHandle_t        g_msc_mutex = NULL;
static TaskHandle_t             g_msc_bridge_task_hdl = NULL;
static EventGroupHandle_t       g_msc_task_event_group = NULL;
#define MSC_TASK_STOP_BIT       (BIT0)
// --- Copied from msc_tx.c ---
static scsi_sense_data_t g_last_sense = {0};


/* ---------------- UART helpers ---------------- */
static void uart_init(void) {
    const uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk= UART_SCLK_DEFAULT,
    };
    uart_driver_delete(UART_NUM);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_GPIO, UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static inline void send_uart_bytes(const void *buf, size_t len) {
    uart_write_bytes(UART_NUM, (const char*)buf, len);
}

static void send_uart_packet(uint8_t type, const void* data, size_t len) {
    uint8_t hdr[2] = { FRAME_HDR, type };
    send_uart_bytes(hdr, 2);
    if (data && len) send_uart_bytes(data, len);
}

/* ---------------- USB Host core task ---------------- */
static void usb_host_task(void *arg) {
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All devices freed");
        }
    }
}

/* ---------------- HID interface callback ---------------- */
static void hid_interface_cb(hid_host_device_handle_t hid_dev_driver_hdl,
                             const hid_host_interface_event_t event,
                             void *arg)
{
    (void)arg;

    if (event == HID_HOST_INTERFACE_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "HID interface disconnected (hdl=%p)", hid_dev_driver_hdl);
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (g_current_mode == MODE_HID && g_hid_driver_handle == hid_dev_driver_hdl) {
                app_event_msg_t msg = { .event_id = APP_EVENT_HID_IF_DISCONNECT, .data.hid_hdl = hid_dev_driver_hdl };
                xQueueSend(app_queue, &msg, 0);
            }
            xSemaphoreGive(g_state_mutex);
        }

    } else if (event == HID_HOST_INTERFACE_EVENT_INPUT_REPORT) {

    if (xSemaphoreTake(g_state_mutex, 0) == pdTRUE) {
        if (g_current_mode == MODE_HID && hid_dev_driver_hdl == g_hid_driver_handle) {

            uint8_t report_data[64] = {0};
            size_t report_len = 0;
            esp_err_t err = hid_host_device_get_raw_input_report_data(
                                hid_dev_driver_hdl, report_data, sizeof(report_data), &report_len);
            if (err == ESP_OK && report_len > 0) {

                // Raw hex (first 32 bytes) for debugging
                char hex[3*32] = {0};
                int idx = 0;
                for (size_t i = 0; i < report_len && i < 32; i++) {
                    idx += snprintf(hex + idx, sizeof(hex) - idx, "%02X ", report_data[i]);
                }

                // Prefer protocol from params (present on your IDF)
                hid_host_dev_params_t p = {0};
                hid_protocol_t proto = HID_PROTOCOL_NONE;
                if (hid_host_device_get_params(hid_dev_driver_hdl, &p) == ESP_OK) {
                    proto = p.proto; // only field we rely on
                }

                // If proto is unknown, use a conservative heuristic
                if (proto != HID_PROTOCOL_MOUSE && proto != HID_PROTOCOL_KEYBOARD) {
                    if (report_len == sizeof(hid_keyboard_report_t)) {
                        proto = HID_PROTOCOL_KEYBOARD;
                    } else if (report_len >= 3 && report_len <= 5) {
                        proto = HID_PROTOCOL_MOUSE;
                    }
                }

                ESP_LOGD(TAG, "HID report (proto=%s, len=%u): %s",
                         (proto == HID_PROTOCOL_MOUSE ? "MOUSE" :
                          proto == HID_PROTOCOL_KEYBOARD ? "KBD" : "UNKNOWN"),
                         (unsigned)report_len, hex);

                if (proto == HID_PROTOCOL_MOUSE) {
                    // Boot mouse: [buttons, dx, dy, (wheel?)]
                    if (report_len >= 3) {
                        hid_mouse_report_t r = {0};
                        r.buttons = report_data[0];
                        r.dx      = (int8_t)report_data[1];
                        r.dy      = (int8_t)report_data[2];
                        r.wheel   = (report_len >= 5) ? (int8_t)report_data[4]
                                                      : ((report_len == 4) ? (int8_t)report_data[3] : 0);

                        ESP_LOGD(TAG, "Mouse: btn=%02X dx=%d dy=%d wheel=%d",
                                 r.buttons, r.dx, r.dy, r.wheel);

                        send_uart_packet(TYPE_MOUSE, &r, sizeof(r));
                    } else {
                        ESP_LOGW(TAG, "Mouse proto but short report (%u bytes)", (unsigned)report_len);
                    }

                } else if (proto == HID_PROTOCOL_KEYBOARD) {
                    if (report_len >= sizeof(hid_keyboard_report_t)) {
                        const hid_keyboard_report_t *k = (const hid_keyboard_report_t*)report_data;

                        ESP_LOGD(TAG, "Keyboard: mod=%02X keys=%02X %02X %02X %02X %02X %02X",
                                 k->modifier,
                                 k->keycode[0], k->keycode[1], k->keycode[2],
                                 k->keycode[3], k->keycode[4], k->keycode[5]);

                        send_uart_packet(TYPE_KEYBOARD, k, sizeof(hid_keyboard_report_t));
                    } else {
                        ESP_LOGW(TAG, "Keyboard proto but short report (%u bytes)", (unsigned)report_len);
                    }

                } else {
                    // Unknown proto — do nothing (or add custom parser if you want)
                    ESP_LOGW(TAG, "Unknown HID proto; report ignored.");
                }
            }
        }
        xSemaphoreGive(g_state_mutex);
    }



    } else if (event == HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR) {
        ESP_LOGW(TAG, "HID Transfer error (hdl=%p) — re-arming", hid_dev_driver_hdl);
        // Re-arm ONLY on error:
        esp_err_t e = hid_host_device_start(hid_dev_driver_hdl);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "hid re-start failed: %s", esp_err_to_name(e));
        }
    }
}

/* Queueing driver-level HID connect events into app task */
static void hid_driver_cb_queue(hid_host_device_handle_t dev_hdl,
                                const hid_host_driver_event_t event,
                                void *arg)
{
    (void)arg;
    if (event == HID_HOST_DRIVER_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "HID DRIVER CONNECTED (hdl=%p)", dev_hdl);
        app_event_msg_t msg = { .event_id = APP_EVENT_HID_CONNECT, .data.hid_hdl = dev_hdl };
        if (xQueueSend(app_queue, &msg, 0) != pdTRUE) { ESP_LOGE(TAG, "App queue full!"); }
    }
}

/* MSC host driver events queued into app task */
static void msc_event_cb_queue(const msc_host_event_t *event, void *arg) {
    (void)arg;
    if (event->event == MSC_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "MSC DRIVER CONNECTED (addr=%d)", event->device.address);
        app_event_msg_t msg = { .event_id = APP_EVENT_MSC_CONNECT, .data.msc_addr = event->device.address };
        if (xQueueSend(app_queue, &msg, 0) != pdTRUE) { ESP_LOGE(TAG, "App queue full!"); }
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        ESP_LOGI(TAG, "MSC DRIVER DISCONNECTED (hdl=%p)", event->device.handle);
        app_event_msg_t msg = { .event_id = APP_EVENT_MSC_DISCONNECT, .data.msc_hdl = event->device.handle };
        if (xQueueSend(app_queue, &msg, 0) != pdTRUE) { ESP_LOGE(TAG, "App queue full!"); }
    }
}

/* ------------------- NEW: MSC Task Logic (from msc_tx.c) ------------------- */

/**
 * @brief Helper to set the last SCSI sense data based on esp_err_t
 * (Copied from msc_tx.c)
 */
static void set_sense_from_err(esp_err_t err) {
    if (err == ESP_OK) {
        g_last_sense = (scsi_sense_data_t){.key = 0x00, .asc = 0x00, .ascq = 0x00}; // NO_SENSE
    } else if (err == ESP_ERR_TIMEOUT) {
        g_last_sense = (scsi_sense_data_t){.key = 0x02, .asc = 0x04, .ascq = 0x01}; // NOT_READY, LOGICAL_UNIT_COMMUNICATION_TIMEOUT
    } else if (err == ESP_ERR_INVALID_STATE) {
        g_last_sense = (scsi_sense_data_t){.key = 0x02, .asc = 0x3A, .ascq = 0x00}; // NOT_READY, MEDIUM_NOT_PRESENT
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
         g_last_sense = (scsi_sense_data_t){.key = 0x05, .asc = 0x20, .ascq = 0x00}; // ILLEGAL_REQUEST, INVALID_COMMAND_OPERATION_CODE
    } else {
        g_last_sense = (scsi_sense_data_t){.key = 0x04, .asc = 0x44, .ascq = 0x00}; // HARDWARE_ERROR
    }
}

/**
 * @brief The main task that waits for UART commands and processes them.
 * (Copied from msc_tx.c and adapted for this file)
 */
static void uart_bridge_task(void *arg)
{
    // Wait for the buffer to be allocated by app_main
    while (g_msc_data_buffer == NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "UART bridge task started");

    while ( (xEventGroupGetBits(g_msc_task_event_group) & MSC_TASK_STOP_BIT) == 0 ) {
        uint8_t cmd_byte;
        // Use a timeout to remain responsive to the STOP_BIT
        int len = uart_read_bytes(UART_NUM, &cmd_byte, 1, pdMS_TO_TICKS(100));
        if (len != 1) continue;

        uint8_t response_byte = RSP_SUCCESS;
        esp_err_t err = ESP_OK;
        set_sense_from_err(ESP_OK); // Clear sense

        // *** ADAPTATION: Use g_msc_mutex ***
        if (xSemaphoreTake(g_msc_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGE(TAG, "UART task failed to get mutex!");
            err = ESP_ERR_TIMEOUT;
            set_sense_from_err(err);
            response_byte = RSP_FAILED;
            // Send fail for all commands except REQUEST_SENSE
            if (cmd_byte == CMD_REQUEST_SENSE) {
                // SENSE is special, it must always succeed
                uint8_t sense_data[18] = {0};
                sense_data[0] = 0x70;
                sense_data[2] = g_last_sense.key;
                sense_data[7] = 10;
                sense_data[12] = g_last_sense.asc;
                sense_data[13] = g_last_sense.ascq;
                uart_write_bytes(UART_NUM, sense_data, 18);
                uart_write_bytes(UART_NUM, &response_byte, 1); // RSP_SUCCESS
                g_last_sense = (scsi_sense_data_t){0}; // Clear
            } else {
                uart_write_bytes(UART_NUM, &response_byte, 1);
            }
            continue; // Go wait for next command
        }

        // --- We now safely own the mutex ---
        // *** ADAPTATION: Use g_msc_driver_handle, g_msc_info, etc. ***

        switch (cmd_byte) {
            case CMD_READ_CAPACITY: {
                ESP_LOGD(TAG, "UART_CMD: READ_CAPACITY");
                if (g_msc_driver_handle == NULL) {
                    err = ESP_FAIL;
                    set_sense_from_err(ESP_ERR_INVALID_STATE);
                    // Send 0s for capacity
                    uart_write_bytes(UART_NUM, &((uint32_t){0}), 4);
                    uart_write_bytes(UART_NUM, &((uint32_t){0}), 4);
                } else {
                    uint32_t sector_count_le = htole32(g_msc_info.sector_count);
                    uint32_t sector_size_le = htole32(g_msc_info.sector_size);
                    uart_write_bytes(UART_NUM, &sector_count_le, 4);
                    uart_write_bytes(UART_NUM, &sector_size_le, 4);
                }
                response_byte = (err == ESP_OK) ? RSP_SUCCESS : RSP_FAILED;
                uart_write_bytes(UART_NUM, &response_byte, 1);
                uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(100));
                break;
            }

            case CMD_READ10: {
                uint32_t lba_le;
                uint16_t sector_count_le;
                len = uart_read_bytes(UART_NUM, &lba_le, 4, pdMS_TO_TICKS(500));
                if (len == 4) len += uart_read_bytes(UART_NUM, &sector_count_le, 2, pdMS_TO_TICKS(500));

                if (len != 6) {
                    ESP_LOGE(TAG, "UART read timeout on READ10 params");
                    err = ESP_ERR_TIMEOUT;
                } else {
                    uint32_t lba = le32toh(lba_le);
                    uint16_t sector_count = le16toh(sector_count_le);
                    ESP_LOGD(TAG, "UART_CMD: READ10 (LBA=%"PRIu32", Count=%u)", lba, sector_count);

                    if (g_msc_driver_handle == NULL) {
                        err = ESP_FAIL;
                        set_sense_from_err(ESP_ERR_INVALID_STATE);
                    } else if (g_msc_buffer_size < g_msc_info.sector_size) {
                        ESP_LOGE(TAG, "Internal error: Buffer size %"PRIu32" < Sector size %"PRIu32, g_msc_buffer_size, g_msc_info.sector_size);
                        err = ESP_FAIL;
                    } else {
                        // Read and send one sector at a time
                        for (uint16_t i = 0; i < sector_count; i++) {
                            err = msc_host_read_sector(g_msc_driver_handle, lba + i, g_msc_data_buffer, g_msc_buffer_size);
                            if (err != ESP_OK) {
                                ESP_LOGE(TAG, "msc_host_read_sector failed: %s", esp_err_to_name(err));
                                set_sense_from_err(err);
                                break;
                            }
                            int written = uart_write_bytes(UART_NUM, g_msc_data_buffer, g_msc_buffer_size);
                            if (written != g_msc_buffer_size) {
                                ESP_LOGE(TAG, "UART write error during READ10!");
                                err = ESP_FAIL;
                                break;
                            }
                        }
                    }
                }
                response_byte = (err == ESP_OK) ? RSP_SUCCESS : RSP_FAILED;
                uart_write_bytes(UART_NUM, &response_byte, 1);
                uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(100));
                break;
            }

            case CMD_WRITE10: {
                uint32_t lba_le;
                uint16_t sector_count_le;
                len = uart_read_bytes(UART_NUM, &lba_le, 4, pdMS_TO_TICKS(500));
                if (len == 4) len += uart_read_bytes(UART_NUM, &sector_count_le, 2, pdMS_TO_TICKS(500));

                if (len != 6) {
                    ESP_LOGE(TAG, "UART read timeout on WRITE10 params");
                    err = ESP_ERR_TIMEOUT;
                } else {
                    uint32_t lba = le32toh(lba_le);
                    uint16_t sector_count = le16toh(sector_count_le);
                    ESP_LOGD(TAG, "UART_CMD: WRITE10 (LBA=%"PRIu32", Count=%u)", lba, sector_count);

                    if (g_msc_driver_handle == NULL) {
                        err = ESP_FAIL;
                        set_sense_from_err(ESP_ERR_INVALID_STATE);
                        // We must still read and discard the data from UART
                        for (uint16_t i = 0; i < sector_count; i++) {
                            uart_read_bytes(UART_NUM, g_msc_data_buffer, g_msc_buffer_size, pdMS_TO_TICKS(1000));
                        }
                    } else if (g_msc_buffer_size < g_msc_info.sector_size) {
                        ESP_LOGE(TAG, "Internal error: Buffer size %"PRIu32" < Sector size %"PRIu32, g_msc_buffer_size, g_msc_info.sector_size);
                        err = ESP_FAIL;
                        // Read and discard data
                        for (uint16_t i = 0; i < sector_count; i++) {
                            uart_read_bytes(UART_NUM, g_msc_data_buffer, g_msc_buffer_size, pdMS_TO_TICKS(1000));
                        }
                    } else {
                        // Read from UART and write one sector at a time
                        for (uint16_t i = 0; i < sector_count; i++) {
                            int bytes_read = uart_read_bytes(UART_NUM, g_msc_data_buffer, g_msc_buffer_size, pdMS_TO_TICKS(5000));
                            if (bytes_read != g_msc_buffer_size) {
                                ESP_LOGE(TAG, "UART read timeout on WRITE10 data");
                                err = ESP_ERR_TIMEOUT;
                                break;
                            }
                            err = msc_host_write_sector(g_msc_driver_handle, lba + i, g_msc_data_buffer, g_msc_buffer_size);
                            if (err != ESP_OK) {
                                ESP_LOGE(TAG, "msc_host_write_sector failed: %s", esp_err_to_name(err));
                                set_sense_from_err(err);
                                break;
                            }
                        }
                    }
                }
                response_byte = (err == ESP_OK) ? RSP_SUCCESS : RSP_FAILED;
                uart_write_bytes(UART_NUM, &response_byte, 1);
                uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(500));
                break;
            }

            case CMD_TEST_UNIT_READY: {
                ESP_LOGD(TAG, "UART_CMD: TEST_UNIT_READY");
                if (g_msc_driver_handle == NULL) {
                    err = ESP_FAIL;
                    set_sense_from_err(ESP_ERR_INVALID_STATE);
                }
                // Note: A "real" TUR might send a command, but this is good enough.
                response_byte = (err == ESP_OK) ? RSP_SUCCESS : RSP_FAILED;
                uart_write_bytes(UART_NUM, &response_byte, 1);
                uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(100));
                break;
            }

            case CMD_REQUEST_SENSE: {
                ESP_LOGD(TAG, "UART_CMD: REQUEST_SENSE");
                // This command doesn't touch the device, just g_last_sense
                uint8_t sense_data[18] = {0};
                sense_data[0] = 0x70; // Response Code: Current Errors
                sense_data[2] = g_last_sense.key;
                sense_data[7] = 10; // Additional Sense Length
                sense_data[12] = g_last_sense.asc;
                sense_data[13] = g_last_sense.ascq;

                uart_write_bytes(UART_NUM, sense_data, sizeof(sense_data));
                response_byte = RSP_SUCCESS; // Always succeed in *sending* sense
                uart_write_bytes(UART_NUM, &response_byte, 1);
                uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(100));
                
                // Clear sense data after reporting it
                g_last_sense = (scsi_sense_data_t){0};

                break;
            }

            default:
                ESP_LOGW(TAG, "Unknown CMD: 0x%02x", cmd_byte);
                response_byte = RSP_FAILED;
                set_sense_from_err(ESP_ERR_NOT_SUPPORTED);
                uart_write_bytes(UART_NUM, &response_byte, 1);
                uart_flush_input(UART_NUM);
                break;
        }

        // *** ADAPTATION: Release g_msc_mutex ***
        xSemaphoreGive(g_msc_mutex);
    }

    ESP_LOGI(TAG, "UART bridge task stopping.");
    vTaskDelete(NULL);
}


/* ---------------- Main ---------------- */
void app_main(void) {
    g_state_mutex = xSemaphoreCreateMutex();          assert(g_state_mutex);
    g_msc_mutex   = xSemaphoreCreateMutex();          assert(g_msc_mutex);
    g_msc_task_event_group = xEventGroupCreate();     assert(g_msc_task_event_group);
    app_queue = xQueueCreate(16, sizeof(app_event_msg_t)); assert(app_queue);

    uart_init();

    ESP_LOGI(TAG, "Installing USB Host Core");
    const usb_host_config_t host_cfg = { .skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    xTaskCreate(usb_host_task, "usb_host_core", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Installing HID Host Driver");
    const hid_host_driver_config_t hid_cfg = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_driver_cb_queue,
        .callback_arg = NULL
    };
    ESP_ERROR_CHECK(hid_host_install(&hid_cfg));

    ESP_LOGI(TAG, "Installing MSC Host Driver");
    const msc_host_driver_config_t msc_cfg = {
        .create_backround_task = true, // spelling per IDF
        .task_priority = 5,
        .stack_size = 4096 * 2,
        .callback = msc_event_cb_queue,
        .callback_arg = NULL
    };
    ESP_ERROR_CHECK(msc_host_install(&msc_cfg));

    ESP_LOGI(TAG, "Unified Host Ready. Plug in a HID or MSC device.");

    app_event_msg_t msg;
    while (1) {
        if (xQueueReceive(app_queue, &msg, pdMS_TO_TICKS(5000)) == pdTRUE) {
            if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) == pdTRUE) {
                switch (msg.event_id) {

                    /* ---------- HID DEVICE CONNECT ---------- */
                    case APP_EVENT_HID_CONNECT: {
                        if (g_current_mode == MODE_HID && g_hid_driver_handle) {
                            ESP_LOGW(TAG, "Ignoring extra HID connect (already active: %p)", g_hid_driver_handle);
                            break;
                        }
                        if (g_current_mode != MODE_NONE) {
                            ESP_LOGW(TAG, "Ignoring HID connect (mode busy: %d)", g_current_mode);
                            break;
                        }

                        g_current_mode = MODE_HID;
                        g_hid_driver_handle = msg.data.hid_hdl;

                        const hid_host_device_config_t hid_if_cfg = {
                            .callback = hid_interface_cb,
                            .callback_arg = NULL
                        };
                        esp_err_t e = hid_host_device_open(g_hid_driver_handle, &hid_if_cfg);
                        if (e != ESP_OK) {
                            ESP_LOGE(TAG, "hid_open: %s", esp_err_to_name(e));
                            g_current_mode = MODE_NONE; g_hid_driver_handle = NULL;
                            break;
                        }

                        e = hid_host_device_start(g_hid_driver_handle);
                        if (e != ESP_OK) {
                            ESP_LOGE(TAG, "hid_start: %s", esp_err_to_name(e));
                            hid_host_device_close(g_hid_driver_handle);
                            g_current_mode = MODE_NONE; g_hid_driver_handle = NULL;
                            break;
                        }

                        ESP_LOGI(TAG, "HID started, announcing HID to RX");
                        send_uart_packet(TYPE_ANNOUNCE_HID, NULL, 0);
                        break;
                    } // <-- End of case APP_EVENT_HID_CONNECT

                    /* ---------- HID DISCONNECT (FIXED) ---------- */
                    case APP_EVENT_HID_IF_DISCONNECT: {
                        if (g_current_mode == MODE_HID && msg.data.hid_hdl == g_hid_driver_handle) {
                            ESP_LOGI(TAG, "HID closed");
                            
                            // *** FIX: Tell driver to release resources ***
                            hid_host_device_close(msg.data.hid_hdl);
                            
                            g_hid_driver_handle = NULL;
                            g_current_mode = MODE_NONE;
                        }
                        break;
                    } // <-- End of case APP_EVENT_HID_IF_DISCONNECT

                    /* ---------- MSC DEVICE CONNECT (RACE CONDITION FIXED) ---------- */
                    case APP_EVENT_MSC_CONNECT: {
                        if (g_current_mode != MODE_NONE) {
                            ESP_LOGW(TAG, "Ignoring MSC connect (mode busy: %d)", g_current_mode);
                            break;
                        }
                        g_current_mode = MODE_MSC;
                        esp_err_t e = msc_host_install_device(msg.data.msc_addr, &g_msc_driver_handle);
                        if (e != ESP_OK) {
                            ESP_LOGE(TAG, "msc_install_dev: %s", esp_err_to_name(e));
                            g_current_mode = MODE_NONE; g_msc_driver_handle = NULL;
                        } else {
                            e = msc_host_get_device_info(g_msc_driver_handle, &g_msc_info);
                            if (e != ESP_OK) {
                                ESP_LOGE(TAG, "msc_get_info: %s", esp_err_to_name(e));
                                msc_host_uninstall_device(g_msc_driver_handle);
                                g_current_mode = MODE_NONE; g_msc_driver_handle = NULL;
                            } else {
                                if (g_msc_info.sector_size == 0) g_msc_info.sector_size = 512;
                                
                                // (Re)allocate buffer
                                if (!g_msc_data_buffer) {
                                    g_msc_data_buffer = malloc(g_msc_info.sector_size);
                                } else if (g_msc_buffer_size < g_msc_info.sector_size) {
                                    uint8_t *nb = realloc(g_msc_data_buffer, g_msc_info.sector_size);
                                    if (nb) g_msc_data_buffer = nb;
                                    else ESP_LOGE(TAG, "MSC buf realloc fail!");
                                }

                                if (!g_msc_data_buffer) {
                                    ESP_LOGE(TAG, "MSC buffer alloc fail");
                                    msc_host_uninstall_device(g_msc_driver_handle);
                                    g_current_mode = MODE_NONE; g_msc_driver_handle = NULL;
                                } else {
                                    g_msc_buffer_size = g_msc_info.sector_size;

                                    // *** FIX: Start the listener task first ***
                                    if (g_msc_bridge_task_hdl == NULL) {
                                        xEventGroupClearBits(g_msc_task_event_group, MSC_TASK_STOP_BIT);
                                        xTaskCreate(uart_bridge_task, "msc_bridge_task", 4096, NULL, 6, &g_msc_bridge_task_hdl);
                                    }
                                    
                                    // *** FIX: Now, announce to the device ***
                                    send_uart_packet(TYPE_ANNOUNCE_MSC, NULL, 0);
                                }
                            }
                        }
                        break;
                    } // <-- End of case APP_EVENT_MSC_CONNECT

                    /* ---------- MSC DISCONNECT (FIXED) ---------- */
                    case APP_EVENT_MSC_DISCONNECT: {
                        if (g_current_mode == MODE_MSC && msg.data.msc_hdl == g_msc_driver_handle) {
                            
                            // *** FIX: Tell driver to release resources ***
                            msc_host_uninstall_device(msg.data.msc_hdl);
                            
                            // *** FIX: Signal task to stop ***
                            if (g_msc_bridge_task_hdl && g_msc_task_event_group) {
                                xEventGroupSetBits(g_msc_task_event_group, MSC_TASK_STOP_BIT);
                                g_msc_bridge_task_hdl = NULL; // Task will delete itself
                            }
                            g_msc_driver_handle = NULL;
                            g_current_mode = MODE_NONE;
                        
                        } else {
                            // This handles disconnects for unknown/stale handles
                            if (msg.data.msc_hdl) {
                                msc_host_uninstall_device(msg.data.msc_hdl);
                            }
                        }
                        break;
                    } // <-- End of case APP_EVENT_MSC_DISCONNECT

                    default: break;

                } // <-- End of switch(msg.event_id)
                xSemaphoreGive(g_state_mutex);
            } // <-- End of if(xSemaphoreTake)
        } else {
            // This else block runs on timeout (every 5 sec)
            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                const char* mode_str = (g_current_mode == MODE_NONE) ? "None" :
                                       (g_current_mode == MODE_HID) ? "HID Active" : "MSC Active";
                ESP_LOGI(TAG, "Current Mode: %s (HID Hdl: %p, MSC Hdl: %p)",
                         mode_str, g_hid_driver_handle, g_msc_driver_handle);
                xSemaphoreGive(g_state_mutex);
            }
        } // <-- End of if(xQueueReceive)
    } // <-- End of while(1)

} // <-- End of app_main
