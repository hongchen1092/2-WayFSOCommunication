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
// Description: HID Host side code
// Defines custom UART FRAME for mouse and keyboard data and push it to UART1 (8n1)
// Dynamically Identify the connected device and initiates the correct framing
// Project Partners: UWA, ANFF
//---------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"

#define TAG "KBD_MOUSE_HOST_TX"

/* ---------- UART Configuration ---------- */
#define UART_NUM        UART_NUM_1
#define UART_TX_GPIO    (17)
#define UART_BAUD       (115200)

/* ---------- Custom Protocol Definition ---------- */
#define FRAME_HDR       (0xA5) // Start-of-frame marker
#define TYPE_MOUSE      (0x01) // Device type ID for Mouse
#define TYPE_KEYBOARD   (0x02) // Device type ID for Keyboard

// Standard HID report structures
typedef struct __attribute__((packed)) {
    uint8_t buttons;
    int8_t  dx;
    int8_t  dy;
    int8_t  wheel;
} hid_mouse_report_t;

typedef struct __attribute__((packed)) {
    uint8_t modifier;
    uint8_t reserved;
    uint8_t keycode[6];
} hid_keyboard_report_t;

// Event queue for HID host events
static QueueHandle_t hid_host_event_queue;

/**
 * @brief HID Host event
 */
typedef struct {
    hid_host_device_handle_t hid_device_handle;
    hid_host_driver_event_t event;
    void *arg;
} hid_host_event_queue_t;


/* ---------- UART Functions ---------- */
static void uart_init(void) {
    const uart_config_t cfg = {
        .baud_rate = UART_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// Sends a structured packet over UART
static void send_uart_packet(uint8_t type, const void* data, size_t len) {
    uint8_t packet[1 + 1 + len]; // HDR + TYPE + PAYLOAD
    packet[0] = FRAME_HDR;
    packet[1] = type;
    memcpy(&packet[2], data, len);

    uart_write_bytes(UART_NUM, (const char*)packet, sizeof(packet));
    ESP_LOGD(TAG, "UART TX -> Type: 0x%02X, Len: %d", type, sizeof(packet));
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, packet, sizeof(packet), ESP_LOG_DEBUG);
}

/* ---------- USB Host Callbacks & Tasks (Based on Official Example) ---------- */

/**
 * @brief HID Host interface callback
 */
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                   const hid_host_interface_event_t event,
                                   void *arg) {
    uint8_t data[64] = {0};
    size_t data_length = 0;
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(hid_device_handle, data, 64, &data_length));

        // Check protocol and send the correct packet
        if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
            if (data_length >= sizeof(hid_keyboard_report_t)) {
                hid_keyboard_report_t *report = (hid_keyboard_report_t*)data;
                ESP_LOGI(TAG, "Keyboard Report: mod=0x%02X, keys=[%02X %02X...]", report->modifier, report->keycode[0], report->keycode[1]);
                send_uart_packet(TYPE_KEYBOARD, report, sizeof(hid_keyboard_report_t));
            }
        } 
        
        else if (HID_PROTOCOL_MOUSE == dev_params.proto) {
            // Check for at least 3 bytes (buttons, dx, dy)
            if (data_length >= 3) {
                
                // Initialize report to all zeros
                hid_mouse_report_t report = {0}; 

                report.buttons = data[0];
                report.dx = data[1];
                report.dy = data[2];
                
                // Conditionally read the wheel data only if it exists
                if (data_length >= 4) {
                    report.wheel = data[3];
                }
                
                ESP_LOGI(TAG, "Mouse Report: buttons=0x%02X, dx=%d, dy=%d, wheel=%d",
                         report.buttons, report.dx, report.dy, report.wheel);
                
                // Send the full 4-byte struct (wheel will be 0 if not present)
                send_uart_packet(TYPE_MOUSE, &report, sizeof(hid_mouse_report_t));
            }
        }
        
        break;
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Device DISCONNECTED (Proto: %d)", dev_params.proto);
        ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
        break;
    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGI(TAG, "Device TRANSFER_ERROR");
        break;
    default:
        ESP_LOGE(TAG, "Unhandled interface event: %d", event);
        break;
    }
}

/**
 * @brief HID Host Device event
 */
void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                             const hid_host_driver_event_t event, void *arg) {
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));
    const hid_host_device_config_t dev_config = {
        .callback = hid_host_interface_callback, .callback_arg = NULL
    };

    switch (event) {
    case HID_HOST_DRIVER_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Device CONNECTED (Proto: %d)", dev_params.proto);
        ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
        
        // --- START OF FIX ---
        // Only set Boot Protocol for the keyboard
        // Let the mouse use its default Report Protocol
        if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
            ESP_LOGI(TAG, "Setting Boot Protocol for Keyboard");
            ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_BOOT));
        }
        // --- END OF FIX ---

        ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
        break;
    default:
        ESP_LOGE(TAG, "Unhandled device event: %d", event);
        break;
    }
}

/**
 * @brief HID Host main task
 */
void hid_host_task(void *pvParameters) {
    hid_host_event_queue_t evt_queue;
    while (1) {
        if (xQueueReceive(hid_host_event_queue, &evt_queue, pdMS_TO_TICKS(100))) {
            hid_host_device_event(evt_queue.hid_device_handle, evt_queue.event, evt_queue.arg);
        }
    }
}

/**
 * @brief HID Host Device callback
 */
void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                const hid_host_driver_event_t event, void *arg) {
    const hid_host_event_queue_t evt_queue = {
        .hid_device_handle = hid_device_handle, .event = event, .arg = arg
    };
    xQueueSend(hid_host_event_queue, &evt_queue, 0);
}

/**
 * @brief Start USB Host install and handle common USB host library events
 */
static void usb_lib_task(void *arg) {
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    xTaskNotifyGive((TaskHandle_t)arg); // Notify app_main that USB host is installed

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

void app_main(void) {
    BaseType_t task_created;
    
    // Initialize UART
    uart_init();
    
    // Create queue
    hid_host_event_queue = xQueueCreate(10, sizeof(hid_host_event_queue_t));

    // Create usb_lib_task
    task_created = xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096,
                                           xTaskGetCurrentTaskHandle(), 2, NULL, 0);
    assert(task_created == pdTRUE);

    // Wait for notification from usb_lib_task to proceed
    ulTaskNotifyTake(false, pdMS_TO_TICKS(1000));

    // HID host driver configuration
    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_device_callback,
        .callback_arg = NULL
    };
    ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

    // Create HID Host task
    task_created = xTaskCreate(&hid_host_task, "hid_task", 4 * 1024, NULL, 2, NULL);
    assert(task_created == pdTRUE);

    ESP_LOGI(TAG, "TX Host Ready. Plug in a keyboard or mouse.");
}
