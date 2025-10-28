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
// File Name: esp1_host_uart_bridge.c
// Description:MSC Host side code
// Using standard usb/host library functions to host a MSC class usb device into the esp and push and pull callbacks 
// Project Partners: UWA, ANFF
//-------------------------------------------------------------------------


#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include <endian.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "usb/msc_host.h"

static const char *TAG = "ESP1_HOST_BRIDGE";

// --- Pins and UART ---
#define APP_QUIT_PIN     GPIO_NUM_0
#define UART_NUM         UART_NUM_1
#define UART_TX_PIN      (GPIO_NUM_17)
#define UART_RX_PIN      (GPIO_NUM_18)
#define UART_BUF_SIZE    (2048)

// --- Protocol ---
#define CMD_TEST_UNIT_READY 0x00
#define CMD_REQUEST_SENSE   0x03
#define CMD_READ_CAPACITY   0x25
#define CMD_READ10          0x28
#define CMD_WRITE10         0x2A
#define RSP_SUCCESS         0x01
#define RSP_FAILED          0x02
// --------------------

typedef struct {
    uint8_t key;
    uint8_t asc;
    uint8_t ascq;
} scsi_sense_data_t;

// --- Application Queue ---
typedef struct {
    enum {
        APP_QUIT,
        APP_DEVICE_CONNECTED,
        APP_DEVICE_DISCONNECTED,
    } id;
    union {
        uint8_t new_dev_address;
        msc_host_device_handle_t device_handle;
    } data;
} app_message_t;

static QueueHandle_t app_queue;

// --- Globals ---
static msc_host_device_handle_t g_msc_device = NULL;
static msc_host_device_info_t g_msc_info;
static uint8_t *g_data_buffer = NULL;
static uint32_t g_buffer_size = 512; // Default
static scsi_sense_data_t g_last_sense = {0};

// *** KEY CHANGE: Mutex to protect all globals above ***
static SemaphoreHandle_t g_device_mutex = NULL;

/**
 * @brief Helper to set the last SCSI sense data based on esp_err_t
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
 * @brief BOOT button pressed callback
 */
static void gpio_cb(void *arg)
{
    BaseType_t xTaskWoken = pdFALSE;
    app_message_t message = { .id = APP_QUIT };
    if (app_queue) {
        xQueueSendFromISR(app_queue, &message, &xTaskWoken);
    }
    if (xTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief MSC driver callback
 */
static void msc_event_cb(const msc_host_event_t *event, void *arg)
{
    app_message_t message;
    if (event->event == MSC_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "MSC device connected (usb_addr=%d)", event->device.address);
        message.id = APP_DEVICE_CONNECTED;
        message.data.new_dev_address = event->device.address;
        xQueueSend(app_queue, &message, portMAX_DELAY);
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        ESP_LOGI(TAG, "MSC device disconnected");
        message.id = APP_DEVICE_DISCONNECTED;
        message.data.device_handle = event->device.handle;
        xQueueSend(app_queue, &message, portMAX_DELAY);
    }
}

/**
 * @brief Initialize UART1
 */
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_LOGI(TAG, "Initializing UART%d", UART_NUM);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

/**
 * @brief The main task that waits for UART commands and processes them.
 */
static void uart_bridge_task(void *arg)
{
    // Wait for the buffer to be allocated by app_main
    while (g_data_buffer == NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "UART task started");

    while (1) {
        uint8_t cmd_byte;
        int len = uart_read_bytes(UART_NUM, &cmd_byte, 1, portMAX_DELAY);
        if (len != 1) continue;

        uint8_t response_byte = RSP_SUCCESS;
        esp_err_t err = ESP_OK;
        set_sense_from_err(ESP_OK); // Clear sense

        // *** KEY CHANGE: Acquire mutex. Use a reasonable timeout. ***
        if (xSemaphoreTake(g_device_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
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

        switch (cmd_byte) {
            case CMD_READ_CAPACITY: {
                ESP_LOGD(TAG, "UART_CMD: READ_CAPACITY");
                if (g_msc_device == NULL) {
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

                    if (g_msc_device == NULL) {
                        err = ESP_FAIL;
                        set_sense_from_err(ESP_ERR_INVALID_STATE);
                    } else if (g_buffer_size < g_msc_info.sector_size) {
                        ESP_LOGE(TAG, "Internal error: Buffer size %"PRIu32" < Sector size %"PRIu32, g_buffer_size, g_msc_info.sector_size);
                        err = ESP_FAIL;
                    } else {
                        // Read and send one sector at a time
                        for (uint16_t i = 0; i < sector_count; i++) {
                            err = msc_host_read_sector(g_msc_device, lba + i, g_data_buffer, g_buffer_size);
                            if (err != ESP_OK) {
                                ESP_LOGE(TAG, "msc_host_read_sector failed: %s", esp_err_to_name(err));
                                set_sense_from_err(err);
                                break;
                            }
                            int written = uart_write_bytes(UART_NUM, g_data_buffer, g_buffer_size);
                            if (written != g_buffer_size) {
                                ESP_LOGE(TAG, "UART write error during READ10!");
                                err = ESP_FAIL;
                                break;
                            }
                        }
                    }
                }
                response_byte = (err == ESP_OK) ? RSP_SUCCESS : RSP_FAILED;
                uart_write_bytes(UART_NUM, &response_byte, 1);
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

                    if (g_msc_device == NULL) {
                        err = ESP_FAIL;
                        set_sense_from_err(ESP_ERR_INVALID_STATE);
                        // We must still read and discard the data from UART
                        for (uint16_t i = 0; i < sector_count; i++) {
                            uart_read_bytes(UART_NUM, g_data_buffer, g_buffer_size, pdMS_TO_TICKS(1000));
                        }
                    } else if (g_buffer_size < g_msc_info.sector_size) {
                        ESP_LOGE(TAG, "Internal error: Buffer size %"PRIu32" < Sector size %"PRIu32, g_buffer_size, g_msc_info.sector_size);
                        err = ESP_FAIL;
                        // Read and discard data
                        for (uint16_t i = 0; i < sector_count; i++) {
                            uart_read_bytes(UART_NUM, g_data_buffer, g_buffer_size, pdMS_TO_TICKS(1000));
                        }
                    } else {
                        // Read from UART and write one sector at a time
                        for (uint16_t i = 0; i < sector_count; i++) {
                            int bytes_read = uart_read_bytes(UART_NUM, g_data_buffer, g_buffer_size, pdMS_TO_TICKS(5000));
                            if (bytes_read != g_buffer_size) {
                                ESP_LOGE(TAG, "UART read timeout on WRITE10 data");
                                err = ESP_ERR_TIMEOUT;
                                break;
                            }
                            err = msc_host_write_sector(g_msc_device, lba + i, g_data_buffer, g_buffer_size);
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
                break;
            }

            case CMD_TEST_UNIT_READY: {
                ESP_LOGD(TAG, "UART_CMD: TEST_UNIT_READY");
                if (g_msc_device == NULL) {
                    err = ESP_FAIL;
                    set_sense_from_err(ESP_ERR_INVALID_STATE);
                }
                // Note: A "real" TUR might send a command, but this is good enough.
                response_byte = (err == ESP_OK) ? RSP_SUCCESS : RSP_FAILED;
                uart_write_bytes(UART_NUM, &response_byte, 1);
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

        // *** KEY CHANGE: Release the mutex ***
        xSemaphoreGive(g_device_mutex);
    }
    vTaskDelete(NULL);
}


/**
 * @brief USB task
 */
static void usb_task(void *args)
{
    const usb_host_config_t host_config = { .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    const msc_host_driver_config_t msc_config = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = msc_event_cb,
    };
    ESP_ERROR_CHECK(msc_host_install(&msc_config));

    bool quit_signaled = false;
    while (!quit_signaled) {
        uint32_t event_flags;
        usb_host_lib_handle_events(pdMS_TO_TICKS(100), &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG,"USB task: MSC client unregistered.");
        }

        app_message_t msg;
        if (xQueueReceive(app_queue, &msg, 0) == pdTRUE) {
             if (msg.id == APP_QUIT) {
                  ESP_LOGI(TAG, "USB task received APP_QUIT signal.");
                  quit_signaled = true;
             } else {
                  xQueueSendToFront(app_queue, &msg, 0); // Put back
             }
        }
    }

    ESP_LOGI(TAG, "Uninstalling USB host...");
    ESP_ERROR_CHECK(msc_host_uninstall());
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}


void app_main(void)
{
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);

    // *** KEY CHANGE: Create the mutex ***
    g_device_mutex = xSemaphoreCreateMutex();
    assert(g_device_mutex);

    // Initialize UART
    uart_init();

    // Allocate the global buffer (must be done before uart_task starts)
    g_data_buffer = malloc(g_buffer_size);
    assert(g_data_buffer);

    // Start tasks
    xTaskCreate(uart_bridge_task, "uart_task", 4096, NULL, 4, NULL);
    xTaskCreate(usb_task, "usb_task", 4096, NULL, 5, NULL);

    // Init BOOT button
    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(APP_QUIT_PIN, gpio_cb, NULL));

    ESP_LOGI(TAG, "Waiting for USB flash drive...");

    bool app_quit = false;
    while (!app_quit) {
        app_message_t msg;
        xQueueReceive(app_queue, &msg, portMAX_DELAY);

        // *** KEY CHANGE: Take mutex before modifying globals ***
        if (xSemaphoreTake(g_device_mutex, portMAX_DELAY) != pdTRUE) {
             ESP_LOGE(TAG, "Main task failed to get mutex!");
             continue; // Should not happen
        }

        switch(msg.id) {
            case APP_DEVICE_CONNECTED:
                if (g_msc_device != NULL) {
                    ESP_LOGW(TAG, "Already a device connected. Ignoring new one.");
                } else {
                    esp_err_t err = msc_host_install_device(msg.data.new_dev_address, &g_msc_device);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "msc_host_install_device failed: %s", esp_err_to_name(err));
                        g_msc_device = NULL;
                        break;
                    }

                    err = msc_host_get_device_info(g_msc_device, &g_msc_info);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "msc_host_get_device_info failed: %s", esp_err_to_name(err));
                        msc_host_uninstall_device(g_msc_device);
                        g_msc_device = NULL;
                        break;
                    }

                    ESP_LOGI(TAG, "Device info:");
                    ESP_LOGI(TAG, "\t Capacity: %llu MB", ((uint64_t)g_msc_info.sector_size * g_msc_info.sector_count) / (1024 * 1024));
                    ESP_LOGI(TAG, "\t Sector size: %"PRIu32, g_msc_info.sector_size);
                    ESP_LOGI(TAG, "\t Sector count: %"PRIu32, g_msc_info.sector_count);

                    if (g_msc_info.sector_size == 0) {
                        ESP_LOGE(TAG, "Device reported sector size 0. Aborting.");
                        msc_host_uninstall_device(g_msc_device);
                        g_msc_device = NULL;
                        break;
                    }

                    // Buffer Re-allocation Logic
                    if (g_msc_info.sector_size > g_buffer_size) {
                        ESP_LOGW(TAG,"Reallocating buffer to %"PRIu32, g_msc_info.sector_size);
                        uint8_t *new_buffer = realloc(g_data_buffer, g_msc_info.sector_size);
                        if (!new_buffer) {
                            ESP_LOGE(TAG, "Failed to reallocate buffer!");
                            msc_host_uninstall_device(g_msc_device);
                            g_msc_device = NULL;
                        } else {
                            g_data_buffer = new_buffer;
                            g_buffer_size = g_msc_info.sector_size;
                        }
                    } else {
                         g_buffer_size = g_msc_info.sector_size; // Adjust to exact size
                    }
                }
                break;

            case APP_DEVICE_DISCONNECTED:
                if (msg.data.device_handle == g_msc_device) {
                    ESP_LOGI(TAG, "Device disconnected.");
                    g_msc_device = NULL;
                    memset(&g_msc_info, 0, sizeof(g_msc_info));
                    g_buffer_size = 512; // Reset buffer size (realloc not needed)
                } else {
                    ESP_LOGW(TAG, "Disconnected event for unknown device handle.");
                    msc_host_uninstall_device(msg.data.device_handle);
                }
                break;

            case APP_QUIT:
                ESP_LOGI(TAG, "APP_QUIT received.");
                app_quit = true;
                if (g_msc_device) {
                    g_msc_device = NULL;
                }
                // Send signal to USB task
                xQueueSend(app_queue, &msg, 0);
                break;

            default:
                break;
        }

        // *** KEY CHANGE: Release the mutex ***
        xSemaphoreGive(g_device_mutex);
    }

    ESP_LOGI(TAG, "Main task cleaning up...");
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for other tasks
    
    gpio_isr_handler_remove(APP_QUIT_PIN);
    gpio_uninstall_isr_service();
    
    if (g_data_buffer) {
        free(g_data_buffer);
    }
    vQueueDelete(app_queue);
    vSemaphoreDelete(g_device_mutex);
    ESP_LOGI(TAG, "Main task exiting.");
}
