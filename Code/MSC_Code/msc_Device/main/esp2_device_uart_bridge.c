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
// File Name: esp2_device_uart_bridge.c
// Description:MSC Device side code
// Using standard tinyusb library functions to enumarate an MSC class usb device on PC
// Project Partners: UWA, ANFF
//-------------------------------------------------------------------------

#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h" // *** FIX: Added missing GPIO header ***
#include "tinyusb.h"
#include "class/msc/msc_device.h"
#include "common/tusb_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // Added for mutex

static const char *TAG = "ESP2_DEVICE_BRIDGE";

// --- Pins and UART ---
#define UART_NUM         UART_NUM_1
#define UART_TX_PIN      (GPIO_NUM_17)
#define UART_RX_PIN      (GPIO_NUM_18)
#define UART_BUF_SIZE    (4096)

// --- Protocol ---
#define CMD_TEST_UNIT_READY 0x00
#define CMD_REQUEST_SENSE   0x03
#define CMD_READ_CAPACITY   0x25
#define CMD_READ10          0x28
#define CMD_WRITE10         0x2A
#define RSP_SUCCESS         0x01
#define RSP_FAILED          0x02
// --------------------

// Global storage for capacity
static uint32_t g_sector_count = 0;
static uint32_t g_sector_size = 512; // Default

// *** KEY CHANGE: Mutex to protect all UART bridge operations ***
static SemaphoreHandle_t g_uart_mutex = NULL;


/* TinyUSB descriptors
 ********************************************************************* */
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)
enum { ITF_NUM_MSC = 0, ITF_NUM_TOTAL };
enum { EDPT_MSC_OUT  = 0x01, EDPT_MSC_IN   = 0x81 };

static tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A, // Espressif VID
    .idProduct = 0x4002,
    .bcdDevice = 0x100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static uint8_t const msc_configuration_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, 64),
};

static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },  // 0: English
    "ESP32-S3",                     // 1: Manufacturer
    "UART MSC Bridge",              // 2: Product
    "123456",                       // 3: Serials
};
/*********************************************************************** TinyUSB descriptors*/

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
    uart_flush_input(UART_NUM);
}

/**
 * @brief Helper function to fetch capacity.
 * MUST be called with g_uart_mutex held.
 */
static bool fetch_capacity_from_esp1(void)
{
    ESP_LOGI(TAG, "Requesting capacity from ESP1...");
    uint8_t cmd_byte = CMD_READ_CAPACITY;
    uart_write_bytes(UART_NUM, &cmd_byte, 1);

    uint32_t sector_count_le;
    uint32_t sector_size_le;
    uint8_t status;

    int len = uart_read_bytes(UART_NUM, &sector_count_le, 4, pdMS_TO_TICKS(3000));
    if (len == 4) len += uart_read_bytes(UART_NUM, &sector_size_le, 4, pdMS_TO_TICKS(1000));
    if (len == 8) len += uart_read_bytes(UART_NUM, &status, 1, pdMS_TO_TICKS(1000));

    if (len != 9) {
        ESP_LOGE(TAG, "uart_get_capacity: Timeout! Expected 9, got %d", len);
        uart_flush_input(UART_NUM);
        return false;
    }

    if (status == RSP_SUCCESS) {
        g_sector_count = tu_le32toh(sector_count_le);
        g_sector_size = tu_le32toh(sector_size_le);
        ESP_LOGI(TAG, "Got capacity: Count=%"PRIu32", Size=%"PRIu32, g_sector_count, g_sector_size);
        if (g_sector_size == 0) g_sector_size = 512; // Failsafe
        if (g_sector_count == 0) return false; // Not really ready
        return true;
    } else {
        ESP_LOGE(TAG, "ESP1 failed to get capacity (RSP_FAILED).");
        g_sector_count = 0; // Reset
        return false;
    }
}


//------------- TinyUSB MSC Callbacks -------------//

// Invoked when received SCSI_CMD_READ_CAPACITY_10
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    ESP_LOGD(TAG, "Callback: tud_msc_capacity_cb");

    // *** KEY CHANGE: Take mutex ***
    if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "tud_msc_capacity_cb: Mutex timeout!");
        *block_count = 0;
        *block_size = 512;
        return;
    }

    // *** KEY CHANGE: Fetch capacity only if we don't have it ***
    if (g_sector_count == 0) {
        ESP_LOGW(TAG, "Capacity is zero, fetching from ESP1...");
        fetch_capacity_from_esp1(); // Updates globals on success
    }

    *block_count = g_sector_count;
    *block_size = g_sector_size;

    // *** KEY CHANGE: Give mutex ***
    xSemaphoreGive(g_uart_mutex);

    ESP_LOGD(TAG, "   Returning count=%"PRIu32", size=%u", *block_count, *block_size);
}

// Invoked when received Test Unit Ready command
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    ESP_LOGD(TAG, "Callback: tud_msc_test_unit_ready_cb");
    bool result = false;

    // *** KEY CHANGE: Take mutex ***
    if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "tud_msc_test_unit_ready_cb: Mutex timeout!");
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x01); // LOGICAL UNIT COMMUNICATION TIMEOUT
        return false;
    }

    uint8_t cmd_byte = CMD_TEST_UNIT_READY;
    uart_write_bytes(UART_NUM, &cmd_byte, 1);

    uint8_t status;
    int len = uart_read_bytes(UART_NUM, &status, 1, pdMS_TO_TICKS(2000)); // 2s timeout

    if (len != 1) {
        ESP_LOGE(TAG, "UART read timeout on TEST_UNIT_READY status");
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x01);
        result = false;
    } else if (status != RSP_SUCCESS) {
        ESP_LOGW(TAG, "ESP1 reported failure on TEST_UNIT_READY");
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00); // Medium not present
        result = false;
    } else {
        ESP_LOGD(TAG,"   TEST_UNIT_READY successful");
        result = true;
    }

    // *** KEY CHANGE: If drive reports not ready, clear our cached capacity ***
    if (result == false) {
        g_sector_count = 0;
    }

    // *** KEY CHANGE: Give mutex ***
    xSemaphoreGive(g_uart_mutex);
    return result;
}


// Invoked when received READ10 command
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void)lun; (void)offset;
    ESP_LOGD(TAG, "Callback: tud_msc_read10_cb (LBA=%"PRIu32", Bufsize=%"PRIu32")", lba, bufsize);

    if (g_sector_size == 0 || bufsize % g_sector_size != 0) {
        ESP_LOGE(TAG, "READ10: Invalid params (size=%"PRIu32", bufsize=%"PRIu32")", g_sector_size, bufsize);
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);
        return -1;
    }
    uint16_t sector_count = bufsize / g_sector_size;
    int32_t return_val = -1; // Assume failure

    // *** KEY CHANGE: Take mutex ***
    if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "tud_msc_read10_cb: Mutex timeout!");
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x01);
        return -1;
    }

    // 1. Send command and params
    uint8_t cmd_byte = CMD_READ10;
    uart_write_bytes(UART_NUM, &cmd_byte, 1);
    uint32_t lba_le = tu_htole32(lba);
    uint16_t sector_count_le = tu_htole16(sector_count);
    uart_write_bytes(UART_NUM, &lba_le, 4);
    uart_write_bytes(UART_NUM, &sector_count_le, 2);

    // 2. Read data block
    // Generous timeout: 10 seconds for 64KB @ 115200 (should be ~6s)
    int bytes_read = uart_read_bytes(UART_NUM, buffer, bufsize, pdMS_TO_TICKS(10000));
    if (bytes_read != bufsize) {
        ESP_LOGE(TAG, "UART read data timeout! Expected %"PRIu32", got %d", bufsize, bytes_read);
        uart_flush_input(UART_NUM);
        tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
        return_val = -1;
    } else {
        // 3. Read status byte
        uint8_t status;
        int len = uart_read_bytes(UART_NUM, &status, 1, pdMS_TO_TICKS(2000));
        if (len != 1) {
            ESP_LOGE(TAG, "UART read timeout on READ10 status byte!");
            tud_msc_set_sense(lun, SCSI_SENSE_ABORTED_COMMAND, 0x00, 0x00);
            return_val = -1;
        } else if (status != RSP_SUCCESS) {
            ESP_LOGE(TAG, "ESP1 failed READ10 (status=0x%02X)", status);
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
            return_val = -1;
        } else {
            ESP_LOGD(TAG, "   READ10 successful");
            return_val = bytes_read; // Success
        }
    }

    // *** KEY CHANGE: Give mutex ***
    xSemaphoreGive(g_uart_mutex);
    return return_val;
}

// Invoked when received WRITE10 command
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    (void)lun; (void)offset;
    ESP_LOGD(TAG, "Callback: tud_msc_write10_cb (LBA=%"PRIu32", Bufsize=%"PRIu32")", lba, bufsize);

    if (g_sector_size == 0 || bufsize % g_sector_size != 0) {
        ESP_LOGE(TAG, "WRITE10: Invalid params (size=%"PRIu32", bufsize=%"PRIu32")", g_sector_size, bufsize);
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);
        return -1;
    }
    uint16_t sector_count = bufsize / g_sector_size;
    int32_t return_val = -1; // Assume failure

    // *** KEY CHANGE: Take mutex ***
    if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "tud_msc_write10_cb: Mutex timeout!");
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x01);
        return -1;
    }

    // 1. Send command and params
    uint8_t cmd_byte = CMD_WRITE10;
    uart_write_bytes(UART_NUM, &cmd_byte, 1);
    uint32_t lba_le = tu_htole32(lba);
    uint16_t sector_count_le = tu_htole16(sector_count);
    uart_write_bytes(UART_NUM, &lba_le, 4);
    uart_write_bytes(UART_NUM, &sector_count_le, 2);

    // 2. Write data block
    int bytes_written = uart_write_bytes(UART_NUM, buffer, bufsize);
     if (bytes_written != bufsize) {
        ESP_LOGE(TAG, "UART write error! Expected %"PRIu32", wrote %d", bufsize, bytes_written);
        tud_msc_set_sense(lun, SCSI_SENSE_ABORTED_COMMAND, 0x4B, 0x00);
        return_val = -1;
     } else {
        // 3. Read status byte
        uint8_t status;
        // Generous timeout, ESP1 might be slow writing to flash
        int len = uart_read_bytes(UART_NUM, &status, 1, pdMS_TO_TICKS(10000));
        if (len != 1) {
            ESP_LOGE(TAG, "UART read timeout on WRITE10 status byte!");
            tud_msc_set_sense(lun, SCSI_SENSE_ABORTED_COMMAND, 0x00, 0x00);
            return_val = -1;
        } else if (status != RSP_SUCCESS) {
            ESP_LOGE(TAG, "ESP1 failed WRITE10 (status=0x%02X)", status);
            tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x0C, 0x00);
            return_val = -1;
        } else {
            ESP_LOGD(TAG, "   WRITE10 successful");
            return_val = bytes_written; // Success
        }
     }

    // *** KEY CHANGE: Give mutex ***
    xSemaphoreGive(g_uart_mutex);
    return return_val;
}

// Invoked when received SCSI_CMD_REQUEST_SENSE
// This is the only other command we *must* implement for the bridge.
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    (void)lun; (void)bufsize;
    const uint8_t cmd = scsi_cmd[0];
    int32_t response_len = 0;
    bool result = false;

    // *** KEY CHANGE: Take mutex ***
    // Use a short timeout, SENSE should be fast.
    if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGE(TAG, "tud_msc_scsi_cb: Mutex timeout!");
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x01);
        return -1;
    }

    switch (cmd)
    {
        case SCSI_CMD_REQUEST_SENSE:
            ESP_LOGD(TAG, "Callback: tud_msc_scsi_cb (REQUEST_SENSE)");
            uint8_t cmd_byte = CMD_REQUEST_SENSE;
            uart_write_bytes(UART_NUM, &cmd_byte, 1);

            // 1. Read 18 bytes of sense data
            int len = uart_read_bytes(UART_NUM, buffer, 18, pdMS_TO_TICKS(1000));
            if (len != 18) {
                ESP_LOGE(TAG, "UART read timeout on REQUEST_SENSE data!");
                result = false;
            } else {
                // 2. Read 1 byte status
                uint8_t status;
                len = uart_read_bytes(UART_NUM, &status, 1, pdMS_TO_TICKS(1000));
                if (len != 1 || status != RSP_SUCCESS) {
                    ESP_LOGE(TAG, "ESP1 failed REQUEST_SENSE command!");
                    result = false;
                } else {
                    response_len = 18;
                    result = true;
                }
            }
            break;
        
        // --- Let TinyUSB handle these with its stubs ---
        case SCSI_CMD_INQUIRY:
        case SCSI_CMD_MODE_SENSE_6:
        case SCSI_CMD_READ_FORMAT_CAPACITY:
        case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        case SCSI_CMD_START_STOP_UNIT:
            ESP_LOGD(TAG, "Callback: tud_msc_scsi_cb (Stubbed: 0x%02X)", cmd);
            // This will be passed to TinyUSB's default weak implementation
            response_len = -1; // Indicate not handled by us
            result = false;
            break;

        default:
            ESP_LOGW(TAG, "Callback: tud_msc_scsi_cb (Unhandled: 0x%02x)", cmd);
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
            response_len = -1;
            result = false;
            break;
    }

    // *** KEY CHANGE: Give mutex ***
    xSemaphoreGive(g_uart_mutex);

    // This is tricky: if we return -1, tud_msc_scsi_cb_default() is called.
    // We only want that for the stubbed commands.
    if (cmd == SCSI_CMD_REQUEST_SENSE) {
        return result ? response_len : -1;
    }
    
    // For unhandled commands
    if (response_len == -1 && result == false) {
        return -1;
    }

    // For other unhandled commands where we set sense
    return -1;
}


void app_main(void)
{
    ESP_LOGI(TAG, "Initializing UART for bridge...");
    uart_init();

    // *** KEY CHANGE: Create the mutex ***
    g_uart_mutex = xSemaphoreCreateMutex();
    assert(g_uart_mutex);

    // *** KEY CHANGE: Don't block here. Let callbacks handle fetching. ***
    ESP_LOGI(TAG, "Initializing TinyUSB...");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &descriptor_config,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        .external_phy = false,
        .configuration_descriptor = msc_configuration_desc,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB MSC initialization DONE");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

