#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

#include "../common/link_proto.h"

#define TAG "DEV_RX"

// === PINS & BAUD (adjust RX pin to your photodiode output) ===
#define UART_PORT         UART_NUM_1
#define UART_TX_PIN       (GPIO_NUM_NC)
#define UART_RX_PIN       (GPIO_NUM_18)     // <-- your RX pin
#define UART_BAUD         (115200)          // proven working
// If your photodiode output idles low (inverted), uncomment the inverse define below
// #define RX_INVERT 

// Optional LED for heartbeat
#define STAT_LED_PIN      (GPIO_NUM_2)

static void uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 4*1024, 4*1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#ifdef RX_INVERT
    ESP_ERROR_CHECK(uart_set_line_inverse(UART_PORT, UART_SIGNAL_RXD_INV));
#endif

    if (STAT_LED_PIN != GPIO_NUM_NC) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << STAT_LED_PIN,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = 0,
            .pull_down_en = 0,
            .intr_type    = GPIO_INTR_DISABLE
        };
        gpio_config(&io);
        gpio_set_level(STAT_LED_PIN, 0);
    }
}

// ---------------- TinyUSB device (mouse-only) descriptors ----------------

enum {
  ITF_NUM_HID,
  ITF_NUM_TOTAL
};

#define EPNUM_HID_IN  0x81

// Mouse-only report descriptor (boot compatible) — **NO Report ID**
static const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_MOUSE()   // <-- no HID_REPORT_ID(...)
};

#define TUSB_DESC_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

static const tusb_desc_device_t device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x303A,   // Espressif VID (dev use)
    .idProduct          = 0x4003,   // bump PID to force re-enum on Windows
    .bcdDevice          = 0x0101,   // bump device rev
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

static const char* hid_string_descriptor[] = {
    (const char[]){ 0x09, 0x04 }, // US English
    "LaserLink",                  // 1: Manufacturer
    "Laser Mouse Bridge",         // 2: Product
    "0002",                       // 3: Serial (bumped)
    "HID Mouse",                  // 4: Interface
};

static const uint8_t hid_configuration_descriptor[] = {
    // Config number, interface count, string index, total length, attributes, power (mA)
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report desc len, EP IN addr, size, polling interval (ms)
    TUD_HID_DESCRIPTOR(ITF_NUM_HID, 4, true, sizeof(hid_report_descriptor), EPNUM_HID_IN, 16, 1),
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t itf)
{
    (void)itf;
    return hid_report_descriptor;
}

// ---- TinyUSB HID required app callbacks ----
uint16_t tud_hid_get_report_cb(uint8_t instance,
                               uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t* buffer,
                               uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0; // no report to send
}

void tud_hid_set_report_cb(uint8_t instance,
                           uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const* buffer,
                           uint16_t bufsize)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)bufsize;
    // nothing to do for mouse-only
}

// ---- Helpful: USB state logs so you know the PC mounted the HID ---
void tud_mount_cb(void)     { ESP_LOGI(TAG, "USB: mounted"); }
void tud_umount_cb(void)    { ESP_LOGW(TAG, "USB: unmounted"); }
void tud_suspend_cb(bool r) { ESP_LOGW(TAG, "USB: suspend (remote_wakeup=%d)", r); }
void tud_resume_cb(void)    { ESP_LOGI(TAG, "USB: resume"); }

static void tinyusb_init_mouse(void)
{
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &device_descriptor,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = hid_configuration_descriptor,
        .hs_configuration_descriptor = hid_configuration_descriptor,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = hid_configuration_descriptor,
#endif
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
}

// ---------------- UART frame RX → TinyUSB mouse report ----------------

static void task_uart_to_mouse(void *arg)
{
    (void)arg;
    uint8_t byte;
    link_mouse_frame_t f;
    size_t idx = 0;

    uint32_t ok=0, crc_fail=0, bad_hdr=0, drops=0, last_seq=0;
    bool have_seq=false;
    TickType_t last_log = xTaskGetTickCount();

    while (1) {
        int n = uart_read_bytes(UART_PORT, &byte, 1, pdMS_TO_TICKS(20));
        if (n == 1) {
            if (idx == 0 && byte != LINK_HDR) { bad_hdr++; continue; }
            ((uint8_t*)&f)[idx++] = byte;

            if (idx == sizeof(link_mouse_frame_t)) {
                uint16_t crc = link_crc16_ccitt((uint8_t*)&f, offsetof(link_mouse_frame_t, crc));
                if (crc == f.crc && f.ver == LINK_VER && (f.len == 3 || f.len == 4)) {
                    ok++;
                    if (have_seq && (uint16_t)(f.seq - last_seq) != 1) drops++;
                    last_seq = f.seq; have_seq = true;

                    uint8_t btn = f.payload[0];
                    int8_t  dx  = (int8_t)f.payload[1];
                    int8_t  dy  = (int8_t)f.payload[2];
                    int8_t  wh  = (f.len >= 4) ? (int8_t)f.payload[3] : 0;

                    if (tud_mounted()) {
                        // No Report ID in descriptor → report_id=0 here is correct
                        tud_hid_mouse_report(0, btn, dx, dy, wh, 0);
                    }
                    if (STAT_LED_PIN != GPIO_NUM_NC) {
                        gpio_set_level(STAT_LED_PIN, !gpio_get_level(STAT_LED_PIN));
                    }
                } else {
                    crc_fail++;
                }
                idx = 0; // resync for next frame
            }
        }

        if ((xTaskGetTickCount() - last_log) >= pdMS_TO_TICKS(1000)) {
            ESP_LOGI(TAG, "FRM: ok=%u, crc_fail=%u, bad_hdr=%u, seq_drops=%u, last_seq=%u",
                     ok, crc_fail, bad_hdr, drops, (unsigned)last_seq);
            last_log = xTaskGetTickCount();
        }
    }
}

void app_main(void)
{
    uart_init();
    tinyusb_init_mouse();
    xTaskCreate(task_uart_to_mouse, "uart_to_mouse", 4096, NULL, 5, NULL);
}
