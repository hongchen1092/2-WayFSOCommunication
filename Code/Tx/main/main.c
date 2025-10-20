// ======================== main.c (TX: USB mouse -> UART) ========================
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

// USB Host + HID Host
#include "usb/usb_host.h"
#include "usb/hid_host.h"

#include "../common/link_proto.h"   // your framing/CRC defs

#define TAG "HOST_TX"

// --------- SELECT YOUR HID HOST API FLAVOR HERE ----------
#define USE_HID_API_V2  1   // try 1 first; if it doesn't compile, set to 0
// ---------------------------------------------------------

// === UART PINS & BAUD ===
#define UART_PORT         UART_NUM_1
#define UART_TX_PIN       (GPIO_NUM_17)   // adjust to your board
#define UART_RX_PIN       (GPIO_NUM_NC)
#define UART_BAUD         (115200)

// Optional heartbeat pin. Leave NC for none.
#define STAT_LED_PIN      (GPIO_NUM_NC)

// --------------------- UART init ---------------------
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
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Only configure if a valid pin (>=0)
    if ((int)STAT_LED_PIN >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << STAT_LED_PIN),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = 0,
            .pull_down_en = 0,
            .intr_type    = GPIO_INTR_DISABLE
        };
        gpio_config(&io);
        gpio_set_level(STAT_LED_PIN, 0);
    }
}

// --------------------- Link send helper ---------------------

static inline int8_t clamp_i8(int v) {
    if (v > 127) return 127;
    if (v < -128) return -128;
    return (int8_t)v;
}

// Pack & send one framed mouse report over UART
static void mouse_link_send_from_hid(uint8_t buttons, int dx, int dy, int wheel)
{
    static uint16_t seq = 0;
    link_mouse_frame_t f;

    f.hdr = LINK_HDR;
    f.ver = LINK_VER;
    f.seq = seq++;
    f.len = 4; // buttons, dx, dy, wheel
    f.payload[0] = buttons & 0x07; // L=1, R=2, M=4 (boot mouse)
    f.payload[1] = (uint8_t)clamp_i8(dx);
    f.payload[2] = (uint8_t)clamp_i8(dy);
    f.payload[3] = (uint8_t)clamp_i8(wheel);
    f.crc = link_crc16_ccitt((const uint8_t*)&f, offsetof(link_mouse_frame_t, crc));

    (void)uart_write_bytes(UART_PORT, (const char*)&f, sizeof(f));

    if ((int)STAT_LED_PIN >= 0)
        gpio_set_level(STAT_LED_PIN, !gpio_get_level(STAT_LED_PIN));
}

#if USE_HID_API_V2
// ========================== V2 HID API (driver callback is (itf, event, arg)) ==========================

// Input report callback: parse boot mouse & forward via UART.
// This symbol name must match your driver header. If your header names it differently,
// adjust here.
static void hid_input_report_cb(struct hid_interface *itf,
                                const uint8_t *report, size_t len,
                                void *arg)
{
    (void)itf; (void)arg;
    if (len < 3) return;

    uint8_t buttons = report[0] & 0x07;
    int8_t  dx      = (int8_t)report[1];
    int8_t  dy      = (int8_t)report[2];
    int8_t  wheel   = (len >= 4) ? (int8_t)report[3] : 0;

    mouse_link_send_from_hid(buttons, dx, dy, wheel);
}

static void hid_driver_event_cb(struct hid_interface *itf,
                                const hid_host_driver_event_t event,
                                void *arg)
{
    (void)arg;

    if (event == HID_HOST_DRIVER_EVENT_CONNECTED) {
        // Put interface into BOOT protocol (mouse) if supported
        // Some releases name the constant HID_PROTOCOL_BOOT; if yours only
        // exposes HID_PROTOCOL_MOUSE/NONE, skip this call.
#ifdef HID_PROTOCOL_BOOT
        (void)hid_class_request_set_protocol(itf, HID_PROTOCOL_BOOT);
#endif
        // Register to receive input reports.
#ifdef hid_host_interface_set_input_report_callback
        hid_host_interface_set_input_report_callback(itf, hid_input_report_cb, NULL);
#else
        // If your header doesn't declare the function above,
        // your component is the "legacy/event-struct" flavor -> set USE_HID_API_V2=0
# warning "hid_host_interface_set_input_report_callback() not declared by this header"
#endif
        // (Optional) idle to 0 for smoother reporting
#ifdef hid_class_request_set_idle
        (void)hid_class_request_set_idle(itf, 0, 0);
#endif
        ESP_LOGI(TAG, "HID interface connected and armed for reports.");
    }
#ifdef HID_HOST_DRIVER_EVENT_DISCONNECTED
    else if (event == HID_HOST_DRIVER_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "HID interface disconnected.");
    }
#endif
}

static void usb_hid_mouse_init(void)
{
    const usb_host_config_t host_cfg = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));

    const hid_host_driver_config_t hid_cfg = {
        .create_background_task = true,
        .task_priority          = 5,
        .stack_size             = 4096,
        .callback               = hid_driver_event_cb,
        .callback_arg           = NULL,
    };
    ESP_ERROR_CHECK(hid_host_install(&hid_cfg));

    ESP_LOGI(TAG, "USB/HID host ready (V2 API). Plug the USB mouse.");
}

#else
// ========================== V1 (legacy) HID API (event is a struct) ==========================
//
// This matches the older component style where the driver raises a single event
// carrying connection info; you then open the device, iterate interfaces, set
// BOOT protocol and attach an input callback per interface.
//

static struct hid_device *s_hdev = NULL;

// Per-interface input report callback (legacy flavor)
static void hid_input_report_cb(struct hid_interface *itf,
                                const uint8_t *report, size_t len,
                                void *arg)
{
    (void)itf; (void)arg;
    if (len < 3) return;

    uint8_t buttons = report[0] & 0x07;
    int8_t  dx      = (int8_t)report[1];
    int8_t  dy      = (int8_t)report[2];
    int8_t  wheel   = (len >= 4) ? (int8_t)report[3] : 0;

    mouse_link_send_from_hid(buttons, dx, dy, wheel);
}

static void hid_host_event_cb(const hid_host_event_t *event, void *arg)
{
    (void)arg;

    switch (event->event) {
    case HID_HOST_DRIVER_EVENT_CONNECTED: {
        const hid_host_dev_info_t *dev = &event->connected.dev_info;

        // Open device using the API your header exposes. Older drops use
        // hid_host_device_open(dev_addr, const hid_host_device_config_t*)
        // Newer legacy uses *params; we pass minimal config (NULL if allowed).
#ifdef hid_host_device_open
        const hid_host_device_config_t cfg = { 0 };
        ESP_ERROR_CHECK(hid_host_device_open(dev->dev_addr, &cfg, &s_hdev));
#else
        // Fallback if your header prefers "..._get_params" workflow.
        hid_host_device_params_t params = { 0 };
        // (fill params as needed)
        ESP_ERROR_CHECK(hid_host_device_open_with_params(&params, &s_hdev));
#endif

        // Iterate interfaces, pick boot mouse, set protocol, attach callback.
        for (int i = 0; i < dev->num_interfaces; ++i) {
            const hid_host_interface_info_t *ifx = &dev->interfaces[i];
            if (ifx->subclass == HID_SUBCLASS_BOOT &&
                ifx->protocol == HID_PROTOCOL_MOUSE) {

                struct hid_interface *itf = NULL;
                ESP_ERROR_CHECK(hid_host_interface_open(s_hdev, ifx->interface_number, &itf));

#ifdef HID_PROTOCOL_BOOT
                (void)hid_host_class_request_set_protocol(itf, HID_PROTOCOL_BOOT);
#endif
#ifdef hid_host_interface_set_input_report_callback
                hid_host_interface_set_input_report_callback(itf, hid_input_report_cb, NULL);
#else
                // If your header uses a different name for the input-cb setter, add it here.
# warning "Please map your header's input-report registration API here."
#endif
                (void)hid_host_class_request_set_idle(itf, 0, 0);

                ESP_LOGI(TAG, "Boot mouse interface %d armed.", ifx->interface_number);
                break;
            }
        }
        break;
    }
#ifdef HID_HOST_DRIVER_EVENT_DISCONNECTED
    case HID_HOST_DRIVER_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "HID device disconnected.");
        if (s_hdev) { hid_host_device_close(s_hdev); s_hdev = NULL; }
        break;
#endif
    default:
        break;
    }
}

static void usb_hid_mouse_init(void)
{
    const usb_host_config_t host_cfg = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));

    const hid_host_driver_config_t hid_cfg = {
        .create_background_task = true,
        .task_priority          = 5,
        .stack_size             = 4096,
        .callback               = hid_host_event_cb,
        .callback_arg           = NULL,
    };
    ESP_ERROR_CHECK(hid_host_install(&hid_cfg));

    ESP_LOGI(TAG, "USB/HID host ready (legacy API). Plug the USB mouse.");
}
#endif // USE_HID_API_V2

// --------------------- app_main ---------------------

void app_main(void)
{
    uart_init();
    usb_hid_mouse_init();

    // Background driver tasks handle the USB; we just idle.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

