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
// Initiates a state machine to identify the connected the device and used host libraries to host it.
// Project Partners: UWA, ANFF
//-------------------------------------------------------------------------
 

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "tinyusb.h"
#include "class/hid/hid.h"
#include "tusb.h"

#define TAG "DEVICE_RXTX"

// ---------- UART & Protocol (MUST MATCH HOST) ----------
#define UART_NUM        UART_NUM_1
#define UART_TX_GPIO    (17)
#define UART_RX_GPIO    (18)
#define UART_BAUD       (460800)
#define UART_RXBUF      (4096)
#define UART_TXBUF      (4096)

#define FRAME_HDR            0xA5
#define TYPE_ANNOUNCE_HID    0x01
#define TYPE_ANNOUNCE_MSC    0x02
#define TYPE_MOUSE           0x03
#define TYPE_KEYBOARD        0x04

// MSC bridge commands/responses (must match host)
#define CMD_TEST_UNIT_READY  0x00
#define CMD_REQUEST_SENSE    0x03
#define CMD_READ_CAPACITY    0x25
#define CMD_READ10           0x28
#define CMD_WRITE10          0x2A
#define RSP_SUCCESS          0x01
#define RSP_FAILED           0x02

// ---------- Endian helpers (ESP32 is little-endian: identity) ----------
#ifndef htole16
#define htole16(x)  ((uint16_t)(x))
#define htole32(x)  ((uint32_t)(x))
#define le16toh(x)  ((uint16_t)(x))
#define le32toh(x)  ((uint32_t)(x))
#endif

// ---------- State ----------
typedef enum { MODE_NONE=0, MODE_HID=1, MODE_MSC=2 } dev_mode_t;
static volatile dev_mode_t g_current_mode = MODE_NONE;

// Protects g_current_mode
static SemaphoreHandle_t g_state_mtx;
// Protects UART while in MSC (callbacks)
static SemaphoreHandle_t g_uart_mtx;

// ---------- HID report structs (packed, same as host used) ----------
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

// ---------- TinyUSB descriptors (Composite: HID Kbd + HID Mouse + MSC) ----------

// HID report descriptors
static const uint8_t hid_report_kbd[] = {
  TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(1))
};
static const uint8_t hid_report_mouse[] = {
  TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(2))
};

// USB device descriptor
static const tusb_desc_device_t desc_device = {
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = 0x0200,
  .bDeviceClass       = TUSB_CLASS_MISC, // IAD
  .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,
  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

  .idVendor           = 0xCafe,   // adjust if you need a fixed VID/PID
  .idProduct          = 0x40ED,   // composite example
  .bcdDevice          = 0x0100,

  .iManufacturer      = 0x01,
  .iProduct           = 0x02,
  .iSerialNumber      = 0x03,

  .bNumConfigurations = 0x01
};

// Endpoint helpers
#define EPNUM_HID_KBD   0x81
#define EPNUM_HID_MOUSE 0x82
#define EPNUM_MSC_OUT   0x01
#define EPNUM_MSC_IN    0x82

// Configuration descriptor: 3 interfaces (HID KBD, HID MOUSE, MSC)
enum {
  ITF_NUM_HID_KBD,
  ITF_NUM_HID_MOUSE,
  ITF_NUM_MSC,
  ITF_NUM_TOTAL
};

#define TUD_CONFIG_TOTAL_LEN ( \
  TUD_CONFIG_DESC_LEN + \
  TUD_HID_INOUT_DESC_LEN + \
  TUD_HID_INOUT_DESC_LEN + \
  TUD_MSC_DESC_LEN )

static const uint8_t desc_configuration[] = {
  // Config
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUD_CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

  // HID Keyboard (ID 1)
  TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID_KBD, 4, HID_ITF_PROTOCOL_KEYBOARD,
                           sizeof(hid_report_kbd), EPNUM_HID_KBD, 8, 5),

  // HID Mouse (ID 2)
  TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID_MOUSE, 5, HID_ITF_PROTOCOL_MOUSE,
                           sizeof(hid_report_mouse), EPNUM_HID_MOUSE, 8, 5),

  // MSC
  TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 6, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

// String descriptors
static const char *string_desc_arr[] = {
  (const char[]) { 0x09, 0x04 }, // 0: English (0x0409)
  "YourCo",                      // 1: Manufacturer
  "RX/TX Composite HID+MSC",     // 2: Product
  "123456",                      // 3: Serial
  "HID Keyboard",                // 4
  "HID Mouse",                   // 5
  "MSC",                         // 6
};

// TinyUSB descriptor callbacks (IDF takes these)
const tusb_desc_device_t *tud_descriptor_device_cb(void) {
  return &desc_device;
}
const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
  (void) index;
  return desc_configuration;
}
uint16_t tud_descriptor_string_cb(uint8_t index, uint16_t langid, uint8_t *buffer, uint16_t buflen) {
  (void) langid;
  const char* str;
  if (index == 0) {
    // supported language is English (0x0409)
    uint16_t* u16 = (uint16_t*) buffer;
    u16[0] = (TUSB_DESC_STRING << 8 ) | (2 + 2);
    u16[1] = 0x0409;
    return 4;
  }
  if (index >= sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) return 0;
  str = string_desc_arr[index];

  uint8_t count = 0;
  while (str[count] && count < (buflen/2 - 1)) {
    ((uint16_t*)buffer)[count + 1] = str[count];
    count++;
  }
  ((uint16_t*)buffer)[0] = (TUSB_DESC_STRING << 8 ) | (2*(count + 1));
  return 2*(count + 1);
}

// HID report descriptor callback
uint8_t const *tud_hid_descriptor_report_cb(uint8_t itf) {
  if (itf == ITF_NUM_HID_KBD)   return hid_report_kbd;
  if (itf == ITF_NUM_HID_MOUSE) return hid_report_mouse;
  return NULL;
}

// ---------- UART ----------
static void uart_init(void) {
  uart_driver_delete(UART_NUM);
  const uart_config_t cfg = {
    .baud_rate = UART_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk= UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RXBUF, UART_TXBUF, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// ---------- MSC <-> UART helpers (device drives commands to host) ----------
static bool uart_write_all(const void* data, size_t len, TickType_t to_ticks) {
  const uint8_t* p = (const uint8_t*)data;
  while (len) {
    int w = uart_write_bytes(UART_NUM, (const char*)p, len);
    if (w <= 0) return false;
    p += w; len -= w;
  }
  // ensure data pushed
  return true;
}
static bool uart_read_all(void* data, size_t len, TickType_t to_ticks) {
  uint8_t* p = (uint8_t*)data;
  TickType_t deadline = xTaskGetTickCount() + to_ticks;
  while (len) {
    TickType_t now = xTaskGetTickCount();
    if (now >= deadline) return false;
    int r = uart_read_bytes(UART_NUM, p, len, deadline - now);
    if (r <= 0) return false;
    p += r; len -= r;
  }
  return true;
}
static bool uart_read_byte(uint8_t* out, TickType_t to_ticks) {
  int r = uart_read_bytes(UART_NUM, out, 1, to_ticks);
  return r == 1;
}

// ---------- MSC callbacks ----------
static int32_t g_block_size = 512;
static int64_t g_block_count = 0;

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
  (void)lun;
  if (g_current_mode != MODE_MSC) return false;

  bool ready = false;
  if (xSemaphoreTake(g_uart_mtx, pdMS_TO_TICKS(2000)) == pdTRUE) {
    uint8_t cmd = CMD_TEST_UNIT_READY;
    uint8_t rsp = 0;
    ready = uart_write_all(&cmd, 1, pdMS_TO_TICKS(200)) &&
            uart_read_byte(&rsp, pdMS_TO_TICKS(1000)) &&
            (rsp == RSP_SUCCESS);
    xSemaphoreGive(g_uart_mtx);
  }
  return ready;
}

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
  (void)lun;
  const char vid[8]  = "YourCo ";
  const char pid[16] = "RTx Bridge     ";
  const char rev[4]  = "1.0 ";
  memcpy(vendor_id,  vid,  8);
  memcpy(product_id, pid, 16);
  memcpy(product_rev,rev,  4);
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) {
  (void)lun; (void)power_condition; (void)load_eject;
  // We don't manage media here; say OK.
  return true;
}

bool tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size) {
  (void)lun;
  if (g_current_mode != MODE_MSC) return false;

  bool ok = false;
  if (xSemaphoreTake(g_uart_mtx, pdMS_TO_TICKS(2000)) == pdTRUE) {
    uint8_t cmd = CMD_READ_CAPACITY, rsp=0;
    uint32_t cnt_le=0, sz_le=0;
    ok = uart_write_all(&cmd, 1, pdMS_TO_TICKS(200)) &&
         uart_read_all(&cnt_le, 4, pdMS_TO_TICKS(1000)) &&
         uart_read_all(&sz_le,  4, pdMS_TO_TICKS(1000)) &&
         uart_read_byte(&rsp,   pdMS_TO_TICKS(1000)) &&
         (rsp == RSP_SUCCESS);
    xSemaphoreGive(g_uart_mtx);

    if (ok) {
      g_block_count = (int64_t)le32toh(cnt_le);
      g_block_size  = (int32_t)le32toh(sz_le);
      if (g_block_size <= 0) g_block_size = 512;
      *block_count = (uint32_t)g_block_count;
      *block_size  = (uint16_t)g_block_size;
      return true;
    }
  }
  return false;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  (void)lun;
  if (g_current_mode != MODE_MSC) return -1;
  if (offset != 0) return 0; // TinyUSB may call in chunks; we use whole-sector I/O

  uint32_t sectors = bufsize / g_block_size;
  if (sectors == 0) return 0;

  int32_t out_bytes = -1;
  if (xSemaphoreTake(g_uart_mtx, pdMS_TO_TICKS(5000)) == pdTRUE) {
    uint8_t cmd = CMD_READ10, rsp=0;
    uint32_t lba_le   = htole32(lba);
    uint16_t cnt_le   = htole16((uint16_t)sectors);

    bool ok = uart_write_all(&cmd, 1, pdMS_TO_TICKS(200)) &&
              uart_write_all(&lba_le, 4, pdMS_TO_TICKS(200)) &&
              uart_write_all(&cnt_le, 2, pdMS_TO_TICKS(200));

    if (ok) {
      ok = uart_read_all(buffer, sectors * g_block_size, pdMS_TO_TICKS(8000)) &&
           uart_read_byte(&rsp, pdMS_TO_TICKS(1000)) &&
           (rsp == RSP_SUCCESS);
    }
    xSemaphoreGive(g_uart_mtx);
    if (ok) out_bytes = (int32_t)(sectors * g_block_size);
  }
  return out_bytes;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  (void)lun;
  if (g_current_mode != MODE_MSC) return -1;
  if (offset != 0) return 0;

  uint32_t sectors = bufsize / g_block_size;
  if (sectors == 0) return 0;

  int32_t written = -1;
  if (xSemaphoreTake(g_uart_mtx, pdMS_TO_TICKS(5000)) == pdTRUE) {
    uint8_t cmd = CMD_WRITE10, rsp=0;
    uint32_t lba_le = htole32(lba);
    uint16_t cnt_le = htole16((uint16_t)sectors);

    bool ok = uart_write_all(&cmd, 1, pdMS_TO_TICKS(200)) &&
              uart_write_all(&lba_le, 4, pdMS_TO_TICKS(200)) &&
              uart_write_all(&cnt_le, 2, pdMS_TO_TICKS(200)) &&
              uart_write_all(buffer, sectors * g_block_size, pdMS_TO_TICKS(8000)) &&
              uart_read_byte(&rsp, pdMS_TO_TICKS(1000)) &&
              (rsp == RSP_SUCCESS);
    xSemaphoreGive(g_uart_mtx);
    if (ok) written = (int32_t)(sectors * g_block_size);
  }
  return written;
}

bool tud_msc_request_sense_cb(uint8_t lun, uint8_t sense_key[3]) {
  (void)lun;
  // Ask host for last sense
  if (g_current_mode != MODE_MSC) { sense_key[0]=0; sense_key[1]=0; sense_key[2]=0; return true; }

  bool ok = false;
  if (xSemaphoreTake(g_uart_mtx, pdMS_TO_TICKS(2000)) == pdTRUE) {
    uint8_t cmd = CMD_REQUEST_SENSE, rsp=0;
    uint8_t sense18[18];
    ok = uart_write_all(&cmd, 1, pdMS_TO_TICKS(200)) &&
         uart_read_all(sense18, 18, pdMS_TO_TICKS(1000)) &&
         uart_read_byte(&rsp, pdMS_TO_TICKS(1000)) &&
         (rsp == RSP_SUCCESS);
    xSemaphoreGive(g_uart_mtx);
    if (ok) {
      // sense18[2] key, [12] asc, [13] ascq
      sense_key[0] = sense18[2];
      sense_key[1] = sense18[12];
      sense_key[2] = sense18[13];
      return true;
    }
  }
  // default no sense
  sense_key[0]=0; sense_key[1]=0; sense_key[2]=0;
  return true;
}

// ---------- HID callbacks ----------
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t type,
                               uint8_t* buf, uint16_t reqlen) {
  (void)itf; (void)report_id; (void)type; (void)buf; (void)reqlen;
  return 0; // we only send reports from UART
}
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t type,
                           uint8_t const* buffer, uint16_t bufsize) {
  (void)itf; (void)report_id; (void)type; (void)buffer; (void)bufsize;
  // Not used
}

// ---------- UART manager (parses frames ONLY in HID/NONE) ----------
static void uart_manager_task(void *arg) {
  uint8_t b;
  for (;;) {
    // If in MSC, DO NOT touch UART: MSC callbacks own it.
    if (g_current_mode == MODE_MSC) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    int r = uart_read_bytes(UART_NUM, &b, 1, pdMS_TO_TICKS(20));
    if (r != 1) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
    if (b != FRAME_HDR) continue;

    uint8_t type=0;
    if (uart_read_bytes(UART_NUM, &type, 1, pdMS_TO_TICKS(50)) != 1) continue;

    if (type == TYPE_ANNOUNCE_HID) {
      if (xSemaphoreTake(g_state_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_current_mode = MODE_HID;
        xSemaphoreGive(g_state_mtx);
        ESP_LOGI(TAG, "Switched to HID mode (announce)");
      }
    } else if (type == TYPE_ANNOUNCE_MSC) {
      if (xSemaphoreTake(g_state_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_current_mode = MODE_MSC;
        xSemaphoreGive(g_state_mtx);
        ESP_LOGI(TAG, "Switched to MSC mode (announce)");
        // On next loop iteration, parser will idle, leaving UART to MSC callbacks
      }
    } else if (type == TYPE_MOUSE) {
      if (g_current_mode == MODE_HID && tud_hid_ready()) {
        hid_mouse_report_t rep;
        if (uart_read_all(&rep, sizeof(rep), pdMS_TO_TICKS(20))) {
          // send as report ID 2
          tud_hid_mouse_report(2, rep.buttons, rep.dx, rep.dy, rep.wheel, 0);
        }
      } else {
        // drain payload if sent in wrong mode
        hid_mouse_report_t tmp; uart_read_all(&tmp, sizeof(tmp), pdMS_TO_TICKS(20));
      }
    } else if (type == TYPE_KEYBOARD) {
      if (g_current_mode == MODE_HID && tud_hid_ready()) {
        hid_keyboard_report_t rep;
        if (uart_read_all(&rep, sizeof(rep), pdMS_TO_TICKS(20))) {
          tud_hid_keyboard_report(1, rep.modifier, rep.keycode);
        }
      } else {
        hid_keyboard_report_t tmp; uart_read_all(&tmp, sizeof(tmp), pdMS_TO_TICKS(20));
      }
    } else {
      // Unknown frame type: ignore (MSC traffic is raw and shouldn't get here)
    }
  }
}

// ---------- TinyUSB events (optional logs) ----------
void tud_mount_cb(void)    { ESP_LOGI(TAG, "USB Mounted"); }
void tud_umount_cb(void)   { ESP_LOGI(TAG, "USB Unmounted"); }
void tud_suspend_cb(bool)  { ESP_LOGI(TAG, "USB Suspended"); }
void tud_resume_cb(void)   { ESP_LOGI(TAG, "USB Resumed"); }

// ---------- app_main ----------
void app_main(void) {
  g_state_mtx = xSemaphoreCreateMutex();  assert(g_state_mtx);
  g_uart_mtx  = xSemaphoreCreateMutex();  assert(g_uart_mtx);

  uart_init();

  // Init TinyUSB (composite once)
  tinyusb_config_t tusb_cfg = {
    .device_descriptor = &desc_device,
    .string_descriptor = string_desc_arr,
    .string_descriptor_count = sizeof(string_desc_arr)/sizeof(string_desc_arr[0]),
    .external_phy = false,
    .configuration_descriptor = desc_configuration,
  };
  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
  ESP_LOGI(TAG, "TinyUSB device started (composite HID+MSC)");

  // Start UART manager
  xTaskCreate(uart_manager_task, "uart_mgr", 4096, NULL, 5, NULL);

  ESP_LOGI(TAG, "Device ready. Waiting for host announcements over UART.");
}

