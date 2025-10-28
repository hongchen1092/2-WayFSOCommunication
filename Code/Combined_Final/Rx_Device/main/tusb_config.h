#pragma once

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUSB_MCU                OPT_MCU_ESP32S3
#define CFG_TUSB_OS                 OPT_OS_FREERTOS
#define CFG_TUSB_RHPORT0_MODE       (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_ENDPOINT0_SIZE      64

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUD_HID                 2       // keyboard + mouse
#define CFG_TUD_MSC                 1
#define CFG_TUD_CDC                 0
#define CFG_TUD_VENDOR              0
#define CFG_TUD_NET                 0
#define CFG_TUD_AUDIO               0
#define CFG_TUD_DFU_RT              0

//--------------------------------------------------------------------
// HID CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUD_HID_EP_BUFSIZE      8
#define CFG_TUD_HID_POLL_INTERVAL   10

//--------------------------------------------------------------------
// MSC CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUD_MSC_EP_BUFSIZE      512
#define CFG_TUD_MSC_BUFSIZE         512

#ifdef __cplusplus
}
#endif
