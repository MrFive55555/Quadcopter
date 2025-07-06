#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUSB_MCU OPT_MCU_STM32H5
#define CFG_TUSB_OS OPT_OS_FREERTOS

// ★★★ 关键定义：将 Port 0 配置为设备模式 ★★★
// 这就是您错误信息中缺失的宏
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE)

// Enable device stack
#define CFG_TUD_ENABLED 1

// A-L-P-H-A
// Temporarily disable the use of pipex which is not available for H5
// For more details, see: https://github.com/hathach/tinyusb/issues/2423
#define TUP_USBIP_DCD_PIPE_XFER_DISABLED 1

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUD_ENDPOINT0_SIZE 64

//------------- CLASS -------------//
#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

// CDC buffer size
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256

#ifdef __cplusplus
}
#endif

#endif