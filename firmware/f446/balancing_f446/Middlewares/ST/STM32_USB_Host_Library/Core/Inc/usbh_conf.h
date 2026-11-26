#pragma once
/* F446専用、RTOS/動的割当/printfなし。固定容量のHost Core設定。 */
#include "stm32f4xx_hal.h"
#include <string.h>
#define USBH_MAX_NUM_ENDPOINTS 8U
#define USBH_MAX_NUM_INTERFACES 16U
#define USBH_MAX_NUM_CONFIGURATION 1U
#define USBH_KEEP_CFG_DESCRIPTOR 1U
#define USBH_MAX_NUM_SUPPORTED_CLASS 1U
#define USBH_MAX_SIZE_CONFIGURATION 512U
#define USBH_MAX_DATA_BUFFER 512U
#define USBH_DEBUG_LEVEL 0U
#define USBH_USE_OS 0U
#define USBH_IN_NAK_PROCESS 0U
#define USBH_memset memset
#define USBH_memcpy memcpy
#define USBH_UsrLog(...) do {} while (0)
#define USBH_ErrLog(...) do {} while (0)
#define USBH_DbgLog(...) do {} while (0)
