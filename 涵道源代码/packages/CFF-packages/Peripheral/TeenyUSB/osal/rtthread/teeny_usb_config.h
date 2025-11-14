/*       
 *         _______                    _    _  _____ ____  
 *        |__   __|                  | |  | |/ ____|  _ \ 
 *           | | ___  ___ _ __  _   _| |  | | (___ | |_) |
 *           | |/ _ \/ _ \ '_ \| | | | |  | |\___ \|  _ < 
 *           | |  __/  __/ | | | |_| | |__| |____) | |_) |
 *           |_|\___|\___|_| |_|\__, |\____/|_____/|____/ 
 *                               __/ |                    
 *                              |___/                     
 *
 * TeenyUSB - light weight usb stack for micro controllers
 * 
 * Copyright (c) 2020 XToolBox  - admin@xtoolbox.org
 *                         www.tusb.org
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __TEENY_USB_CONFIG_H__
#define __TEENY_USB_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>

/* Use external phy for high speed core */
// #define  OTG_HS_EXTERNAL_PHY

/* Use embedded phy for high speed core */
// #define  OTG_HS_EMBEDDED_PHY

/* Use embedded phy for full speed core */ 
#define  OTG_FS_EMBEDDED_PHY

/* Enable DMA for High speed phy */
// #define  OTG_HS_ENABLE_DMA

/* Support for other speed config and device qualifier descriptor */
#define  SUPPORT_OTHER_SPEED

/** Setup descriptor buffer size, used for other speed config and DMA */
#define DESCRIPTOR_BUFFER_SIZE TEENYUSB_DESCRIPTOR_BUFFER_SIZE

/* Interanl pullup for FS core */
#define  USB_FS_INTERNAL_PULLUP

/* Use CRS clock source for FS core */
#define  USB_FS_CLOCK_SOURCE_CRS

/* USB core ID used in test app, 0 - FS core, 1 - HS core */
#define  USB_CORE_ID_FS             0
#define  USB_CORE_ID_HS             1

#define  TUSB_APP_USB_CORE          USB_CORE_ID_FS


/** debug info level, one of TUSB_DBG_LEVEL_NONE, TUSB_DBG_LEVEL_DEBUG, TUSB_DBG_LEVEL_WARNING, TUSB_DBG_LEVEL_ERROR */
#if defined(TEENYUSB_DBG_DEBUG)
#define TUSB_DBG_LEVEL TUSB_DBG_LEVEL_DEBUG
#elif defined(TEENYUSB_DBG_WARNING)
#define TUSB_DBG_LEVEL TUSB_DBG_LEVEL_WARNING
#elif defined(TEENYUSB_DBG_ERROR)
#define TUSB_DBG_LEVEL TUSB_DBG_LEVEL_ERROR
#else
#define TUSB_DBG_LEVEL TUSB_DBG_LEVEL_NONE
#endif

#define TUSB_HAS_OS

#ifndef TEENYUSB_HOST_ENABLE
#define TUSB_NO_HOST
#endif

#ifndef TEENYUSB_DEVICE_ENABLE
#define TUSB_NO_DEVICE
#endif

#define RTOS_INTERRUPT_ENTER()      rt_interrupt_enter()
#define RTOS_INTERRUPT_LEAVE()      rt_interrupt_leave()
#define TUSB_PRINTF                 rt_kprintf
#define TUSB_MQ_T                   struct rt_messagequeue



#ifdef __cplusplus
}
#endif

#endif
