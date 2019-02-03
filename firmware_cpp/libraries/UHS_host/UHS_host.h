/* Copyright (C) 2015-2016 Andrew J. Kroll
   and
Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

Circuits At Home, LTD
Web      :  http://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */
/* USB functions */
#ifndef _UHS_host_h_
#define _UHS_host_h_

// WARNING: Do not change the order of includes, or stuff will break!
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>

#include <ISR_safe_memory.h>
#include <UHS_ByteBuffer.h>
#include "UHS_macros.h"

// None of these should ever be directly included by a driver, or a user's sketch.
#include "UHS_USB_IDs.h"
#include "UHS_settings.h"
#include "UHS_usb_ch9.h"
#include "UHS_UsbCore.h"
#include "UHS_address.h"
#include "UHS_usbhost.h"
//#include "UHS_printhex.h"
#include "UHS_message.h"

// Load system components as required
#if defined(LOAD_USB_HOST_SYSTEM) && !defined(USB_HOST_SYSTEM_LOADED)
//#include "UHS_util_INLINE.h"
#include "UHS_host_INLINE.h"
//#include "UHS_printf_HELPER.h"

#if defined(LOAD_USB_HOST_SHIELD)
#include "USB_LEMIDI/USB_HOST_SHIELD.h"
#endif

// Load USB drivers and multiplexers

#if defined(LOAD_UHS_HUB)
#include "UHS_HUB/UHS_HUB.h"
#endif // HUB loaded

// Add HID
#if defined(LOAD_UHS_HID)
#include "UHS_HID/UHS_HID.h"
#endif // HID loaded

// Add CDC multiplexers (currently only ACM)
#if defined(LOAD_UHS_CDC_ACM) || defined(LOAD_UHS_CDC_ACM_FTDI) || defined(LOAD_UHS_CDC_ACM_PROLIFIC) || defined(LOAD_UHS_CDC_ACM_XR21B1411)
#include "UHS_CDC/UHS_CDC.h"
#endif // CDC loaded

#endif // System code loaded

#endif // _UHS_host_h_
