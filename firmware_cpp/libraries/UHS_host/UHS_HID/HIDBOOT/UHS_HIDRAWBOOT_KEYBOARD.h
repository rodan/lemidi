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

// TO-DO: parse config.
// TO-DO: implement cooked interface.

#if !defined(__UHS_HIDBOOT_KEYBOARD_H__)
#define __UHS_HIDBOOT_KEYBOARD_H__

#if !defined(DEBUG_PRINTF_EXTRA_HUGE_USB_HIDBOOT_KEYBOARD)
#define DEBUG_PRINTF_EXTRA_HUGE_USB_HIDBOOT_KEYBOARD 0
#endif

#if DEBUG_PRINTF_EXTRA_HUGE
#if DEBUG_PRINTF_EXTRA_HUGE_USB_HIDBOOT_KEYBOARD
#define HIDBOOT_KEYBOARD_DUBUG(...) printf(__VA_ARGS__)
#else
#define HID_DUBUGBOOT_KEYBOARD(...) VOID0
#endif
#else
#define HID_DUBUGBOOT_KEYBOARD(...) VOID0
#endif

class UHS_HIDBOOT_keyboard : public UHS_HID_base {
public:

        uint8_t led_states;

        UHS_HIDBOOT_keyboard(UHS_HID *p) {
                parent = p;
                driver = UHS_HID_keyboard;
                led_states = 0;
        }

        ~UHS_HIDBOOT_keyboard() {
        };

        void AJK_NI driverPoll() {
                uint8_t data[parent->epInfo[parent->epInterruptInIndex].maxPktSize];
                uint16_t length = parent->epInfo[parent->epInterruptInIndex].maxPktSize;
                uint8_t rv = parent->pUsb->inTransfer(parent->bAddress, parent->epInfo[parent->epInterruptInIndex].epAddr, &length, data);

                if(rv == 0) {
                        parent->hidProcessor->onPoll(this, data, length);
                } else if(rv != UHS_HOST_ERROR_NAK) {
                        HID_DUBUGBOOT_KEYBOARD("DP %02x A %02x EI %02x EA %02x\r\n", rv, parent->bAddress, parent->epInterruptInIndex, parent->epInfo[parent->epInterruptInIndex].epAddr);
                }
        }

        uint8_t AJK_NI SetLEDs(uint8_t leds) {
                uint8_t rv;
                parent->pUsb->DisablePoll();
                led_states = leds;
                rv = parent->SetReport(parent->bIface, 2, 0, 1, &leds);
                parent->pUsb->EnablePoll();
                return rv;
        }

        uint8_t AJK_NI getLEDs(void) {
                return led_states;
        }

        void AJK_NI driverStart() {
                printf("BOOT_KEYBOARD\r\n");
                {
                        uint16_t length = 128;
                        uint8_t buffer[length];
                        parent->ReportDescr(parent->bIface, length, buffer);
                }
                parent->SetProtocol(parent->bIface, 0);
                uint8_t rv;
                uint8_t led = 0x40U;
                while(led) {
                        led >>= 1;
                        rv = SetLEDs(led);
                        if(rv != 0) return; // skip onStart if unplugged.
                        if(!parent->UHS_SLEEP_MS(50)) return; // skip onStart if unplugged.
                }
                parent->SetIdle(parent->bIface, 0, 0);

                parent->hidProcessor->onStart(this);
        }

        void AJK_NI driverRelease() {
                parent->hidProcessor->onRelease(this);
        }
};

#endif // __UHS_HIDBOOT_KEYBOARD_H__
