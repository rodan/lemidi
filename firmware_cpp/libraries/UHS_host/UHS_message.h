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
#if !defined(_UHS_host_h_) || defined(__MESSAGE_H__)
#error "Never include UHS_message.h directly; include UHS_Usb.h instead"
#else
#define __MESSAGE_H__

extern int UsbDEBUGlvl;

void E_Notify(char const * msg, int lvl);
void E_Notify(uint8_t b, int lvl);
void E_NotifyStr(char const * msg, int lvl);
void E_Notifyc(char c, int lvl);

#ifdef DEBUG_USB_HOST
#define Notify E_Notify
#define NotifyStr E_NotifyStr
#define Notifyc E_Notifyc
void NotifyFailGetDevDescr(uint8_t reason);
void NotifyFailSetDevTblEntry(uint8_t reason);
void NotifyFailGetConfDescr(uint8_t reason);
void NotifyFailSetConfDescr(uint8_t reason);
void NotifyFailGetDevDescr(void);
void NotifyFailSetDevTblEntry(void);
void NotifyFailGetConfDescr(void);
void NotifyFailSetConfDescr(void);
void NotifyFailUnknownDevice(uint16_t VID, uint16_t PID);
void NotifyFail(uint8_t rcode);
#else
#define Notify(...) VOID0
#define NotifyStr(...) VOID0
#define Notifyc(...) VOID0
#define NotifyFailGetDevDescr(...) VOID0
#define NotifyFailSetDevTblEntry(...) VOID0
#define NotifyFailGetConfDescr(...) VOID0
#define NotifyFailGetDevDescr(...) VOID0
#define NotifyFailSetDevTblEntry(...) VOID0
#define NotifyFailGetConfDescr(...) VOID0
#define NotifyFailSetConfDescr(...) VOID0
#define NotifyFailUnknownDevice(...) VOID0
#define NotifyFail(...) VOID0
#endif

#ifdef DEBUG_USB_HOST
template <class ERROR_TYPE> void ErrorMessage(uint8_t level, char const * msg, ERROR_TYPE rcode = 0) {
        Notify(msg, level);
        Notify(PSTR(": "), level);
        D_PrintHex<ERROR_TYPE > (rcode, level);
        Notify(PSTR("\r\n"), level);
#else
template <class ERROR_TYPE> void ErrorMessage(NOTUSED(uint8_t level), NOTUSED(char const * msg), ERROR_TYPE rcode = 0) {
        (void)rcode;
#endif
}

#ifdef DEBUG_USB_HOST
template <class ERROR_TYPE> void ErrorMessage(char const * msg, ERROR_TYPE rcode = 0) {
        Notify(msg, 0x80);
        Notify(PSTR(": "), 0x80);
        D_PrintHex<ERROR_TYPE > (rcode, 0x80);
        Notify(PSTR("\r\n"), 0x80);
#else
template <class ERROR_TYPE> void ErrorMessage(NOTUSED(char const * msg), ERROR_TYPE rcode = 0) {
        (void)rcode;
#endif
}

#endif // __MESSAGE_H__
