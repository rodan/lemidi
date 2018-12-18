
#include <stdio.h>
#include <string.h>

#include "drivers/uart0.h"
#include "drivers/timer_a0.h"
#include "drivers/rtc.h"
#include "drivers/lcd.h"
#include "version.h"
#include "qa.h"


void display_menu(void)
{
    snprintf(str_temp, STR_LEN,
            "\r\n --- retrojoy build #%d\r\n  available commands:\r\n", BUILD);
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m?\e[0m              - show menu\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!led [on/off]\e[0m  - led on/off\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!lcd [init/s]\e[0m  - led on/off\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!stat\e[0m          - system status\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

}

void parse_user_input(void)
{
    char f = uart0_rx_buf[0];
    char *in = (char *) uart0_rx_buf;

    if (f == '?') {
        display_menu();
    } else if (f == '!') {
        if (strstr(in, "led")) {
            if (strstr(in, "on")) {
                LED_ON;
            } else if (strstr(in, "off")) {
                LED_OFF;
            }
        } else if (strstr(in, "stat")) {
            snprintf(str_temp, STR_LEN, "  rtca %lus\r\n", rtca_time.sys );
            uart0_tx_str(str_temp, strlen(str_temp));
        } else if (strstr(in, "lcd")) {
            if (strstr(in, "init")) {
                LCD_Init();
            } else if (strstr(in, "s")) {
                snprintf(str_temp, STR_LEN, "%8lu", rtca_time.sys );
                LCD_Send_STR(1, str_temp);
            }
        }
    }
}

