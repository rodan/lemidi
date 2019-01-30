
#include <stdio.h>
#include <string.h>

#include "drivers/uart0.h"
#include "drivers/timer_a0.h"
#include "drivers/mcp42xxx.h"
#include "drivers/max3421.h"
#include "drivers/helper.h"
#include "version.h"
#include "proj.h"
#include "qa.h"


void display_menu(void)
{
    snprintf(str_temp, STR_LEN,
            "\r\n --- lemidi build #%d\r\n  available commands:\r\n", BUILD);
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m?\e[0m                - show menu\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1mb[0-3] [on/off]\e[0m  - button [0-3] on/off\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1mj[0-1] [0-255]\e[0m   - set joystick wiper\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1mstat\e[0m             - system status\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

}

void parse_user_input(void)
{
    char d = uart0_rx_buf[0];
    char id = uart0_rx_buf[1];
    char *in = (char *) uart0_rx_buf;

    uint16_t inum;
    uint8_t reg[32];
    uint8_t i;

    if (d == '?') {
        display_menu();
    } else if (d == 'b') {
        if (id == '0') {
            if (strstr(in, "on")) {
                b0_on;
            } else if (strstr(in, "off")) {
                b0_off;
            }
        } else if (id == '1') {
            if (strstr(in, "on")) {
                b1_on;
            } else if (strstr(in, "off")) {
                b1_off;
            }
        } else if (id == '2') {
            if (strstr(in, "on")) {
                b2_on;
            } else if (strstr(in, "off")) {
                b2_off;
            }
        } else if (id == '3') {
            if (strstr(in, "on")) {
                b3_on;
            } else if (strstr(in, "off")) {
                b3_off;
            }
        }
    } else if (d == 'j') {
        if (id == '0') {
            if (str_to_uint16(in, &inum, 1, strlen(in), 0, 255) == EXIT_SUCCESS) {
                ctrl.j0[0] = inum;
                ctrl.j0[1] = inum;
                //mcp42_set_pot( 0, ctrl.j1[0], ctrl.j1[1]);
                mcp42_set_pot_ch( 0, 0, ctrl.j0[0]);
                mcp42_set_pot_ch( 0, 1, ctrl.j0[1]);
                snprintf(str_temp, STR_LEN, "j0 0x%x 0x%x\r\n", ctrl.j0[0], ctrl.j0[1] );
                uart0_tx_str(str_temp, strlen(str_temp));
            }

        } else if (id == '1') {
            if (str_to_uint16(in, &inum, 1, strlen(in), 0, 255) == EXIT_SUCCESS) {
                ctrl.j1[0] = inum;
                ctrl.j1[1] = inum;
                //mcp42_set_pot( 0, ctrl.j1[0], ctrl.j1[1]);
                mcp42_set_pot_ch( 1, 0, ctrl.j1[0]);
                mcp42_set_pot_ch( 1, 1, ctrl.j1[1]);
                snprintf(str_temp, STR_LEN, "j1 0x%x 0x%x\r\n", ctrl.j1[0], ctrl.j1[1] );
                uart0_tx_str(str_temp, strlen(str_temp));
            }
        }
    } else if (d == 'l') {
        /*
        if (strstr(in, "1 on")) {
            regWr(rIOPINS2, bmGPOUT6); //status led1 on
        } else if (strstr(in, "2 on")) {
            regWr(rIOPINS2, bmGPOUT7); //status led2 on
        }
        */
        regWr(rIOPINS2, 0);
    } else if (d == 'i') {
        //reg = regRd(rIOPINS1);
        //snprintf(str_temp, STR_LEN, "IOPINS1 0x%x\r\n", reg);
        //uart0_tx_str(str_temp, strlen(str_temp));

        snprintf(str_temp, STR_LEN, "R6 0x%x\r\n", regRd(rRCVBC));
        uart0_tx_str(str_temp, strlen(str_temp));

        snprintf(str_temp, STR_LEN, "R7 0x%x\r\n", regRd(rSNDBC));
        uart0_tx_str(str_temp, strlen(str_temp));

        memset(reg, 0xff, 32);
        bytesRd(13<<3, 8, reg);
        for (i=0;i<8;i++) {
            snprintf(str_temp, STR_LEN, "R%u 0x%x\r\n", i+13, reg[i]);
            uart0_tx_str(str_temp, strlen(str_temp));
        }
        bytesRd(21<<3, 11, reg);
        for (i=0;i<11;i++) {
            snprintf(str_temp, STR_LEN, "R%u 0x%x\r\n", i+21, reg[i]);
            uart0_tx_str(str_temp, strlen(str_temp));
        }
        snprintf(str_temp, STR_LEN, "vbusState 0x%x\r\n", MAX3421_getVbusState());
        uart0_tx_str(str_temp, strlen(str_temp));
        snprintf(str_temp, STR_LEN, "int cnt %lu handled %lu\r\n", int_cnt, int_cnt_hl);
        uart0_tx_str(str_temp, strlen(str_temp));
        snprintf(str_temp, STR_LEN, "gpx cnt %lu handled %lu\r\n", gpx_cnt, gpx_cnt_hl);
        uart0_tx_str(str_temp, strlen(str_temp));
        } else if (strstr(in, "stat")) {
            snprintf(str_temp, STR_LEN, "  fault 0x%x, UCSCTL7 0x%x\r\n", SFRIFG1&OFIFG, UCSCTL7 );
            uart0_tx_str(str_temp, strlen(str_temp));
            snprintf(str_temp, STR_LEN, "  UCSCTL4 0x%x, UCSCTL6 0x%x\r\n", UCSCTL4, UCSCTL6 );
            uart0_tx_str(str_temp, strlen(str_temp));
            UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
            SFRIFG1 &= ~OFIFG;
    }
}


