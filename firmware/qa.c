
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
    char buf[18];

    uart0_print("\r\n --- lemidi build #");
    uart0_print(_utoa(buf, BUILD));
    uart0_print(" available commands:\r\n");
    uart0_print(" \e[33;1m?\e[0m                - show menu\r\n");
    uart0_print(" \e[33;1mb[0-3] [on/off]\e[0m  - button [0-3] on/off\r\n");
    uart0_print(" \e[33;1mj[0-1] [0-255]\e[0m   - set joystick wiper\r\n");
    uart0_print(" \e[33;1mstat\e[0m             - system status\r\n");
}

void parse_user_input(void)
{
    char *in = uart0_get_rx_buf();
    char d = in[0];
    char id = in[1];

    uint16_t inum;
    uint8_t reg[32];
    uint8_t i;
    char buf[18];

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
                
                uart0_print("j0 ");
                uart0_print(_utoa(buf, ctrl.j0[0]));
                uart0_print(" ");
                uart0_print(_utoa(buf, ctrl.j0[1]));
                uart0_print("\r\n");
            }

        } else if (id == '1') {
            if (str_to_uint16(in, &inum, 1, strlen(in), 0, 255) == EXIT_SUCCESS) {
                ctrl.j1[0] = inum;
                ctrl.j1[1] = inum;
                //mcp42_set_pot( 0, ctrl.j1[0], ctrl.j1[1]);
                mcp42_set_pot_ch( 1, 0, ctrl.j1[0]);
                mcp42_set_pot_ch( 1, 1, ctrl.j1[1]);

                uart0_print("j1 ");
                uart0_print(_utoa(buf, ctrl.j1[0]));
                uart0_print(" ");
                uart0_print(_utoa(buf, ctrl.j1[1]));
                uart0_print("\r\n");
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

        //snprintf(str_temp, STR_LEN, "R6 0x%x\r\n", regRd(rRCVBC));
        //uart0_tx_str(str_temp, strlen(str_temp));
        //snprintf(str_temp, STR_LEN, "R7 0x%x\r\n", regRd(rSNDBC));
        //uart0_tx_str(str_temp, strlen(str_temp));

        memset(reg, 0xff, 32);
        bytesRd(13<<3, 8, reg);
        for (i=0;i<8;i++) {
            uart0_print("\r\nR");
            uart0_print(_utoa(buf, i+13));
            uart0_print(" ");
            uart0_print(_utob(buf, reg[i]));
        }
        bytesRd(21<<3, 11, reg);
        for (i=0;i<11;i++) {
            uart0_print("\r\nR");
            uart0_print(_utoa(buf, i+21));
            uart0_print(" ");
            uart0_print(_utob(buf, reg[i]));
        }
        uart0_print("\r\nvbusState ");
        uart0_print(_utoa(buf, MAX3421_getVbusState()));
 
        uart0_print("\r\nint cnt ");
        uart0_print(_utoa(buf, int_cnt));
        uart0_print(" handled ");
        uart0_print(_utoa(buf, int_cnt_hl));
 
        uart0_print("\r\ngpx cnt ");
        uart0_print(_utoa(buf, gpx_cnt));
        uart0_print(" handled ");
        uart0_print(_utoa(buf, gpx_cnt_hl));
        
        } else if (strstr(in, "stat")) {
            /*
            snprintf(str_temp, STR_LEN, "  fault 0x%x, UCSCTL7 0x%x\r\n", SFRIFG1&OFIFG, UCSCTL7 );
            uart0_tx_str(str_temp, strlen(str_temp));
            snprintf(str_temp, STR_LEN, "  UCSCTL4 0x%x, UCSCTL6 0x%x\r\n", UCSCTL4, UCSCTL6 );
            uart0_tx_str(str_temp, strlen(str_temp));
            UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
            SFRIFG1 &= ~OFIFG;
            */
    }
}


