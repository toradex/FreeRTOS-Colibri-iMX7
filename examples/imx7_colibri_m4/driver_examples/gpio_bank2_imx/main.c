/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include "board.h"
#include "gpio_pins.h"
#include "gpio_imx.h"
#include "gpio_ctrl.h"
#include "debug_console_imx.h"


/* button relevent variables */
static uint8_t keyPressCount;
static uint8_t keyPressCountOld;


/******************************************************************************
*
* Function Name: main
* Comments: Hello World Example with GPIO.
*   This example include:
*   Configure BUTTON1 as GPIO functionality
*     and check the button's state(pressed or released). According to the Button
*     status, copy it to the LED
*
******************************************************************************/
int main(void)
{
    /* hardware initialiize, include RDC, IOMUX, Uart debug initialize */
    hardware_init();
    PRINTF("\n\r\n\r\n\r");
    PRINTF("=======================================================\n\r");
    PRINTF("================= GPIO Bank 2 Example =================\n\r");
    PRINTF("===== Change the Led Status according to the Key ======\n\r");
    PRINTF("=======================================================\n\r");

    /* GPIO module initialize, configure "LED" as output and button as interrupt mode. */
    GPIO_Ctrl_Init();

    keyPressCount = 0;
    keyPressCountOld = 0;

    /* Initialize enviroments and led status */
    keyPressCount = GPIO_Ctrl_GetKey(BOARD_GPIO_SWITCH1_CONFIG);
    keyPressCountOld = keyPressCount;
    GPIO_Ctrl_WriteLed(BOARD_GPIO_LED1_CONFIG, BOARD_GPIO_LED1_RDC_PDAP, keyPressCount);

    PRINTF("================== Hello World GPIO ===================\n\r");

    while(true)
    {
        /* wait for user change button leve */
        keyPressCount = GPIO_Ctrl_GetKey(BOARD_GPIO_SWITCH1_CONFIG);
        /* If button change, than print and update LED */
        if(keyPressCount != keyPressCountOld)
        {
            keyPressCountOld = keyPressCount;
            GPIO_Ctrl_WriteLed(BOARD_GPIO_LED1_CONFIG, BOARD_GPIO_LED1_RDC_PDAP, keyPressCount);
            PRINTF("================== keyPressCount = %d ==================\n\r", keyPressCount);
        }
    }
}


/*******************************************************************************
 * EOF
 ******************************************************************************/
