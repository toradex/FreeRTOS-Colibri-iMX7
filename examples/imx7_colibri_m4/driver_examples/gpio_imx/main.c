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
#include "debug_console_imx.h"

/* button relevent variables */
static volatile uint8_t keyPressCount;
static uint8_t keyLastState;
static uint8_t keyState;

/******************************************************************************
*
* Function Name: main
* Comments: GPIO module initialize, interrupt and IO operation.
*   This example include 2 step:
*   1)Configure BUTTON1 as interrupt mode, falling edge, and test 
*     by pressing the button 3 times to trigger interrupt.
*   2)Configure BUTTON1 as GPIO functionality
*     and check the button's state(pressed or released) through switch LED
*     to on or off if this board has LED.
*
******************************************************************************/
int main( void )  
{
    /* hardware initialiize, include RDC, IOMUX, Uart debug initialize */
    hardware_init(); 

    PRINTF("\n\r====================== GPIO Example ========================\n\r");

#ifdef BOARD_GPIO_LED_CONFIG
    /* GPIO module initialize, configure "LED" as output and drive the output high level */
    gpio_init_t ledInitConfig = {
        .pin = BOARD_GPIO_LED_CONFIG->pin,
        .direction = gpioDigitalOutput,
        .interruptMode = gpioNoIntmode
    };
    GPIO_Init(BOARD_GPIO_LED_CONFIG->base, &ledInitConfig);

    /* drive the LED output high level. */
    GPIO_WritePinOutput(BOARD_GPIO_LED_CONFIG->base, BOARD_GPIO_LED_CONFIG->pin, gpioPinSet);
#endif

#ifdef BOARD_GPIO_KEY_CONFIG
    /* GPIO module initialize, configure button as interrupt mode. */
    gpio_init_t keyInitConfig = {
        .pin = BOARD_GPIO_KEY_CONFIG->pin,
        .direction = gpioDigitalInput,
        .interruptMode = gpioIntFallingEdge
    };
    GPIO_Init(BOARD_GPIO_KEY_CONFIG->base, &keyInitConfig);

    NVIC_EnableIRQ(BOARD_GPIO_KEY_IRQ_NUM);
    /* Clear the interrupt state, this operation is necessary, because the GPIO module maybe confuse
       the first rising edge as interrupt*/
    GPIO_ClearStatusFlag(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin);
    /* Enable GPIO pin interrupt */
    GPIO_SetPinIntMode(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin, true);
    /* press button trigger interrupt */
    PRINTF("\n\r=================== GPIO Interrupt =====================\n\r");
    PRINTF("The (%s) button is configured to trigger GPIO interrupt.\n\r", BOARD_GPIO_KEY_CONFIG->name);
    PRINTF("Press the (%s) button 3 times to continue.\n\n\r", BOARD_GPIO_KEY_CONFIG->name);

    keyPressCount = 1;
    while(keyPressCount < 4);

    /* Now disable the interrupt */
    NVIC_DisableIRQ(BOARD_GPIO_KEY_IRQ_NUM);
    GPIO_SetPinIntMode(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin, false);

    /* GPIO module initialize, configure button as GPIO functionality. */
    keyInitConfig.pin = BOARD_GPIO_KEY_CONFIG->pin;
    keyInitConfig.direction = gpioDigitalInput;
    keyInitConfig.interruptMode = gpioNoIntmode;
    GPIO_Init(BOARD_GPIO_KEY_CONFIG->base, &keyInitConfig);

    /* Configure button as GPIO functionality
       and check the button's state(pressed or released) to switch LED on or off */

    /* Check the buttion's status(pressed or released) */
    PRINTF("\n\r================= GPIO Functionality==================\n\r");
    PRINTF("The (%s) button state is now polled.\n\r", BOARD_GPIO_KEY_CONFIG->name);
    PRINTF("Press the (%s) button to switch LED on or off\n\n\r", BOARD_GPIO_KEY_CONFIG->name);

    keyLastState = 1;   //initial button released, logic 1

    for(;;)
    {
        keyState = GPIO_ReadPinInput(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin);
        if(keyState != keyLastState)
        {
            PRINTF("Button %s\n\r", keyState ? "pressed" : "released");
            keyLastState = keyState;
#ifdef BOARD_GPIO_LED_CONFIG
            GPIO_WritePinOutput(BOARD_GPIO_LED_CONFIG->base, BOARD_GPIO_LED_CONFIG->pin, keyState ? gpioPinSet : gpioPinClear);
#endif
        }
    }
#endif
}

/******************************************************************************
* Function Name: BOARD_GPIO_BTN_HANDLER
* Comments: The interrupt service routine triggered by gpio
* Note: Need to consider how to eliminate the button shake problem
******************************************************************************/
void BOARD_GPIO_KEY_HANDLER(void)
{
    PRINTF("Button pressed %d time. \n\r", keyPressCount);
    keyPressCount++;
    /* clear the interrupt status */
    GPIO_ClearStatusFlag(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
