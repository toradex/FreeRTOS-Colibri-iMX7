/*
 * Copyright (c) 2017, Toradex
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

#define GPIO_INTERRUPT         (1)
#define GPIO_POLLING           (0)
#define GPIO_DEBOUNCE_DELAY    (100000)

gpio_config_t gpioLcdA0 = {
	.name = "LCD A0 [SSPRXD]",
	.muxReg = &IOMUXC_SW_MUX_CTL_PAD_I2C1_SCL, /* GPIO4 IO08 */
	.muxConfig = 0,
	.padReg = &IOMUXC_SW_PAD_CTL_PAD_I2C1_SCL,
	.padConfig = 0,
	.base = GPIO4,
	.pin = 8,
};

/*!
 * @brief Initialize LCD display
 */
static void LCD_Init(void)
{
	configure_gpio_pin(&gpioLcdA0);

	gpio_init_config_t lcdA0InitConfig = {
		.pin = gpioLcdA0.pin,
		.direction = gpioDigitalOutput,
		.interruptMode = gpioNoIntmode
	};
	GPIO_Init(gpioLcdA0.base, &lcdA0InitConfig);


}

/*!
 * @brief Main entry point
 */
int main(void)
{
	/* hardware initialiize, include RDC, IOMUX and UART debug */
	hardware_init();
	PRINTF("\n\r=> Low Power Demo\n\r");

	/* GPIO module initialize, configure "LED" as output and button as interrupt mode. */
	LCD_Init();

	while(true);

	return 0;
}

