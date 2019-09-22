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
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "debug_console_imx.h"
#include "ecspi.h"
#include "gpio_pins.h"
#include "gpio_imx.h"

#define GPIO_INTERRUPT         (1)
#define GPIO_POLLING           (0)
#define GPIO_DEBOUNCE_DELAY    (100000)

#define BURST_LENGTH_IN_BYTES(x)        ((8 * x) - 1)

enum lcd_cmd_type {
	LCD_COMMAND = 0,
	LCD_DATA = 1,
};

gpio_config_t gpioLcdA0 = {
	.name = "LCD A0 [SSPRXD]",
	.muxReg = &IOMUXC_SW_MUX_CTL_PAD_I2C1_SCL, /* GPIO4 IO08 */
	.muxConfig = 5,
	.padReg = &IOMUXC_SW_PAD_CTL_PAD_I2C1_SCL,
	.padConfig = 0x4, /* SRE=Slow */
	.base = GPIO4,
	.pin = 8,
};

gpio_config_t gpioLcdReset = {
	.name = "GPIO1 IO9 [PWM B]",
	.muxReg = &IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO09,
	.muxConfig = 0,
	.padReg = &IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO09,
	.padConfig = 0,
	.base = GPIO1,
	.pin = 9,
};

/*!
 * @brief Initialize LCD display
 */
static void LCD_Init(void)
{
	gpio_init_config_t lcdA0InitConfig = {
		.pin = gpioLcdA0.pin,
		.direction = gpioDigitalOutput,
		.interruptMode = gpioNoIntmode
	};

	gpio_init_config_t lcdResetConfig = {
		.pin = gpioLcdReset.pin,
		.direction = gpioDigitalOutput,
		.interruptMode = gpioNoIntmode
	};

	ecspi_init_config_t ecspiMasterInitConfig = {
		.baudRate = 1000000,
		.mode = ecspiMasterMode,
		.burstLength = 7,
		.channelSelect = ecspiSelectChannel0,
		.clockPhase = ecspiClockPhaseSecondEdge,
		.clockPolarity = ecspiClockPolarityActiveLow,
		.ecspiAutoStart = false,
	};

	configure_gpio_pin(&gpioLcdReset);
	GPIO_Init(gpioLcdReset.base, &lcdResetConfig);

	GPIO_WritePinOutput(gpioLcdReset.base, gpioLcdReset.pin, gpioPinClear);
	vTaskDelay(2);
	GPIO_WritePinOutput(gpioLcdReset.base, gpioLcdReset.pin, gpioPinSet);

	configure_gpio_pin(&gpioLcdA0);
	GPIO_Init(gpioLcdA0.base, &lcdA0InitConfig);

	ecspiMasterInitConfig.clockRate =
		get_ecspi_clock_freq(BOARD_ECSPI_BASEADDR);

	ECSPI_Init(BOARD_ECSPI_BASEADDR, &ecspiMasterInitConfig);
	ECSPI_SetSCLKInactiveState(BOARD_ECSPI_BASEADDR, 0, ecspiSclkStayHigh);
	NVIC_EnableIRQ(BOARD_ECSPI_IRQ_NUM);
}

static void LCD_SendBytes(const uint8_t *buf, int count, enum lcd_cmd_type cmd)
{
	int bytes;
	uint32_t data;
	gpio_pin_action_t a0;

	if (cmd == LCD_COMMAND)
		a0 = gpioPinClear;
	else
		a0 = gpioPinSet;
	GPIO_WritePinOutput(gpioLcdA0.base, gpioLcdA0.pin, a0);

	ECSPI_SetBurstLength(BOARD_ECSPI_BASEADDR, BURST_LENGTH_IN_BYTES(count));

	while (count > 0 && !ECSPI_GetStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoFull)) {
		int i;
		bytes = count & 0x3;
		bytes = bytes ? bytes : 4;
		
		data = 0;
		for(i = 0; i < bytes; i++)
			data = (data << 8) | *(buf)++;

		ECSPI_SendData(BOARD_ECSPI_BASEADDR, data);
		count -= bytes;
	}

//	PRINTF("LCD_SendBytes, end count %d\n\r", count);

	//ECSPI_SetIntCmd(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoEmpty, true);
	//ECSPI_SetIntCmd(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoTc, true);

	ECSPI_StartBurst(BOARD_ECSPI_BASEADDR);
//PRINTF("Status %x\n\r", ECSPI_GetStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoTc));
	while (!ECSPI_GetStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoTc));
	ECSPI_ClearStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoTc);
}

#define CLAMP(x, low, high) { if ( (x) < (low) ) x = (low); if ( (x) > (high) ) x = (high); }
#define LCDWIDTH 128
#define LCDHEIGHT 64
#define LCDPAGES  (LCDHEIGHT+7)/8

static void LCD_SetXY(int x, int y)
{
	unsigned char cmd[3];

	PRINTF("LCD_SetXY(%d,%d)\n\r", x, y);
	CLAMP(x, 0, LCDWIDTH-1);
	CLAMP(y, 0, LCDPAGES-1);

	cmd[0] = 0xB0 | (y&0xF);
	cmd[1] = 0x10 | (x&0xF);
	cmd[2] = (x>>4)&0xF;
	LCD_SendBytes(cmd, 3, LCD_COMMAND);
}

void UITask(void *pvParameters)
{
	const uint8_t LCD_init_seq[] = {
		0x40, // Display start line 0
		0xa1, // ADC reverse
		0xc0, // Normal COM0~COM63
		0xa6, // Display normal
		0xa2, // Set bias 1/9 (Duty 1/65)
		0x2f, // Booster, Regulator and Follower on
		0xf8, // Set internal Booster to 4x
		0x00,
		0x27, // Contrast set
		0x81,
		0x16,
		0xac, // No indicator
		0x00,
		0xaf, // Display on
	};


	/* GPIO module initialize, configure "LED" as output and button as interrupt mode. */
	LCD_Init();

	LCD_SendBytes(LCD_init_seq, sizeof(LCD_init_seq), LCD_COMMAND);
	PRINTF("After init\n\r");
/*
	uint8_t cmd = 0xA4 | 1;
	LCD_SendBytes(&cmd, 1, LCD_COMMAND);
*/
	uint8_t data = 0xff;
	while (true) {
		data = ~data;
		for (int y = 0; y < 8; y++) {
			LCD_SetXY(0,y);
			for (int x = 0; x < 128; x++) {
				LCD_SendBytes(&data, 1, LCD_DATA);
				vTaskDelay(5);
			}
		}
	}
/*
	while (true) {
		LCD_SetXY(x,y);
		LCD_SendBytes(&data, 1, LCD_DATA);
		x++;
		if (x == 128) {
			x = 0;
			y++;
		}
		if (y == 64) {
			x = 0;
			y = 0;
		}
	}*/
}

/*!
 * @brief Main entry point
 */
int main(void)
{
	/* hardware initialiize, include RDC, IOMUX and UART debug */
	hardware_init();
	PRINTF("\n\r=> Low Power Demo\n\r");

	xTaskCreate(UITask, "UI Task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY+1, NULL);

	/* Start FreeRTOS scheduler. */
	vTaskStartScheduler();

	return 0;
}

void eCSPI3_Handler(void)
{
	uint32_t flags;

	flags = ECSPI_GetStatusFlag(BOARD_ECSPI_BASEADDR, ~0);

	//PRINTF("IRQ, flags %08x\n\r", flags);

	if (flags & ecspiFlagTxfifoTc) {
		PRINTF("Transfer Complete\n\r");
		//ECSPI_ClearStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoTc);
	}
}
