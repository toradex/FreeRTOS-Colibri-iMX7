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

#include <limits.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "debug_console_imx.h"
#include "ecspi.h"
#include "gpio_pins.h"
#include "gpio_imx.h"
#include "ugui/ugui.h"
#include "lpm_mcore.h"
#include "mu_imx.h"

TaskHandle_t xLcdTaskHandle;

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
	ECSPI_ClearStatusFlag(BOARD_ECSPI_BASEADDR, ~0);

	/*
	 * IRQ priority must be lower than configMAX_SYSCALL_INTERRUPT_PRIORITY
	 * (8) to allow FreeRTOS syscalls.
	 */
	NVIC_SetPriority(BOARD_ECSPI_IRQ_NUM, 3);
	NVIC_EnableIRQ(BOARD_ECSPI_IRQ_NUM);
}

static void LCD_SendBytes(const uint8_t *buf, int count, enum lcd_cmd_type cmd)
{
	int bytes;
	uint32_t data;
	gpio_pin_action_t a0;
	uint32_t ulNotifiedValue = 0;

	// For simplicity, we only allow up to 256 bytes in one transfer. This
	// fits into the FIFO and is below the maximum burst size of 512 bytes.
	// It simplifies the code here and is good enouth for the ST7565R
	// controller, where we have to send a "Page Address Set Command" to
	// select the next page after 128 * 8 bit => 128 bytes of data
	// anyway.
	if (count > 64 * 4) {
		PRINTF("%s: Maximum 256 bytes!\n\r", __func__);
		return;
	}

	if (cmd == LCD_COMMAND)
		a0 = gpioPinClear;
	else
		a0 = gpioPinSet;
	GPIO_WritePinOutput(gpioLcdA0.base, gpioLcdA0.pin, a0);

	ECSPI_SetBurstLength(BOARD_ECSPI_BASEADDR, BURST_LENGTH_IN_BYTES(count));

	while (count > 0) {
		int i;
		bytes = count & 0x3;
		bytes = bytes ? bytes : 4;
		
		data = 0;
		for(i = 0; i < bytes; i++)
			data = (data << 8) | *(buf)++;

		ECSPI_SendData(BOARD_ECSPI_BASEADDR, data);
		count -= bytes;
		if (ECSPI_GetStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoFull)) {
			PRINTF("%s: FIFO Full!?\n\r", __func__);
			break;
		}
	}

	ECSPI_SetIntCmd(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoTc, true);

	ECSPI_StartBurst(BOARD_ECSPI_BASEADDR);

	ulNotifiedValue = ulTaskNotifyTake(pdFALSE, 10);

	if (ulNotifiedValue < 1) {
		PRINTF("%s: Transfer timeout\n\r", __func__);
	}
}

#define CLAMP(x, low, high) { if ( (x) < (low) ) x = (low); if ( (x) > (high) ) x = (high); }
#define LCDWIDTH 128
#define LCDHEIGHT 64
#define LCDPAGES  (LCDHEIGHT+7)/8

static void LCD_SetXY(int x, int y)
{
	unsigned char cmd[3];

	//PRINTF("LCD_SetXY(%d,%d)\n\r", x, y);
	CLAMP(x, 0, LCDWIDTH-1);
	CLAMP(y, 0, LCDPAGES-1);

	cmd[0] = 0xB0 | (y&0xF);
	cmd[1] = 0x10 | (x&0xF);
	cmd[2] = (x>>4)&0xF;
	LCD_SendBytes(cmd, 3, LCD_COMMAND);
}

uint8_t fb[LCDWIDTH * LCDPAGES];
static void LCD_SendFB(uint8_t *fb)
{
	for (int y = 0; y < LCDPAGES; y++) {
		LCD_SetXY(0,y);
		LCD_SendBytes(fb + y * LCDWIDTH, LCDWIDTH, LCD_DATA);
	}
}

void LCD_SetPixel(UG_S16 x, UG_S16 y, UG_COLOR c)
{
	int page = y >> 3;
	uint8_t bit = 1 << (y & 0x7);
//PRINTF("LCD_SetPixel, x %d, y %d, page %d, bit %x\n\r", x, y, page, bit);

	if (c == C_WHITE)
		fb[128 * page + x] &= ~bit;
	else
		fb[128 * page + x] |= bit;
}

void mu_handle_message(uint32_t msg)
{
	static bool firstready = true;
	switch(msg) {
	case MU_LPM_M4_LPM_READY:
		if (firstready)
			LPM_MCORE_SendMessage(MU_LPM_M4_RELEASE_HIGH_BUS);
		firstready = false;

		PRINTF("\n\rA7 ready\n\r");
		break;
	case MU_LPM_M4_LPM_SLEEP:
		PRINTF("\n\rA7 asleep\n\r");
		break;
	default:
		PRINTF("MSG %08x, \n\r", msg);
		break;
	}
}

void MU_M4_Handler(void)
{
	uint32_t msg;

	if (MU_TryReceiveMsg(MUB, 0, &msg) == kStatus_MU_Success)
		mu_handle_message(msg);


	return;
}

UG_GUI gui;

void LCD_Task(void *pvParameters)
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
	PRINTF("Before init\n\r");
	LCD_Init();

	LCD_SendBytes(LCD_init_seq, sizeof(LCD_init_seq), LCD_COMMAND);
	PRINTF("After init\n\r");
/*
	uint8_t cmd = 0xA4 | 1;
	LCD_SendBytes(&cmd, 1, LCD_COMMAND);
*/
	UG_Init(&gui, LCD_SetPixel, LCDWIDTH, LCDHEIGHT);
	UG_SetBackcolor(C_WHITE);
	UG_SetForecolor(C_BLACK);

	UG_FontSetHSpace(0);
	UG_FontSelect(&FONT_8X8);
	UG_DrawLine(0, 0, 0, 8, C_BLACK);
	UG_DrawLine(0, 8, 8, 8, C_BLACK);
	UG_DrawLine(8, 0, 0, 8, C_BLACK);
	UG_PutString(10, 0, "Inertial");
	UG_FontSelect(&FONT_6X8);
	UG_PutString(0, 14, "Accel.");
	UG_PutString(0, 24, "X: 0.0g");
	UG_PutString(0, 34, "Y: 0.0g");
	UG_PutString(0, 44, "Z: 0.0g");

	UG_PutString(64, 14, "Gyroscope");
	UG_PutString(64, 24, "X: 4.5\xb0/s");
	UG_PutString(64, 34, "Y: 7.0\xb0/s");
	UG_PutString(64, 44, "Z: 2.4\xb0/s");

	UG_FontSelect(&FONT_4X6);
	UG_PutString(0, 54, "0123456789 Hello World");
	UG_DrawCircle(60, 30, 20, C_BLACK);
/*
	//UG_FontSelect(&FONT_5X8);
	UG_FontSelect(&FONT_4X6);
	UG_PutString(0, 16, "0123456789");

*/

	LCD_SendFB(fb);

	int x = 0;
	char control_char;
	while (true) {
		LCD_SetPixel(x, 7, C_BLACK);
		LCD_SetPixel(x, 8, C_BLACK);

		x++;
		LCD_SendFB(fb);
		//vTaskDelay(100);
		if (x >= LCDWIDTH)
			x = 0;

		PRINTF("\r\nCPU delaying for 5s: ");
		vTaskDelay(5000);
		PRINTF("done");

		PRINTF("\r\nCPU spinning, press any character (s sends msg): ");
		control_char = GETCHAR();
		PRINTF("%c", control_char);
		if ((control_char == 's') || (control_char == 'S')) {
			PRINTF("\r\nSending msg to A7...");
			LPM_MCORE_SendMessage(MSG_LPM_M4_WAIT);
		}
	}
/*
	uint8_t page[128];
	while (true) {
		data = ~data;
		for (int x = 0; x < 128; x++)
			page[x] = data;
		for (int y = 0; y < 8; y++) {
			LCD_SetXY(0,y);
			LCD_SendBytes(page, 128, LCD_DATA);
			vTaskDelay(100);
		}
	}

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
 * @brief Custom function to be run in idletask
 */
void vApplicationIdleHook(void)
{
	/* Waiting for Wake up event. */
	LPM_MCORE_WaitForInt();
}

/*!
 * @brief Main entry point
 */
int main(void)
{
	/* hardware initialiize, include RDC, IOMUX and UART debug */
	hardware_init();
	LPM_MCORE_ChangeM4Clock(LPM_M4_LOW_FREQ);
	PRINTF("\n\r=> Low Power Demo\n\r");

	NVIC_SetPriority(MU_M4_IRQn, 3);
	NVIC_EnableIRQ(MU_M4_IRQn);
	MU_EnableRxFullInt(MUB, 0);

	xTaskCreate(LCD_Task, "LCD Task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY+1, &xLcdTaskHandle);

	/* Start FreeRTOS scheduler. */
	vTaskStartScheduler();

	return 0;
}

void eCSPI3_Handler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	uint32_t flags;

	flags = ECSPI_GetStatusFlag(BOARD_ECSPI_BASEADDR, ~0);

	//PRINTF("IRQ, flags %08x\n\r", flags);

	if (flags & ecspiFlagTxfifoTc) {
		ECSPI_ClearStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoTc);
		vTaskNotifyGiveFromISR(xLcdTaskHandle, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}
