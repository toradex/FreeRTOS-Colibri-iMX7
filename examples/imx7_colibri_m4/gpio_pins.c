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

#include <assert.h>
#include "gpio_pins.h"
#include "board.h"
#include "gpio_imx.h"

gpio_config_t gpioLed = {
    "EXT_IO0 LED",                      /* name */
    &IOMUXC_LPSR_SW_MUX_CTL_PAD_GPIO1_IO02,  /* muxReg */
    0,                                  /* muxConfig */
    &IOMUXC_LPSR_SW_PAD_CTL_PAD_GPIO1_IO02,  /* padReg */
    0,                                  /* padConfig */
    GPIO1,                              /* base */
    2                                   /* pin */
};

gpio_config_t gpioKeyFunc1 = {
    "EXT_IO1",                                      /* name */
    &IOMUXC_SW_MUX_CTL_PAD_EPDC_GDRL,               /* muxReg */
    5,                                              /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL,               /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL_PS(2) |        /* padConfig */
        IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL_PE_MASK |
	IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL_HYS_MASK,
    GPIO2,                                          /* base */
    26                                              /* pin */
};

gpio_config_t gpioKeyFunc2 = {
    "EXT_IO2",                                       /* name */
    &IOMUXC_SW_MUX_CTL_PAD_EPDC_SDCE2,               /* muxReg */
    5,                                              /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_EPDC_SDCE2,                  /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_EPDC_SDCE2_PS(2) |        /* padConfig */
        IOMUXC_SW_PAD_CTL_PAD_EPDC_SDCE2_PE_MASK |
	IOMUXC_SW_PAD_CTL_PAD_EPDC_SDCE2_HYS_MASK,
    GPIO2,                                          /* base */
    22                                              /* pin */
};


/* Switch 1
 * SODIMM                BALL NAME                GPIO                ALT
 *    133                EPDC_GDRL           GPIO02_26                  5
 */
gpio_config_t gpioSwitch1 = {
    "SODIMM 133",                                      /* name */
    &IOMUXC_SW_MUX_CTL_PAD_EPDC_GDRL,               /* muxReg */
    5,                                              /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL,               /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL_PS(2)  |        /* padConfig */
    	IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL_PE_MASK |
    IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL_HYS_MASK,
    GPIO2,                                          /* base */
    26                                              /* pin */
};
gpio_init_config_t Switch1 = {
    .pin           = 26, //pin number
    .direction     = gpioDigitalInput,
    .interruptMode = gpioNoIntmode
};

/* LED 1
 * SODIMM                BALL NAME                GPIO                ALT
 *    127               EPDC_SDCE2           GPIO02_22                  5
 */
gpio_config_t gpioLed1 = {
    "SODIMM 127",                                       /* name */
    &IOMUXC_SW_MUX_CTL_PAD_EPDC_SDCE2,               /* muxReg */
    5,                                              /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_EPDC_SDCE2,                  /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_EPDC_SDCE2_PS(2) |        /* padConfig */
        IOMUXC_SW_PAD_CTL_PAD_EPDC_SDCE2_PE_MASK |
	IOMUXC_SW_PAD_CTL_PAD_EPDC_SDCE2_HYS_MASK,
    GPIO2,                                          /* base */
    22                                              /* pin */
};
gpio_init_config_t Led1 = {
    .pin           = 22, //pin number
    .direction     = gpioDigitalOutput,
    .interruptMode = gpioNoIntmode
};


/* Switch 2
 * SODIMM                BALL NAME                GPIO                ALT
 *    107                 EPDC_DATA15           GPIO02_15                  5
 */
gpio_config_t gpioSwitch2 = {
    "SODIMM 107",                                      /* name */
    &IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA15,               /* muxReg */
    5,                                              /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_EPDC_DATA15,               /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_EPDC_DATA15_PS(2)  |        /* padConfig */
    	IOMUXC_SW_PAD_CTL_PAD_EPDC_DATA15_PE_MASK |
    IOMUXC_SW_PAD_CTL_PAD_EPDC_DATA15_HYS_MASK,
    GPIO2,                                          /* base */
    15                                              /* pin */
};
gpio_init_config_t Switch2 = {
    .pin           = 15, //pin number
    .direction     = gpioDigitalInput,
    .interruptMode = gpioNoIntmode
};

/* LED 2
 * SODIMM                BALL NAME                GPIO                ALT
 *    105                 EPDC_DATA10           GPIO02_10                  5
 */
gpio_config_t gpioLed2 = {
    "SODIMM 105",                                       /* name */
    &IOMUXC_SW_MUX_CTL_PAD_EPDC_DATA10,               /* muxReg */
    5,                                              /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_EPDC_DATA10,                  /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_EPDC_DATA10_PS(2) |        /* padConfig */
        IOMUXC_SW_PAD_CTL_PAD_EPDC_DATA10_PE_MASK |
	IOMUXC_SW_PAD_CTL_PAD_EPDC_DATA10_HYS_MASK,
    GPIO2,                                          /* base */
    10                                              /* pin */
};

gpio_init_config_t Led2 = {
    .pin           = 10, //pin number
    .direction     = gpioDigitalOutput,
    .interruptMode = gpioNoIntmode
};

void configure_gpio_pin(gpio_config_t *config)
{
    assert(config);

    *(config->muxReg) = config->muxConfig;
    *(config->padReg) = config->padConfig;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
