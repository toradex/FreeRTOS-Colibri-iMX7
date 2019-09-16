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
 
#ifndef __FXOS8700_H__
#define __FXOS8700_H__

#include <stdint.h>
#include <stdbool.h>

/* I2C Slave Address define */
#define FXOS8700_ADDRESS_0            (0x1C)
#define FXOS8700_ADDRESS_1            (0x1D)
#define FXOS8700_ADDRESS_2            (0x1E)
#define FXOS8700_ADDRESS_3            (0x1F)
#define FXOS8700_ADDRESS_DEFAULT      (FXOS8700_ADDRESS_0)

/* FXOS8700 device ID number */
#define FXOS8700_DEVICE_ID            (0xC7)

/* FXOS8700 Registers address definition */
#define FXOS8700_OUT_X_MSB            (0x01)
#define FXOS8700_WHO_AM_I             (0x0D)
#define FXOS8700_XYZ_DATA_CFG         (0x0E)
#define FXOS8700_CTRL_REG1            (0x2A)
#define FXOS8700_CTRL_REG2            (0x2B)
#define FXOS8700_M_CTRL_REG1          (0x5B)
#define FXOS8700_M_CTRL_REG2          (0x5C)

/* Function prototypes */
#if defined(__cplusplus)
extern "C" {
#endif

bool fxos8700_init(void);
bool fxos8700_read_data(int16_t *, int16_t *, int16_t *, int16_t *, int16_t *, int16_t *);

#ifdef __cplusplus
}
#endif


#endif/* __FXOS8700_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
