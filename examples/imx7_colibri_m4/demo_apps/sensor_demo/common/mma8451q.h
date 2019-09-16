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

#ifndef __MMA8451Q_H__
#define __MMA8451Q_H__

#include <stdint.h>
#include <stdbool.h>

/* I2C Slave Address define */
#define MMA8451Q_ADDRESS_0         (0x1C)       /* SA0 = 0, low logic */
#define MMA8451Q_ADDRESS_1         (0x1D)       /* SA0 = 1, high logic */

/* MMA8451Q device ID number */
#define MMA8451Q_DEVICE_ID         (0x1A)

/* MMA8451Q Registers address definition */
#define MMA8451Q_STATUS            (0x00)
#define MMA8451Q_OUT_X_MSB         (0x01)
#define MMA8451Q_OUT_X_LSB         (0x02)
#define MMA8451Q_OUT_Y_MSB         (0x03)
#define MMA8451Q_OUT_Y_LSB         (0x04)
#define MMA8451Q_OUT_Z_MSB         (0x05)
#define MMA8451Q_OUT_Z_LSB         (0x06)
#define MMA8451Q_F_SETUP           (0x09)
#define MMA8451Q_TRIG_CFG          (0x0A)
#define MMA8451Q_SYSMOD            (0x0B)
#define MMA8451Q_INT_SOURCE        (0x0C)
#define MMA8451Q_WHO_AM_I          (0x0D)
#define MMA8451Q_XYZ_DATA_CFG      (0x0E)
#define MMA8451Q_HP_FILTER_CUTOFF  (0x0F)
#define MMA8451Q_PL_STATUS         (0x10)
#define MMA8451Q_PL_CFG            (0x11)
#define MMA8451Q_PL_COUNT          (0x12)
#define MMA8451Q_PL_BF_ZCOMP       (0x13)
#define MMA8451Q_PL_THS_REG        (0x14)
#define MMA8451Q_FF_MT_CFG         (0x15)
#define MMA8451Q_FF_MT_SRC         (0x16)
#define MMA8451Q_FF_MT_THS         (0x17)
#define MMA8451Q_FF_MT_COUNT       (0x18)
#define MMA8451Q_TRANSIENT_CFG     (0x1D)
#define MMA8451Q_TRANSIENT_SRC     (0x1E)
#define MMA8451Q_TRANSIENT_THS     (0x1F)
#define MMA8451Q_TRANSIENT_COUNT   (0x20)
#define MMA8451Q_PULSE_CFG         (0x21)
#define MMA8451Q_PULSE_SRC         (0x22)
#define MMA8451Q_PULSE_THSX        (0x23)
#define MMA8451Q_PULSE_THSY        (0x24)
#define MMA8451Q_PULSE_THSZ        (0x25)
#define MMA8451Q_PULSE_TMLT        (0x26)
#define MMA8451Q_PULSE_LTCY        (0x27)
#define MMA8451Q_PULSE_WIND        (0x28)
#define MMA8451Q_ASLP_COUNT        (0x29)
#define MMA8451Q_CTRL_REG1         (0x2A)
#define MMA8451Q_CTRL_REG2         (0x2B)
#define MMA8451Q_CTRL_REG3         (0x2C)
#define MMA8451Q_CTRL_REG4         (0x2D)
#define MMA8451Q_CTRL_REG5         (0x2E)
#define MMA8451Q_OFF_X             (0x2F)
#define MMA8451Q_OFF_Y             (0x30)
#define MMA8451Q_OFF_Z             (0x31)

enum _mma8451q_mode
{
    mma8451qMode_2G = 0U,
    mma8451qMode_4G = 1U,
    mma8451qMode_8G = 2U
};

/* Function prototypes */
#if defined(__cplusplus)
extern "C" {
#endif

bool MMA8451Q_Init(void);
bool MMA8451Q_ReadData(int16_t *x, int16_t *y, int16_t *z);
bool MMA8451Q_ChangeMode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __MMA8451Q_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
