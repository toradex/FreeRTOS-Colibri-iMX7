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

#ifndef __MPL3115_H__
#define __MPL3115_H__

#include <stdint.h>
#include <stdbool.h>

/* I2C Slave Address define */
#define MPL3115_ADDRESS            (0x60)

/* MPL3115 device ID number */
#define MPL3115_DEVICE_ID          (0xC4)

/* MPL3115 Registers address definition */
#define MPL3115_STATUS             (0x00)
#define MPL3115_OUT_P_MSB          (0x01)
#define MPL3115_OUT_P_CSB          (0x02)
#define MPL3115_OUT_P_LSB          (0x03)
#define MPL3115_OUT_T_MSB          (0x04)
#define MPL3115_OUT_T_LSB          (0x05)
#define MPL3115_DR_STATUS          (0x06)
#define MPL3115_OUT_P_DELTA_MSB    (0x07)
#define MPL3115_OUT_P_DELTA_CSB    (0x08)
#define MPL3115_OUT_P_DELTA_LSB    (0x09)
#define MPL3115_OUT_T_DELTA_MSB    (0x0A)
#define MPL3115_OUT_T_DELTA_LSB    (0x0B)
#define MPL3115_WHO_AM_I           (0x0C)
#define MPL3115_F_STATUS           (0x0D)
#define MPL3115_F_DATA             (0x0E)
#define MPL3115_F_SETUP            (0x0F)
#define MPL3115_TIME_DLY           (0x10)
#define MPL3115_SYSMOD             (0x11)
#define MPL3115_INT_SOURCE         (0x12)
#define MPL3115_PT_DATA_CFG        (0x13)
#define MPL3115_BAR_IN_MSB         (0x14)
#define MPL3115_BAR_IN_LSB         (0x15)
#define MPL3115_P_TGT_MSB          (0x16)
#define MPL3115_P_TGT_LSB          (0x17)
#define MPL3115_T_TGT              (0x18)
#define MPL3115_P_WND_MSB          (0x19)
#define MPL3115_P_WND_LSB          (0x1A)
#define MPL3115_T_WND              (0x1B)
#define MPL3115_P_MIN_MSB          (0x1C)
#define MPL3115_P_MIN_CSB          (0x1D)
#define MPL3115_P_MIN_LSB          (0x1E)
#define MPL3115_T_MIN_MSB          (0x1F)
#define MPL3115_T_MIN_LSB          (0x20)
#define MPL3115_P_MAX_MSB          (0x21)
#define MPL3115_P_MAX_CSB          (0x22)
#define MPL3115_P_MAX_LSB          (0x23)
#define MPL3115_T_MAX_MSB          (0x24)
#define MPL3115_T_MAX_LSB          (0x25)
#define MPL3115_CTRL_REG1          (0x26)
#define MPL3115_CTRL_REG2          (0x27)
#define MPL3115_CTRL_REG3          (0x28)
#define MPL3115_CTRL_REG4          (0x29)
#define MPL3115_CTRL_REG5          (0x2A)
#define MPL3115_OFF_P              (0x2B)
#define MPL3115_OFF_T              (0x2C)
#define MPL3115_OFF_H              (0x2D)

typedef struct _pressure_sensor
{
    int32_t iHp;                    // slow (typically 25Hz) height (counts)
    int32_t iHpFast;                // fast (typically 200Hz) height (counts)
    int16_t iTp;                    // slow (typically 25Hz) temperature (count)
    int16_t iTpFast;                // fast (typically 200Hz) temperature (counts)
    float fHp;                      // slow (typically 25Hz) height (m)
    float fTp;                      // slow (typically 25Hz) temperature (C)
    float fmPerCount;               // initialized to FMPERCOUNT
    float fCPerCount;               // initialized to FCPERCPOUNT
} pressure_sensor_t;

/* Function prototypes */
#if defined(__cplusplus)
extern "C" {
#endif

bool mpl3115_init(pressure_sensor_t*);
bool mpl3115_read_data(pressure_sensor_t*);

#ifdef __cplusplus
}
#endif

#endif /* __MPL3115_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
