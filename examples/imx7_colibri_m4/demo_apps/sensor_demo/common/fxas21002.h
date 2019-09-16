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

#ifndef __FXAS21002_H__
#define __FXAS21002_H__

#define OVERSAMPLE_RATIO                (64)         // int32: 8x: 3DOF, 6DOF, 9DOF run at SENSORFS / OVERSAMPLE_RATIO Hz

// place the gain in the gyro structure
#define FXAS21002_COUNTSPERDEGPERSEC    (16.0F)      // for production 2000dps range (2000dps=32000)
#define FXAS21002_DEGPERSECPERCOUNT     (0.0625F)    // must be reciprocal of FCOUNTSPERDEGPERSEC

/* I2C Slave Address define */
#define FXAS21002_ADDRESS_0             (0x20)
#define FXAS21002_ADDRESS_1             (0x21)
#define FXAS21002_ADDRESS_DEFAULT       (FXAS21002_ADDRESS_0)

/* FXAS21002 device ID number */
#define FXAS21002_DEVICE_ID             (0xD7)

/* FXAS21002 Registers address definition */
#define FXAS21002_STATUS                (0x00)
#define FXAS21002_OUT_X_MSB             (0x01)
#define FXAS21002_OUT_X_LSB             (0x02)
#define FXAS21002_OUT_Y_MSB             (0x03)
#define FXAS21002_OUT_Y_LSB             (0x04)
#define FXAS21002_OUT_Z_MSB             (0x05)
#define FXAS21002_OUT_Z_LSB             (0x06)
#define FXAS21002_DR_STATUS             (0x07)
#define FXAS21002_F_STATUS              (0x08)
#define FXAS21002_F_SETUP               (0x09)
#define FXAS21002_F_EVENT               (0x0A)
#define FXAS21002_INT_SRC_FLAG          (0x0B)
#define FXAS21002_WHO_AM_I              (0x0C)
#define FXAS21002_CTRL_REG0             (0x0D)
#define FXAS21002_RT_CFG                (0x0E)
#define FXAS21002_RT_SRC                (0x0F)
#define FXAS21002_RT_THS                (0x10)
#define FXAS21002_RT_COUNT              (0x11)
#define FXAS21002_TEMP                  (0x12)
#define FXAS21002_CTRL_REG1             (0x13)
#define FXAS21002_CTRL_REG2             (0x14)
#define FXAS21002_CTRL_REG3             (0x15)

// gyro sensor structure definition
typedef struct _gyro_sensor
{
    int32_t iSumYpFast[3];                  // sum of fast measurements
    float   fYp[3];                         // raw gyro sensor output (deg/s)
    float   fDegPerSecPerCount;             // initialized to FDEGPERSECPERCOUNT
    int16_t iYpFast[3];                     // fast (typically 200Hz) readings
    int16_t iYp[3];                         // averaged gyro sensor output (counts)
} gyro_sensor_t;

/* Function prototypes */
#if defined(__cplusplus)
extern "C" {
#endif

bool fxas21002_init(gyro_sensor_t*);
bool fxas21002_read_data(gyro_sensor_t*);

#ifdef __cplusplus
}
#endif


#endif /* __FXAS21002_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
