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

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "i2c_xfer.h"
#include "fxas21002.h"

/*FUNCTION****************************************************************
*
* Function Name    : fxas21002_init
* Returned Value   : result
* Comments         : Initialize FXAS21002 Gyro sensor.
*
*END*********************************************************************/
bool fxas21002_init(gyro_sensor_t* pThisGyro)
{
    uint8_t txBuffer;
    uint8_t cmdBuffer[2];

    pThisGyro->fDegPerSecPerCount = FXAS21002_DEGPERSECPERCOUNT;

    // Write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS21002 in Standby
    // [7]: ZR_cond=0
    // [6]: RST=0
    // [5]: ST=0 self test disabled
    // [4-2]: DR[2-0]=000 for 800Hz
    // [1-0]: Active=0, Ready=0 for Standby mode
    cmdBuffer[0] = BOARD_I2C_FXAS21002_ADDR << 1;
    cmdBuffer[1] = FXAS21002_CTRL_REG1;
    txBuffer = 0x00;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    // write 0000 0000 = 0x00 to CTRL_REG0 to configure range and filters
    // [7-6]: BW[1-0]=00, LPF disabled
    // [5]: SPIW=0 4 wire SPI (irrelevant)
    // [4-3]: SEL[1-0]=00 for 10Hz HPF at 200Hz ODR
    // [2]: HPF_EN=0 disable HPF
    // [1-0]: FS[1-0]=00 for 1600dps (TBD CHANGE TO 2000dps when final trimmed parts available)
    cmdBuffer[0] = BOARD_I2C_FXAS21002_ADDR << 1;
    cmdBuffer[1] = FXAS21002_CTRL_REG0;
    txBuffer = 0x00;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    // write 0000 0010 = 0x02 to CTRL_REG1 to configure 800Hz ODR and enter Active mode
    // [7]: ZR_cond=0
    // [6]: RST=0
    // [5]: ST=0 self test disabled
    // [4-2]: DR[2-0]=000 for 800Hz ODR
    // [1-0]: Active=1, Ready=0 for Active mode
    cmdBuffer[0] = BOARD_I2C_FXAS21002_ADDR << 1;
    cmdBuffer[1] = FXAS21002_CTRL_REG1;
    txBuffer = 0x02;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    return true;
}

/*FUNCTION****************************************************************
*
* Function Name    : fxas21002_read_data
* Returned Value   : result
* Comments         : Get current height and temperature from fxas21002.
*
*END*********************************************************************/
bool fxas21002_read_data(gyro_sensor_t* pThisGyro)
{
    uint8_t rxBuffer[6];
    uint8_t cmdBuffer[3];

    // store the gain terms in the GyroSensor structure
    cmdBuffer[0] = BOARD_I2C_FXAS21002_ADDR << 1;
    cmdBuffer[1] = FXAS21002_OUT_X_MSB;
    cmdBuffer[2] = (BOARD_I2C_FXAS21002_ADDR << 1) + 1;
    if (!I2C_XFER_ReceiveDataBlocking(cmdBuffer, 3, rxBuffer, 6))
        return false;

    pThisGyro->iYpFast[0] = (rxBuffer[0] << 8) | rxBuffer[1];
    pThisGyro->iYpFast[1] = (rxBuffer[2] << 8) | rxBuffer[3];
    pThisGyro->iYpFast[2] = (rxBuffer[4] << 8) | rxBuffer[5];

    return true;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
