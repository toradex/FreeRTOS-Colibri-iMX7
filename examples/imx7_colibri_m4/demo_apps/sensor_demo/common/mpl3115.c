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
#include "mpl3115.h"

#define MPL3115_MPERCOUNT  0.0000152587890625F      // 1/65536 fixed range for MPL3115
#define MPL3115_CPERCPOUNT 0.00390625F              // 1/256 fixed range for MPL3115

/*FUNCTION****************************************************************
*
* Function Name    : mpl3115_init
* Returned Value   : result
* Comments         : Initialize MPL3115 pressure and temperature sensor.
*
*END*********************************************************************/
bool mpl3115_init(pressure_sensor_t* pThisPressure)
{
    uint8_t txBuffer;
    uint8_t cmdBuffer[2];

    pThisPressure->fmPerCount = MPL3115_MPERCOUNT;
    pThisPressure->fCPerCount = MPL3115_CPERCPOUNT;

    /* Place the MPL3115 in Standby */
    cmdBuffer[0] = MPL3115_ADDRESS << 1;
    cmdBuffer[1] = MPL3115_CTRL_REG1;
    txBuffer = 0x00;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    /* Enable Data Flags in PT_DATA_CFG */
    cmdBuffer[0] = MPL3115_ADDRESS << 1;
    cmdBuffer[1] = MPL3115_PT_DATA_CFG;
    txBuffer = 0x07;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    // write 1011 1001 = 0xB9 to configure MPL3115 and enter Active mode
    // [7]: ALT=1 for altitude measurements
    // [6]: RAW=0 to disable raw measurements
    // [5-3]: OS=111 for OS ratio=128 for maximum internal averaging with 512ms output interval
    // [2]: RST=0 do not enter reset
    // [1]: OST=0 do not initiate a reading
    // [0]: SBYB=1 to enter active mode
    cmdBuffer[0] = MPL3115_ADDRESS << 1;
    cmdBuffer[1] = MPL3115_CTRL_REG1;
    txBuffer = 0xB9;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    return true;
}

/*FUNCTION****************************************************************
*
* Function Name    : mpl3115_read_data
* Returned Value   : result
* Comments         : Get current height and temperature from mpl3115.
*
*END*********************************************************************/
bool mpl3115_read_data(pressure_sensor_t* pThisPressure)
{
    uint8_t rxBuffer[5];
    uint8_t cmdBuffer[3];

    cmdBuffer[0] = MPL3115_ADDRESS << 1;
    cmdBuffer[1] = MPL3115_OUT_P_MSB;
    cmdBuffer[2] = (MPL3115_ADDRESS << 1) + 1;
    if (!I2C_XFER_ReceiveDataBlocking(cmdBuffer, 3, rxBuffer, 5))
        return false;

    // place the read buffer into the 32 bit altitude and 16 bit temperature
    pThisPressure->iHpFast = (rxBuffer[0] << 24) | (rxBuffer[1] << 16) | (rxBuffer[2] << 8);
    pThisPressure->iTpFast = (rxBuffer[3] << 8) | rxBuffer[4];

    return true;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
