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
#include "debug_console_imx.h"
#include "i2c_xfer.h"
#include "mma8451q.h"

static uint8_t mma8451qMode;

/*FUNCTION****************************************************************
*
* Function Name    : MMA8451Q_Init
* Returned Value   : true or false
* Comments         : Initialize MMA8451Q 3-axis accelerometer sensor.
*
*END*********************************************************************/
bool MMA8451Q_Init(void)
{
    uint8_t txBuffer;
    uint8_t rxBuffer;
    uint8_t cmdBuffer[3];

    /* Place the MMA8451Q in Standby */
    PRINTF("Place the MMA8451Q in standby mode... ");
    cmdBuffer[0] = BOARD_I2C_MMA8451Q_ADDR << 1;
    cmdBuffer[1] = MMA8451Q_CTRL_REG1;
    txBuffer = 0x00;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
    {
        PRINTF("ERROR\n\r");
        return false;
    }
    PRINTF("OK\n\r");

    // write 0000 0000= 0x00 to MMA8451Q_XYZ_DATA_CFG register
    // [7]: reserved
    // [6]: reserved
    // [5]: reserved
    // [4]: hpf_out=0
    // [3]: reserved
    // [2]: reserved
    // [1-0]: fs=00 for 2g mode.
    PRINTF("Set the mode: 2G ... ");
    cmdBuffer[0] = BOARD_I2C_MMA8451Q_ADDR << 1;
    cmdBuffer[1] = MMA8451Q_XYZ_DATA_CFG;
    txBuffer = 0x00;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
    {
        PRINTF("ERROR\n\r");
        return false;
    }
    mma8451qMode = 0U;
    PRINTF("OK\n\r");

    // write 0000 0001 = 0x01 to MMA8451Q_CTRL_REG1
    // [7-6]: aslp_rate=00
    // [5-3]: dr=000
    // [2]: lnoise=0
    // [1]: f_read=0 for normal read mode
    // [0]: active=1 to take the part out of standby and enable sampling
    PRINTF("Fast read clear and active mode ... ");
    cmdBuffer[0] = BOARD_I2C_MMA8451Q_ADDR << 1;
    cmdBuffer[1] = MMA8451Q_CTRL_REG1;
    txBuffer = 0x01;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
    {
        PRINTF("ERROR\n\r");
        return false;
    }
    PRINTF("OK\n\r");

    // read WHO_AM_I device register, 0x1A
    PRINTF("Test WHO_AM_I check... ");
    cmdBuffer[0] = BOARD_I2C_MMA8451Q_ADDR << 1;
    cmdBuffer[1] = MMA8451Q_WHO_AM_I;
    cmdBuffer[2] = (BOARD_I2C_MMA8451Q_ADDR << 1) + 1;
    if (!I2C_XFER_ReceiveDataBlocking(cmdBuffer, 3, &rxBuffer, 1))
    {
        PRINTF("ERROR\n\r");
        return false;
    }
    if(rxBuffer == MMA8451Q_DEVICE_ID)
        PRINTF("OK\n\r");
    else
    {
        PRINTF("ERROR\n\r");
        return false;
    }
    return true;
}

/*FUNCTION****************************************************************
*
* Function Name    : MMA8451Q_ReadData
* Returned Value   : true or false
* Comments         : Get current acceleration from MMA8451Q.
*
*END*********************************************************************/
bool MMA8451Q_ReadData(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t rxBuffer[7];
    uint8_t cmdBuffer[3];

    cmdBuffer[0] = BOARD_I2C_MMA8451Q_ADDR << 1;
    cmdBuffer[1] = MMA8451Q_OUT_X_MSB;
    cmdBuffer[2] = (BOARD_I2C_MMA8451Q_ADDR << 1) + 1;
    if (!I2C_XFER_ReceiveDataBlocking(cmdBuffer, 3, rxBuffer, 7))
        return false;

    *x = ((rxBuffer[0] << 8) & 0xff00) | rxBuffer[1];
    *y = ((rxBuffer[2] << 8) & 0xff00) | rxBuffer[3];
    *z = ((rxBuffer[4] << 8) & 0xff00) | rxBuffer[5];
    *x = (int16_t)(*x) >> 2;
    *y = (int16_t)(*y) >> 2;
    *z = (int16_t)(*z) >> 2;

    if(mma8451qMode == mma8451qMode_4G)
    {
        (*x) = (*x) << 1;
        (*y) = (*y) << 1;
        (*z) = (*z) << 1;
    }
    else if(mma8451qMode == mma8451qMode_8G)
    {
        (*x) = (*x) << 2;
        (*y) = (*y) << 2;
        (*z) = (*z) << 2;
    }

    return true;
}

 /*FUNCTION****************************************************************
*
* Function Name    : MMA8451Q_ChangeMode
* Returned Value   : true or false
* Comments         : Change the current mode.
*
*END*********************************************************************/
bool MMA8451Q_ChangeMode(uint8_t mode)
{
    uint8_t txBuffer;
    uint8_t cmdBuffer[2];

    cmdBuffer[0] = BOARD_I2C_MMA8451Q_ADDR << 1;
    cmdBuffer[1] = MMA8451Q_XYZ_DATA_CFG;
    txBuffer = mode;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;
    return true;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
