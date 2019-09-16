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
#include "fxos8700.h"

/*FUNCTION****************************************************************
*
* Function Name    : fxos8700_init
* Returned Value   : result
* Comments         : Initialize FXOS8700 Acc and Mag sensor.
*
*END*********************************************************************/
bool fxos8700_init(void)
{
    uint8_t txBuffer;
    uint8_t cmdBuffer[2];

    // write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS8700 into standby
    // [7-1] = 0000 000
    // [0]: active=0
    cmdBuffer[0] = BOARD_I2C_FXOS8700_ADDR << 1;
    cmdBuffer[1] = FXOS8700_CTRL_REG1;
    txBuffer = 0x00;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    // write 0001 1111 = 0x1F to M_CTRL_REG1
    // [7]: m_acal=0: auto calibration disabled
    // [6]: m_rst=0: one-shot magnetic reset disabled
    // [5]: m_ost=0: one-shot magnetic measurement disabled
    // [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
    // [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
    cmdBuffer[0] = BOARD_I2C_FXOS8700_ADDR << 1;
    cmdBuffer[1] = FXOS8700_M_CTRL_REG1;
    txBuffer = 0x1F;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    // write 0010 0000 = 0x20 to magnetometer control register 2
    // [7]: reserved
    // [6]: reserved
    // [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the accelerometer registers
    // [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
    // [3]: m_maxmin_dis_ths=0
    // [2]: m_maxmin_rst=0
    // [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
    cmdBuffer[0] = BOARD_I2C_FXOS8700_ADDR << 1;
    cmdBuffer[1] = FXOS8700_M_CTRL_REG2;
    txBuffer = 0x20;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    // write 0000 0001= 0x01 to XYZ_DATA_CFG register
    // [7]: reserved
    // [6]: reserved
    // [5]: reserved
    // [4]: hpf_out=0
    // [3]: reserved
    // [2]: reserved
    // [1-0]: fs=01 for 4g mode: 2048 counts / g = 8192 counts / g after 2 bit left shift
    cmdBuffer[0] = BOARD_I2C_FXOS8700_ADDR << 1;
    cmdBuffer[1] = FXOS8700_XYZ_DATA_CFG;
    txBuffer = 0x01;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    // write 0000 0010 = 0x02 to CTRL_REG2 to set MODS bits
    // [7]: st=0: self test disabled
    // [6]: rst=0: reset disabled
    // [5]: unused
    // [4-3]: smods=00
    // [2]: slpe=0: auto sleep disabled
    // [1-0]: mods=10 for high resolution (maximum over sampling)
    cmdBuffer[0] = BOARD_I2C_FXOS8700_ADDR << 1;
    cmdBuffer[1] = FXOS8700_CTRL_REG2;
    txBuffer = 0x02;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    // write 0000 1101 = 0x0D to accelerometer control register 1
    // [7-6]: aslp_rate=00
    // [5-3]: dr=001=1 for 200Hz data rate (when in hybrid mode)
    // [2]: lnoise=1 for low noise mode (since we're in 4g mode)
    // [1]: f_read=0 for normal 16 bit reads
    // [0]: active=1 to take the part out of standby and enable sampling
    cmdBuffer[0] = BOARD_I2C_FXOS8700_ADDR << 1;
    cmdBuffer[1] = FXOS8700_CTRL_REG1;
    txBuffer = 0x0D;
    if (!I2C_XFER_SendDataBlocking(cmdBuffer, 2, &txBuffer, 1))
        return false;

    return true;
}

/*FUNCTION****************************************************************
*
* Function Name    : fxos8700_read_data
* Returned Value   : result
* Comments         : Get current Acc and Mag from FXOS8700 6-axis sensor.
*
*END*********************************************************************/
bool fxos8700_read_data(int16_t *Ax, int16_t *Ay, int16_t *Az,
                        int16_t *Mx, int16_t *My, int16_t *Mz)
{
    uint8_t rxBuffer[12];
    uint8_t cmdBuffer[3];

    // Fetch Current Acc and Mag in all Axis
    cmdBuffer[0] = BOARD_I2C_FXOS8700_ADDR << 1;
    cmdBuffer[1] = FXOS8700_OUT_X_MSB;
    cmdBuffer[2] = (BOARD_I2C_FXOS8700_ADDR << 1) + 1;
    if (!I2C_XFER_ReceiveDataBlocking(cmdBuffer, 3, rxBuffer, 12))
        return false;

    *Ax = (rxBuffer[2] << 8) | rxBuffer[3];
    *Ay = (rxBuffer[0] << 8) | rxBuffer[1];
    *Az = (rxBuffer[4] << 8) | rxBuffer[5];
    *Mx = (rxBuffer[8] << 8) | rxBuffer[9];
    *My = (rxBuffer[6] << 8) | rxBuffer[7];
    *Mz = (rxBuffer[10] << 8) | rxBuffer[11];

    return true;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
