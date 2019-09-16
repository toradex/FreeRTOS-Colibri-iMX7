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

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "debug_console_imx.h"
#include "i2c_xfer.h"
#include "fxas21002.h"
#include "fxos8700.h"
#include "mpl3115.h"

////////////////////////////////////////////////////////////////////////////////
// Definition
////////////////////////////////////////////////////////////////////////////////
#define HEIGHT_UPDATE_THRESHOLD (0.5f)
#define TEMP_UPDATE_THRESHOLD   (0.3f)
#define GYRO_UPDATE_THRESHOLD   (5.0f)

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
void fxas21002Demo(void)
{
    gyro_sensor_t thisGyroSensor;
    uint32_t count = 0, printCount = 0;

    PRINTF("\n\r-------------- FXAS21002 Gyro data acquisition --------------\n\r\n\r");

    PRINTF("FXAS21002 initialization ... ");
    if (fxas21002_init(&thisGyroSensor))
        PRINTF("OK\n\r");
    else
        PRINTF("ERROR\n\r");

    /* delay 10ms to wait sensor init finish */
    vTaskDelay(10);

    while(1)
    {
        fxas21002_read_data(&thisGyroSensor);
        thisGyroSensor.iSumYpFast[0] += thisGyroSensor.iYpFast[0];
        thisGyroSensor.iSumYpFast[1] += thisGyroSensor.iYpFast[1];
        thisGyroSensor.iSumYpFast[2] += thisGyroSensor.iYpFast[2];

        count++;
        if (count == OVERSAMPLE_RATIO)
        {
            count = 0;
            thisGyroSensor.iYp[0] = thisGyroSensor.iSumYpFast[0] / OVERSAMPLE_RATIO;
            thisGyroSensor.iYp[1] = thisGyroSensor.iSumYpFast[1] / OVERSAMPLE_RATIO;
            thisGyroSensor.iYp[2] = thisGyroSensor.iSumYpFast[2] / OVERSAMPLE_RATIO;
            thisGyroSensor.fYp[0] = thisGyroSensor.iYp[0] * thisGyroSensor.fDegPerSecPerCount;
            thisGyroSensor.fYp[1] = thisGyroSensor.iYp[1] * thisGyroSensor.fDegPerSecPerCount;
            thisGyroSensor.fYp[2] = thisGyroSensor.iYp[2] * thisGyroSensor.fDegPerSecPerCount;
            thisGyroSensor.iSumYpFast[0] = 0;
            thisGyroSensor.iSumYpFast[1] = 0;
            thisGyroSensor.iSumYpFast[2] = 0;
        }

        if ((thisGyroSensor.fYp[0] > GYRO_UPDATE_THRESHOLD) ||
            (thisGyroSensor.fYp[0] < -GYRO_UPDATE_THRESHOLD) ||
            (thisGyroSensor.fYp[1] > GYRO_UPDATE_THRESHOLD) ||
            (thisGyroSensor.fYp[1] < -GYRO_UPDATE_THRESHOLD) ||
            (thisGyroSensor.fYp[2] > GYRO_UPDATE_THRESHOLD) ||
            (thisGyroSensor.fYp[2] < -GYRO_UPDATE_THRESHOLD))
        {
            printCount++;
            if (40 == printCount)
            {
                printCount = 0;
                PRINTF("[FXAS21002] Rotate detected: X:%5.1fdps, Y:%5.1fdps, Z:%5.1fdps\n\r",\
                       thisGyroSensor.fYp[0], thisGyroSensor.fYp[1], thisGyroSensor.fYp[2]);
            }
        }
    }
}

void fxos8700Demo(void)
{
    int16_t Ax, Ay, Az, Mx, My, Mz;
    float Axf, Ayf, Azf, Mxf, Myf, Mzf;

    PRINTF("\n\r-------------- FXOS8700 Acc+Mag data acquisition --------------\n\r\n\r");

    PRINTF("FXOS8700 initialization  ... ");
    if (fxos8700_init())
        PRINTF("OK\n\r");
    else
        PRINTF("ERROR\n\r");

    while(1)
    {
        vTaskDelay(500);
        fxos8700_read_data(&Ax, &Ay, &Az, &Mx, &My, &Mz);
        Axf = Ax / 8192.0;
        Ayf = Ay / 8192.0;
        Azf = Az / 8192.0;
        Mxf = Mx * 0.1;
        Myf = My * 0.1;
        Mzf = Mz * 0.1;
        PRINTF("[FXOS8700]Current Acc:X=%7.1fg Y=%7.1fg Z=%7.1fg\n\r",Axf, Ayf, Azf);
        PRINTF("[FXOS8700]Current Mag:X=%6.1fuT Y=%6.1fuT Z=%6.1fuT\n\r",Mxf, Myf, Mzf);
    }
}

void mpl3115Demo(void)
{
    pressure_sensor_t thisPressureSensor;
    float lastHight = 0, lastTemp = 0;

    PRINTF("\n\r-------------- MPL3115 Pressure data acquisition --------------\n\r\n\r");

    PRINTF("MPL3115 initialization   ... ");
    if (mpl3115_init(&thisPressureSensor))
        PRINTF("OK\n\r");
    else
        PRINTF("ERROR\n\r");

    while(1)
    {
        vTaskDelay(100);
        mpl3115_read_data(&thisPressureSensor);
        thisPressureSensor.iHp = thisPressureSensor.iHpFast;;
        thisPressureSensor.iTp = thisPressureSensor.iTpFast;
        thisPressureSensor.fHp = (float) thisPressureSensor.iHp *\
                  thisPressureSensor.fmPerCount;
        thisPressureSensor.fTp = (float) thisPressureSensor.iTp *\
                  thisPressureSensor.fCPerCount;
        if (((thisPressureSensor.fHp - lastHight) > HEIGHT_UPDATE_THRESHOLD)  ||
            ((thisPressureSensor.fHp - lastHight) < -HEIGHT_UPDATE_THRESHOLD) ||
            ((thisPressureSensor.fTp - lastTemp) > TEMP_UPDATE_THRESHOLD)   ||
            ((thisPressureSensor.fTp - lastTemp) < -TEMP_UPDATE_THRESHOLD))
        {
            lastHight = thisPressureSensor.fHp;
            lastTemp = thisPressureSensor.fTp;
            PRINTF("[MPL3115]Current Height = %6.1fMeter, Current Temp = %5.1fCelsius\n\r",
                   thisPressureSensor.fHp,
                   thisPressureSensor.fTp);
        }
    }
}

void MainTask(void *pvParameters)
{
    uint8_t demoSel;

    PRINTF("\n\r-------------- iMX7D SDB on board sensor example --------------\n\r\n\r");

    /* Setup I2C init structure. */
    i2c_init_config_t i2cInitConfig = {
        .clockRate       = get_i2c_clock_freq(BOARD_I2C_BASEADDR),
        .baudRate        = 400000u,
        .slaveAddress    = 0x00
    };

    /* Initialize I2C module with I2C init structure. */
    I2C_XFER_Config(&i2cInitConfig);

    /* Print the initial banner. */
    PRINTF("\n\rPlease select the sensor demo you want to run:\n\r");
    PRINTF("[1].FXAS21002 3-axes Gyro sensor\n\r");
    PRINTF("[2].FXOS8700 6-axes Acc+Mag sensor\n\r");
    PRINTF("[3].MPL3115 Pressure sensor\n\r");

    while(1)
    {
        demoSel = GETCHAR();
        if (('1' == demoSel) || ('2' == demoSel) || ('3' == demoSel))
            break;
    }

    switch(demoSel)
    {
        case '1':
            fxas21002Demo();
            break;
        case '2':
            fxos8700Demo();
            break;
        case '3':
            mpl3115Demo();
            break;
    }
}

int main(void)
{
    /* Initialize board specified hardware. */
    hardware_init();

    /* Create a the APP main task. */
    xTaskCreate(MainTask, "Main Task", configMINIMAL_STACK_SIZE + 100,
                NULL, tskIDLE_PRIORITY+1, NULL);

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* should never reach this point. */
    while (true);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
