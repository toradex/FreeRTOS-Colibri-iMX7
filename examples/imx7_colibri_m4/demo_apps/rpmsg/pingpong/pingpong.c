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

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "board.h"
#include "rpmsg/rpmsg.h"
#include "debug_console_imx.h"
#include "mu_imx.h"
#include "plat_porting.h"

/*
 * APP decided interrupt priority
 */
#define APP_MU_IRQ_PRIORITY 3

/* Internal functions */
static void rpmsg_channel_created(struct rpmsg_channel *rp_chnl);
static void rpmsg_channel_deleted(struct rpmsg_channel *rp_chnl);
static void rpmsg_read_cb(struct rpmsg_channel *, void *, int, void *, unsigned long);

/* Globals */
static struct remote_device *rdev;
static struct rpmsg_channel *app_chnl;
static uint32_t msg_var;
static SemaphoreHandle_t app_sema;

/*!
 * @brief A basic RPMSG task
 */
void PingPongTask(void *pvParameters)
{
    PRINTF("RPMSG PingPong Demo...\r\n");

    app_sema = xSemaphoreCreateCounting(2, 0);

    PRINTF("RPMSG Init as Remote\r\n");
    /*
     * RPMSG Init as REMOTE
     */
    rpmsg_init(0, &rdev, rpmsg_channel_created, rpmsg_channel_deleted, rpmsg_read_cb, RPMSG_MASTER);

    /*
     * rpmsg_channel_created will post the first semaphore
     */
    xSemaphoreTake(app_sema, portMAX_DELAY);
    PRINTF("Name service handshake is done, M4 has setup a rpmsg channel [%d ---> %d]\r\n", app_chnl->src, app_chnl->dst);


    /*
     * pingpong demo loop
     */
    for (;;) {
        xSemaphoreTake(app_sema, portMAX_DELAY);
        PRINTF("Get Data From A7 : %d\r\n", msg_var);
        msg_var++;
        rpmsg_send(app_chnl, (void*)&msg_var, sizeof(uint32_t));
    }
}


/*
 * MU Interrrupt ISR
 */
void BOARD_MU_HANDLER(void)
{
    /*
     * calls into rpmsg_handler provided by middleware
     */
    rpmsg_handler();
}


int main(void)
{
    hardware_init();

    /*
     * Prepare for the MU Interrupt
     *  MU must be initialized before rpmsg init is called
     */
    MU_Init(BOARD_MU_BASE_ADDR);
    NVIC_SetPriority(BOARD_MU_IRQ_NUM, APP_MU_IRQ_PRIORITY);
    NVIC_EnableIRQ(BOARD_MU_IRQ_NUM);

    // Create a demo task which will print Hello world and echo user's input.
    xTaskCreate(PingPongTask, "Ping Pong Task", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY+1, NULL);

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    // Should never reach this point.
    while (true);
}

/* rpmsg_rx_callback will call into this for a channel creation event*/
static void rpmsg_channel_created(struct rpmsg_channel *rp_chnl) 
{
    /*
     * we should give the created rp_chnl handler to app layer
     */
    app_chnl = rp_chnl;

    /*
     * sync to application layer
     */
    xSemaphoreGiveFromISR(app_sema, NULL);
}

static void rpmsg_channel_deleted(struct rpmsg_channel *rp_chnl)
{
    rpmsg_destroy_ept(rp_chnl->rp_ept);
}

static void rpmsg_read_cb(struct rpmsg_channel *rp_chnl, void *data, int len,
                void * priv, unsigned long src) 
{
    msg_var = *(uint32_t*)data;
    /*
     * sync to application layer
     */
    xSemaphoreGiveFromISR(app_sema, NULL);
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
