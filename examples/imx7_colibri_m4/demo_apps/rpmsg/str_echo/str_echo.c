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
#include "string.h"
#include "assert.h"
#include "board.h"
#include "rpmsg/rpmsg.h"
#include "debug_console_imx.h"
#include "mu_imx.h"
#include "plat_porting.h"

/*
 * APP decided interrupt priority
 */
#define APP_MU_IRQ_PRIORITY 3

#define MAX_STRING_SIZE 496         /* Maximum size to hold the data A7 gives */

/*
 * For the most worst case, master will send 3 consecutive messages which remote
 * do not process.
 * The synchronization between remote and master is that each time endpoint callback
 * is called, the MU Receive interrupt is temperorily disabled. Until the next time
 * remote consumes the message, the interrupt will not be enabled again.
 * When the interrupt is not enabled, Master can not send the notify, it will blocks
 * there and can not send further message.
 * In the worst case, master send the first message, it triggles the ISR in remote
 * side, remote ISR clear the MU status bit so master can send the second message
 * and notify again, master can continue to send the 3rd message but will blocks
 * when trying to notify. Meanwhile, remote side is still in the first ISR which
 * has a loop to receive all the 3 messages.
 * Master is blocked and can not send the 4th message, remote side ISR copies all
 * this 3 message to app buffer and informs the app layer to consume them. After
 * a message is consumed, the ISR is enabled again and the second notify is received.
 * This unblocks the master to complete the 3rd notify and send the next message.
 * The 4th notify will not complete until remote consumes the second message.
 * The situation goes on and we can see application layer need a maximum size 3
 * buffer to hold the unconsumed messages. STRING_BUFFER_CNT is therefore set to 3
 */
#define STRING_BUFFER_CNT 3

/* Internal functions */
static void rpmsg_channel_created(struct rpmsg_channel *rp_chnl);
static void rpmsg_channel_deleted(struct rpmsg_channel *rp_chnl);
static void rpmsg_read_cb(struct rpmsg_channel *, void *, int, void *, unsigned long);

/* Globals */
static struct remote_device *rdev;
static struct rpmsg_channel *app_chnl;
static char strVar[STRING_BUFFER_CNT][MAX_STRING_SIZE + 1];
static uint8_t app_idx = 0;
static uint8_t handler_idx = 0;
static SemaphoreHandle_t app_sema;

/*!
 * @brief A basic RPMSG task
 */
void StrEchoTask(void *pvParameters)
{
    PRINTF("RPMSG String Echo Demo...\r\n");

    app_sema = xSemaphoreCreateCounting(STRING_BUFFER_CNT + 1, 0);

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
        /*
         * Take from next app string buffer
         */
        if ((strlen(strVar[app_idx]) == 2) && (strVar[app_idx][0] == 0xd) && (strVar[app_idx][1] == 0xa)) 
            PRINTF("Get New Line From A7 From Slot %d\r\n", app_idx);
        else
            PRINTF("Get Message From A7 : \"%s\" [len : %d] from slot %d\r\n", strVar[app_idx], strlen(strVar[app_idx]), app_idx);

        /*
         * echo back
         */
        rpmsg_send(app_chnl, (void*)strVar[app_idx], strlen(strVar[app_idx]));
        app_idx = (app_idx + 1) % STRING_BUFFER_CNT;
        /*
         * once a message is consumed, the MU receive interrupt can be enabled
         * again
         */
        MU_EnableRxFullInt(MU0_B, MU_RPMSG_CHANNEL);
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
    xTaskCreate(StrEchoTask, "String Echo Task", configMINIMAL_STACK_SIZE,
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
    /*
     * Temperorily Disable MU Receive Interrupt to avoid master 
     * sending too many messages and remote will fail to keep pace
     * to consume
     */
    MU_DisableRxFullInt(MU0_B, MU_RPMSG_CHANNEL);
    /*
     * Copy to next app string buffer
     */
    assert(len <= MAX_STRING_SIZE);
    memcpy((void*)strVar[handler_idx], data, len);
    /*
     * Add trailing '\0'
     */
    strVar[handler_idx][len] = 0;
    handler_idx = (handler_idx + 1) % STRING_BUFFER_CNT;
    xSemaphoreGiveFromISR(app_sema, NULL);
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
