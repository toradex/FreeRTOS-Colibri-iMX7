/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
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
#include <string.h>
#include "board.h"
#include "lpm_mcore.h"
#include "debug_console_imx.h"
#include "ccm_imx7d.h"
#include "mu_imx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpt.h"

unsigned long configCPU_CLOCK_HZ = configCPU_CLOCK_HZ_DEFAULT;
extern void vPortSetupTimerInterrupt(void);

#define MAXIMUM_24M_DIV 15

static LPM_POWER_STATUS_M4 m4_lpm_state = LPM_M4_STATE_RUN;

static P_WAKEUP_INT_ELE g_wakeup_int_list;

/*
 * Send Message to A7
 */
void LPM_MCORE_SendMessage(uint32_t msg)
{
    while (0 == (MUB_SR & MU_SR_TEn(0x8 >> LPM_MCORE_MU_CHANNEL)));
    MUB->TR[LPM_MCORE_MU_CHANNEL] = msg;
    while ((MUB_SR & MU_SR_EP_MASK) != 0);
}

#if (defined(LPM_MCORE_PRINT_DEBUG_INFO) && (LPM_MCORE_PRINT_DEBUG_INFO))
/*
 * Provide a spinning delay, the actual delay time is dependent on the CPU freq
 */
static void my_delay(void)
{
    uint32_t i, j, k;
    for (i=0; i!=DELAY_CNT; i++)
        for (j=0; j!=DELAY_CNT; j++)
            for (k=0; k!=DELAY_CNT; k++)
                __NOP();
    return;
}

/*
 * Use the delay function to demostrate current CPU running freq
 */
static void verify_clock_speed(void)
{
    uint32_t i;
    for (i=0; i!=DELAY_LOOP_CNT_LOW_SPEED; i++) {
        my_delay();
        PRINTF("\rVerify M4 Speed : %2d of %d ... ", i+1, DELAY_LOOP_CNT_LOW_SPEED);
    }
    PRINTF("Done.\r\n");
}
#endif

/*
 * initialize the wakeup interrupt list
 */
static void lpm_init_wakeup_interrupt_list() {
    g_wakeup_int_list = NULL;
}

/*
 * add a new irq to wakeup interrupt link list
 */
static void lpm_add_wakeup_interrupt_list(uint32_t irq_no)
{
    P_WAKEUP_INT_ELE cur_ele = g_wakeup_int_list;
    P_WAKEUP_INT_ELE p;

    if (cur_ele == NULL) {
        /*
         * first element to add
         */
        p = pvPortMalloc(sizeof(WAKEUP_INT_ELE));
        p->irq_no = irq_no;
        p->next = NULL;
        g_wakeup_int_list = p;
    } else {
        for (;;) {
            if (cur_ele->irq_no == irq_no) {
                /*
                 * already in the link list
                 *      - return directly
                 */
                break;
            }
            else if (cur_ele->next == NULL) {
                /*
                 * can't find the element
                 *      - insert into the end
                 */
                p = pvPortMalloc(sizeof(WAKEUP_INT_ELE));
                p->irq_no = irq_no;
                p->next = NULL;
                cur_ele->next = p;
            } else {
                cur_ele = cur_ele->next;
            }
        }
    }
}

/*
 * remove an exsiting irq to wakeup interrupt link list
 */
static void lpm_del_wakeup_interrupt_list(uint32_t irq_no)
{
    P_WAKEUP_INT_ELE cur_ele = g_wakeup_int_list;
    P_WAKEUP_INT_ELE p;

    if (cur_ele != NULL) {
        if (cur_ele->irq_no == irq_no) {
            /*first element is the target*/
            p = g_wakeup_int_list;
            g_wakeup_int_list = p->next;
            vPortFree(p);
        } else {
            for (;;) {
                p = cur_ele->next;
                if (p == NULL) {
                    /*
                     * can't find the element
                     *      - return directly
                     */
                    break;
                } else {
                    if (p->irq_no == irq_no) {
                        /*
                         * Find the target "p"
                         */
                        cur_ele->next = p->next;
                        vPortFree(p);
                        break;
                    } else {
                        cur_ele = cur_ele->next;
                    }
                }
            }
        }
    }
}


/*
 * register a IRQ source as M4 wakeup source
 */
void LPM_MCORE_RegisterWakeupInterrupt(GPC_Type * base, uint32_t irq_no, GPC_IRQ_WAKEUP_MODE wakeup_mode)
{
    /*register wakeup interrupt for M4 in GPC*/
    GPC_EnableM4WakeupIRQ(base, irq_no, wakeup_mode);

    if (wakeup_mode == GPC_IRQ_WAKEUP_ENABLE) {
        /*add an element to link list*/
        lpm_add_wakeup_interrupt_list(irq_no);
    } else {
        /*delete an element to link list*/
        lpm_del_wakeup_interrupt_list(irq_no);
    }
}


/*
 * Low Power Management initialization
 */
void LPM_MCORE_Init(GPC_Type * base)
{
    // Init GPC
    GPC_Init(base);

    // Init the wakeup interrupt link list
    lpm_init_wakeup_interrupt_list();
}


/*
 * get the current m4 LPM state
 */
LPM_POWER_STATUS_M4 LPM_MCORE_GetPowerStatus(GPC_Type * base)
{
    return m4_lpm_state;
} 

/*
 * on-the-fly change m4 parent clock between 24MHz and 240MHz
 */
void LPM_MCORE_ChangeM4Clock(LPM_M4_CLOCK_SPEED target)
{
    // change CCM Root to change M4 clock
    switch (target) {
        case LPM_M4_LOW_FREQ:
            if (CCM_GetRootMux(CCM, ccmRootM4) != ccmRootmuxM4Osc24m) {
            #if (defined(LPM_MCORE_PRINT_DEBUG_INFO) && (LPM_MCORE_PRINT_DEBUG_INFO))
                PRINTF("Change M4 clock freq to 24M\r\n");
            #endif
                CCM_SetRootMux(CCM, ccmRootM4, ccmRootmuxM4Osc24m);
            }
            CCM_ControlGate(CCM, ccmPllGateSys, ccmClockNotNeeded);
            CCM_ControlGate(CCM, ccmPllGateSysDiv2, ccmClockNotNeeded);
            configCPU_CLOCK_HZ = 24000000ul;
            break;
        case LPM_M4_HIGH_FREQ:
            CCM_ControlGate(CCM, ccmPllGateSys, ccmClockNeededRun);
            CCM_ControlGate(CCM, ccmPllGateSysDiv2, ccmClockNeededRun);
            if (CCM_GetRootMux(CCM, ccmRootM4) != ccmRootmuxM4SysPllDiv2) {
            #if (defined(LPM_MCORE_PRINT_DEBUG_INFO) && (LPM_MCORE_PRINT_DEBUG_INFO))
                PRINTF("Change M4 clock freq to SysPLL Div2 (240M)\r\n");
            #endif
                CCM_SetRootMux(CCM, ccmRootM4, ccmRootmuxM4SysPllDiv2);
            }
            configCPU_CLOCK_HZ = 240000000ul;
            break;
        default:
            break;
    }
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        vPortSetupTimerInterrupt();
#if (defined(LPM_MCORE_PRINT_DEBUG_INFO) && (LPM_MCORE_PRINT_DEBUG_INFO))
    verify_clock_speed();
#endif
}

/*
 * cycle M4 low power mode to next state, the state machine is
 *
 *        +---> "RUN" ---> "WAIT" ---> "STOP" ---+
 *        |                                      |
 *        +--------------------------------------+
 */
void LPM_MCORE_SetPowerStatus(GPC_Type * base, LPM_POWER_STATUS_M4 m4_next_lpm)
{
    uint32_t next_lpm = GPC_LPCR_M4_LPM0(0);
    switch (m4_next_lpm) {
        case LPM_M4_STATE_RUN:
            next_lpm = GPC_LPCR_M4_LPM0(0);
            break;
        case LPM_M4_STATE_WAIT:
            next_lpm = GPC_LPCR_M4_LPM0(1);
            break;
        case LPM_M4_STATE_STOP:
            next_lpm = GPC_LPCR_M4_LPM0(2);
            break;
        default:
            break;
    }

    /*
     * Patch, let GPC-M4 observe the GPR0 interrupt for a period as long
     * as 5 32KHz clock cycle before set it to a Low power status
     */
    if (m4_next_lpm != LPM_M4_STATE_RUN)
    {
        uint32_t i;
        LPM_MCORE_RegisterWakeupInterrupt(GPC, GPT4_IRQn, GPC_IRQ_WAKEUP_ENABLE);
        for (i=0; i!=GPC_SYNC_DELAY_CNT; i++)
            __NOP();
        LPM_MCORE_RegisterWakeupInterrupt(GPC, GPT4_IRQn, GPC_IRQ_WAKEUP_DISABLE);
    }

    GPC_SetM4NextLPM(base, next_lpm);

    /*change lpm state variable*/
    m4_lpm_state = m4_next_lpm;
}



/*
 * Give readable string of current M4 lpm state
 */
const char* LPM_MCORE_GetPowerStatusString(void)
{
    switch (m4_lpm_state) {
        case LPM_M4_STATE_RUN:
            return "RUN";
        case LPM_M4_STATE_WAIT:
            return "WAIT";
        case LPM_M4_STATE_STOP:
            return "STOP";
        default:
            return "UNKNOWN";
    }
}

/*
 * Check if A7 LPM Driver is ready, an "Once Ready, Always Ready" logic is used
 */
uint32_t LPM_MCORE_CheckPeerReady(void) 
{
    static uint32_t a7_ready = 0;
    if (!a7_ready) {
        a7_ready = MU_GetFlags(MUB) & MU_SR_Fn(1);
    }
    return a7_ready;
}

/*
 * Use MU Flag to indicate to A7 that low power management in M4 is ready
 */
void LPM_MCORE_SetSelfReady(void)
{
    MU_SetFlags(MUB, MU_CR_Fn(1));
}

/*
 * This function modify BASEPRI to configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY and
 * wakeup interrupt's NVIC->Priority to configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1
 * The effect is all non-wakeup interrupt gets mute
 * The original basepri settings are stored into pBasepriBackup
 * The original wakeup interrupt nvic->priority settings are stored into linklist
 */
void lpm_disable_non_wakeup_interrupt(uint32_t* pBasepriBackup)
{
    P_WAKEUP_INT_ELE ele;
    uint32_t irq_no;
#if defined(__CC_ARM)
    register uint32_t __regBasePri __ASM("basepri");
    *pBasepriBackup = __regBasePri;
    __regBasePri  = (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - __NVIC_PRIO_BITS));
#else
    *pBasepriBackup = __get_BASEPRI();
    __set_BASEPRI(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - __NVIC_PRIO_BITS));
#endif
    /*
     * Make exceptions to wakeup interrupts, they are stored in "g_wakeup_int_list"
     */
    ele = g_wakeup_int_list;
    for (;;) {
        if (ele == NULL)
            break;

        /* 
         * Store the current Priority into ele backup field
         * Change the Priority to "configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1"
         */
        irq_no = ele->irq_no;
        ele->irq_priority_backup = NVIC->IP[irq_no];
        NVIC->IP[irq_no] = (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1) << (8 - __NVIC_PRIO_BITS);

        /*
         * Move to next
         */
        ele = ele->next;
    }
    __DSB();
    __ISB();
}

/*
 * This function restores BASEPRI and wakeup interrupt nvic priority settings
 * It recover interrupt settings made by lpm_disable_non_wakeup_interrupt
 */
void lpm_enable_non_wakeup_interrupt(uint32_t basePriBackup)
{
    P_WAKEUP_INT_ELE ele;
    uint32_t irq_no;
#if defined(__CC_ARM)
    register uint32_t __regBasePri __ASM("basepri");
#endif
    /*
     * first restore wakeup interrupt priority
     */
    ele = g_wakeup_int_list;
    for (;;) {
        if (ele == NULL)
            break;

        /* 
         * Restore the original priority
         */
        irq_no = ele->irq_no;
        NVIC->IP[irq_no] = ele->irq_priority_backup;

        /*
         * Move to next
         */
        ele = ele->next;
    }
#if defined(__CC_ARM)
    __regBasePri = basePriBackup & 0xFF;
#else
    __set_BASEPRI(basePriBackup);
#endif
    // infinite_loop();
    // Are these necessary?
    __DSB();
    __ISB();
}


/*
 * The sleep function inserted into FreeRTOS idle task hook
 */
void LPM_MCORE_WaitForInt(void)
{
    uint32_t priMaskBackup;

    /* 
     * Only when 
     *      1. A7 peer is ready
     *      2. safe sleep function has been put into TCM
     *      3. m4 true sleep mode has been allowed
     *      4. m4 current lpm mode is wait / stop
     *  The "power save wfi" routine will be executed
     *  Otherwise "normal wfi" will be executed
     *
     *  In Power Save WFI
     *      - PRIMASK is set, so all interrupt handler won't be executed
     *      - BASEPRI and NVIC->Priority is modified so that only wakeup interrupt can
     *        wake up M4 from WFI
     *      - After M4 wake up, NVIC->Priority, BASEPRI and PRIMASK are restored so that
     *        the system return to normal mode
     *
     *  There is a critical section code which is in "runInRAM", inside it M4 will
     *  inform A7 it release the high bus. A7 can then shutdown the high bus which
     *  will make all highbus related peripherals losing functionality, including
     *  DDR, so code in "runInRAM" should run in TCM and don't have any access to
     *  other part of memory
     */
    if (LPM_MCORE_CheckPeerReady() && false)
    {
        volatile uint32_t* reg_lpcr_m4 = &GPC->LPCR_M4;

        uint32_t next_lpm_mode = *reg_lpcr_m4 & GPC_LPCR_M4_LPM0_MASK;
        uint32_t basePriBackup;

        /* Save current PRIMASK value. */
        priMaskBackup = __get_PRIMASK();

        /* 
         * Set PRIMASK to avoid execution of any enabled ISR. 
         *      Note : PRIMASK will not prevent interrupt to wake up M4 from WFI
         *             but it will prevent interrupt handler from running
         */
        __set_PRIMASK(1);
        /*
         * Some of the code should be moved out of "runInRAM"
         */
        switch (next_lpm_mode) {
            case LPCR_M4_RUN:
                /*
                 * STOP -> RUN
                 */
                /*
                 * tell A7 the next LPM mode is RUN
                 */
                /*
                 * the WFI will be wakeup by any enabled interrupt
                 */
                __WFI();
                break;
            case LPCR_M4_WAIT:
            case LPCR_M4_STOP:
                /*
                 * RUN -> WAIT or WAIT -> STOP
                 */
                /*
                 * tell A7 the next LPM mode is WAIT/STOP
                 */
                if (next_lpm_mode == LPCR_M4_WAIT)
                    LPM_MCORE_SendMessage(MSG_LPM_M4_WAIT);
                else if (next_lpm_mode == LPCR_M4_STOP)
                    LPM_MCORE_SendMessage(MSG_LPM_M4_STOP);
                /* 
                 * do modification to BASEPRI and NVIC->Priority settings so that
                 * all interrupt except wakeup interrupt are disabled
                 */
                lpm_disable_non_wakeup_interrupt(&basePriBackup);

                /*
                 * Inside "runInRAM", M4 will inform A7 that it release the highbus. Later
                 * when M4 is waken up, it will request A7 to resume highbus. This section
                 * of code must run in TCM to avoid accessing highbus dependent resouces
                 */
                __WFI();

                // Restore Basepri and NVIC->Priority settings
                lpm_enable_non_wakeup_interrupt(basePriBackup);
                break;
            default:
                break;
        }
        /* 
         * Recover PRIMASK register value. this will enable the wakeup interrupt
         * handler and will activate the main task immediately
         */
        __set_PRIMASK(priMaskBackup);
    }
    else {
        /*
         * Normal WFI which will be wakeup by any enabled interrupt
         */
        __WFI();
    }
}

