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

#include "gpio_pins.h"
#include "board.h"

void hardware_init(void)
{
    int i;

    /* Board specific RDC settings */
    BOARD_RdcInit();
    /* Board specific clock settings */
    BOARD_ClockInit();
    /* initialize debug uart */
    dbg_uart_init();

    /*
     * In order to wakeup M4 from LPM, some PLLCTRLs need to be set to "NeededRun"
     */
    CCM_BASE_PTR->PLL_CTRL[0].PLL_CTRL = ccmClockNeededRun;
    CCM_BASE_PTR->PLL_CTRL[6].PLL_CTRL = ccmClockNeededRun;
    CCM_BASE_PTR->PLL_CTRL[7].PLL_CTRL = ccmClockNeededRun;
    CCM_BASE_PTR->PLL_CTRL[8].PLL_CTRL = ccmClockNeededRun;
    CCM_BASE_PTR->PLL_CTRL[9].PLL_CTRL = ccmClockNeededRun;
    CCM_BASE_PTR->PLL_CTRL[10].PLL_CTRL = ccmClockNeededRun;
    CCM_BASE_PTR->PLL_CTRL[11].PLL_CTRL = ccmClockNeededRun;
    CCM_BASE_PTR->PLL_CTRL[12].PLL_CTRL = ccmClockNeededRun;
    CCM_BASE_PTR->PLL_CTRL[13].PLL_CTRL = ccmClockNeededRun;
    CCM_BASE_PTR->PLL_CTRL[14].PLL_CTRL = ccmClockNeededRun;
    CCM_BASE_PTR->PLL_CTRL[15].PLL_CTRL = ccmClockNeededRun;

    /* Enable clock gate for wakeup mix*/
    CCM_ControlGate(CCM, BOARD_SIM_WAKEUP_CCGR, ccmClockNeededAll);
    /* In this demo, we need to share board GPIO without RDC SEMAPHORE */
    RDC_SetPdapAccess(RDC, rdcPdapGpio1, 0xFF, false, false);
    RDC_SetPdapAccess(RDC, rdcPdapGpio4, 0xFF, false, false);

    /* Enable gpio clock gate */
    CCM_ControlGate(CCM, ccmCcgrGateGpio1, ccmClockNeededRunWait);
    CCM_ControlGate(CCM, ccmCcgrGateGpio4, ccmClockNeededRunWait);

    /* RDC ECSPI */
    RDC_SetPdapAccess(RDC, rdcPdapEcspi3, 0xff, false, false);
    //RDC_SetPdapAccess(RDC, rdcPdapEcspi3, 3 << (BOARD_DOMAIN_ID * 2), false, false);
    /* Select board ecspi clock derived from OSC clock(24M) */
    CCM_UpdateRoot(CCM, BOARD_ECSPI_CCM_ROOT, ccmRootmuxEcspiOsc24m, 0, 0);
    /* Enable ecspi clock gate */
    CCM_EnableRoot(CCM, BOARD_ECSPI_CCM_ROOT);
    CCM_ControlGate(CCM, BOARD_ECSPI_CCM_CCGR, ccmClockNeededAll);
    /* Configure ecspi pin IOMUX */
    configure_ecspi_pins(BOARD_ECSPI_BASEADDR);

    /* Enable MU clock*/
    CCM_ControlGate(CCM, BOARD_MU_CCM_CCGR, ccmClockNeededAll);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/