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

#ifndef __UART_IMX_H__
#define __UART_IMX_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "device_imx.h"

/*!
 * @addtogroup uart_imx_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Uart module initialize structure. */
typedef struct _uart_init_config
{
    uint32_t clockRate;     /*!< Current UART module clock freq. */
    uint32_t baudRate;      /*!< Desired UART baud rate. */
    uint32_t wordLength;    /*!< Data bits in one frame. */
    uint32_t stopBitNum;    /*!< Number of stop bits in one frame. */
    uint32_t parity;        /*!< Parity error check mode of this module. */
    uint32_t direction;     /*!< Data transfer direction of this module. */
} uart_init_config_t;

/*!
 * @brief UART number of data bits in a character.
 */
enum _uart_word_length
{
    uartWordLength7Bits     = 0x0,
    uartWordLength8Bits     = UART_UCR2_WS_MASK,
};

/*!
 * @brief UART number of stop bits.
 */
enum _uart_stop_bit_num
{
    uartStopBitNumOne       = 0x0,
    uartStopBitNumTwo       = UART_UCR2_STPB_MASK,
};

/*!
 * @brief UART parity mode.
 */
enum _uart_partity_mode
{
    uartParityDisable       = 0x0,
    uartParityEven          = UART_UCR2_PREN_MASK,
    uartParityOdd           = UART_UCR2_PREN_MASK | UART_UCR2_PROE_MASK
};

/*!
 * @brief Data transfer direction.
 */
enum _uart_direction_mode
{
    uartDirectionDisable    = 0x0,
    uartDirectionTx         = UART_UCR2_TXEN_MASK,
    uartDirectionRx         = UART_UCR2_RXEN_MASK,
    uartDirectionTxRx       = UART_UCR2_TXEN_MASK | UART_UCR2_RXEN_MASK
};

/*!
 * @brief This enumeration contains the settings for all of the UART
 *        interrupt configurations.
 */
enum _uart_interrupt
{
    uartIntAutoBaud                       = 0x0080000F,
    uartIntTxReady                        = 0x0080000D,
    uartIntIdle                           = 0x0080000C,
    uartIntRxReady                        = 0x00800009,
    uartIntTxEmpty                        = 0x00800006,
    uartIntRtsDelta                       = 0x00800005,
    uartIntEscape                         = 0x0084000F,
    uartIntRts                            = 0x00840004,
    uartIntAgingTimer                     = 0x00840003,
    uartIntDtr                            = 0x0088000D,
    uartIntParityError                    = 0x0088000C,
    uartIntFrameError                     = 0x0088000B,
    uartIntDcd                            = 0x00880009,
    uartIntRi                             = 0x00880008,
    uartIntRxDs                           = 0x00880006,
    uartInttAirWake                       = 0x00880005,
    uartIntAwake                          = 0x00880004,
    uartIntDtrDelta                       = 0x00880003,
    uartIntAutoBaudCnt                    = 0x00880000,
    uartIntIr                             = 0x008C0008,
    uartIntWake                           = 0x008C0007,
    uartIntTxComplete                     = 0x008C0003,
    uartIntBreakDetect                    = 0x008C0002,
    uartIntRxOverrun                      = 0x008C0001,
    uartIntRxDataReady                    = 0x008C0000,
    uartIntRs485SlaveAddrMatch            = 0x00B80003
};

/*!
 * @brief Flag for UART interrupt/DMA status check or polling status.
 */
enum _uart_status_flag
{
    uartStatusRxCharReady                 = 0x0000000F,
    uartStatusRxError                     = 0x0000000E,
    uartStatusRxOverrunError              = 0x0000000D,
    uartStatusRxFrameError                = 0x0000000C,
    uartStatusRxBreakDetect               = 0x0000000B,
    uartStatusRxParityError               = 0x0000000A,
    uartStatusParityError                 = 0x0094000F,
    uartStatusRtsStatus                   = 0x0094000E,
    uartStatusTxReady                     = 0x0094000D,
    uartStatusRtsDelta                    = 0x0094000C,
    uartStatusEscape                      = 0x0094000B,
    uartStatusFrameError                  = 0x0094000A,
    uartStatusRxReady                     = 0x00940009,
    uartStatusAgingTimer                  = 0x00940008,
    uartStatusDtrDelta                    = 0x00940007,
    uartStatusRxDs                        = 0x00940006,
    uartStatustAirWake                    = 0x00940005,
    uartStatusAwake                       = 0x00940004,
    uartStatusRs485SlaveAddrMatch         = 0x00940003,
    uartStatusAutoBaud                    = 0x0098000F,
    uartStatusTxEmpty                     = 0x0098000E,
    uartStatusDtr                         = 0x0098000D,
    uartStatusIdle                        = 0x0098000C,
    uartStatusAutoBaudCntStop             = 0x0098000B,
    uartStatusRiDelta                     = 0x0098000A,
    uartStatusRi                          = 0x00980009,
    uartStatusIr                          = 0x00980008,
    uartStatusWake                        = 0x00980007,
    uartStatusDcdDelta                    = 0x00980006,
    uartStatusDcd                         = 0x00980005,
    uartStatusRts                         = 0x00980004,
    uartStatusTxComplete                  = 0x00980003,
    uartStatusBreakDetect                 = 0x00980002,
    uartStatusRxOverrun                   = 0x00980001,
    uartStatusRxDataReady                 = 0x00980000
};

/*!
 * @brief The events will generate DMA Request.
 */
enum _uart_dma
{
    uartDmaRxReady                        = 0x00800008,
    uartDmaTxReady                        = 0x00800003,
    uartDmaAgingTimer                     = 0x00800002,
    uartDmaIdle                           = 0x008C0006
};

/*!
 * @brief RTS pin interrupt trigger edge.
 */
enum _uart_rts_int_trigger_edge
{
    uartRtsTriggerEdgeRising              = UART_UCR2_RTEC(0),
    uartRtsTriggerEdgeFalling             = UART_UCR2_RTEC(1),
    uartRtsTriggerEdgeBoth                = UART_UCR2_RTEC(2)
};

/*!
 * @brief UART module modem role selections.
 */
enum _uart_modem_mode
{
    uartModemModeDce                      = 0,
    uartModemModeDte                      = UART_UFCR_DCEDTE_MASK
};

/*!
 * @brief DTR pin interrupt trigger edge.
 */
enum _uart_dtr_int_trigger_edge
{
    uartDtrTriggerEdgeRising              = UART_UCR3_DPEC(0),
    uartDtrTriggerEdgeFalling             = UART_UCR3_DPEC(1),
    uartDtrTriggerEdgeBoth                = UART_UCR3_DPEC(2)
};

/*!
 * @brief IrDA vote clock selections.
 */
enum _uart_irda_vote_clock
{
    uartIrdaVoteClockSampling             = 0x0,
    uartIrdaVoteClockReference            = UART_UCR4_IRSC_MASK
};

/*!
 * @brief UART module Rx Idle condition selections.
 */
enum _uart_rx_idle_condition
{
     uartRxIdleMoreThan4Frames            = UART_UCR1_ICD(0),
     uartRxIdleMoreThan8Frames            = UART_UCR1_ICD(1),
     uartRxIdleMoreThan16Frames           = UART_UCR1_ICD(2),
     uartRxIdleMoreThan32Frames           = UART_UCR1_ICD(3),
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name UART Initialization and Configuration functions
 * @{
 */

/*!
 * @brief Initialize UART module with given initialize structure.
 *
 * @param base UART base pointer.
 * @param initConfig UART initialize structure(see uart_init_config_t above).
 */
void UART_Init(UART_Type* base, uart_init_config_t* initConfig);

/*!
 * @brief This function reset Uart module register content to its default value.
 *
 * @param base UART base pointer.
 */
void UART_Deinit(UART_Type* base);

/*!
 * @brief This function is used to Enable the UART Module.
 *
 * @param base UART base pointer.
 */
static inline void UART_Enable(UART_Type* base)
{
    UART_UCR1_REG(base) |= UART_UCR1_UARTEN_MASK;
}

/*!
 * @brief This function is used to Disable the UART Module.
 *
 * @param base UART base pointer.
 */
static inline void UART_Disable(UART_Type* base)
{
    UART_UCR1_REG(base) &= ~UART_UCR1_UARTEN_MASK;
}

/*!
 * @brief This function is used to set the baud rate of UART Module.
 *
 * @param base UART base pointer.
 * @param clockRate UART module clock frequency.
 * @param baudRate Desired UART module baud rate.
 */
void UART_SetBaudRate(UART_Type* base, uint32_t clockRate, uint32_t baudRate);

/*!
 * @brief This function is used to set the transform direction of UART Module.
 *
 * @param base UART base pointer.
 * @param direction UART transfer direction(see _uart_direction_mode enumeration above).
 */
static inline void UART_SetDirMode(UART_Type* base, uint32_t direction)
{
    assert((direction & uartDirectionTx) || (direction & uartDirectionRx));
    UART_UCR2_REG(base) = (UART_UCR2_REG(base) & ~(UART_UCR2_RXEN_MASK | UART_UCR2_TXEN_MASK)) | direction;
}

/*!
 * @brief This function is used to set the number of frames RXD is allowed to
 *        be idle before an idle condition is reported. The available condition
 *        can be select from _uart_idle_condition enumeration.
 *
 * @param base UART base pointer.
 * @param idleCondition The condition that an idle condition is reported
 *                      (see _uart_idle_condition enumeration above).
 */
static inline void UART_SetRxIdleCondition(UART_Type* base, uint32_t idleCondition)
{
    assert(idleCondition <= uartRxIdleMoreThan32Frames);
    UART_UCR1_REG(base) = (UART_UCR1_REG(base) & ~UART_UCR1_ICD_MASK) | idleCondition;
}

/*!
 * @brief This function is used to set the polarity of UART signal. The polarity
 *        of Tx and Rx can be set separately.
 *
 * @param base UART base pointer.
 * @param direction UART transfer direction(see _uart_direction_mode enumeration above).
 * @param invert Set true to invert the polarity of UART signal.
 */
void UART_SetInvertCmd(UART_Type* base, uint32_t direction, bool invert);

/*@}*/

/*!
 * @name Low Power Mode functions.
 * @{
 */

/*!
 * @brief This function is used to set UART enable condition in the DOZE state.
 *
 * @param base UART base pointer.
 * @param enable Set true to enable UART module in doze mode.
 */
void UART_SetDozeMode(UART_Type* base, bool enable);

/*!
 * @brief This function is used to set UART enable condition of the UART low power feature.
 *
 * @param base UART base pointer.
 * @param enable Set true to enable UART module low power feature.
 */
void UART_SetLowPowerMode(UART_Type* base, bool enable);

/*@}*/

/*!
 * @name Data transfer functions.
 * @{
 */

/*!
 * @brief This function is used to send data in RS-232 and IrDA Mode.
 *        A independent 9 Bits RS-485 send data function is provided.
 *
 * @param base UART base pointer.
 * @param data Data to be set through Uart module.
 */
static inline void UART_Putchar(UART_Type* base, uint8_t data)
{
    UART_UTXD_REG(base) = (data & UART_UTXD_TX_DATA_MASK);
}

/*!
 * @brief This function is used to receive data in RS-232 and IrDA Mode.
 *        A independent 9 Bits RS-485 receive data function is provided.
 *
 * @param base UART base pointer.
 * @return The data received from UART module.
 */
static inline uint8_t UART_Getchar(UART_Type* base)
{
    return (uint8_t)(UART_URXD_REG(base) & UART_URXD_RX_DATA_MASK);
}

/*@}*/

/*!
 * @name Interrupt and Flag control functions.
 * @{
 */

/*!
 * @brief This function is used to set the enable condition of
 *        specific UART interrupt source. The available interrupt
 *        source can be select from uart_interrupt enumeration.
 *
 * @param base UART base pointer.
 * @param intSource Available interrupt source for this module.
 * @param enable Set true to enable corresponding interrupt.
 */
void UART_SetIntCmd(UART_Type* base, uint32_t intSource, bool enable);

/*!
 * @brief This function is used to get the current status of specific
 *        UART status flag(including interrupt flag). The available
 *        status flag can be select from _uart_status_flag enumeration.
 *
 * @param base UART base pointer.
 * @param flag Status flag to check.
 * @return current state of corresponding status flag.
 */
bool UART_GetStatusFlag(UART_Type* base, uint32_t flag);

/*!
 * @brief This function is used to get the current status
 *        of specific UART status flag. The available status
 *        flag can be select from _uart_status_flag enumeration.
 *
 * @param base UART base pointer.
 * @param flag Status flag to clear.
 */
void UART_ClearStatusFlag(UART_Type* base, uint32_t flag);

/*@}*/

/*!
 * @name DMA control functions.
 * @{
 */

/*!
 * @brief This function is used to set the enable condition of
 *        specific UART DMA source. The available DMA source
 *        can be select from _uart_dma enumeration.
 *
 * @param base UART base pointer.
 * @param dmaSource The Event that can generate DMA request.
 * @param enable Set true to enable corresponding DMA source.
 */
void UART_SetDmaCmd(UART_Type* base, uint32_t dmaSource, bool enable);

/*@}*/

/*!
 * @name FIFO control functions.
 * @{
 */

/*!
 * @brief This function is used to set the watermark of UART Tx FIFO.
 *        A maskable interrupt is generated whenever the data level in
 *        the TxFIFO falls below the Tx FIFO watermark.
 *
 * @param base UART base pointer.
 * @param watermark The Tx FIFO watermark.
 */
static inline void UART_SetTxFifoWatermark(UART_Type* base, uint8_t watermark)
{
    assert((watermark >= 2) && (watermark <= 32));
    UART_UFCR_REG(base) = (UART_UFCR_REG(base) & ~UART_UFCR_TXTL_MASK) | UART_UFCR_TXTL(watermark);
}

/*!
 * @brief This function is used to set the watermark of UART Rx FIFO.
 *        A maskable interrupt is generated whenever the data level in
 *        the RxFIFO reaches the Rx FIFO watermark.
 *
 * @param base UART base pointer.
 * @param watermark The Rx FIFO watermark.
 */
static inline void UART_SetRxFifoWatermark(UART_Type* base, uint8_t watermark)
{
    assert(watermark <= 32);
    UART_UFCR_REG(base) = (UART_UFCR_REG(base) & ~UART_UFCR_RXTL_MASK) | UART_UFCR_RXTL(watermark);
}

/*@}*/

/*!
 * @name Hardware Flow control and Modem Signal functions.
 * @{
 */

/*!
 * @brief This function is used to set the enable condition of RTS
 *        Hardware flow control.
 *
 * @param base UART base pointer.
 * @param enable Set true to enable RTS hardware flow control.
 */
void UART_SetRtsFlowCtrlCmd(UART_Type* base, bool enable);

/*!
 * @brief This function is used to set the RTS interrupt trigger edge.
 *        The available trigger edge can be select from
 *        _uart_rts_trigger_edge enumeration.
 *
 * @param base UART base pointer.
 * @param triggerEdge Available RTS pin interrupt trigger edge.
 */
static inline void UART_SetRtsIntTriggerEdge(UART_Type* base, uint32_t triggerEdge)
{
    assert((triggerEdge == uartRtsTriggerEdgeRising)  || \
           (triggerEdge == uartRtsTriggerEdgeFalling) || \
           (triggerEdge == uartRtsTriggerEdgeBoth));

    UART_UCR2_REG(base) = (UART_UCR2_REG(base) & ~UART_UCR2_RTEC_MASK) | triggerEdge;
}


/*!
 * @brief This function is used to set the enable condition of CTS
 *        auto control. if CTS control is enabled, the CTS_B pin will
 *        be controlled by the receiver, otherwise the CTS_B pin will
 *        controlled by UART_CTSPinCtrl function. 
 *
 * @param base UART base pointer.
 * @param enable Set true to enable CTS auto control.
 */
void UART_SetCtsFlowCtrlCmd(UART_Type* base, bool enable);

/*!
 * @brief This function is used to control the CTS_B pin state when 
 *        auto CTS control is disabled.
 *        The CTS_B pin is low(active)
 *        The CTS_B pin is high(inactive)
 *
 * @param base UART base pointer.
 * @param active Set true: the CTS_B pin active;
 *               Set false: the CTS_B pin inactive.
 */
void UART_SetCtsPinLevel(UART_Type* base, bool active);

/*!
 * @brief This function is used to set the auto CTS_B pin control
 *        trigger level. The CTS_B pin will be de-asserted when 
 *        Rx FIFO reach CTS trigger level.
 *
 * @param base UART base pointer.
 * @param triggerLevel Auto CTS_B pin control trigger level.
 */
static inline void UART_SetCtsTriggerLevel(UART_Type* base, uint8_t triggerLevel)
{
    assert(triggerLevel <= 32);
    UART_UCR4_REG(base) = (UART_UCR4_REG(base) & ~UART_UCR4_CTSTL_MASK) | UART_UCR4_CTSTL(triggerLevel);
}

/*!
 * @brief This function is used to set the role(DTE/DCE) of UART module
 *        in RS-232 communication.
 *
 * @param base UART base pointer.
 * @param mode The role(DTE/DCE) of UART module(see _uart_modem_mode enumeration above).
 */
void UART_SetModemMode(UART_Type* base, uint32_t mode);

/*!
 * @brief This function is used to set the edge of DTR_B (DCE) or
 *        DSR_B (DTE) on which an interrupt will be generated. 
 *
 * @param base UART base pointer.
 * @param triggerEdge The trigger edge on which an interrupt will be generated.
 *                    (see _uart_dtr_trigger_edge enumeration above)
 */
static inline void UART_SetDtrIntTriggerEdge(UART_Type* base, uint32_t triggerEdge)
{
    assert((triggerEdge == uartDtrTriggerEdgeRising)  || \
           (triggerEdge == uartDtrTriggerEdgeFalling) || \
           (triggerEdge == uartDtrTriggerEdgeBoth));
    UART_UCR3_REG(base) = (UART_UCR3_REG(base) & ~UART_UCR3_DPEC_MASK) | triggerEdge;
}

/*!
 * @brief This function is used to set the pin state of DSR pin(for DCE mode)
 *        or DTR pin(for DTE mode) for the modem interface.
 *
 * @param base UART base pointer.
 * @param active Set true: DSR/DTR pin is logic one.
 *               Set false: DSR/DTR pin is logic zero.
 */
void UART_SetDtrPinLevel(UART_Type* base, bool active);

/*!
 * @brief This function is used to set the pin state of 
 *        DCD pin. THIS FUNCTION IS FOR DCE MODE ONLY.
 *
 * @param base UART base pointer.
 * @param active Set true: DCD_B pin is logic one (DCE mode)
 *               Set false: DCD_B pin is logic zero (DCE mode)
 */
void UART_SetDcdPinLevel(UART_Type* base, bool active);

/*!
 * @brief This function is used to set the pin state of 
 *        RI pin. THIS FUNCTION IS FOR DCE MODE ONLY.
 *
 * @param base UART base pointer.
 * @param active Set true: RI_B pin is logic one (DCE mode)
 *               Set false: RI_B pin is logic zero (DCE mode)
 */
void UART_SetRiPinLevel(UART_Type* base, bool active);

/*@}*/

/*!
 * @name Multi-processor and RS-485 functions.
 * @{
 */

/*!
 * @brief This function is used to send 9 Bits length data in
 *        RS-485 Multidrop mode.
 *
 * @param base UART base pointer.
 * @param data Data(9 bits) to be set through uart module.
 */
void UAER_Putchar9(UART_Type* base, uint16_t data);

/*!
 * @brief This functions is used to receive 9 Bits length data in
 *        RS-485 Multidrop mode.
 *
 * @param base UART base pointer.
 * @return The data(9 bits) received from UART module.
 */
uint16_t UAER_Getchar9(UART_Type* base);

/*!
 * @brief This function is used to set the enable condition of 
 *        9-Bits data or Multidrop mode.
 *
 * @param base UART base pointer.
 * @param enable Set true to enable Multidrop mode.
 */
void UART_SetMultidropMode(UART_Type* base, bool enable);

/*!
 * @brief This function is used to set the enable condition of
 *        Automatic Address Detect Mode.
 *
 * @param base UART base pointer.
 * @param enable Set true to enable Automatic Address Detect mode.
 */
void UART_SetSlaveAddressDetectCmd(UART_Type* base, bool enable);

/*!
 * @brief This function is used to set the slave address char
 *        that the receiver will try to detect.
 *
 * @param base UART base pointer.
 * @param slaveAddress The slave to detect.
 */
static inline void UART_SetSlaveAddress(UART_Type* base, uint8_t slaveAddress)
{
    UART_UMCR_REG(base) = (UART_UMCR_REG(base) & ~UART_UMCR_SLADDR_MASK) | \
                          UART_UMCR_SLADDR(slaveAddress);
}

/*@}*/

/*!
 * @name IrDA control functions.
 * @{
 */

/*!
 * @brief This function is used to set the enable condition of
 *        IrDA Mode.
 *
 * @param base UART base pointer.
 * @param enable Set true to enable IrDA mode.
 */
void UART_SetIrDACmd(UART_Type* base, bool enable);

/*!
 * @brief This function is used to set the clock for the IR pulsed
 *        vote logic. The available clock can be select from
 *        _uart_irda_vote_clock enumeration.
 *
 * @param base UART base pointer.
 * @param voteClock The available IrDA vote clock selection.
 */
void UART_SetIrDAVoteClock(UART_Type* base, uint32_t voteClock);

/*@}*/

/*!
 * @name Misc. functions.
 * @{
 */

/*!
 * @brief This function is used to set the enable condition of
 *        Automatic Baud Rate Detection feature.
 *
 * @param base UART base pointer.
 * @param enable Set true to enable Automatic Baud Rate Detection feature.
 */
void UART_SetAutoBaudRateCmd(UART_Type* base, bool enable);

/*!
 * @brief This function is used to read the current value of Baud Rate
 *        Count Register value. this counter is used by Auto Baud Rate
 *        Detect feature.
 *
 * @param base UART base pointer.
 * @return Current Baud Rate Count Register value.
 */
static inline uint16_t UART_ReadBaudRateCount(UART_Type* base)
{
    return (uint16_t)(UART_UBRC_REG(base) & UART_UBRC_BCNT_MASK);
}

/*!
 * @brief This function is used to send BREAK character.It is 
 *        important that SNDBRK is asserted high for a sufficient
 *        period of time to generate a valid BREAK. 
 *
 * @param base UART base pointer.
 * @param active Asserted high to generate BREAK.
 */
void UART_SendBreakChar(UART_Type* base, bool active);

/*!
 * @brief This function is used to send BREAK character.It is 
 *        important that SNDBRK is asserted high for a sufficient
 *        period of time to generate a valid BREAK. 
 *
 * @param base UART base pointer.
 * @param active Asserted high to generate BREAK.
 */
void UART_SetEscapeDecectCmd(UART_Type* base, bool enable);

/*!
 * @brief This function is used to set the enable condition of
 *        Escape Sequence Detection feature.
 *
 * @param base UART base pointer.
 * @param escapeChar The Escape Character to detect.
 */
static inline void UART_SetEscapeChar(UART_Type* base, uint8_t escapeChar)
{
    UART_UESC_REG(base) = (UART_UESC_REG(base) & ~UART_UESC_ESC_CHAR_MASK) | \
                          UART_UESC_ESC_CHAR(escapeChar);
}

/*!
 * @brief This function is used to set the maximum time interval (in ms)
 *                 allowed between escape characters.
 *
 * @param base UART base pointer.
 * @param timerInterval Maximum time interval allowed between escape characters.
 */
static inline void UART_SetEscapeTimerInterval(UART_Type* base, uint16_t timerInterval)
{
    assert(timerInterval <= 0xFFF);
    UART_UTIM_REG(base) = (UART_UTIM_REG(base) & ~UART_UTIM_TIM_MASK) | \
                          UART_UTIM_TIM(timerInterval);
}

/*@}*/

#ifdef __cplusplus
}
#endif

/*! @}*/

#endif /* __UART_IMX_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
