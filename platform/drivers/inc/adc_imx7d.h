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

#ifndef __ADC_IMX7D_H__
#define __ADC_IMX7D_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "device_imx.h"

/*!
 * @addtogroup adc_imx7d_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 *  @brief ADC module initialize structure.
 */
typedef struct _adc_init_config
{
    uint32_t sampleRate;            /*!< The desired ADC sample rate.*/
    bool     levelShifterEnable;    /*!< The level shifter module configuration(Enable to power on ADC module).*/
} adc_init_config_t;

/*!
 * @brief ADC logic channel initialize structure.
 */
typedef struct _adc_logic_ch_init_config
{
    uint8_t  inputChannel;          /*!< The logic channel to be set.*/
    bool     coutinuousEnable;      /*!< Continuous sample mode enable configuration.*/
    uint32_t convertRate;           /*!< The continuous rate when continuous sample enabled.*/
    bool     averageEnable;         /*!< Hardware average enable configuration.*/
    uint8_t  averageNumber;         /*!< The average number for hardware average function.*/
} adc_logic_ch_init_config_t;

/*!
 * @brief ADC logic channel selection enumeration.
 */
enum _adc_logic_ch_selection
{
    adcLogicChA     = 0x0,          /*!< ADC Logic Channel A.*/
    adcLogicChB     = 0x1,          /*!< ADC Logic Channel B.*/
    adcLogicChC     = 0x2,          /*!< ADC Logic Channel C.*/
    adcLogicChD     = 0x3,          /*!< ADC Logic Channel D.*/
    adcLogicChSW    = 0x4           /*!< ADC Logic Channel Software.*/
};

/*!
 * @brief ADC hardware average number enumeration.
 */
enum _adc_average_number
{
    adcAvgNum4      = 0x0,          /*!< ADC Hardware Average Number is set to 4.*/
    adcAvgNum8      = 0x1,          /*!< ADC Hardware Average Number is set to 8.*/
    adcAvgNum16     = 0x2,          /*!< ADC Hardware Average Number is set to 16.*/
    adcAvgNum32     = 0x3           /*!< ADC Hardware Average Number is set to 32.*/
};

/*!
 * @brief ADC build-in comparer work mode configuration enumeration.
 */
enum _adc_compare_mode
{
    adcCmpModeDisable             = 0x0,    /*!< ADC build-in comparator is disabled.*/
    adcCmpModeGreaterThanLow      = 0x1,    /*!< ADC build-in comparator will be triggered when sample value greater than low threshold.*/
    adcCmpModeLessThanLow         = 0x2,    /*!< ADC build-in comparator will be triggered when sample value less than low threshold.*/
    adcCmpModeInInterval          = 0x3,    /*!< ADC build-in comparator will be triggered when sample value in interval between low and high threshold.*/
    adcCmpModeGreaterThanHigh     = 0x5,    /*!< ADC build-in comparator will be triggered when sample value greater than high threshold.*/
    adcCmpModeLessThanHigh        = 0x6,    /*!< ADC build-in comparator will be triggered when sample value less than high threshold.*/
    adcCmpModeOutOffInterval      = 0x7     /*!< ADC build-in comparator will be triggered when sample value out of interval between low and high threshold.*/
};

/*!
 * @brief This enumeration contains the settings for all of the ADC
 *        interrupt configurations.
 */
enum _adc_interrupt
{
    adcIntLastFifoDataRead         = ADC_INT_EN_LAST_FIFO_DATA_READ_EN_MASK,
    adcIntConvertTimeoutChSw       = ADC_INT_EN_SW_CH_COV_TO_INT_EN_MASK,
    adcIntConvertTimeoutChD        = ADC_INT_EN_CHD_COV_TO_INT_EN_MASK,
    adcIntConvertTimeoutChC        = ADC_INT_EN_CHC_COV_TO_INT_EN_MASK,
    adcIntConvertTimeoutChB        = ADC_INT_EN_CHB_COV_TO_INT_EN_MASK,
    adcIntConvertTimeoutChA        = ADC_INT_EN_CHA_COV_TO_INT_EN_MASK,
    adcIntConvertChSw              = ADC_INT_EN_SW_CH_COV_INT_EN_MASK,
    adcIntConvertChD               = ADC_INT_EN_CHD_COV_INT_EN_MASK,
    adcIntConvertChC               = ADC_INT_EN_CHC_COV_INT_EN_MASK,
    adcIntConvertChB               = ADC_INT_EN_CHB_COV_INT_EN_MASK,
    adcIntConvertChA               = ADC_INT_EN_CHA_COV_INT_EN_MASK,
    adcIntFifoOverrun              = ADC_INT_EN_FIFO_OVERRUN_INT_EN_MASK,
    adcIntFifoUnderrun             = ADC_INT_EN_FIFO_UNDERRUN_INT_EN_MASK,
    adcIntDmaReachWatermark        = ADC_INT_EN_DMA_REACH_WM_INT_EN_MASK,
    adcIntCmpChD                   = ADC_INT_EN_CHD_CMP_INT_EN_MASK,
    adcIntCmpChC                   = ADC_INT_EN_CHC_CMP_INT_EN_MASK,
    adcIntCmpChB                   = ADC_INT_EN_CHB_CMP_INT_EN_MASK,
    adcIntCmpChA                   = ADC_INT_EN_CHA_CMP_INT_EN_MASK
};

/*!
 * @brief Flag for ADC interrupt/DMA status check or polling status.
 */
enum _adc_status_flag
{
    adcStatusLastFifoDataRead      = ADC_INT_STATUS_LAST_FIFO_DATA_READ_MASK,
    adcStatusConvertTimeoutChSw    = ADC_INT_STATUS_SW_CH_COV_TO_MASK,
    adcStatusConvertTimeoutChD     = ADC_INT_STATUS_CHD_COV_TO_MASK,
    adcStatusConvertTimeoutChC     = ADC_INT_STATUS_CHC_COV_TO_MASK,
    adcStatusConvertTimeoutChB     = ADC_INT_STATUS_CHB_COV_TO_MASK,
    adcStatusConvertTimeoutChA     = ADC_INT_STATUS_CHA_COV_TO_MASK,
    adcStatusConvertChSw           = ADC_INT_STATUS_SW_CH_COV_MASK,
    adcStatusConvertChD            = ADC_INT_STATUS_CHD_COV_MASK,
    adcStatusConvertChC            = ADC_INT_STATUS_CHC_COV_MASK,
    adcStatusConvertChB            = ADC_INT_STATUS_CHB_COV_MASK,
    adcStatusConvertChA            = ADC_INT_STATUS_CHA_COV_MASK,
    adcStatusFifoOverrun           = ADC_INT_STATUS_FIFO_OVERRUN_MASK,
    adcStatusFifoUnderrun          = ADC_INT_STATUS_FIFO_UNDERRUN_MASK,
    adcStatusDmaReachWatermark     = ADC_INT_STATUS_DMA_REACH_WM_MASK,
    adcStatusCmpChD                = ADC_INT_STATUS_CHD_CMP_MASK,
    adcStatusCmpChC                = ADC_INT_STATUS_CHC_CMP_MASK,
    adcStatusCmpChB                = ADC_INT_STATUS_CHB_CMP_MASK,
    adcStatusCmpChA                = ADC_INT_STATUS_CHA_CMP_MASK
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name ADC Module Initialization and Configuration functions.
 * @{
 */

/*!
 * @brief Initialize ADC to reset state and initialize with initialize structure.
 *
 * @param base ADC base pointer.
 * @param initConfig ADC initialize structure.
 */
void ADC_Init(ADC_Type* base, adc_init_config_t* initConfig);

/*!
 * @brief This function reset ADC module register content to its default value.
 *
 * @param base ADC base pointer.
 */
void ADC_Deinit(ADC_Type* base);

/*!
 * @brief This function Enable ADC module build-in Level Shifter.
 *        For iMX7D, Level Shifter should always be enabled.
 *        User can disable Level Shifter to save power.
 *
 * @param base ADC base pointer.
 */
static inline void ADC_LevelShifterEnable(ADC_Type* base)
{
    ADC_ADC_CFG_REG(base) |= ADC_ADC_CFG_ADC_EN_MASK;
}

/*!
 * @brief This function Disable ADC module build-in Level Shifter
 *        to save power.
 *
 * @param base ADC base pointer.
 */
static inline void ADC_LevelShifterDisable(ADC_Type* base)
{
    ADC_ADC_CFG_REG(base) &= ~ADC_ADC_CFG_ADC_EN_MASK;
}

/*!
 * @brief This function is used to set ADC module sample rate.
 *
 * @param base ADC base pointer.
 * @param sampleRate Desired ADC sample rate.
 */
void ADC_SetSampleRate(ADC_Type* base, uint32_t sampleRate);

/*@}*/

/*!
 * @name ADC Low power control functions.
 * @{
 */

/*!
 * @brief This function is used to stop all digital part power.
 *
 * @param base ADC base pointer.
 * @param clockDown - true:  Clock down.
 *                  - false: Clock running.
 */
void ADC_SetClockDownCmd(ADC_Type* base, bool clockDown);

/*!
 * @brief This function is used to power down ADC analogue core.
 *        Before entering into stop-mode, power down ADC analogue core first.
 * @param base ADC base pointer.
 * @param powerDown - true:  Power down the ADC analogue core.
 *                  - false: Do not power down the ADC analogue core.
 */
void ADC_SetPowerDownCmd(ADC_Type* base, bool powerDown);

/*@}*/

/*!
 * @name ADC Convert Channel Initialization and Configuration functions.
 * @{
 */

/*!
 * @brief Initialize ADC Logic channel with initialize structure.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param chInitConfig ADC logic channel initialize structure.
 */
void ADC_LogicChInit(ADC_Type* base, uint8_t logicCh, adc_logic_ch_init_config_t* chInitConfig);

/*!
 * @brief Reset target ADC logic channel registers to default value.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 */
void ADC_LogicChDeinit(ADC_Type* base, uint8_t logicCh);

/*!
 * @brief Select input channel for target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param inputCh Input channel selection for target logic channel(vary from 0 to 15).
 */
void ADC_SelectInputCh(ADC_Type* base, uint8_t logicCh, uint8_t inputCh);

/*!
 * @brief Set ADC conversion rate of target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param convertRate ADC conversion rate in Hz.
 */
void ADC_SetConvertRate(ADC_Type* base, uint8_t logicCh, uint32_t convertRate);

/*!
 * @brief Set work state of hardware average feature of target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param enable - true: Enable hardware average.
 *               - faluse: Disable hardware average.
 */
void ADC_SetAverageCmd(ADC_Type* base, uint8_t logicCh, bool enable);

/*!
 * @brief Set hardware average number of target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param avgNum hardware average number(should select from _adc_average_number enumeration).
 */
void ADC_SetAverageNum(ADC_Type* base, uint8_t logicCh, uint8_t avgNum);

/*@}*/

/*!
 * @name ADC Conversion Control functions.
 * @{
 */

/*!
 * @brief Set continuous convert work mode of target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param enable - true:  Enable continuous convert.
 *               - false: Disable continuous convert.
 */
void ADC_SetConvertCmd(ADC_Type* base, uint8_t logicCh, bool enable);

/*!
 * @brief Trigger single time convert on target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 */
void ADC_TriggerSingleConvert(ADC_Type* base, uint8_t logicCh);

/*!
 * @brief Get 12-bit length right aligned convert result.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @return convert result on target logic channel.
 */
uint16_t ADC_GetConvertResult(ADC_Type* base, uint8_t logicCh);

/*@}*/

/*!
 * @name ADC Comparer Control functions.
 * @{
 */

/*!
 * @brief Set the work mode of ADC module build-in comparer on target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param cmpMode Comparer work mode selected from _adc_compare_mode enumeration.
 */
void ADC_SetCmpMode(ADC_Type* base, uint8_t logicCh, uint8_t cmpMode);

/*!
 * @brief Set ADC module build-in comparer high threshold on target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param threshold Comparer threshold in 12-bit unsigned int formate.
 */
void ADC_SetCmpHighThres(ADC_Type* base, uint8_t logicCh, uint16_t threshold);

/*!
 * @brief Set ADC module build-in comparer low threshold on target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param threshold Comparer threshold in 12-bit unsigned int formate.
 */
void ADC_SetCmpLowThres(ADC_Type* base, uint8_t logicCh, uint16_t threshold);

/*!
 * @brief Set the working mode of ADC module auto disable feature on target logic channel.
 *        This feature can disable continuous conversion when CMP condition matched.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param enable  - true:  Enable Auto Disable feature.
 *                - false: Disable Auto Disable feature.
 */
void ADC_SetAutoDisableCmd(ADC_Type* base, uint8_t logicCh, bool enable);

/*@}*/

/*!
 * @name Interrupt and Flag control functions.
 * @{
 */

/*!
 * @brief Enables or disables ADC interrupt requests.
 *
 * @param base ADC base pointer.
 * @param intSource ADC interrupt sources to config.
 * @param enable   Pass true to enable interrupt, false to disable.
 */
void ADC_SetIntCmd(ADC_Type* base, uint32_t intSource, bool enable);

/*!
 * @brief Enables or disables ADC interrupt flag when interrupt condition met.
 *
 * @param base ADC base pointer.
 * @param intSignal ADC interrupt signals to config.
 * @param intSignal Should be select from _adc_interrupt enumeration.
 */
void ADC_SetIntSigCmd(ADC_Type* base, uint32_t intSignal, bool enable);

/*!
 * @brief Gets the ADC status flag state.
 *
 * @param base ADC base pointer.
 * @param flags ADC status flag mask defined in _adc_status_flag enumeration.
 * @return ADC status, each bit represents one status flag
 */
static inline uint32_t ADC_GetStatusFlag(ADC_Type* base, uint32_t flags)
{
    return (ADC_INT_STATUS_REG(base) & flags);
}

/*!
 * @brief Clear one or more ADC status flag state.
 *
 * @param base ADC base pointer.
 * @param flags ADC status flag mask defined in _adc_status_flag enumeration.
 */
static inline void ADC_ClearStatusFlag(ADC_Type* base, uint32_t flags)
{
    ADC_INT_STATUS_REG(base) &= ~flags;
}

/*@}*/

/*!
 * @name DMA & FIFO control functions.
 * @{
 */

/*!
 * @brief Set the reset state of ADC internal DMA part.
 *
 * @param base ADC base pointer.
 * @param active - true  :Reset the DMA and DMA FIFO return to its reset value.
 *               - false :de-active DMA reset. 
 */
void ADC_SetDmaReset(ADC_Type* base, bool active);

/*!
 * @brief Set the work mode of ADC DMA part.
 *
 * @param base ADC base pointer.
 * @param enable - true  :Enable DMA, the data in DMA FIFO should move by SDMA.
 *               - false :Disable DMA, the data in DMA FIFO can only move by CPU.
 */
void ADC_SetDmaCmd(ADC_Type* base, bool enable);

/*!
 * @brief Set the work mode of ADC DMA FIFO part.
 *
 * @param base ADC base pointer.
 * @param enable - true  :Enable DMA FIFO.
 *               - false :Disable DMA FIFO.
 */
void ADC_SetDmaFifoCmd(ADC_Type* base, bool enable);

/*!
 * @brief Select the logic channel that will use DMA transfer.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 */
static inline void ADC_SetDmaCh(ADC_Type* base, uint32_t logicCh)
{
    assert(logicCh <= adcLogicChD);
    ADC_DMA_FIFO_REG(base) = (ADC_DMA_FIFO_REG(base) & ~ADC_DMA_FIFO_DMA_CH_SEL_MASK) | \
                             ADC_DMA_FIFO_DMA_CH_SEL(logicCh);
}

/*!
 * @brief Set the DMA request trigger watermark.
 *
 * @param base ADC base pointer.
 * @param watermark DMA request trigger watermark.
 */
static inline void ADC_SetDmaWatermark(ADC_Type* base, uint32_t watermark)
{
    assert(watermark <= 0x1FF);
    ADC_DMA_FIFO_REG(base) = (ADC_DMA_FIFO_REG(base) & ~ADC_DMA_FIFO_DMA_WM_LVL_MASK) | \
                             ADC_DMA_FIFO_DMA_WM_LVL(watermark);
}

/*!
 * @brief Get the convert result from DMA FIFO.
 *        Data position:
 *            DMA_FIFO_DATA1(27~16bits)
 *            DMA_FIFO_DATA0(11~0bits)
 *
 * @param base ADC base pointer.
 * @return Get 2 ADC transfer result from DMA FIFO.
 */
static inline uint32_t ADC_GetFifoData(ADC_Type* base)
{
    return ADC_DMA_FIFO_DAT_REG(base);
}

/*!
 * @brief Get the DMA FIFO full status
 *
 * @param base ADC base pointer.
 * @return - true:  DMA FIFO full
 *         - false: DMA FIFO not full
 */
static inline bool ADC_IsFifoFull(ADC_Type* base)
{
    return (bool)(ADC_FIFO_STATUS_REG(base) & ADC_FIFO_STATUS_FIFO_FULL_MASK);
}

/*!
 * @brief Get the DMA FIFO empty status
 *
 * @param base ADC base pointer.
 * @return - true:  DMA FIFO empty
 *         - false: DMA FIFO not empty
 */
static inline bool ADC_IsFifoEmpty(ADC_Type* base)
{
    return (bool)(ADC_FIFO_STATUS_REG(base) & ADC_FIFO_STATUS_FIFO_EMPTY_MASK);
}

/*!
 * @brief Get the entries number in DMA FIFO.
 *
 * @param base ADC base pointer.
 * @return The numbers of data in DMA FIFO.
 */
static inline uint8_t ADC_GetFifoEntries(ADC_Type* base)
{
    return ADC_FIFO_STATUS_REG(base) & ADC_FIFO_STATUS_FIFO_ENTRIES_MASK;
}

/*@}*/

#ifdef __cplusplus
}
#endif

/*! @}*/

#endif /* __ADC_IMX7D_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
