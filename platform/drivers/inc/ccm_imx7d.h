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

#ifndef __CCM_IMX7D_H__
#define __CCM_IMX7D_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include "device_imx.h"

/*!
 * @addtogroup ccm_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CCM_REG_OFF(root, off)            (*((volatile uint32_t *)((uint32_t)root + off)))
#define CCM_REG(root)                     CCM_REG_OFF(root, 0)
#define CCM_REG_SET(root)                 CCM_REG_OFF(root, 4)
#define CCM_REG_CLR(root)                 CCM_REG_OFF(root, 8)

/*!
 * @brief Root control names for root clock setting.
 */
enum _ccm_root_control {
    ccmRootM4           = (uint32_t)(&CCM_TARGET_ROOT1),
    ccmRootAxi          = (uint32_t)(&CCM_TARGET_ROOT16),
    ccmRootAhb          = (uint32_t)(&CCM_TARGET_ROOT32),
    ccmRootIpg          = (uint32_t)(&CCM_TARGET_ROOT33),
    ccmRootQspi         = (uint32_t)(&CCM_TARGET_ROOT85),
    ccmRootCan1         = (uint32_t)(&CCM_TARGET_ROOT89),
    ccmRootCan2         = (uint32_t)(&CCM_TARGET_ROOT90),
    ccmRootI2c1         = (uint32_t)(&CCM_TARGET_ROOT91),
    ccmRootI2c2         = (uint32_t)(&CCM_TARGET_ROOT92),
    ccmRootI2c3         = (uint32_t)(&CCM_TARGET_ROOT93),
    ccmRootI2c4         = (uint32_t)(&CCM_TARGET_ROOT94),
    ccmRootUart1        = (uint32_t)(&CCM_TARGET_ROOT95),
    ccmRootUart2        = (uint32_t)(&CCM_TARGET_ROOT96),
    ccmRootUart3        = (uint32_t)(&CCM_TARGET_ROOT97),
    ccmRootUart4        = (uint32_t)(&CCM_TARGET_ROOT98),
    ccmRootUart5        = (uint32_t)(&CCM_TARGET_ROOT99),
    ccmRootUart6        = (uint32_t)(&CCM_TARGET_ROOT100),
    ccmRootUart7        = (uint32_t)(&CCM_TARGET_ROOT101),
    ccmRootEcspi1       = (uint32_t)(&CCM_TARGET_ROOT102),
    ccmRootEcspi2       = (uint32_t)(&CCM_TARGET_ROOT103),
    ccmRootEcspi3       = (uint32_t)(&CCM_TARGET_ROOT104),
    ccmRootEcspi4       = (uint32_t)(&CCM_TARGET_ROOT105),
    ccmRootFtm1         = (uint32_t)(&CCM_TARGET_ROOT110),
    ccmRootFtm2         = (uint32_t)(&CCM_TARGET_ROOT111),
    ccmRootGpt1         = (uint32_t)(&CCM_TARGET_ROOT114),
    ccmRootGpt2         = (uint32_t)(&CCM_TARGET_ROOT115),
    ccmRootGpt3         = (uint32_t)(&CCM_TARGET_ROOT116),
    ccmRootGpt4         = (uint32_t)(&CCM_TARGET_ROOT117),
    ccmRootWdog         = (uint32_t)(&CCM_TARGET_ROOT119)
};

/*!
 * @brief Clock source enumeration for M4 core.
 */
enum _ccm_rootmux_m4 {
    ccmRootmuxM4Osc24m       = 0U,
    ccmRootmuxM4SysPllDiv2   = 1U,
    ccmRootmuxM4EnetPll250m  = 2U,
    ccmRootmuxM4SysPllPfd2   = 3U,
    ccmRootmuxM4DdrPllDiv2   = 4U,
    ccmRootmuxM4AudioPll     = 5U,
    ccmRootmuxM4VideoPll     = 6U,
    ccmRootmuxM4UsbPll       = 7U
};

/*!
 * @brief Clock source enumeration for AXI bus.
 */
enum _ccm_rootmux_axi {
    ccmRootmuxAxiOsc24m      = 0U,
    ccmRootmuxAxiSysPllPfd1  = 1U,
    ccmRootmuxAxiDdrPllDiv2  = 2U,
    ccmRootmuxAxiEnetPll250m = 3U,
    ccmRootmuxAxiSysPllPfd5  = 4U,
    ccmRootmuxAxiAudioPll    = 5U,
    ccmRootmuxAxiVideoPll    = 6U,
    ccmRootmuxAxiSysPllPfd7  = 7U
};

/*!
 * @brief Clock source enumeration for AHB bus.
 */
enum _ccm_rootmux_ahb {
    ccmRootmuxAhbOsc24m      = 0U,
    ccmRootmuxAhbSysPllPfd2  = 1U,
    ccmRootmuxAhbDdrPllDiv2  = 2U,
    ccmRootmuxAhbSysPllPfd0  = 3U,
    ccmRootmuxAhbEnetPll125m = 4U,
    ccmRootmuxAhbUsbPll      = 5U,
    ccmRootmuxAhbAudioPll    = 6U,
    ccmRootmuxAhbVideoPll    = 7U
};

/*!
 * @brief Clock source enumeration for IPG bus.
 */
enum _ccm_rootmux_ipg {
    ccmRootmuxIpgAHB = 0U
};

/*!
 * @brief Clock source enumeration for QSPI peripheral.
 */
enum _ccm_rootmux_qspi {
    ccmRootmuxQspiOsc24m      = 0U,
    ccmRootmuxQspiSysPllPfd4  = 1U,
    ccmRootmuxQspiDdrPllDiv2  = 2U,
    ccmRootmuxQspiEnetPll500m = 3U,
    ccmRootmuxQspiSysPllPfd3  = 4U,
    ccmRootmuxQspiSysPllPfd2  = 5U,
    ccmRootmuxQspiSysPllPfd6  = 6U,
    ccmRootmuxQspiSysPllPfd7  = 7U
};

/*!
 * @brief Clock source enumeration for CAN peripheral.
 */
enum _ccm_rootmux_can {
    ccmRootmuxCanOsc24m       = 0U,
    ccmRootmuxCanSysPllDiv4   = 1U,
    ccmRootmuxCanDdrPllDiv2   = 2U,
    ccmRootmuxCanSysPllDiv1   = 3U,
    ccmRootmuxCanEnetPll40m   = 4U,
    ccmRootmuxCanUsbPll       = 5U,
    ccmRootmuxCanExtClk1      = 6U,
    ccmRootmuxCanExtClk34     = 7U
};

/*!
 * @brief Clock source enumeration for ECSPI peripheral.
 */
enum _ccm_rootmux_ecspi {
    ccmRootmuxEcspiOsc24m      = 0U,
    ccmRootmuxEcspiSysPllDiv2  = 1U,
    ccmRootmuxEcspiEnetPll40m  = 2U,
    ccmRootmuxEcspiSysPllDiv4  = 3U,
    ccmRootmuxEcspiSysPllDiv1  = 4U,
    ccmRootmuxEcspiSysPllPfd4  = 5U,
    ccmRootmuxEcspiEnetPll250m = 6U,
    ccmRootmuxEcspiUsbPll      = 7U
};

/*!
 * @brief Clock source enumeration for I2C peripheral.
 */
enum _ccm_rootmux_i2c {
    ccmRootmuxI2cOsc24m         = 0U,
    ccmRootmuxI2cSysPllDiv4     = 1U,
    ccmRootmuxI2cEnetPll50m     = 2U,
    ccmRootmuxI2cDdrPllDiv2     = 3U,
    ccmRootmuxI2cAudioPll       = 4U,
    ccmRootmuxI2cVideoPll       = 5U,
    ccmRootmuxI2cUsbPll         = 6U,
    ccmRootmuxI2cSysPllPfd2Div2 = 7U
};

/*!
 * @brief Clock source enumeration for UART peripheral.
 */
enum _ccm_rootmux_uart {
    ccmRootmuxUartOsc24m        = 0U,
    ccmRootmuxUartSysPllDiv2    = 1U,
    ccmRootmuxUartEnetPll40m    = 2U,
    ccmRootmuxUartEnetPll100m   = 3U,
    ccmRootmuxUartSysPllDiv1    = 4U,
    ccmRootmuxUartExtClk2       = 5U,
    ccmRootmuxUartExtClk34      = 6U,
    ccmRootmuxUartUsbPll        = 7U
};

/*!
 * @brief Clock source enumeration for FlexTimer peripheral.
 */
enum _ccm_rootmux_ftm {
    ccmRootmuxFtmOsc24m         = 0U,
    ccmRootmuxFtmEnetPll100m    = 1U,
    ccmRootmuxFtmSysPllDiv4     = 2U,
    ccmRootmuxFtmEnetPll40m     = 3U,
    ccmRootmuxFtmAudioPll       = 4U,
    ccmRootmuxFtmExtClk3        = 5U,
    ccmRootmuxFtmRef1m          = 6U,
    ccmRootmuxFtmVideoPll       = 7U
};

/*!
 * @brief Clock source enumeration for GPT peripheral.
 */
enum _ccm_rootmux_gpt {
    ccmRootmuxGptOsc24m         = 0U,
    ccmRootmuxGptEnetPll100m    = 1U,
    ccmRootmuxGptSysPllPfd0     = 2U,
    ccmRootmuxGptEnetPll40m     = 3U,
    ccmRootmuxGptVideoPll       = 4U,
    ccmRootmuxGptRef1m          = 5U,
    ccmRootmuxGptAudioPll       = 6U,
    ccmRootmuxGptExtClk         = 7U
};

/*!
 * @brief Clock source enumeration for WDOG peripheral.
 */
enum _ccm_rootmux_wdog {
    ccmRootmuxWdogOsc24m         = 0U,
    ccmRootmuxWdogSysPllPfd2Div2 = 1U,
    ccmRootmuxWdogSysPllDiv4     = 2U,
    ccmRootmuxWdogDdrPllDiv2     = 3U,
    ccmRootmuxWdogEnetPll125m    = 4U,
    ccmRootmuxWdogUsbPll         = 5U,
    ccmRootmuxWdogRef1m          = 6U,
    ccmRootmuxWdogSysPllPfd1Div2 = 7U
};

/*!
 * @brief CCM PLL gate control
 */
enum _ccm_pll_gate {
    ccmPllGateCkil       = (uint32_t)(&CCM_PLL_CTRL0_REG(CCM_BASE_PTR)),
    ccmPllGateArm        = (uint32_t)(&CCM_PLL_CTRL1_REG(CCM_BASE_PTR)),
    ccmPllGateArmDiv1    = (uint32_t)(&CCM_PLL_CTRL2_REG(CCM_BASE_PTR)),
    ccmPllGateDdr        = (uint32_t)(&CCM_PLL_CTRL3_REG(CCM_BASE_PTR)),
    ccmPllGateDdrDiv1    = (uint32_t)(&CCM_PLL_CTRL4_REG(CCM_BASE_PTR)),
    ccmPllGateDdrDiv2    = (uint32_t)(&CCM_PLL_CTRL5_REG(CCM_BASE_PTR)),
    ccmPllGateSys        = (uint32_t)(&CCM_PLL_CTRL6_REG(CCM_BASE_PTR)),
    ccmPllGateSysDiv1    = (uint32_t)(&CCM_PLL_CTRL7_REG(CCM_BASE_PTR)),
    ccmPllGateSysDiv2    = (uint32_t)(&CCM_PLL_CTRL8_REG(CCM_BASE_PTR)),
    ccmPllGateSysDiv4    = (uint32_t)(&CCM_PLL_CTRL9_REG(CCM_BASE_PTR)),
    ccmPllGatePfd0       = (uint32_t)(&CCM_PLL_CTRL10_REG(CCM_BASE_PTR)),
    ccmPllGatePfd0Div2   = (uint32_t)(&CCM_PLL_CTRL11_REG(CCM_BASE_PTR)),
    ccmPllGatePfd1       = (uint32_t)(&CCM_PLL_CTRL12_REG(CCM_BASE_PTR)),
    ccmPllGatePfd1Div2   = (uint32_t)(&CCM_PLL_CTRL13_REG(CCM_BASE_PTR)),
    ccmPllGatePfd2       = (uint32_t)(&CCM_PLL_CTRL14_REG(CCM_BASE_PTR)),
    ccmPllGatePfd2Div2   = (uint32_t)(&CCM_PLL_CTRL15_REG(CCM_BASE_PTR)),
    ccmPllGatePfd3       = (uint32_t)(&CCM_PLL_CTRL16_REG(CCM_BASE_PTR)),
    ccmPllGatePfd4       = (uint32_t)(&CCM_PLL_CTRL17_REG(CCM_BASE_PTR)),
    ccmPllGatePfd5       = (uint32_t)(&CCM_PLL_CTRL18_REG(CCM_BASE_PTR)),
    ccmPllGatePfd6       = (uint32_t)(&CCM_PLL_CTRL19_REG(CCM_BASE_PTR)),
    ccmPllGatePfd7       = (uint32_t)(&CCM_PLL_CTRL20_REG(CCM_BASE_PTR)),
    ccmPllGateEnet       = (uint32_t)(&CCM_PLL_CTRL21_REG(CCM_BASE_PTR)),
    ccmPllGateEnet500m   = (uint32_t)(&CCM_PLL_CTRL22_REG(CCM_BASE_PTR)),
    ccmPllGateEnet250m   = (uint32_t)(&CCM_PLL_CTRL23_REG(CCM_BASE_PTR)),
    ccmPllGateEnet125m   = (uint32_t)(&CCM_PLL_CTRL24_REG(CCM_BASE_PTR)),
    ccmPllGateEnet100m   = (uint32_t)(&CCM_PLL_CTRL25_REG(CCM_BASE_PTR)),
    ccmPllGateEnet50m    = (uint32_t)(&CCM_PLL_CTRL26_REG(CCM_BASE_PTR)),
    ccmPllGateEnet40m    = (uint32_t)(&CCM_PLL_CTRL27_REG(CCM_BASE_PTR)),
    ccmPllGateEnet25m    = (uint32_t)(&CCM_PLL_CTRL28_REG(CCM_BASE_PTR)),
    ccmPllGateAudio      = (uint32_t)(&CCM_PLL_CTRL29_REG(CCM_BASE_PTR)),
    ccmPllGateAudioDiv1  = (uint32_t)(&CCM_PLL_CTRL30_REG(CCM_BASE_PTR)),
    ccmPllGateVideo      = (uint32_t)(&CCM_PLL_CTRL31_REG(CCM_BASE_PTR)),
    ccmPllGateVideoDiv1  = (uint32_t)(&CCM_PLL_CTRL32_REG(CCM_BASE_PTR))
};

/*!
 * @brief CCM CCGR gate control
 */
enum _ccm_ccgr_gate {
    ccmCcgrGateIpmux1    = (uint32_t)(&CCM_CCGR10),
    ccmCcgrGateIpmux2    = (uint32_t)(&CCM_CCGR11),
    ccmCcgrGateIpmux3    = (uint32_t)(&CCM_CCGR12),
    ccmCcgrGateOcram     = (uint32_t)(&CCM_CCGR17),
    ccmCcgrGateOcramS    = (uint32_t)(&CCM_CCGR18),
    ccmCcgrGateQspi      = (uint32_t)(&CCM_CCGR21),
    ccmCcgrGateAdc       = (uint32_t)(&CCM_CCGR32),
    ccmCcgrGateRdc       = (uint32_t)(&CCM_CCGR38),
    ccmCcgrGateMu        = (uint32_t)(&CCM_CCGR39),
    ccmCcgrGateSemaHs    = (uint32_t)(&CCM_CCGR40),
    ccmCcgrGateSema1     = (uint32_t)(&CCM_CCGR64),
    ccmCcgrGateSema2     = (uint32_t)(&CCM_CCGR65),
    ccmCcgrGateCan1      = (uint32_t)(&CCM_CCGR116),
    ccmCcgrGateCan2      = (uint32_t)(&CCM_CCGR117),
    ccmCcgrGateEcspi1    = (uint32_t)(&CCM_CCGR120),
    ccmCcgrGateEcspi2    = (uint32_t)(&CCM_CCGR121),
    ccmCcgrGateEcspi3    = (uint32_t)(&CCM_CCGR122),
    ccmCcgrGateEcspi4    = (uint32_t)(&CCM_CCGR123),
    ccmCcgrGateGpt1      = (uint32_t)(&CCM_CCGR124),
    ccmCcgrGateGpt2      = (uint32_t)(&CCM_CCGR125),
    ccmCcgrGateGpt3      = (uint32_t)(&CCM_CCGR126),
    ccmCcgrGateGpt4      = (uint32_t)(&CCM_CCGR127),
    ccmCcgrGateI2c1      = (uint32_t)(&CCM_CCGR136),
    ccmCcgrGateI2c2      = (uint32_t)(&CCM_CCGR137),
    ccmCcgrGateI2c3      = (uint32_t)(&CCM_CCGR138),
    ccmCcgrGateI2c4      = (uint32_t)(&CCM_CCGR139),
    ccmCcgrGateUart1     = (uint32_t)(&CCM_CCGR148),
    ccmCcgrGateUart2     = (uint32_t)(&CCM_CCGR149),
    ccmCcgrGateUart3     = (uint32_t)(&CCM_CCGR150),
    ccmCcgrGateUart4     = (uint32_t)(&CCM_CCGR151),
    ccmCcgrGateUart5     = (uint32_t)(&CCM_CCGR152),
    ccmCcgrGateUart6     = (uint32_t)(&CCM_CCGR153),
    ccmCcgrGateUart7     = (uint32_t)(&CCM_CCGR154),
    ccmCcgrGateWdog1     = (uint32_t)(&CCM_CCGR156),
    ccmCcgrGateWdog2     = (uint32_t)(&CCM_CCGR157),
    ccmCcgrGateWdog3     = (uint32_t)(&CCM_CCGR158),
    ccmCcgrGateWdog4     = (uint32_t)(&CCM_CCGR159),
    ccmCcgrGateGpio1     = (uint32_t)(&CCM_CCGR160),
    ccmCcgrGateGpio2     = (uint32_t)(&CCM_CCGR161),
    ccmCcgrGateGpio3     = (uint32_t)(&CCM_CCGR162),
    ccmCcgrGateGpio4     = (uint32_t)(&CCM_CCGR163),
    ccmCcgrGateGpio5     = (uint32_t)(&CCM_CCGR164),
    ccmCcgrGateGpio6     = (uint32_t)(&CCM_CCGR165),
    ccmCcgrGateGpio7     = (uint32_t)(&CCM_CCGR166),
    ccmCcgrGateIomux     = (uint32_t)(&CCM_CCGR168),
    ccmCcgrGateIomuxLpsr = (uint32_t)(&CCM_CCGR169),
    ccmCcgrGatePwm1      = (uint32_t)(&CCM_CCGR132),
    ccmCcgrGatePwm2      = (uint32_t)(&CCM_CCGR133),
    ccmCcgrGatePwm3      = (uint32_t)(&CCM_CCGR134),
    ccmCcgrGatePwm4      = (uint32_t)(&CCM_CCGR135)
};

/*!
 * @brief CCM gate control value
 */
enum _ccm_gate_value {
    ccmClockNotNeeded        = 0x0U,      /*!< Clock always disabled.*/
    ccmClockNeededRun        = 0x1111U,   /*!< Clock enabled when CPU is running.*/
    ccmClockNeededRunWait    = 0x2222U,   /*!< Clock enabled when CPU is running or in WAIT mode.*/
    ccmClockNeededAll        = 0x3333U    /*!< Clock always enabled.*/
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name CCM Root Setting
 * @{
 */

/*!
 * @brief Set clock root mux
 *
 * @param base CCM base pointer.
 * @param ccmRoot Root control (see _ccm_root_control enumeration)
 * @param mux Root mux value (see _ccm_rootmux_xxx enumeration)
 */
static inline void CCM_SetRootMux(CCM_Type * base, uint32_t ccmRoot, uint32_t mux)
{
    CCM_REG(ccmRoot) = (CCM_REG(ccmRoot) & (~CCM_TARGET_ROOT0_MUX_MASK)) |
                              CCM_TARGET_ROOT0_MUX(mux);
}

/*!
 * @brief Get clock root mux
 *
 * @param base CCM base pointer.
 * @param ccmRoot Root control (see _ccm_root_control enumeration)
 * @return root mux value (see _ccm_rootmux_xxx enumeration)
 */
static inline uint32_t CCM_GetRootMux(CCM_Type * base, uint32_t ccmRoot)
{
    return (CCM_REG(ccmRoot) & CCM_TARGET_ROOT0_MUX_MASK) >> CCM_TARGET_ROOT0_MUX_SHIFT;
}

/*!
 * @brief Enable clock root
 *
 * @param base CCM base pointer.
 * @param ccmRoot Root control (see _ccm_root_control enumeration)
 */
static inline void CCM_EnableRoot(CCM_Type * base, uint32_t ccmRoot)
{
    CCM_REG_SET(ccmRoot) = CCM_TARGET_ROOT0_SET_ENABLE_MASK;
}

/*!
 * @brief Disable clock root
 *
 * @param base CCM base pointer.
 * @param ccmRoot Root control (see _ccm_root_control enumeration)
 */
static inline void CCM_DisableRoot(CCM_Type * base, uint32_t ccmRoot)
{
    CCM_REG_CLR(ccmRoot) = CCM_TARGET_ROOT0_CLR_ENABLE_MASK;
}

/*!
 * @brief Check whether clock root is enabled
 *
 * @param base CCM base pointer.
 * @param ccmRoot Root control (see _ccm_root_control enumeration)
 * @return CCM root enabled or not (true: enabled, false: disabled)
 */
static inline bool CCM_IsRootEnabled(CCM_Type * base, uint32_t ccmRoot)
{
    return (bool)(CCM_REG(ccmRoot) & CCM_TARGET_ROOT0_ENABLE_MASK);
}

/*!
 * @brief Set root clock divider
 *
 * @param base CCM base pointer.
 * @param ccmRoot Root control (see _ccm_root_control enumeration)
 * @param pre Pre divider value (0-7, divider=n+1)
 * @param post Post divider value (0-63, divider=n+1)
 */
void CCM_SetRootDivider(CCM_Type * base, uint32_t ccmRoot, uint32_t pre, uint32_t post);

/*!
 * @brief Get root clock divider
 *
 * @param base CCM base pointer.
 * @param ccmRoot Root control (see _ccm_root_control enumeration)
 * @param pre Pointer to pre divider value store address
 * @param post Pointer to post divider value store address
 */
void CCM_GetRootDivider(CCM_Type * base, uint32_t ccmRoot, uint32_t *pre, uint32_t *post);

/*!
 * @brief Update clock root in one step, for dynamical clock switching
 *
 * @param base CCM base pointer.
 * @param ccmRoot Root control (see _ccm_root_control enumeration)
 * @param root mux value (see _ccm_rootmux_xxx enumeration)
 * @param pre Pre divider value (0-7, divider=n+1)
 * @param post Post divider value (0-63, divider=n+1)
 */
void CCM_UpdateRoot(CCM_Type * base, uint32_t ccmRoot, uint32_t mux, uint32_t pre, uint32_t post);

/*@}*/

/*!
 * @name CCM Gate Control
 * @{
 */

/*!
 * @brief Set PLL or CCGR gate control
 *
 * @param base CCM base pointer.
 * @param ccmGate Gate control (see _ccm_pll_gate and _ccm_ccgr_gate enumeration)
 * @param control Gate control value (see _ccm_gate_value)
 */
static inline void CCM_ControlGate(CCM_Type * base, uint32_t ccmGate, uint32_t control)
{
    CCM_REG(ccmGate) = control;
}

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __CCM_IMX7D_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
