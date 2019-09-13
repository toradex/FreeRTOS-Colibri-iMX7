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

#ifndef __CCM_ANALOG_IMX7D_H__
#define __CCM_ANALOG_IMX7D_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include "device_imx.h"

/*!
 * @addtogroup ccm_analog_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CCM_ANALOG_TUPLE(reg, shift)               ((offsetof(CCM_ANALOG_Type, reg) & 0xFFFF) | ((shift) << 16))
#define CCM_ANALOG_TUPLE_REG_OFF(base, tuple, off) (*((volatile uint32_t *)((uint32_t)base + ((tuple) & 0xFFFF) + off)))
#define CCM_ANALOG_TUPLE_REG(base, tuple)          CCM_ANALOG_TUPLE_REG_OFF(base, tuple, 0)
#define CCM_ANALOG_TUPLE_REG_SET(base, tuple)      CCM_ANALOG_TUPLE_REG_OFF(base, tuple, 4)
#define CCM_ANALOG_TUPLE_REG_CLR(base, tuple)      CCM_ANALOG_TUPLE_REG_OFF(base, tuple, 8)
#define CCM_ANALOG_TUPLE_SHIFT(tuple)              (((tuple) >> 16) & 0x1F)

/*!
 * @brief PLL control names for PLL power/bypass/lock operations.
 *
 * These constants define the PLL control names for PLL power/bypass/lock operations.\n
 * 0:15   : REG offset to CCM_ANALOG_BASE in bytes\n
 * 16:20  : Powerdown bit shift
 */
enum _ccm_analog_pll_control {
    ccmAnalogPllArmControl             = CCM_ANALOG_TUPLE(PLL_ARM, CCM_ANALOG_PLL_ARM_POWERDOWN_SHIFT),
    ccmAnalogPllDdrControl             = CCM_ANALOG_TUPLE(PLL_DDR, CCM_ANALOG_PLL_DDR_POWERDOWN_SHIFT),
    ccmAnalogPll480Control             = CCM_ANALOG_TUPLE(PLL_480, CCM_ANALOG_PLL_480_POWERDOWN_SHIFT),
    ccmAnalogPllEnetControl            = CCM_ANALOG_TUPLE(PLL_ENET, CCM_ANALOG_PLL_ENET_POWERDOWN_SHIFT),
    ccmAnalogPllAudioControl           = CCM_ANALOG_TUPLE(PLL_AUDIO, CCM_ANALOG_PLL_AUDIO_POWERDOWN_SHIFT),
    ccmAnalogPllVideoControl           = CCM_ANALOG_TUPLE(PLL_VIDEO, CCM_ANALOG_PLL_VIDEO_POWERDOWN_SHIFT)
};

/*!
 * @brief PLL clock names for clock enable/disable settings.
 *
 * These constants define the PLL clock names for PLL clock enable/disable operations.\n
 * 0:15   : REG offset to CCM_ANALOG_BASE in bytes\n
 * 16:20  : Clock enable bit shift
 */
enum _ccm_analog_pll_clock {
    ccmAnalogPllArmClock               = CCM_ANALOG_TUPLE(PLL_ARM, CCM_ANALOG_PLL_ARM_ENABLE_CLK_SHIFT),

    ccmAnalogPllDdrClock               = CCM_ANALOG_TUPLE(PLL_DDR, CCM_ANALOG_PLL_DDR_ENABLE_CLK_SHIFT),
    ccmAnalogPllDdrDiv2Clock           = CCM_ANALOG_TUPLE(PLL_DDR, CCM_ANALOG_PLL_DDR_DIV2_ENABLE_CLK_SHIFT),

    ccmAnalogPll480Clock               = CCM_ANALOG_TUPLE(PLL_480, CCM_ANALOG_PLL_480_ENABLE_CLK_SHIFT),

    ccmAnalogPllEnet25MhzClock         = CCM_ANALOG_TUPLE(PLL_ENET, CCM_ANALOG_PLL_ENET_ENABLE_CLK_25MHZ_SHIFT),
    ccmAnalogPllEnet40MhzClock         = CCM_ANALOG_TUPLE(PLL_ENET, CCM_ANALOG_PLL_ENET_ENABLE_CLK_40MHZ_SHIFT),
    ccmAnalogPllEnet50MhzClock         = CCM_ANALOG_TUPLE(PLL_ENET, CCM_ANALOG_PLL_ENET_ENABLE_CLK_50MHZ_SHIFT),
    ccmAnalogPllEnet100MhzClock        = CCM_ANALOG_TUPLE(PLL_ENET, CCM_ANALOG_PLL_ENET_ENABLE_CLK_100MHZ_SHIFT),
    ccmAnalogPllEnet125MhzClock        = CCM_ANALOG_TUPLE(PLL_ENET, CCM_ANALOG_PLL_ENET_ENABLE_CLK_125MHZ_SHIFT),
    ccmAnalogPllEnet250MhzClock        = CCM_ANALOG_TUPLE(PLL_ENET, CCM_ANALOG_PLL_ENET_ENABLE_CLK_250MHZ_SHIFT),
    ccmAnalogPllEnet500MhzClock        = CCM_ANALOG_TUPLE(PLL_ENET, CCM_ANALOG_PLL_ENET_ENABLE_CLK_500MHZ_SHIFT),

    ccmAnalogPllAudioClock             = CCM_ANALOG_TUPLE(PLL_AUDIO, CCM_ANALOG_PLL_AUDIO_ENABLE_CLK_SHIFT),
    ccmAnalogPllVideoClock             = CCM_ANALOG_TUPLE(PLL_VIDEO, CCM_ANALOG_PLL_VIDEO_ENABLE_CLK_SHIFT)
};

/*!
 * @brief PFD gate names for clock gate settings, clock source is system PLL(PLL_480)
 *
 * These constants define the PFD gate names for PFD clock enable/disable operations.\n
 * 0:15   : REG offset to CCM_ANALOG_BASE in bytes\n
 * 16:20  : Clock gate bit shift
 */
enum _ccm_analog_pfd_clkgate {
    ccmAnalogMainDiv1ClkGate           = CCM_ANALOG_TUPLE(PLL_480, CCM_ANALOG_PLL_480_MAIN_DIV1_CLKGATE_SHIFT),
    ccmAnalogMainDiv2ClkGate           = CCM_ANALOG_TUPLE(PLL_480, CCM_ANALOG_PLL_480_MAIN_DIV2_CLKGATE_SHIFT),
    ccmAnalogMainDiv4ClkGate           = CCM_ANALOG_TUPLE(PLL_480, CCM_ANALOG_PLL_480_MAIN_DIV4_CLKGATE_SHIFT),

    ccmAnalogPfd0Div2ClkGate           = CCM_ANALOG_TUPLE(PLL_480, CCM_ANALOG_PLL_480_PFD0_DIV2_CLKGATE_SHIFT),
    ccmAnalogPfd1Div2ClkGate           = CCM_ANALOG_TUPLE(PLL_480, CCM_ANALOG_PLL_480_PFD1_DIV2_CLKGATE_SHIFT),
    ccmAnalogPfd2Div2ClkGate           = CCM_ANALOG_TUPLE(PLL_480, CCM_ANALOG_PLL_480_PFD2_DIV2_CLKGATE_SHIFT),

    ccmAnalogPfd0Div1ClkGate           = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD0_DIV1_CLKGATE_SHIFT),
    ccmAnalogPfd1Div1ClkGate           = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD1_DIV1_CLKGATE_SHIFT),
    ccmAnalogPfd2Div1ClkGate           = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD2_DIV1_CLKGATE_SHIFT),
    ccmAnalogPfd3Div1ClkGate           = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD3_DIV1_CLKGATE_SHIFT),

    ccmAnalogPfd4Div1ClkGate           = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD4_DIV1_CLKGATE_SHIFT),
    ccmAnalogPfd5Div1ClkGate           = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD5_DIV1_CLKGATE_SHIFT),
    ccmAnalogPfd6Div1ClkGate           = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD6_DIV1_CLKGATE_SHIFT),
    ccmAnalogPfd7Div1ClkGate           = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD7_DIV1_CLKGATE_SHIFT)
};

/*!
 * @brief PFD fraction names for clock fractional divider operations
 *
 * These constants define the PFD fraction names for PFD fractional divider operations.\n
 * 0:15   : REG offset to CCM_ANALOG_BASE in bytes\n
 * 16:20  : Fraction bits shift
 */
enum _ccm_analog_pfd_frac {
    ccmAnalogPfd0Frac                  = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD0_FRAC_SHIFT),
    ccmAnalogPfd1Frac                  = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD1_FRAC_SHIFT),
    ccmAnalogPfd2Frac                  = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD2_FRAC_SHIFT),
    ccmAnalogPfd3Frac                  = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD3_FRAC_SHIFT),

    ccmAnalogPfd4Frac                  = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD4_FRAC_SHIFT),
    ccmAnalogPfd5Frac                  = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD5_FRAC_SHIFT),
    ccmAnalogPfd6Frac                  = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD6_FRAC_SHIFT),
    ccmAnalogPfd7Frac                  = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD7_FRAC_SHIFT)
};

/*!
 * @brief PFD stable names for clock stable query
 *
 * These constants define the PFD stable names for clock stable query.\n
 * 0:15   : REG offset to CCM_ANALOG_BASE in bytes\n
 * 16:20  : Stable bit shift
 */
enum _ccm_analog_pfd_stable {
    ccmAnalogPfd0Stable                = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD0_STABLE_SHIFT),
    ccmAnalogPfd1Stable                = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD1_STABLE_SHIFT),
    ccmAnalogPfd2Stable                = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD2_STABLE_SHIFT),
    ccmAnalogPfd3Stable                = CCM_ANALOG_TUPLE(PFD_480A, CCM_ANALOG_PFD_480A_PFD3_STABLE_SHIFT),

    ccmAnalogPfd4Stable                = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD4_STABLE_SHIFT),
    ccmAnalogPfd5Stable                = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD5_STABLE_SHIFT),
    ccmAnalogPfd6Stable                = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD6_STABLE_SHIFT),
    ccmAnalogPfd7Stable                = CCM_ANALOG_TUPLE(PFD_480B, CCM_ANALOG_PFD_480B_PFD7_STABLE_SHIFT)
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name CCM Analog PLL Operations
 * @{
 */

/*!
 * @brief Power up PLL
 *
 * @param base CCM_ANALOG base pointer.
 * @param pllControl PLL control name (see _ccm_analog_pll_control enumeration)
 */
static inline void CCM_ANALOG_PowerUpPll(CCM_ANALOG_Type * base, uint32_t pllControl)
{
    CCM_ANALOG_TUPLE_REG_CLR(base, pllControl) = 1 << CCM_ANALOG_TUPLE_SHIFT(pllControl);
}

/*!
 * @brief Power down PLL
 *
 * @param base CCM_ANALOG base pointer.
 * @param pllControl PLL control name (see _ccm_analog_pll_control enumeration)
 */
static inline void CCM_ANALOG_PowerDownPll(CCM_ANALOG_Type * base, uint32_t pllControl)
{
    CCM_ANALOG_TUPLE_REG_SET(base, pllControl) = 1 << CCM_ANALOG_TUPLE_SHIFT(pllControl);
}

/*!
 * @brief PLL bypass setting
 *
 * @param base CCM_ANALOG base pointer.
 * @param pllControl PLL control name (see _ccm_analog_pll_control enumeration)
 * @param bypass Bypass the PLL (true: bypass, false: not bypass)
 */
static inline void CCM_ANALOG_SetPllBypass(CCM_ANALOG_Type * base, uint32_t pllControl, bool bypass)
{
    if (bypass)
        CCM_ANALOG_TUPLE_REG_SET(base, pllControl) = CCM_ANALOG_PLL_ARM_BYPASS_MASK;
    else
        CCM_ANALOG_TUPLE_REG_CLR(base, pllControl) = CCM_ANALOG_PLL_ARM_BYPASS_MASK;
}

/*!
 * @brief Check if PLL is bypassed
 *
 * @param base CCM_ANALOG base pointer.
 * @param pllControl PLL control name (see _ccm_analog_pll_control enumeration)
 * @return PLL bypass status (true: bypassed, false: not bypassed)
 */
static inline bool CCM_ANALOG_IsPllBypassed(CCM_ANALOG_Type * base, uint32_t pllControl)
{
    return (bool)(CCM_ANALOG_TUPLE_REG(base, pllControl) & CCM_ANALOG_PLL_ARM_BYPASS_MASK);
}

/*!
 * @brief Check if PLL clock is locked
 *
 * @param base CCM_ANALOG base pointer.
 * @param pllControl PLL control name (see _ccm_analog_pll_control enumeration)
 * @return PLL lock status (true: locked, false: not locked)
 */
static inline bool CCM_ANALOG_IsPllLocked(CCM_ANALOG_Type * base, uint32_t pllControl)
{
    return (bool)(CCM_ANALOG_TUPLE_REG(base, pllControl) & CCM_ANALOG_PLL_ARM_LOCK_MASK);
}

/*!
 * @brief Enable PLL clock
 *
 * @param base CCM_ANALOG base pointer.
 * @param pllClock PLL clock name (see _ccm_analog_pll_clock enumeration)
 */
static inline void CCM_ANALOG_EnablePllClock(CCM_ANALOG_Type * base, uint32_t pllClock)
{
    CCM_ANALOG_TUPLE_REG_SET(base, pllClock) = 1 << CCM_ANALOG_TUPLE_SHIFT(pllClock);
}

/*!
 * @brief Disable PLL clock
 *
 * @param base CCM_ANALOG base pointer.
 * @param pllClock PLL clock name (see _ccm_analog_pll_clock enumeration)
 */
static inline void CCM_ANALOG_DisablePllClock(CCM_ANALOG_Type * base, uint32_t pllClock)
{
    CCM_ANALOG_TUPLE_REG_CLR(base, pllClock) = 1 << CCM_ANALOG_TUPLE_SHIFT(pllClock);
}

/*!
 * @brief Get System PLL (PLL_480) clock frequency
 *
 * @param base CCM_ANALOG base pointer.
 * @return System PLL clock frequency in HZ
 */
uint32_t CCM_ANALOG_GetSysPllFreq(CCM_ANALOG_Type * base);

/*@}*/

/*!
 * @name CCM Analog PFD Operations
 * @{
 */

/*!
 * @brief Enable PFD clock
 *
 * @param base CCM_ANALOG base pointer.
 * @param pfdClkGate PFD clock gate (see _ccm_analog_pfd_clkgate enumeration)
 */
static inline void CCM_ANALOG_EnablePfdClock(CCM_ANALOG_Type * base, uint32_t pfdClkGate)
{
    CCM_ANALOG_TUPLE_REG_CLR(base, pfdClkGate) = 1 << CCM_ANALOG_TUPLE_SHIFT(pfdClkGate);
}

/*!
 * @brief Disable PFD clock
 *
 * @param base CCM_ANALOG base pointer.
 * @param pfdClkGate PFD clock gate (see _ccm_analog_pfd_clkgate enumeration)
 */
static inline void CCM_ANALOG_DisablePfdClock(CCM_ANALOG_Type * base, uint32_t pfdClkGate)
{
    CCM_ANALOG_TUPLE_REG_SET(base, pfdClkGate) = 1 << CCM_ANALOG_TUPLE_SHIFT(pfdClkGate);
}

/*!
 * @brief Check if PFD clock is stable
 *
 * @param base CCM_ANALOG base pointer.
 * @param pfdStable PFD stable identifier (see _ccm_analog_pfd_stable enumeration)
 * @return PFD clock stable status (true: stable, false: not stable)
 */
static inline bool CCM_ANALOG_IsPfdStable(CCM_ANALOG_Type * base, uint32_t pfdStable)
{
    return (bool)(CCM_ANALOG_TUPLE_REG(base, pfdStable) & (1 << CCM_ANALOG_TUPLE_SHIFT(pfdStable)));
}

/*!
 * @brief Set PFD clock fraction
 *
 * @param base CCM_ANALOG base pointer.
 * @param pfdFrac PFD clock fraction (see _ccm_analog_pfd_frac enumeration)
 * @param value PFD clock fraction value
 */
static inline void CCM_ANALOG_SetPfdFrac(CCM_ANALOG_Type * base, uint32_t pfdFrac, uint32_t value)
{
    assert(value >= 12 && value <= 35);
    CCM_ANALOG_TUPLE_REG_CLR(base, pfdFrac) = CCM_ANALOG_PFD_480A_CLR_PFD0_FRAC_MASK << CCM_ANALOG_TUPLE_SHIFT(pfdFrac);
    CCM_ANALOG_TUPLE_REG_SET(base, pfdFrac) = value << CCM_ANALOG_TUPLE_SHIFT(pfdFrac);
}

/*!
 * @brief Get PFD clock fraction
 *
 * @param base CCM_ANALOG base pointer.
 * @param pfdFrac PFD clock fraction (see _ccm_analog_pfd_frac enumeration)
 * @return PFD clock fraction value
 */
static inline uint32_t CCM_ANALOG_GetPfdFrac(CCM_ANALOG_Type * base, uint32_t pfdFrac)
{
    return (CCM_ANALOG_TUPLE_REG(base, pfdFrac) >> CCM_ANALOG_TUPLE_SHIFT(pfdFrac)) & CCM_ANALOG_PFD_480A_PFD0_FRAC_MASK;
}

/*!
 * @brief Get PFD clock frequency
 *
 * @param base CCM_ANALOG base pointer.
 * @param pfdFrac PFD clock fraction (see _ccm_analog_pfd_frac enumeration)
 * @return PFD clock frequency in HZ
 */
uint32_t CCM_ANALOG_GetPfdFreq(CCM_ANALOG_Type * base, uint32_t pfdFrac);

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __CCM_ANALOG_IMX7D_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
