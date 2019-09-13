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

#include "ccm_analog_imx7d.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : CCM_ANALOG_GetSysPllFreq
 * Description   : Get system PLL frequency
 *
 *END**************************************************************************/
uint32_t CCM_ANALOG_GetSysPllFreq(CCM_ANALOG_Type * base)
{
    if (CCM_ANALOG_IsPllBypassed(base, ccmAnalogPll480Control))
        return 24000000;

    if (CCM_ANALOG_PLL_480 & CCM_ANALOG_PLL_480_DIV_SELECT_MASK)
        return 528000000;
    else
        return 480000000;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CCM_ANALOG_GetPfdFreq
 * Description   : Get PFD frequency
 *
 *END**************************************************************************/
uint32_t CCM_ANALOG_GetPfdFreq(CCM_ANALOG_Type * base, uint32_t pfdFrac)
{
    uint32_t main, frac;

    /* PFD should work with system PLL without bypass */
    assert(!CCM_ANALOG_IsPllBypassed(base, ccmAnalogPll480Control));

    main = CCM_ANALOG_GetSysPllFreq(base);
    frac = CCM_ANALOG_GetPfdFrac(base, pfdFrac);

    return main / frac * 18; 
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
