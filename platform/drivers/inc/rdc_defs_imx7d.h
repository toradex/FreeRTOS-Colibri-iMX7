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

#ifndef __RDC_DEFS_IMX7D__
#define __RDC_DEFS_IMX7D__

/*!
 * @addtogroup rdc_def_imx7d
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief RDC master assignment
 */
enum _rdc_mda {
    rdcMdaA7          = 0U,
    rdcMdaM4          = 1U,
    rdcMdaPcie        = 2U,
    rdcMdaCsi         = 3U,
    rdcMdaEpdc        = 4U,
    rdcMdaLcdif       = 5U,
    rdcMdaDisplayPort = 6U,
    rdcMdaPxp         = 7U,
    rdcMdaCoresight   = 8U,
    rdcMdaDap         = 9U,
    rdcMdaCaam        = 10U,
    rdcMdaSdmaPeriph  = 11U,
    rdcMdaSdmaBurst   = 12U,
    rdcMdaApbhdma     = 13U,
    rdcMdaRawnand     = 14U,
    rdcMdaUsdhc1      = 15U,
    rdcMdaUsdhc2      = 16U,
    rdcMdaUsdhc3      = 17U,
    rdcMdaNc1         = 18U,
    rdcMdaUsb         = 19U,
    rdcMdaNc2         = 20U,
    rdcMdaTest        = 21U,
    rdcMdaEnet1Tx     = 22U,
    rdcMdaEnet1Rx     = 23U,
    rdcMdaEnet2Tx     = 24U,
    rdcMdaEnet2Rx     = 25U,
    rdcMdaSdmaPort    = 26U
};

/*!
 * @brief RDC peripheral assignment
 */
enum _rdc_pdap {
    rdcPdapGpio1                 = 0U,
    rdcPdapGpio2                 = 1U,
    rdcPdapGpio3                 = 2U,
    rdcPdapGpio4                 = 3U,
    rdcPdapGpio5                 = 4U,
    rdcPdapGpio6                 = 5U,
    rdcPdapGpio7                 = 6U,
    rdcPdapIomuxcLpsrGpr         = 7U,
    rdcPdapWdog1                 = 8U,
    rdcPdapWdog2                 = 9U,
    rdcPdapWdog3                 = 10U,
    rdcPdapWdog4                 = 11U,
    rdcPdapIomuxcLpsr            = 12U,
    rdcPdapGpt1                  = 13U,
    rdcPdapGpt2                  = 14U,
    rdcPdapGpt3                  = 15U,
    rdcPdapGpt4                  = 16U,
    rdcPdapRomcp                 = 17U,
    rdcPdapKpp                   = 18U,
    rdcPdapIomuxc                = 19U,
    rdcPdapIomuxcGpr             = 20U,
    rdcPdapOcotpCtrl             = 21U,
    rdcPdapAnatopDig             = 22U,
    rdcPdapSnvs                  = 23U,
    rdcPdapCcm                   = 24U,
    rdcPdapSrc                   = 25U,
    rdcPdapGpc                   = 26U,
    rdcPdapSemaphore1            = 27U,
    rdcPdapSemaphore2            = 28U,
    rdcPdapRdc                   = 29U,
    rdcPdapCsu                   = 30U,
    rdcPdapReserved1             = 31U,
    rdcPdapReserved2             = 32U,
    rdcPdapAdc1                  = 33U,
    rdcPdapAdc2                  = 34U,
    rdcPdapEcspi4                = 35U,
    rdcPdapFlexTimer1            = 36U,
    rdcPdapFlexTimer2            = 37U,
    rdcPdapPwm1                  = 38U,
    rdcPdapPwm2                  = 39U,
    rdcPdapPwm3                  = 40U,
    rdcPdapPwm4                  = 41U,
    rdcPdapSystemCounterRead     = 42U,
    rdcPdapSystemCounterCompare  = 43U,
    rdcPdapSystemCounterControl  = 44U,
    rdcPdapPcie                  = 45U,
    rdcPdapReserved3             = 46U,
    rdcPdapEpdc                  = 47U,
    rdcPdapPxp                   = 48U,
    rdcPdapCsi                   = 49U,
    rdcPdapReserved4             = 50U,
    rdcPdapLcdif                 = 51U,
    rdcPdapReserved5             = 52U,
    rdcPdapMipiCsi               = 53U,
    rdcPdapMipiDsi               = 54U,
    rdcPdapReserved6             = 55U,
    rdcPdapTzasc                 = 56U,
    rdcPdapDdrPhy                = 57U,
    rdcPdapDdrc                  = 58U,
    rdcPdapReserved7             = 59U,
    rdcPdapPerfMon1              = 60U,
    rdcPdapPerfMon2              = 61U,
    rdcPdapAxi                   = 62U,
    rdcPdapQosc                  = 63U,
    rdcPdapFlexCan1              = 64U,
    rdcPdapFlexCan2              = 65U,
    rdcPdapI2c1                  = 66U,
    rdcPdapI2c2                  = 67U,
    rdcPdapI2c3                  = 68U,
    rdcPdapI2c4                  = 69U,
    rdcPdapUart4                 = 70U,
    rdcPdapUart5                 = 71U,
    rdcPdapUart6                 = 72U,
    rdcPdapUart7                 = 73U,
    rdcPdapMuA                   = 74U,
    rdcPdapMuB                   = 75U,
    rdcPdapSemaphoreHs           = 76U,
    rdcPdapUsbPl301              = 77U,
    rdcPdapReserved8             = 78U,
    rdcPdapReserved9             = 79U,
    rdcPdapReserved10            = 80U,
    rdcPdapUSB1Otg1              = 81U,
    rdcPdapUSB2Otg2              = 82U,
    rdcPdapUSB3Host              = 83U,
    rdcPdapUsdhc1                = 84U,
    rdcPdapUsdhc2                = 85U,
    rdcPdapUsdhc3                = 86U,
    rdcPdapReserved11            = 87U,
    rdcPdapReserved12            = 88U,
    rdcPdapSim1                  = 89U,
    rdcPdapSim2                  = 90U,
    rdcPdapQspi                  = 91U,
    rdcPdapWeim                  = 92U,
    rdcPdapSdma                  = 93U,
    rdcPdapEnet1                 = 94U,
    rdcPdapEnet2                 = 95U,
    rdcPdapReserved13            = 96U,
    rdcPdapReserved14            = 97U,
    rdcPdapEcspi1                = 98U,
    rdcPdapEcspi2                = 99U,
    rdcPdapEcspi3                = 100U,
    rdcPdapReserved15            = 101U,
    rdcPdapUart1                 = 102U,
    rdcPdapReserved16            = 103U,
    rdcPdapUart3                 = 104U,
    rdcPdapUart2                 = 105U,
    rdcPdapSai1                  = 106U,
    rdcPdapSai2                  = 107U,
    rdcPdapSai3                  = 108U,
    rdcPdapReserved17            = 109U,
    rdcPdapReserved18            = 110U,
    rdcPdapSpba                  = 111U,
    rdcPdapDap                   = 112U,
    rdcPdapReserved19            = 113U,
    rdcPdapReserved20            = 114U,
    rdcPdapReserved21            = 115U,
    rdcPdapCaam                  = 116U,
    rdcPdapReserved22            = 117U
};

/*!
 * @brief RDC memory region
 */
enum _rdc_mr {
    rdcMrMmdc           = 0U,  /* alignment 4096 */
    rdcMrMmdcLast       = 7U,  /* alignment 4096 */
    rdcMrQspi           = 8U,  /* alignment 4096 */
    rdcMrQspiLast       = 15U, /* alignment 4096 */
    rdcMrWeim           = 16U, /* alignment 4096 */
    rdcMrWeimLast       = 23U, /* alignment 4096 */
    rdcMrPcie           = 24U, /* alignment 4096 */
    rdcMrPcieLast       = 31U, /* alignment 4096 */
    rdcMrOcram          = 32U, /* alignment 128 */
    rdcMrOcramLast      = 36U, /* alignment 128 */
    rdcMrOcramS         = 37U, /* alignment 128 */
    rdcMrOcramSLast     = 41U, /* alignment 128 */
    rdcMrOcramEpdc      = 42U, /* alignment 128 */
    rdcMrOcramEpdcLast  = 46U, /* alignment 128 */
    rdcMrOcramPxp       = 47U, /* alignment 128 */
    rdcMrOcramPxpLast   = 51U  /* alignment 128 */
};

#endif /* __RDC_DEFS_IMX7D__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
