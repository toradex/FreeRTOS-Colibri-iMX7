/*
** ###################################################################
**     Processors:          MCIMX7D7DVK10SA
**                          MCIMX7D7DVM10SA
**                          MCIMX7D3DVK10SA
**                          MCIMX7D3EVM10SA
**
**     Compiler:            GNU C Compiler
**     Reference manual:    IMX7DRM, Rev.A, February 2015
**     Version:             rev. 1.0, 2015-05-19
**
**     Abstract:
**         Linker file for the GNU C Compiler
**
**     Copyright (c) 2015 Freescale Semiconductor, Inc.
**     All rights reserved.
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
** ###################################################################
*/
/*
 Linker file for Colibri i.MX7S and Colibri i.MX7D
 Storing code and data in the TCML / TCMU sections

               +-------------+
               |             |  ^
               |   m_text    |  |LENGTH(m_text)
               |             |  v
        m_text +-------------+  

               +-------------+  
               |             |  ^
               |m_interrupts |  |LENGTH(m_interrupts)
               |             |  v
  m_interrupts +-------------+


    __StackTop +-------------+
               |             |  ^  ^
               |    stack    |  |  |STACK_SIZE
               |             |  |  v
  __StackLimit +-------------+  |
   __HeapLimit +-------------+  |
               |             |  |  ^
               |    heap     |  |  |HEAP_SIZE
               |             |  |  v
    __HeapBase +-------------+  |
               |             |  |LENGTH(m_data)
               | normal data |  |
               |             |  v
  m_data_start +-------------+
*/

TCML_code         = 0x1fff8000;
TCMU_system       = 0x20000000;
OCRAM_S_code      = 0x00000000;
OCRAM_S_system    = 0x00180000;
OCRAM_code        = 0x00900000;
OCRAM_system      = 0x20200000;
OCRAM_EPDC_code   = 0x00920000;
OCRAM_EPDC_system = 0x20220000;
DDR_code          = 0x10000000;
DDR_system        = 0x80000000;


/* Code execution after reset starts here: */
ENTRY(Reset_Handler)

HEAP_SIZE  = DEFINED(__heap_size__)  ? __heap_size__  : 0x0400;
STACK_SIZE = DEFINED(__stack_size__) ? __stack_size__ : 0x0400;

/* Specify the memory areas */
MEMORY
{
  m_interrupts  (RX)  : ORIGIN = OCRAM_S_code,         LENGTH = 0x00000240
  m_text        (RX)  : ORIGIN = OCRAM_code + 0x10000, LENGTH = 0x00010000
  m_data        (RW)  : ORIGIN = OCRAM_EPDC_system,    LENGTH = 0x00020000
}

INCLUDE "MCIMX7D_M4.ld"
