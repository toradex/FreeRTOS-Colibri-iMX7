; * ---------------------------------------------------------------------------------------
; *  @file:    startup_MCIMX7D_M4.s
; *  @purpose: CMSIS Cortex-M4 Core Device Startup File
; *            IMX7D_M4
; *  @version: 0.1
; *  @date:    2015-5-27
; *  @build:   b54573
; * ---------------------------------------------------------------------------------------
; *
; * Copyright (c) 2015 , Freescale Semiconductor, Inc.
; * All rights reserved.
; *
; * Redistribution and use in source and binary forms, with or without modification,
; * are permitted provided that the following conditions are met:
; *
; * o Redistributions of source code must retain the above copyright notice, this list
; *   of conditions and the following disclaimer.
; *
; * o Redistributions in binary form must reproduce the above copyright notice, this
; *   list of conditions and the following disclaimer in the documentation and/or
; *   other materials provided with the distribution.
; *
; * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
; *   contributors may be used to endorse or promote products derived from this
; *   software without specific prior written permission.
; *
; * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
; * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
; * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; *
; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------
; *
; *****************************************************************************/


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size
                IMPORT |Image$$ARM_LIB_STACK$$ZI$$Limit|
__Vectors       DCD    |Image$$ARM_LIB_STACK$$ZI$$Limit|  ; Top of Stack
                DCD     Reset_Handler  ; Reset Handler
                DCD     NMI_Handler                         ;NMI Handler
                DCD     HardFault_Handler                   ;Hard Fault Handler
                DCD     MemManage_Handler                   ;MPU Fault Handler
                DCD     BusFault_Handler                    ;Bus Fault Handler
                DCD     UsageFault_Handler                  ;Usage Fault Handler
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     0                                   ;Reserved
                DCD     SVC_Handler                         ;SVCall Handler
                DCD     DebugMon_Handler                    ;Debug Monitor Handler
                DCD     0                                   ;Reserved
                DCD     PendSV_Handler                      ;PendSV Handler
                DCD     SysTick_Handler                     ;SysTick Handler

                                                            ;External Interrupts
                DCD     DefaultISR                          ;Reserved Interrupt 16
                DCD     DefaultISR                          ;Reserved Interrupt 17
                DCD     DefaultISR                          ;Reserved Interrupt 18
                DCD     DefaultISR                          ;Reserved Interrupt 19
                DCD     DefaultISR                          ;Reserved Interrupt 20
                DCD     DefaultISR                          ;Reserved Interrupt 21
                DCD     DefaultISR                          ;Reserved Interrupt 22
                DCD     DefaultISR                          ;Reserved Interrupt 23
                DCD     DefaultISR                          ;Reserved Interrupt 24
                DCD     DefaultISR                          ;Reserved Interrupt 25
                DCD     WDOG3_Handler                       ;WDOG3 Handler
                DCD     SEMA4_Handler                       ;SEMA4_Handler
                DCD     DefaultISR                          ;Reserved Interrupt 28
                DCD     DefaultISR                          ;Reserved Interrupt 29
                DCD     DefaultISR                          ;Reserved Interrupt 30
                DCD     DefaultISR                          ;Reserved Interrupt 31
                DCD     UART6_Handler                       ;UART6 Handler
                DCD     DefaultISR                          ;Reserved Interrupt 33
                DCD     DefaultISR                          ;Reserved Interrupt 34
                DCD     DefaultISR                          ;Reserved Interrupt 35
                DCD     DefaultISR                          ;Reserved Interrupt 36
                DCD     DefaultISR                          ;Reserved Interrupt 37
                DCD     DefaultISR                          ;Reserved Interrupt 38
                DCD     DefaultISR                          ;Reserved Interrupt 39
                DCD     DefaultISR                          ;Reserved Interrupt 40
                DCD     DefaultISR                          ;Reserved Interrupt 41
                DCD     UART1_Handler                       ;UART1 Handler
                DCD     UART2_Handler                       ;UART2 Handler
                DCD     UART3_Handler                       ;UART3 Handler
                DCD     UART4_Handler                       ;UART4 Handler
                DCD     UART5_Handler                       ;UART5 Handler
                DCD     eCSPI1_Handler                      ;eCSPI1 Handler
                DCD     eCSPI2_Handler                      ;eCSPI2 Handler
                DCD     eCSPI3_Handler                      ;eCSPI3 Handler
                DCD     eCSPI4_Handler                      ;eCSPI4 Handler
                DCD     I2C1_Handler                        ;I2C1 Handler
                DCD     I2C2_Handler                        ;I2C2 Handler
                DCD     I2C3_Handler                        ;I2C3 Handler
                DCD     I2C4_Handler                        ;I2C4 Handler
                DCD     DefaultISR                          ;Reserved Interrupt 55
                DCD     DefaultISR                          ;Reserved Interrupt 56
                DCD     DefaultISR                          ;Reserved Interrupt 57
                DCD     DefaultISR                          ;Reserved Interrupt 58
                DCD     DefaultISR                          ;Reserved Interrupt 59
                DCD     DefaultISR                          ;Reserved Interrupt 60
                DCD     DefaultISR                          ;Reserved Interrupt 61
                DCD     DefaultISR                          ;Reserved Interrupt 62
                DCD     DefaultISR                          ;Reserved Interrupt 63
                DCD     DefaultISR                          ;Reserved Interrupt 64
                DCD     DefaultISR                          ;Reserved Interrupt 65
                DCD     DefaultISR                          ;Reserved Interrupt 66
                DCD     DefaultISR                          ;Reserved Interrupt 67
                DCD     GPT4_Handler                        ;GPT4 handler
                DCD     GPT3_Handler                        ;GPT3 handler
                DCD     GPT2_Handler                        ;GPT2 handler
                DCD     GPT1_Handler                        ;GPT1 handler
                DCD     GPIO1_INT7_Handler                  ;Active HIGH Interrupt from INT7 from GPIO
                DCD     GPIO1_INT6_Handler                  ;Active HIGH Interrupt from INT6 from GPIO
                DCD     GPIO1_INT5_Handler                  ;Active HIGH Interrupt from INT5 from GPIO
                DCD     GPIO1_INT4_Handler                  ;Active HIGH Interrupt from INT4 from GPIO
                DCD     GPIO1_INT3_Handler                  ;Active HIGH Interrupt from INT3 from GPIO
                DCD     GPIO1_INT2_Handler                  ;Active HIGH Interrupt from INT2 from GPIO
                DCD     GPIO1_INT1_Handler                  ;Active HIGH Interrupt from INT1 from GPIO
                DCD     GPIO1_INT0_Handler                  ;Active HIGH Interrupt from INT0 from GPIO
                DCD     GPIO1_INT15_0_Handler               ;Combined interrupt indication for GPIO1 signal 0 throughout 15
                DCD     GPIO1_INT31_16_Handler              ;Combined interrupt indication for GPIO1 signal 16 throughout 31
                DCD     GPIO2_INT15_0_Handler               ;Combined interrupt indication for GPIO2 signal 0 throughout 15
                DCD     GPIO2_INT31_16_Handler              ;Combined interrupt indication for GPIO2 signal 16 throughout 31
                DCD     GPIO3_INT15_0_Handler               ;Combined interrupt indication for GPIO3 signal 0 throughout 15
                DCD     GPIO3_INT31_16_Handler              ;Combined interrupt indication for GPIO3 signal 16 throughout 31
                DCD     GPIO4_INT15_0_Handler               ;Combined interrupt indication for GPIO4 signal 0 throughout 15
                DCD     GPIO4_INT31_16_Handler              ;Combined interrupt indication for GPIO4 signal 16 throughout 31
                DCD     GPIO5_INT15_0_Handler               ;Combined interrupt indication for GPIO5 signal 0 throughout 15
                DCD     GPIO5_INT31_16_Handler              ;Combined interrupt indication for GPIO5 signal 16 throughout 31
                DCD     GPIO6_INT15_0_Handler               ;Combined interrupt indication for GPIO6 signal 0 throughout 15
                DCD     GPIO6_INT31_16_Handler              ;Combined interrupt indication for GPIO6 signal 16 throughout 31
                DCD     GPIO7_INT15_0_Handler               ;Combined interrupt indication for GPIO7 signal 0 throughout 15
                DCD     GPIO7_INT31_16_Handler              ;Combined interrupt indication for GPIO7 signal 16 throughout 31
                DCD     DefaultISR                          ;Reserved Interrupt 94
                DCD     DefaultISR                          ;Reserved Interrupt 95
                DCD     DefaultISR                          ;Reserved Interrupt 96
                DCD     DefaultISR                          ;Reserved Interrupt 97
                DCD     DefaultISR                          ;Reserved Interrupt 98
                DCD     DefaultISR                          ;Reserved Interrupt 99
                DCD     DefaultISR                          ;Reserved Interrupt 100
                DCD     DefaultISR                          ;Reserved Interrupt 101
                DCD     DefaultISR                          ;Reserved Interrupt 102
                DCD     DefaultISR                          ;Reserved Interrupt 103
                DCD     DefaultISR                          ;Reserved Interrupt 104
                DCD     DefaultISR                          ;Reserved Interrupt 105
                DCD     DefaultISR                          ;Reserved Interrupt 106
                DCD     DefaultISR                          ;Reserved Interrupt 107
                DCD     DefaultISR                          ;Reserved Interrupt 108
                DCD     DefaultISR                          ;Reserved Interrupt 109
                DCD     DefaultISR                          ;Reserved Interrupt 110
                DCD     DefaultISR                          ;Reserved Interrupt 111
                DCD     DefaultISR                          ;Reserved Interrupt 112
                DCD     MU_Handler                          ;MU_Handler
                DCD     ADC1_Handler                        ;ADC1 Handler
                DCD     ADC2_Handler                        ;ADC2 Handler
                DCD     DefaultISR                          ;Reserved Interrupt 116
                DCD     DefaultISR                          ;Reserved Interrupt 117
                DCD     DefaultISR                          ;Reserved Interrupt 118
                DCD     DefaultISR                          ;Reserved Interrupt 119
                DCD     DefaultISR                          ;Reserved Interrupt 120
                DCD     DefaultISR                          ;Reserved Interrupt 121
                DCD     DefaultISR                          ;Reserved Interrupt 122
                DCD     DefaultISR                          ;Reserved Interrupt 123
                DCD     DefaultISR                          ;Reserved Interrupt 124
                DCD     DefaultISR                          ;Reserved Interrupt 125
                DCD     FLEXCAN1_Handler                    ;FLEXCAN1 Handler
                DCD     FLEXCAN2_Handler                    ;FLEXCAN2 Handler
                DCD     DefaultISR                          ;Reserved Interrupt 128
                DCD     DefaultISR                          ;Reserved Interrupt 129
                DCD     DefaultISR                          ;Reserved Interrupt 130
                DCD     DefaultISR                          ;Reserved Interrupt 131
                DCD     DefaultISR                          ;Reserved Interrupt 132
                DCD     DefaultISR                          ;Reserved Interrupt 133
                DCD     DefaultISR                          ;Reserved Interrupt 134
                DCD     DefaultISR                          ;Reserved Interrupt 135
                DCD     DefaultISR                          ;Reserved Interrupt 136
                DCD     DefaultISR                          ;Reserved Interrupt 137
                DCD     DefaultISR                          ;Reserved Interrupt 138
                DCD     DefaultISR                          ;Reserved Interrupt 139
                DCD     DefaultISR                          ;Reserved Interrupt 140
                DCD     DefaultISR                          ;Reserved Interrupt 141
                DCD     UART7_Handler                       ;UART7 Handler
                DCD     DefaultISR                          ;Reserved Interrupt 143

__Vectors_End

__Vectors_Size     EQU     __Vectors_End - __Vectors



                AREA    |.text|, CODE, READONLY
; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                CPSID   I               ; Mask interrupts
                LDR     R0, =SystemInit
                BLX     R0
                CPSIE   i               ; Unmask interrupts
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)
NMI_Handler\
                PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler\
                PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP
WDOG3_Handler\
                PROC
                EXPORT  WDOG3_Handler             [WEAK]
                B       .
                ENDP
SEMA4_Handler\
                PROC
                EXPORT  SEMA4_Handler             [WEAK]
                B       .
                ENDP
UART6_Handler\
                PROC
                EXPORT  UART6_Handler             [WEAK]
                B       .
                ENDP
UART1_Handler\
                PROC
                EXPORT  UART1_Handler             [WEAK]
                B       .
                ENDP
UART2_Handler\
                PROC
                EXPORT  UART2_Handler             [WEAK]
                B       .
                ENDP
UART3_Handler\
                PROC
                EXPORT  UART3_Handler             [WEAK]
                B       .
                ENDP
UART4_Handler\
                PROC
                EXPORT  UART4_Handler             [WEAK]
                B       .
                ENDP
UART5_Handler\
                PROC
                EXPORT  UART5_Handler             [WEAK]
                B       .
                ENDP
eCSPI1_Handler\
                PROC
                EXPORT  eCSPI1_Handler            [WEAK]
                B       .
                ENDP
eCSPI2_Handler\
                PROC
                EXPORT  eCSPI2_Handler            [WEAK]
                B       .
                ENDP
eCSPI3_Handler\
                PROC
                EXPORT  eCSPI3_Handler            [WEAK]
                B       .
                ENDP
eCSPI4_Handler\
                PROC
                EXPORT  eCSPI4_Handler            [WEAK]
                B       .
                ENDP
I2C1_Handler\
                PROC
                EXPORT  I2C1_Handler              [WEAK]
                B       .
                ENDP
I2C2_Handler\
                PROC
                EXPORT  I2C2_Handler              [WEAK]
                B       .
                ENDP
I2C3_Handler\
                PROC
                EXPORT  I2C3_Handler              [WEAK]
                B       .
                ENDP
I2C4_Handler\
                PROC
                EXPORT  I2C4_Handler              [WEAK]
                B       .
                ENDP
GPT4_Handler\
                PROC
                EXPORT  GPT4_Handler              [WEAK]
                B       .
                ENDP
GPT3_Handler\
                PROC
                EXPORT  GPT3_Handler              [WEAK]
                B       .
                ENDP
GPT2_Handler\
                PROC
                EXPORT  GPT2_Handler              [WEAK]
                B       .
                ENDP
GPT1_Handler\
                PROC
                EXPORT  GPT1_Handler              [WEAK]
                B       .
                ENDP
GPIO1_INT7_Handler\
                PROC
                EXPORT  GPIO1_INT7_Handler        [WEAK]
                B       .
                ENDP
GPIO1_INT6_Handler\
                PROC
                EXPORT  GPIO1_INT6_Handler        [WEAK]
                B       .
                ENDP
GPIO1_INT5_Handler\
                PROC
                EXPORT  GPIO1_INT5_Handler        [WEAK]
                B       .
                ENDP
GPIO1_INT4_Handler\
                PROC
                EXPORT  GPIO1_INT4_Handler        [WEAK]
                B       .
                ENDP
GPIO1_INT3_Handler\
                PROC
                EXPORT  GPIO1_INT3_Handler        [WEAK]
                B       .
                ENDP
GPIO1_INT2_Handler\
                PROC
                EXPORT  GPIO1_INT2_Handler        [WEAK]
                B       .
                ENDP
GPIO1_INT1_Handler\
                PROC
                EXPORT  GPIO1_INT1_Handler        [WEAK]
                B       .
                ENDP
GPIO1_INT0_Handler\
                PROC
                EXPORT  GPIO1_INT0_Handler        [WEAK]
                B       .
                ENDP
GPIO1_INT15_0_Handler\
                PROC
                EXPORT  GPIO1_INT15_0_Handler     [WEAK]
                B       .
                ENDP
GPIO1_INT31_16_Handler\
                PROC
                EXPORT  GPIO1_INT31_16_Handler    [WEAK]
                B       .
                ENDP
GPIO2_INT15_0_Handler\
                PROC
                EXPORT  GPIO2_INT15_0_Handler     [WEAK]
                B       .
                ENDP
GPIO2_INT31_16_Handler\
                PROC
                EXPORT  GPIO2_INT31_16_Handler    [WEAK]
                B       .
                ENDP
GPIO3_INT15_0_Handler\
                PROC
                EXPORT  GPIO3_INT15_0_Handler     [WEAK]
                B       .
                ENDP
GPIO3_INT31_16_Handler\
                PROC
                EXPORT  GPIO3_INT31_16_Handler    [WEAK]
                B       .
                ENDP
GPIO4_INT15_0_Handler\
                PROC
                EXPORT  GPIO4_INT15_0_Handler     [WEAK]
                B       .
                ENDP
GPIO4_INT31_16_Handler\
                PROC
                EXPORT  GPIO4_INT31_16_Handler    [WEAK]
                B       .
                ENDP
GPIO5_INT15_0_Handler\
                PROC
                EXPORT  GPIO5_INT15_0_Handler     [WEAK]
                B       .
                ENDP
GPIO5_INT31_16_Handler\
                PROC
                EXPORT  GPIO5_INT31_16_Handler    [WEAK]
                B       .
                ENDP
GPIO6_INT15_0_Handler\
                PROC
                EXPORT  GPIO6_INT15_0_Handler     [WEAK]
                B       .
                ENDP
GPIO6_INT31_16_Handler\
                PROC
                EXPORT  GPIO6_INT31_16_Handler    [WEAK]
                B       .
                ENDP
GPIO7_INT15_0_Handler\
                PROC
                EXPORT  GPIO7_INT15_0_Handler     [WEAK]
                B       .
                ENDP
GPIO7_INT31_16_Handler\
                PROC
                EXPORT  GPIO7_INT31_16_Handler    [WEAK]
                B       .
                ENDP
MU_Handler\
                PROC
                EXPORT  MU_Handler                [WEAK]
                B       .
                ENDP
ADC1_Handler\
                PROC
                EXPORT  ADC1_Handler              [WEAK]
                B       .
                ENDP
ADC2_Handler\
                PROC
                EXPORT  ADC2_Handler              [WEAK]
                B       .
                ENDP
FLEXCAN1_Handler\
                PROC
                EXPORT  FLEXCAN1_Handler          [WEAK]
                B       .
                ENDP
FLEXCAN2_Handler\
                PROC
                EXPORT  FLEXCAN2_Handler          [WEAK]
                B       .
                ENDP
UART7_Handler\
                PROC
                EXPORT  UART7_Handler             [WEAK]
                B       .
                ENDP

Default_Handler\
                PROC

                EXPORT  DefaultISR                [WEAK]


DefaultISR

                B      DefaultISR
                ENDP
                  ALIGN


                END
