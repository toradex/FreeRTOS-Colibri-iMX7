; ---------------------------------------------------------------------------------------
;  @file:    startup_MCIMX7D_M4.s
;  @purpose: CMSIS Cortex-M4 Core Device Startup File
;            IMX7D_M4
;  @version: 0.1
;  @date:    2015-04-06
;  @build:   b49163
; ---------------------------------------------------------------------------------------
;
; Copyright (c) 2015 , Freescale Semiconductor, Inc.
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without modification,
; are permitted provided that the following conditions are met:
;
; o Redistributions of source code must retain the above copyright notice, this list
;   of conditions and the following disclaimer.
;
; o Redistributions in binary form must reproduce the above copyright notice, this
;   list of conditions and the following disclaimer in the documentation and/or
;   other materials provided with the distribution.
;
; o Neither the name of Freescale Semiconductor, Inc. nor the names of its
;   contributors may be used to endorse or promote products derived from this
;   software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
; ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
; ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler                                   ;NMI Handler
        DCD     HardFault_Handler                             ;Hard Fault Handler
        DCD     MemManage_Handler                             ;MPU Fault Handler
        DCD     BusFault_Handler                              ;Bus Fault Handler
        DCD     UsageFault_Handler                            ;Usage Fault Handler
        DCD     0                                             ;Reserved
        DCD     0                                             ;Reserved
        DCD     0                                             ;Reserved
        DCD     0                                             ;Reserved
        DCD     SVC_Handler                                   ;SVCall Handler
        DCD     DebugMon_Handler                              ;Debug Monitor Handler
        DCD     0                                             ;Reserved
        DCD     PendSV_Handler                                ;PendSV Handler
        DCD     SysTick_Handler                               ;SysTick Handler

;External Interrupts
        DCD     DefaultISR                                    ;Reserved Interrupt 16
        DCD     DefaultISR                                    ;Reserved Interrupt 17
        DCD     DefaultISR                                    ;Reserved Interrupt 18
        DCD     DefaultISR                                    ;Reserved Interrupt 19
        DCD     DefaultISR                                    ;Reserved Interrupt 20
        DCD     DefaultISR                                    ;Reserved Interrupt 21
        DCD     DefaultISR                                    ;Reserved Interrupt 22
        DCD     DefaultISR                                    ;Reserved Interrupt 23
        DCD     DefaultISR                                    ;Reserved Interrupt 24
        DCD     DefaultISR                                    ;Reserved Interrupt 25
        DCD     WDOG3_Handler                                 ;WDOG3 Handler
        DCD     SEMA4_Handler                                 ;SEMA4 handler
        DCD     DefaultISR                                    ;Reserved Interrupt 28
        DCD     DefaultISR                                    ;Reserved Interrupt 29
        DCD     DefaultISR                                    ;Reserved Interrupt 30
        DCD     DefaultISR                                    ;Reserved Interrupt 31
        DCD     UART6_Handler                                 ;UART6 Handler
        DCD     DefaultISR                                    ;Reserved Interrupt 33
        DCD     DefaultISR                                    ;Reserved Interrupt 34
        DCD     DefaultISR                                    ;Reserved Interrupt 35
        DCD     DefaultISR                                    ;Reserved Interrupt 36
        DCD     DefaultISR                                    ;Reserved Interrupt 37
        DCD     DefaultISR                                    ;Reserved Interrupt 38
        DCD     DefaultISR                                    ;Reserved Interrupt 39
        DCD     DefaultISR                                    ;Reserved Interrupt 40
        DCD     DefaultISR                                    ;Reserved Interrupt 41
        DCD     UART1_Handler                                 ;UART1 Handler
        DCD     UART2_Handler                                 ;UART2 Handler
        DCD     UART3_Handler                                 ;UART3 Handler
        DCD     UART4_Handler                                 ;UART4 Handler
        DCD     UART5_Handler                                 ;UART5 Handler
        DCD     eCSPI1_Handler                                ;eCSPI1 Handler
        DCD     eCSPI2_Handler                                ;eCSPI2 Handler
        DCD     eCSPI3_Handler                                ;eCSPI3 Handler
        DCD     eCSPI4_Handler                                ;eCSPI4 Handler
        DCD     I2C1_Handler                                  ;I2C1 Handler
        DCD     I2C2_Handler                                  ;I2C2 Handler
        DCD     I2C3_Handler                                  ;I2C3 Handler
        DCD     I2C4_Handler                                  ;I2C4 Handler
        DCD     DefaultISR                                    ;Reserved Interrupt 55
        DCD     DefaultISR                                    ;Reserved Interrupt 56
        DCD     DefaultISR                                    ;Reserved Interrupt 57
        DCD     DefaultISR                                    ;Reserved Interrupt 58
        DCD     DefaultISR                                    ;Reserved Interrupt 59
        DCD     DefaultISR                                    ;Reserved Interrupt 60
        DCD     DefaultISR                                    ;Reserved Interrupt 61
        DCD     DefaultISR                                    ;Reserved Interrupt 62
        DCD     DefaultISR                                    ;Reserved Interrupt 63
        DCD     DefaultISR                                    ;Reserved Interrupt 64
        DCD     DefaultISR                                    ;Reserved Interrupt 65
        DCD     DefaultISR                                    ;Reserved Interrupt 66
        DCD     DefaultISR                                    ;Reserved Interrupt 67
        DCD     GPT4_Handler                                  ;GPT4 handler
        DCD     GPT3_Handler                                  ;GPT3 handler
        DCD     GPT2_Handler                                  ;GPT2 handler
        DCD     GPT1_Handler                                  ;GPT1 handler
        DCD     GPIO1_INT7_Handler                            ;Active HIGH Interrupt from INT7 from GPIO
        DCD     GPIO1_INT6_Handler                            ;Active HIGH Interrupt from INT6 from GPIO
        DCD     GPIO1_INT5_Handler                            ;Active HIGH Interrupt from INT5 from GPIO
        DCD     GPIO1_INT4_Handler                            ;Active HIGH Interrupt from INT4 from GPIO
        DCD     GPIO1_INT3_Handler                            ;Active HIGH Interrupt from INT3 from GPIO
        DCD     GPIO1_INT2_Handler                            ;Active HIGH Interrupt from INT2 from GPIO
        DCD     GPIO1_INT1_Handler                            ;Active HIGH Interrupt from INT1 from GPIO
        DCD     GPIO1_INT0_Handler                            ;Active HIGH Interrupt from INT0 from GPIO
        DCD     GPIO1_INT15_0_Handler                         ;Combined interrupt indication for GPIO1 signal 0 throughout 15
        DCD     GPIO1_INT31_16_Handler                        ;Combined interrupt indication for GPIO1 signal 16 throughout 31
        DCD     GPIO2_INT15_0_Handler                         ;Combined interrupt indication for GPIO2 signal 0 throughout 15
        DCD     GPIO2_INT31_16_Handler                        ;Combined interrupt indication for GPIO2 signal 16 throughout 31
        DCD     GPIO3_INT15_0_Handler                         ;Combined interrupt indication for GPIO3 signal 0 throughout 15
        DCD     GPIO3_INT31_16_Handler                        ;Combined interrupt indication for GPIO3 signal 16 throughout 31
        DCD     GPIO4_INT15_0_Handler                         ;Combined interrupt indication for GPIO4 signal 0 throughout 15
        DCD     GPIO4_INT31_16_Handler                        ;Combined interrupt indication for GPIO4 signal 16 throughout 31
        DCD     GPIO5_INT15_0_Handler                         ;Combined interrupt indication for GPIO5 signal 0 throughout 15
        DCD     GPIO5_INT31_16_Handler                        ;Combined interrupt indication for GPIO5 signal 16 throughout 31
        DCD     GPIO6_INT15_0_Handler                         ;Combined interrupt indication for GPIO6 signal 0 throughout 15
        DCD     GPIO6_INT31_16_Handler                        ;Combined interrupt indication for GPIO6 signal 16 throughout 31
        DCD     GPIO7_INT15_0_Handler                         ;Combined interrupt indication for GPIO7 signal 0 throughout 15
        DCD     GPIO7_INT31_16_Handler                        ;Combined interrupt indication for GPIO7 signal 16 throughout 31
        DCD     DefaultISR                                    ;Reserved Interrupt 94
        DCD     DefaultISR                                    ;Reserved Interrupt 95
        DCD     DefaultISR                                    ;Reserved Interrupt 96
        DCD     DefaultISR                                    ;Reserved Interrupt 97
        DCD     DefaultISR                                    ;Reserved Interrupt 98
        DCD     DefaultISR                                    ;Reserved Interrupt 99
        DCD     DefaultISR                                    ;Reserved Interrupt 100
        DCD     DefaultISR                                    ;Reserved Interrupt 101
        DCD     DefaultISR                                    ;Reserved Interrupt 102
        DCD     DefaultISR                                    ;Reserved Interrupt 103
        DCD     DefaultISR                                    ;Reserved Interrupt 104
        DCD     DefaultISR                                    ;Reserved Interrupt 105
        DCD     DefaultISR                                    ;Reserved Interrupt 106
        DCD     DefaultISR                                    ;Reserved Interrupt 107
        DCD     DefaultISR                                    ;Reserved Interrupt 108
        DCD     DefaultISR                                    ;Reserved Interrupt 109
        DCD     DefaultISR                                    ;Reserved Interrupt 110
        DCD     DefaultISR                                    ;Reserved Interrupt 111
        DCD     DefaultISR                                    ;Reserved Interrupt 112
        DCD     MU_Handler                                    ;MU Handler
        DCD     ADC1_Handler                                  ;ADC1 Handler
        DCD     ADC2_Handler                                  ;ADC2 Handler
        DCD     DefaultISR                                    ;Reserved Interrupt 116
        DCD     DefaultISR                                    ;Reserved Interrupt 117
        DCD     DefaultISR                                    ;Reserved Interrupt 118
        DCD     DefaultISR                                    ;Reserved Interrupt 119
        DCD     DefaultISR                                    ;Reserved Interrupt 120
        DCD     DefaultISR                                    ;Reserved Interrupt 121
        DCD     DefaultISR                                    ;Reserved Interrupt 122
        DCD     DefaultISR                                    ;Reserved Interrupt 123
        DCD     DefaultISR                                    ;Reserved Interrupt 124
        DCD     DefaultISR                                    ;Reserved Interrupt 125
        DCD     FLEXCAN1_Handler                              ;FLEXCAN1 Handler
        DCD     FLEXCAN2_Handler                              ;FLEXCAN2 Handler
        DCD     DefaultISR                                    ;Reserved Interrupt 128
        DCD     DefaultISR                                    ;Reserved Interrupt 129
        DCD     DefaultISR                                    ;Reserved Interrupt 130
        DCD     DefaultISR                                    ;Reserved Interrupt 131
        DCD     DefaultISR                                    ;Reserved Interrupt 132
        DCD     DefaultISR                                    ;Reserved Interrupt 133
        DCD     DefaultISR                                    ;Reserved Interrupt 134
        DCD     DefaultISR                                    ;Reserved Interrupt 135
        DCD     DefaultISR                                    ;Reserved Interrupt 136
        DCD     DefaultISR                                    ;Reserved Interrupt 137
        DCD     DefaultISR                                    ;Reserved Interrupt 138
        DCD     DefaultISR                                    ;Reserved Interrupt 139
        DCD     DefaultISR                                    ;Reserved Interrupt 140
        DCD     DefaultISR                                    ;Reserved Interrupt 141
        DCD     UART7_Handler                                 ;UART7 Handler
        DCD     DefaultISR                                    ;Reserved Interrupt 143


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
        CPSID   I               ; Mask interrupts
        LDR     R0, =SystemInit
        BLX     R0
        CPSIE   I               ; Unmask interrupts
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B .

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B .

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
        B .

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
        B .

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
        B .

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B .

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
        B .

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B .

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B .

        PUBWEAK WDOG3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDOG3_Handler
        B .

        PUBWEAK UART1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART1_Handler
        B .

        PUBWEAK UART2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART2_Handler
        B .

        PUBWEAK UART3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART3_Handler
        B .

        PUBWEAK UART4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART4_Handler
        B .

        PUBWEAK UART5_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART5_Handler
        B .

        PUBWEAK UART6_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART6_Handler
        B .

        PUBWEAK UART7_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART7_Handler
        B .

        PUBWEAK eCSPI1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
eCSPI1_Handler
        B .

        PUBWEAK eCSPI2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
eCSPI2_Handler
        B .

        PUBWEAK eCSPI3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
eCSPI3_Handler
        B .

        PUBWEAK eCSPI4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
eCSPI4_Handler
        B .

        PUBWEAK I2C1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C1_Handler
        B .

        PUBWEAK I2C2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C2_Handler
        B .

        PUBWEAK I2C3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C3_Handler
        B .

        PUBWEAK I2C4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C4_Handler
        B .

        PUBWEAK GPT4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPT4_Handler
        B .

        PUBWEAK GPT3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPT3_Handler
        B .

        PUBWEAK GPT2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPT2_Handler
        B .

        PUBWEAK GPT1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPT1_Handler
        B .

        PUBWEAK GPIO1_INT7_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO1_INT7_Handler
        B .

        PUBWEAK GPIO1_INT6_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO1_INT6_Handler
        B .

        PUBWEAK GPIO1_INT5_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO1_INT5_Handler
        B .

        PUBWEAK GPIO1_INT4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO1_INT4_Handler
        B .

       PUBWEAK GPIO1_INT3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO1_INT3_Handler
        B .

        PUBWEAK GPIO1_INT2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO1_INT2_Handler
        B .

        PUBWEAK GPIO1_INT1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO1_INT1_Handler
        B .

        PUBWEAK GPIO1_INT0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO1_INT0_Handler
        B .

        PUBWEAK GPIO1_INT15_0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO1_INT15_0_Handler
        B .

        PUBWEAK GPIO1_INT31_16_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO1_INT31_16_Handler
        B .

        PUBWEAK GPIO2_INT15_0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO2_INT15_0_Handler
        B .

        PUBWEAK GPIO2_INT31_16_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO2_INT31_16_Handler
        B .

        PUBWEAK GPIO3_INT15_0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO3_INT15_0_Handler
        B .

        PUBWEAK GPIO3_INT31_16_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO3_INT31_16_Handler
        B .

        PUBWEAK GPIO4_INT15_0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO4_INT15_0_Handler
        B .

        PUBWEAK GPIO4_INT31_16_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO4_INT31_16_Handler
        B .

        PUBWEAK GPIO5_INT15_0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO5_INT15_0_Handler
        B .

        PUBWEAK GPIO5_INT31_16_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO5_INT31_16_Handler
        B .

        PUBWEAK GPIO6_INT15_0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO6_INT15_0_Handler
        B .

        PUBWEAK GPIO6_INT31_16_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO6_INT31_16_Handler
        B .

        PUBWEAK GPIO7_INT15_0_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO7_INT15_0_Handler
        B .

        PUBWEAK GPIO7_INT31_16_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIO7_INT31_16_Handler
        B .

        PUBWEAK MU_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MU_Handler
        B .

        PUBWEAK ADC1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC1_Handler
        B .

        PUBWEAK ADC2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC2_Handler
        B .

        PUBWEAK SEMA4_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SEMA4_Handler
        B .

        PUBWEAK FLEXCAN1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
FLEXCAN1_Handler

        PUBWEAK FLEXCAN2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
FLEXCAN2_Handler

        PUBWEAK DefaultISR
        SECTION .text:CODE:REORDER:NOROOT(1)
DefaultISR
        B DefaultISR

        END

