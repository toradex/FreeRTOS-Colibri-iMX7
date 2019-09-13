/* ---------------------------------------------------------------------------------------*/
/*  @file:    startup_MCIMX7D_M4.s                                                        */
/*  @purpose: CMSIS Cortex-M4 Core Device Startup File                                    */
/*            IMX7D_M4                                                                    */
/*  @version: 0.1                                                                         */
/*  @date:    2015-04-06                                                                  */
/*  @build:   b54573                                                                      */
/* ---------------------------------------------------------------------------------------*/
/*                                                                                        */
/* Copyright (c) 2015 , Freescale Semiconductor, Inc.                                     */
/* All rights reserved.                                                                   */
/*                                                                                        */
/* Redistribution and use in source and binary forms, with or without modification,       */
/* are permitted provided that the following conditions are met:                          */
/*                                                                                        */
/* o Redistributions of source code must retain the above copyright notice, this list     */
/*   of conditions and the following disclaimer.                                          */
/*                                                                                        */
/* o Redistributions in binary form must reproduce the above copyright notice, this       */
/*   list of conditions and the following disclaimer in the documentation and/or          */
/*   other materials provided with the distribution.                                      */
/*                                                                                        */
/* o Neither the name of Freescale Semiconductor, Inc. nor the names of its               */
/*   contributors may be used to endorse or promote products derived from this            */
/*   software without specific prior written permission.                                  */
/*                                                                                        */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND        */
/* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED          */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                 */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR       */
/* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES         */
/* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;           */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON         */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                */
/* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS          */
/* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/*****************************************************************************/
/* Version: GCC for ARM Embedded Processors                                  */
/*****************************************************************************/


    .word __etext
    .word __data_start__
    .word __data_end__
    .word __bss_end__




    .syntax unified
    .arch armv7-m

    .section .isr_vector, "a"
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __StackTop                                      /* Top of Stack */
    .long   Reset_Handler                                   /* Reset Handler */
    .long   NMI_Handler                                     /* NMI Handler*/
    .long   HardFault_Handler                               /* Hard Fault Handler*/
    .long   MemManage_Handler                               /* MPU Fault Handler*/
    .long   BusFault_Handler                                /* Bus Fault Handler*/
    .long   UsageFault_Handler                              /* Usage Fault Handler*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   SVC_Handler                                     /* SVCall Handler*/
    .long   DebugMon_Handler                                /* Debug Monitor Handler*/
    .long   0                                               /* Reserved*/
    .long   PendSV_Handler                                  /* PendSV Handler*/
    .long   SysTick_Handler                                 /* SysTick Handler*/

                                                            /* External Interrupts*/
    .long   DefaultISR                                      /* 16*/
    .long   DefaultISR                                      /* 17*/
    .long   DefaultISR                                      /* 18*/
    .long   DefaultISR                                      /* 19*/
    .long   DefaultISR                                      /* 20*/
    .long   DefaultISR                                      /* 21*/
    .long   DefaultISR                                      /* 22*/
    .long   DefaultISR                                      /* 23*/
    .long   DefaultISR                                      /* 24*/
    .long   DefaultISR                                      /* 25*/
    .long   WDOG3_Handler                                   /* WDOG3 Handler*/
    .long   SEMA4_Handler                                   /* SEMA4_Handler*/
    .long   DefaultISR                                      /* 28*/
    .long   DefaultISR                                      /* 29*/
    .long   DefaultISR                                      /* 30*/
    .long   DefaultISR                                      /* 31*/
    .long   UART6_Handler                                   /* UART6 Handler*/
    .long   DefaultISR                                      /* 33*/
    .long   DefaultISR                                      /* 34*/
    .long   DefaultISR                                      /* 35*/
    .long   DefaultISR                                      /* 36*/
    .long   DefaultISR                                      /* 37*/
    .long   DefaultISR                                      /* 38*/
    .long   DefaultISR                                      /* 39*/
    .long   DefaultISR                                      /* 40*/
    .long   DefaultISR                                      /* 41*/
    .long   UART1_Handler                                   /* UART1 Handler*/
    .long   UART2_Handler                                   /* UART2 Handler*/
    .long   UART3_Handler                                   /* UART3 Handler*/
    .long   UART4_Handler                                   /* UART4 Handler*/
    .long   UART5_Handler                                   /* UART5 Handler*/
    .long   eCSPI1_Handler                                  /* eCSPI1 Handler*/
    .long   eCSPI2_Handler                                  /* eCSPI2 Handler*/
    .long   eCSPI3_Handler                                  /* eCSPI3 Handler*/
    .long   eCSPI4_Handler                                  /* eCSPI4 Handler*/
    .long   I2C1_Handler                                    /* I2C1 Handler*/
    .long   I2C2_Handler                                    /* I2C2 Handler*/
    .long   I2C3_Handler                                    /* I2C3 Handler*/
    .long   I2C4_Handler                                    /* I2C4 Handler*/
    .long   DefaultISR                                      /* 55*/
    .long   DefaultISR                                      /* 56*/
    .long   DefaultISR                                      /* 57*/
    .long   DefaultISR                                      /* 58*/
    .long   DefaultISR                                      /* 59*/
    .long   DefaultISR                                      /* 60*/
    .long   DefaultISR                                      /* 61*/
    .long   DefaultISR                                      /* 62*/
    .long   DefaultISR                                      /* 63*/
    .long   DefaultISR                                      /* 64*/
    .long   DefaultISR                                      /* 65*/
    .long   DefaultISR                                      /* 66*/
    .long   DefaultISR                                      /* 67*/
    .long   GPT4_Handler                                    /* GPT4 handler*/
    .long   GPT3_Handler                                    /* GPT3 handler*/
    .long   GPT2_Handler                                    /* GPT2 handler*/
    .long   GPT1_Handler                                    /* GPT1 handler*/
    .long   GPIO1_INT7_Handler                              /* Active HIGH Interrupt from INT7 from GPIO*/
    .long   GPIO1_INT6_Handler                              /* Active HIGH Interrupt from INT6 from GPIO*/
    .long   GPIO1_INT5_Handler                              /* Active HIGH Interrupt from INT5 from GPIO*/
    .long   GPIO1_INT4_Handler                              /* Active HIGH Interrupt from INT4 from GPIO*/
    .long   GPIO1_INT3_Handler                              /* Active HIGH Interrupt from INT3 from GPIO*/
    .long   GPIO1_INT2_Handler                              /* Active HIGH Interrupt from INT2 from GPIO*/
    .long   GPIO1_INT1_Handler                              /* Active HIGH Interrupt from INT1 from GPIO*/
    .long   GPIO1_INT0_Handler                              /* Active HIGH Interrupt from INT0 from GPIO*/
    .long   GPIO1_INT15_0_Handler                           /* Combined interrupt indication for GPIO1 signal 0 throughout 15*/
    .long   GPIO1_INT31_16_Handler                          /* Combined interrupt indication for GPIO1 signal 16 throughout 31*/
    .long   GPIO2_INT15_0_Handler                           /* Combined interrupt indication for GPIO2 signal 0 throughout 15*/
    .long   GPIO2_INT31_16_Handler                          /* Combined interrupt indication for GPIO2 signal 16 throughout 31*/
    .long   GPIO3_INT15_0_Handler                           /* Combined interrupt indication for GPIO3 signal 0 throughout 15*/
    .long   GPIO3_INT31_16_Handler                          /* Combined interrupt indication for GPIO3 signal 16 throughout 31*/
    .long   GPIO4_INT15_0_Handler                           /* Combined interrupt indication for GPIO4 signal 0 throughout 15*/
    .long   GPIO4_INT31_16_Handler                          /* Combined interrupt indication for GPIO4 signal 16 throughout 31*/
    .long   GPIO5_INT15_0_Handler                           /* Combined interrupt indication for GPIO5 signal 0 throughout 15*/
    .long   GPIO5_INT31_16_Handler                          /* Combined interrupt indication for GPIO5 signal 16 throughout 31*/
    .long   GPIO6_INT15_0_Handler                           /* Combined interrupt indication for GPIO6 signal 0 throughout 15*/
    .long   GPIO6_INT31_16_Handler                          /* Combined interrupt indication for GPIO6 signal 16 throughout 31*/
    .long   GPIO7_INT15_0_Handler                           /* Combined interrupt indication for GPIO7 signal 0 throughout 15*/
    .long   GPIO7_INT31_16_Handler                          /* Combined interrupt indication for GPIO7 signal 16 throughout 31*/
    .long   DefaultISR                                      /* 94*/
    .long   DefaultISR                                      /* 95*/
    .long   DefaultISR                                      /* 96*/
    .long   DefaultISR                                      /* 97*/
    .long   DefaultISR                                      /* 98*/
    .long   DefaultISR                                      /* 99*/
    .long   DefaultISR                                      /* 100*/
    .long   DefaultISR                                      /* 101*/
    .long   DefaultISR                                      /* 102*/
    .long   DefaultISR                                      /* 103*/
    .long   DefaultISR                                      /* 104*/
    .long   DefaultISR                                      /* 105*/
    .long   DefaultISR                                      /* 106*/
    .long   DefaultISR                                      /* 107*/
    .long   DefaultISR                                      /* 108*/
    .long   DefaultISR                                      /* 109*/
    .long   DefaultISR                                      /* 110*/
    .long   DefaultISR                                      /* 111*/
    .long   DefaultISR                                      /* 112*/
    .long   MU_Handler                                      /* MU Handler*/
    .long   ADC1_Handler                                    /* ADC1 Handler*/
    .long   ADC2_Handler                                    /* ADC2 Handler*/
    .long   DefaultISR                                      /* 116*/
    .long   DefaultISR                                      /* 117*/
    .long   DefaultISR                                      /* 118*/
    .long   DefaultISR                                      /* 119*/
    .long   DefaultISR                                      /* 120*/
    .long   DefaultISR                                      /* 121*/
    .long   DefaultISR                                      /* 122*/
    .long   DefaultISR                                      /* 123*/
    .long   DefaultISR                                      /* 124*/
    .long   DefaultISR                                      /* 125*/
    .long   FLEXCAN1_Handler                                /* FLEXCAN1 Handler*/
    .long   FLEXCAN2_Handler                                /* FLEXCAN2 Handler*/
    .long   DefaultISR                                      /* 128*/
    .long   DefaultISR                                      /* 129*/
    .long   DefaultISR                                      /* 130*/
    .long   DefaultISR                                      /* 131*/
    .long   DefaultISR                                      /* 132*/
    .long   DefaultISR                                      /* 133*/
    .long   DefaultISR                                      /* 134*/
    .long   DefaultISR                                      /* 135*/
    .long   DefaultISR                                      /* 136*/
    .long   DefaultISR                                      /* 137*/
    .long   DefaultISR                                      /* 138*/
    .long   DefaultISR                                      /* 139*/
    .long   DefaultISR                                      /* 140*/
    .long   DefaultISR                                      /* 141*/
    .long   UART7_Handler                                   /* UART7 Handler*/
    .long   DefaultISR                                      /* 143*/

    .size    __isr_vector, . - __isr_vector



    .text
    .thumb

/* Reset Handler */

    .thumb_func
    .align 2
    .globl   Reset_Handler
    .weak    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    cpsid   i               /* Mask interrupts */
#ifndef __NO_SYSTEM_INIT
    bl SystemInit
#endif
    /* data copy */
    ldr r0,=__DATA_ROM
    subs r0,r0,#0x1
    ldr r1,=__data_start__
    subs r1,r1,#0x1
    ldr r2,=__data_end__
    subs r3,r2,r1
    b  Copy_init_data
  Loop_copy_init_data:
    adds r1,r1,#0x1
    adds r0,r0,#0x1
    ldrb r4,[r0]
    str r4,[r1]

  Copy_init_data:
    subs r3,r3,#0x1
    cmp r3,#0x0
    bne Loop_copy_init_data

    cpsie   i               /* Unmask interrupts */
    bl _start
    .pool
    .size Reset_Handler, . - Reset_Handler

    .align    1
    .thumb_func
    .weak DefaultISR
    .type DefaultISR, %function
DefaultISR:
    b    DefaultISR
    .size DefaultISR, . - DefaultISR

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro def_irq_handler    handler_name
    .weak \handler_name
    .set  \handler_name, DefaultISR
    .endm

/* Exception Handlers */
    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    WDOG3_Handler
    def_irq_handler    SEMA4_Handler
    def_irq_handler    UART6_Handler
    def_irq_handler    UART1_Handler
    def_irq_handler    UART2_Handler
    def_irq_handler    UART3_Handler
    def_irq_handler    UART4_Handler
    def_irq_handler    UART5_Handler
    def_irq_handler    eCSPI1_Handler
    def_irq_handler    eCSPI2_Handler
    def_irq_handler    eCSPI3_Handler
    def_irq_handler    eCSPI4_Handler
    def_irq_handler    I2C1_Handler
    def_irq_handler    I2C2_Handler
    def_irq_handler    I2C3_Handler
    def_irq_handler    I2C4_Handler
    def_irq_handler    GPT4_Handler
    def_irq_handler    GPT3_Handler
    def_irq_handler    GPT2_Handler
    def_irq_handler    GPT1_Handler
    def_irq_handler    GPIO1_INT7_Handler
    def_irq_handler    GPIO1_INT6_Handler
    def_irq_handler    GPIO1_INT5_Handler
    def_irq_handler    GPIO1_INT4_Handler
    def_irq_handler    GPIO1_INT3_Handler
    def_irq_handler    GPIO1_INT2_Handler
    def_irq_handler    GPIO1_INT1_Handler
    def_irq_handler    GPIO1_INT0_Handler
    def_irq_handler    GPIO1_INT15_0_Handler
    def_irq_handler    GPIO1_INT31_16_Handler
    def_irq_handler    GPIO2_INT15_0_Handler
    def_irq_handler    GPIO2_INT31_16_Handler
    def_irq_handler    GPIO3_INT15_0_Handler
    def_irq_handler    GPIO3_INT31_16_Handler
    def_irq_handler    GPIO4_INT15_0_Handler
    def_irq_handler    GPIO4_INT31_16_Handler
    def_irq_handler    GPIO5_INT15_0_Handler
    def_irq_handler    GPIO5_INT31_16_Handler
    def_irq_handler    GPIO6_INT15_0_Handler
    def_irq_handler    GPIO6_INT31_16_Handler
    def_irq_handler    GPIO7_INT15_0_Handler
    def_irq_handler    GPIO7_INT31_16_Handler
    def_irq_handler    MU_Handler
    def_irq_handler    ADC1_Handler
    def_irq_handler    ADC2_Handler
    def_irq_handler    FLEXCAN1_Handler
    def_irq_handler    FLEXCAN2_Handler
    def_irq_handler    UART7_Handler
    .end
