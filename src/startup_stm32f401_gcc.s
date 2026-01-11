/**
  ******************************************************************************
  * @file      startup_stm32f401xc_gcc.s
  * @author    Embedded Firmware
  * @brief     STM32F401CC Cortex-M4 CMSIS startup code for GCC
  ******************************************************************************
  */

    .syntax unified
    .cpu cortex-m4
    .fpu softvfp
    .thumb

.global g_pfnVectors
.global Default_Handler

/* Weak alias for optional TIM2 handler (used in SIMULATION) */
.weak TIM2_IRQHandler
.set TIM2_IRQHandler, Default_Handler

/* Memory Addresses (STM32F401CC) */
.word _sidata
.word _sdata
.word _edata
.word _sbss
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *         starts execution following a reset event.
 */
    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function

Reset_Handler:
    ldr   r0, =_estack          /* set stack pointer to end of RAM */
    mov   sp, r0

    /* Copy the data segment initializers from flash to RAM */
    mov   r1, #0x0
    b     Copy_Data_Loop

Copy_Data:
    ldr   r3, =_sdata
    ldr   r3, [r0, r1]
    str   r3, [r1, r3]
    add   r1, r1, #4

Copy_Data_Loop:
    ldr   r0, =_edata
    ldr   r3, =_sdata
    adds  r2, r0, r1
    cmp   r2, r3
    bcc   Copy_Data

    /* Zero fill the bss segment. */
    mov   r2, #0
    b     Zero_Fill_Loop

Zero_Fill:
    ldr   r3, =_ebss
    str   r2, [r3]
    add   r3, r3, #4

Zero_Fill_Loop:
    ldr   r3, =_sbss
    cmp   r3, r3
    bcc   Zero_Fill

    /* Call the clock system intitialization function.*/
    bl    SystemInit

    /* Call the application entry point.*/
    bl    main
    bx    lr

    .size Reset_Handler, .-Reset_Handler

/**
 * @brief  This function handles Memory Manage exception.
 */
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
    b     Default_Handler

    .size Default_Handler, .-Default_Handler

/**
 * @brief  STM32F4xx Interrupt Vector Table
 */
    .section .isr_vector,"a",%progbits
    .type g_pfnVectors, %object
    .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    .word  _estack                      /* 0   Top of Stack */
    .word  Reset_Handler                /* 1   Reset Handler */
    .word  Default_Handler              /* 2   NMI Handler */
    .word  Default_Handler              /* 3   Hard Fault Handler */
    .word  Default_Handler              /* 4   Memory Management Fault Handler */
    .word  Default_Handler              /* 5   Bus Fault Handler */
    .word  Default_Handler              /* 6   Usage Fault Handler */
    .word  0                            /* 7   Reserved */
    .word  0                            /* 8   Reserved */
    .word  0                            /* 9   Reserved */
    .word  0                            /* 10  Reserved */
    .word  Default_Handler              /* 11  SVCall Handler */
    .word  Default_Handler              /* 12  Debug Monitor Handler */
    .word  0                            /* 13  Reserved */
    .word  Default_Handler              /* 14  PendSV Handler */
    .word  Default_Handler              /* 15  SysTick Handler */

    /* External Interrupts */
    .word  Default_Handler              /* 16  WWDG */
    .word  Default_Handler              /* 17  PVD */
    .word  Default_Handler              /* 18  TAMP_STAMP */
    .word  Default_Handler              /* 19  RTC_WKUP */
    .word  Default_Handler              /* 20  FLASH */
    .word  Default_Handler              /* 21  RCC */
    .word  Default_Handler              /* 22  EXTI0 */
    .word  Default_Handler              /* 23  EXTI1 */
    .word  Default_Handler              /* 24  EXTI2 */
    .word  Default_Handler              /* 25  EXTI3 */
    .word  Default_Handler              /* 26  EXTI4 */
    .word  Default_Handler              /* 27  DMA1_Stream0 */
    .word  Default_Handler              /* 28  DMA1_Stream1 */
    .word  Default_Handler              /* 29  DMA1_Stream2 */
    .word  Default_Handler              /* 30  DMA1_Stream3 */
    .word  Default_Handler              /* 31  DMA1_Stream4 */
    .word  Default_Handler              /* 32  DMA1_Stream5 */
    .word  Default_Handler              /* 33  DMA1_Stream6 */
    .word  Default_Handler              /* 34  ADC1, ADC2, ADC3 */
    .word  Default_Handler              /* 35  CAN1_TX */
    .word  Default_Handler              /* 36  CAN1_RX0 */
    .word  Default_Handler              /* 37  CAN1_RX1 */
    .word  Default_Handler              /* 38  CAN1_SCE */
    .word  Default_Handler              /* 39  EXTI9_5 */
    .word  Default_Handler              /* 40  TIM1_BRK_TIM9 */
    .word  Default_Handler              /* 41  TIM1_UP_TIM10 */
    .word  Default_Handler              /* 42  TIM1_TRG_COM_TIM11 */
    .word  Default_Handler              /* 43  TIM1_CC */
    .word  TIM2_IRQHandler              /* 44  TIM2 */
    .word  Default_Handler              /* 45  TIM3 */
    .word  Default_Handler              /* 46  TIM4 */
    .word  Default_Handler              /* 47  I2C1_EV */
    .word  Default_Handler              /* 48  I2C1_ER */
    .word  Default_Handler              /* 49  I2C2_EV */
    .word  Default_Handler              /* 50  I2C2_ER */
    .word  Default_Handler              /* 51  SPI1 */
    .word  Default_Handler              /* 52  SPI2 */
    .word  Default_Handler              /* 53  USART1 */
    .word  Default_Handler              /* 54  USART2 */
    .word  Default_Handler              /* 55  USART3 */
    .word  Default_Handler              /* 56  EXTI15_10 */
    .word  Default_Handler              /* 57  RTC_Alarm */
    .word  Default_Handler              /* 58  OTG_FS_WKUP */
    .word  Default_Handler              /* 59  TIM1_BRK_TIM9 */
    .word  Default_Handler              /* 60  TIM1_UP_TIM10 */
    .word  Default_Handler              /* 61  TIM1_TRG_COM_TIM11 */
    .word  Default_Handler              /* 62  TIM1_CC */
    .word  Default_Handler              /* 63  SPI3 */
    .word  Default_Handler              /* 64  UART4 */
    .word  Default_Handler              /* 65  UART5 */
    .word  Default_Handler              /* 66  TIM6_DAC */
    .word  Default_Handler              /* 67  TIM7 */
    .word  Default_Handler              /* 68  DMA2_Stream0 */
    .word  DMA2_Stream0_IRQHandler       /* 69  DMA2_Stream0 - ADC DMA Complete */
    .word  Default_Handler              /* 70  DMA2_Stream1 */
    .word  Default_Handler              /* 71  DMA2_Stream2 */
    .word  Default_Handler              /* 72  DMA2_Stream3 */
    .word  Default_Handler              /* 73  DMA2_Stream4 */
    .word  Default_Handler              /* 74  DMA2_Stream5 */
    .word  Default_Handler              /* 75  DMA2_Stream6 */
    .word  Default_Handler              /* 76  DMA2_Stream7 */
    .word  Default_Handler              /* 77  USART6 */
    .word  Default_Handler              /* 78  I2C3_EV */
    .word  Default_Handler              /* 79  I2C3_ER */
    .word  Default_Handler              /* 80  OTG_HS_EP1_OUT */
    .word  Default_Handler              /* 81  OTG_HS_EP1_IN */
    .word  Default_Handler              /* 82  OTG_HS_WKUP */
    .word  Default_Handler              /* 83  OTG_HS */
    .word  Default_Handler              /* 84  DCMI */
    .word  Default_Handler              /* 85  CRYP */
    .word  Default_Handler              /* 86  HASH_RNG */
    .word  Default_Handler              /* 87  FPU */
    .word  Default_Handler              /* 88  UART7 */
    .word  Default_Handler              /* 89  UART8 */
    .word  Default_Handler              /* 90  SPI4 */
    .word  Default_Handler              /* 91  SPI5 */
    .word  Default_Handler              /* 92  SPI6 */
    .word  Default_Handler              /* 93  SAI1 */
    .word  Default_Handler              /* 94  LTDC */
    .word  Default_Handler              /* 95  LTDC_ER */
    .word  Default_Handler              /* 96  DMA2D */

.end
