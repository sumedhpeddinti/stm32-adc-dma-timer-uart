/**
  ******************************************************************************
  * @file    stm32f4xx.h
  * @brief   Minimal STM32F4xx CMSIS header stub
  *
  * This is a minimal header file providing basic CMSIS definitions.
  * For production use, consider using official CMSIS-Core files from ST.
  ******************************************************************************
  */

#ifndef STM32F4XX_H
#define STM32F4XX_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
   ARM Cortex-M4 Processor and Core Peripherals
   ============================================================================ */

#define __CM4_REV              0x0001U  /*!< Cortex-M4 revision r0p1 */
#define __MPU_PRESENT          1U       /*!< STM32F4xx provides an MPU */
#define __NVIC_PRIO_BITS       4U       /*!< STM32F4xx uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig 0U       /*!< Set to 1 if different SysTick Config is used */

/* Processor and Core Peripherals */
#include <core_cm4.h>

/* ============================================================================
   Device-specific defines
   ============================================================================ */

#define STM32F401xC  1U

/* ============================================================================
   Memory mapping
   ============================================================================ */

#define FLASH_BASE           0x08000000U    /*!< FLASH base address in the alias region */
#define SRAM1_BASE           0x20000000U    /*!< SRAM1(112 KB) base address in the alias region */
#define PERIPH_BASE          0x40000000U    /*!< Peripheral base address in the alias region */

#define APB1PERIPH_BASE      (PERIPH_BASE)
#define APB2PERIPH_BASE      (PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE      (PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE      (PERIPH_BASE + 0x08000000U)

/* ============================================================================
   Peripheral Base Addresses
   ============================================================================ */

#define TIM2_BASE            (APB1PERIPH_BASE + 0x0000U)
#define USART2_BASE          (APB1PERIPH_BASE + 0x4400U)
#define ADC1_BASE            (APB2PERIPH_BASE + 0x2000U)
#define GPIOA_BASE           (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE           (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE           (AHB1PERIPH_BASE + 0x0800U)
#define DMA2_BASE            (AHB1PERIPH_BASE + 0x6400U)
#define RCC_BASE             (AHB1PERIPH_BASE + 0x3800U)

/* ============================================================================
   Type definitions
   ============================================================================ */

typedef int32_t  s32;
typedef int16_t  s16;
typedef int8_t   s8;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;
typedef volatile uint32_t vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

/* ============================================================================
   Exported functions
   ============================================================================ */

extern void SystemInit(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_H */
