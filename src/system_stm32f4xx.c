/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @brief   Minimal STM32F4xx system clock initialization for bare-metal
  *          Configures PLL to 84 MHz from 8 MHz HSE
  *          Sets up clock distribution for APB1 (42 MHz) and APB2 (84 MHz)
  ******************************************************************************
  */

#include <stdint.h>

/* RCC Register Base Address */
#define RCC_BASE           0x40023800

/* RCC Registers */
#define RCC_CR             (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_PLLCFGR        (*(volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CFGR           (*(volatile uint32_t *)(RCC_BASE + 0x08))
#define RCC_AHB1ENR        (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR        (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_APB2ENR        (*(volatile uint32_t *)(RCC_BASE + 0x44))

/* FLASH Register Base Address */
#define FLASH_BASE         0x40023C00
#define FLASH_ACR          (*(volatile uint32_t *)(FLASH_BASE + 0x00))

/* NVIC Register Base Address */
#define NVIC_BASE          0xE000E000
#define NVIC_ISER0         (*(volatile uint32_t *)(NVIC_BASE + 0x100))

/**
 * @brief Initialize the system clock
 *
 * This function configures:
 *   - HSE at 8 MHz
 *   - PLL with M=4, N=336, P=4, Q=7
 *   - Main clock = 84 MHz
 *   - APB1 = 42 MHz (clock for TIM2, UART)
 *   - APB2 = 84 MHz (clock for ADC, DMA2)
 *
 * Calculation: Fvco = Fin * (N/M) = 8 * (336/4) = 168 MHz
 *              Fout = Fvco / P = 168 / 2 = 84 MHz
 */
void SystemInit(void)
{
    /* Enable HSE */
    RCC_CR |= (1 << 16);                    /* HSEON = 1 */
    while (!(RCC_CR & (1 << 17)));          /* Wait for HSERDY */

    /* Set FLASH latency to 2 wait states (required for 84 MHz) */
    FLASH_ACR = (FLASH_ACR & ~0x07) | 2;

    /* Configure PLL for 84 MHz SYSCLK from 8 MHz HSE
     * VCO input  = 8 MHz / PLLM = 2 MHz
     * VCO output = 2 MHz * PLLN = 336 MHz
     * SYSCLK     = 336 MHz / PLLP = 84 MHz
     * USB clock  = 336 MHz / PLLQ = 48 MHz
     * Settings: PLLM=4, PLLN=168, PLLP=/4, PLLQ=7
     */
    RCC_PLLCFGR = 0;
    RCC_PLLCFGR |= (4 << 0);                /* PLLM = 4 (VCO in = 2 MHz) */
    RCC_PLLCFGR |= (168 << 6);              /* PLLN = 168 (VCO out = 336 MHz) */
    RCC_PLLCFGR |= (1 << 16);               /* PLLP = 01 => divide by 4 */
    RCC_PLLCFGR |= (7 << 24);               /* PLLQ = 7 (USB 48 MHz) */
    RCC_PLLCFGR |= (1 << 22);               /* PLLSRC = HSE */

    /* Enable PLL */
    RCC_CR |= (1 << 24);                    /* PLLON = 1 */
    while (!(RCC_CR & (1 << 25)));          /* Wait for PLLRDY */

    /* Configure CFGR: Set prescalers and select PLL as system clock */
    /* CFGR = (APB2 << 13) | (APB1 << 10) | (AHB << 4) | (SW) */
    RCC_CFGR = 0;
    RCC_CFGR |= (0 << 13);                  /* APB2 prescaler = 1 (84 MHz) */
    RCC_CFGR |= (4 << 10);                  /* APB1 prescaler = 2 (42 MHz) */
    RCC_CFGR |= (0 << 4);                   /* AHB prescaler = 1 (84 MHz) */
    RCC_CFGR |= (2);                        /* SW = 2 (select PLL) */

    /* Wait until PLL is selected as system clock */
    while ((RCC_CFGR & (3 << 2)) != (2 << 2));

    /* Enable peripheral clocks needed for this project */
    /* AHB1 Peripherals */
    RCC_AHB1ENR |= (1 << 0);                /* GPIOA clock enable */
    RCC_AHB1ENR |= (1 << 1);                /* GPIOB clock enable */
    RCC_AHB1ENR |= (1 << 2);                /* GPIOC clock enable */
    RCC_AHB1ENR |= (1 << 21);               /* DMA2 clock enable */

    /* APB1 Peripherals (42 MHz clock) */
    RCC_APB1ENR |= (1 << 0);                /* TIM2 clock enable */
    RCC_APB1ENR |= (1 << 17);               /* USART2 clock enable */

    /* APB2 Peripherals (84 MHz clock) */
    RCC_APB2ENR |= (1 << 8);                /* ADC1 clock enable */
}
