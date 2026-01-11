/**
  ******************************************************************************
  * @file    main.c
  * @brief   STM32F401CC/F411CE Bare-metal ADC DMA UART firmware
  *
  * Configuration:
  *   - System Clock: 84 MHz (HSE 8 MHz + PLL)
  *   - TIM2: 100 Hz timer with TRGO output
  *   - ADC1: Triggered by TIM2 TRGO, samples PA0 (ADC1_IN0)
  *   - DMA2 Stream0: Transfers ADC results to RAM buffer (16 samples)
  *   - USART2: Sends formatted ADC data at 115200 baud
  *   - GPIO: Debug LED on PC13 (Blackpill onboard LED)
  *
  ******************************************************************************
  */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ============================================================================
   REGISTER DEFINITIONS (CMSIS-like bare-metal)
   ============================================================================ */

/* Base addresses */
#define PERIPH_BASE         0x40000000U
#define AHB1PERIPH_BASE     (PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE     (PERIPH_BASE + 0x08000000U)
#define APB1PERIPH_BASE     PERIPH_BASE
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x00010000U)

#define GPIOA_BASE          (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE          (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE          (AHB1PERIPH_BASE + 0x0800U)
#define DMA2_BASE           (AHB1PERIPH_BASE + 0x6400U)
#define RCC_BASE            (AHB1PERIPH_BASE + 0x3800U)
#define TIM2_BASE           (APB1PERIPH_BASE + 0x0000U)
#define USART2_BASE         (APB1PERIPH_BASE + 0x4400U)
#define ADC1_BASE           (APB2PERIPH_BASE + 0x2000U)

/* GPIO Registers */
#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE)

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
} GPIO_TypeDef;

/* TIM2 Registers */
#define TIM2                ((TIM_TypeDef *)TIM2_BASE)

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_TypeDef;

/* ADC Registers */
#define ADC1                ((ADC_TypeDef *)ADC1_BASE)

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t JOFR1;
    volatile uint32_t JOFR2;
    volatile uint32_t JOFR3;
    volatile uint32_t JOFR4;
    volatile uint32_t HTR;
    volatile uint32_t LTR;
    volatile uint32_t SQR1;
    volatile uint32_t SQR2;
    volatile uint32_t SQR3;
    volatile uint32_t JSQR;
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t JDR4;
    volatile uint32_t DR;
} ADC_TypeDef;

/* USART Registers */
#define USART2              ((USART_TypeDef *)USART2_BASE)

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

/* DMA2 Registers */
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t NDTR;
    volatile uint32_t PAR;
    volatile uint32_t M0AR;
    volatile uint32_t M1AR;
    volatile uint32_t FCR;
} DMA_Stream_TypeDef;

typedef struct {
    volatile uint32_t LISR;
    volatile uint32_t HISR;
    volatile uint32_t LIFCR;
    volatile uint32_t HIFCR;
    DMA_Stream_TypeDef Stream[8];
} DMA_TypeDef;

#define DMA2                ((DMA_TypeDef *)DMA2_BASE)

/* ============================================================================
   GLOBAL VARIABLES
   ============================================================================ */

/* ADC buffer: 16 samples of 16-bit values */
volatile uint16_t adc_buf[16];

/* DMA transfer complete flag */
volatile uint8_t dma_complete = 0;

/* Sample counter for debug/testing */
volatile uint32_t sample_count = 0;
#ifdef SIMULATION
static volatile uint32_t sim_index = 0;
#endif

/* ============================================================================
   FUNCTION PROTOTYPES
   ============================================================================ */

void SystemInit(void);
void GPIO_Init(void);
void TIM2_Init(void);
void ADC1_Init(void);
void DMA2_Init(void);
void USART2_Init(void);
void USART2_SendString(const char *str);
void USART2_SendChar(char c);
void DMA2_Stream0_IRQHandler(void);
#ifdef SIMULATION
void TIM2_IRQHandler(void);
#endif

/* ============================================================================
   GPIO INITIALIZATION
   ============================================================================ */

void GPIO_Init(void)
{
    /* PA0: ADC input (analog) - already in default analog mode */
    /* PA2: USART2 TX - set to AF7 (USART2) */
    /* PA3: USART2 RX - set to AF7 (USART2) */
    /* PC13: Debug LED output */

    /* Configure PA2 and PA3 as USART2 alternate functions */
    /* PA2: AF7 (USART2_TX) */
    GPIOA->MODER &= ~(3U << 4);             /* Clear mode for PA2 */
    GPIOA->MODER |= (2U << 4);              /* AF mode for PA2 */
    GPIOA->AFRH = 0;
    GPIOA->AFRL &= ~(0xFU << 8);            /* Clear AF for PA2 */
    GPIOA->AFRL |= (7U << 8);               /* AF7 for PA2 */

    /* PA3: AF7 (USART2_RX) */
    GPIOA->MODER &= ~(3U << 6);             /* Clear mode for PA3 */
    GPIOA->MODER |= (2U << 6);              /* AF mode for PA3 */
    GPIOA->AFRL &= ~(0xFU << 12);           /* Clear AF for PA3 */
    GPIOA->AFRL |= (7U << 12);              /* AF7 for PA3 */

    /* Configure PC13 as debug LED output (push-pull) */
    GPIOC->MODER &= ~(3U << 26);            /* Clear mode for PC13 */
    GPIOC->MODER |= (1U << 26);             /* Output mode for PC13 */
    GPIOC->OTYPER &= ~(1U << 13);           /* Push-pull */
    GPIOC->OSPEEDR &= ~(3U << 26);          /* Low speed */
    GPIOC->PUPDR &= ~(3U << 26);            /* No pull-up/down */

    /* Set PC13 initially low */
    GPIOC->ODR &= ~(1U << 13);
}

/* ============================================================================
   TIMER2 INITIALIZATION (100 Hz)
   ============================================================================ */

void TIM2_Init(void)
{
    /**
     * TIM2 Configuration for 100 Hz update rate:
     *   - Timer clock: 84 MHz (APB1 timer clock)
     *   - Desired frequency: 100 Hz (10 ms period)
     *   - Counter period = 84,000,000 / 100 = 840,000 clocks
     *   - Using PSC = 839, ARR = 99,999 gives: 84MHz / 840 / 100 = ~100 Hz
     *     More precisely: 84MHz / 840 = 100kHz, then 100kHz / 1000 = 100 Hz
     *     Actually: PSC = 839 (divide by 840), ARR = 9999 (divide by 10000)
     *     Frequency = 84MHz / 840 / 10000 = 10 Hz... need to recalculate
     *
     *   Correct calculation:
     *     For 100 Hz: f = 84MHz / ((PSC+1) * (ARR+1))
     *     100 = 84,000,000 / ((PSC+1) * (ARR+1))
     *     (PSC+1) * (ARR+1) = 840,000
     *     Let PSC+1 = 1000, then ARR+1 = 840
     *     So PSC = 999, ARR = 839
     *     Frequency = 84MHz / 1000 / 840 = 100 Hz ✓
     */
    TIM2->PSC = 999;                        /* Prescaler: divide by 1000 */
    TIM2->ARR = 839;                        /* Auto-reload: 840 counts = 100 Hz */
    TIM2->DIER |= (1 << 0);                 /* Enable update interrupt (for debug) */

    /* Configure TRGO output on update event */
    /* CR2: MMS = 010 (Update event) */
    TIM2->CR2 &= ~(7 << 4);                 /* Clear MMS bits */
    TIM2->CR2 |= (2 << 4);                  /* MMS = 010 (Update event as TRGO) */

    /* Counter mode: up-counting */
    TIM2->CR1 &= ~(1 << 4);                 /* DIR = 0 (up-counting) */

    /* Edge-aligned counting */
    TIM2->CR1 &= ~(3 << 5);                 /* CMS = 00 (edge-aligned) */

    /* Enable TIM2 counter */
    TIM2->CR1 |= (1 << 0);                  /* CEN = 1 */
}

/* ============================================================================
   ADC1 INITIALIZATION
   ============================================================================ */

void ADC1_Init(void)
{
#ifdef SIMULATION
    for (volatile int i = 0; i < 1000; i++);
    return;
#endif
    /**
     * ADC1 Configuration:
     *   - Input: PA0 (ADC1_IN0)
     *   - Trigger: TIM2 TRGO (external trigger)
     *   - Mode: Single conversion
     *   - Sampling time: 144 cycles (slow but stable)
     *   - Resolution: 12-bit
     *   - Right-aligned data
     */

    /* Disable ADC during configuration */
    ADC1->CR2 &= ~(1 << 0);                 /* ADON = 0 */

    /* ADC CR1: Configure interrupt flags, resolution, etc. */
    ADC1->CR1 = 0;
    /* Default: 12-bit resolution, no interrupt masks */

    /* ADC CR2: Configure trigger and mode */
    ADC1->CR2 = 0;
    /* Enable DMA requests and DMA in continuous mode (DDS) */
    ADC1->CR2 |= (1 << 8);                  /* DMA = 1 */
    ADC1->CR2 |= (1 << 9);                  /* DDS = 1 (DMA requests after last transfer) */
    /* End-of-conversion at each conversion */
    ADC1->CR2 |= (1 << 10);                 /* EOCS = 1 */
    /* External trigger on rising edge from TIM2_TRGO */
    ADC1->CR2 |= (1 << 28);                 /* EXTEN = 01 (rising edge) */
    ADC1->CR2 &= ~(0xF << 24);
    ADC1->CR2 |= (6 << 24);                 /* EXTSEL = 0110 (TIM2_TRGO) */

    /* ADC SMPR2: Sampling time for channels 0-9 */
    /* For PA0 (ADC1_IN0), set 144 cycles (111 in 3-bit field) */
    ADC1->SMPR2 &= ~(7 << 0);               /* Clear SMP0 bits */
    ADC1->SMPR2 |= (7 << 0);                /* SMP0 = 111 (144 cycles) */

    /* ADC SQR1: Sequence length = 1 conversion */
    ADC1->SQR1 &= ~(15 << 20);              /* Clear L bits */
    ADC1->SQR1 |= (0 << 20);                /* L = 0 (1 conversion) */

    /* ADC SQR3: Sequence 1 = channel 0 (PA0) */
    ADC1->SQR3 &= ~(31 << 0);               /* Clear SQ1 bits */
    ADC1->SQR3 |= (0 << 0);                 /* SQ1 = 0 (ADC1_IN0) */

    /* Enable ADC */
    ADC1->CR2 |= (1 << 0);                  /* ADON = 1 */

    /* Wait for ADC stabilization (~3 microseconds) */
    for (volatile int i = 0; i < 1000; i++);
}

/* ============================================================================
   DMA2 INITIALIZATION
   ============================================================================ */

void DMA2_Init(void)
{
#ifdef SIMULATION
    return;
#endif
    /**
     * DMA2 Stream0 Configuration:
     *   - Channel: 0 (ADC1)
     *   - Source: ADC1->DR (peripheral)
     *   - Destination: adc_buf (RAM)
     *   - Mode: Circular
     *   - Data width: 16-bit (half-word)
     *   - Burst: No burst
     *   - Flow control: DMA
     *
     * Note: DMA2 Stream0 Channel0 is used for ADC1 on STM32F4
     */

    /* Disable DMA Stream0 during configuration */
    DMA2->Stream[0].CR &= ~(1 << 0);        /* EN = 0 */
    while (DMA2->Stream[0].CR & (1 << 0));  /* Wait until disabled */

    /* Clear all interrupt flags for Stream0 */
    DMA2->LIFCR = (0x3D << 0);              /* Clear all flags for Stream0 */

    /* DMA2 Stream0 CR Configuration */
    DMA2->Stream[0].CR = 0;
    DMA2->Stream[0].CR |= (0 << 25);        /* CHSEL = 000 (Channel 0 for ADC1) */
    DMA2->Stream[0].CR |= (3 << 16);        /* PL = 11 (very high priority) */
    DMA2->Stream[0].CR &= ~(1 << 9);        /* PINC = 0 (peripheral address fixed) */
    DMA2->Stream[0].CR |= (1 << 10);        /* MINC = 1 (memory address increment) */
    DMA2->Stream[0].CR &= ~(3 << 6);        /* DIR = 00 (peripheral-to-memory) */
    DMA2->Stream[0].CR |= (1 << 8);         /* CIRC = 1 (circular mode) */
    DMA2->Stream[0].CR &= ~(3 << 11);       /* Clear PSIZE */
    DMA2->Stream[0].CR |= (1 << 11);        /* PSIZE = 01 (16-bit) */
    DMA2->Stream[0].CR &= ~(3 << 13);       /* Clear MSIZE */
    DMA2->Stream[0].CR |= (1 << 13);        /* MSIZE = 01 (16-bit) */
    DMA2->Stream[0].CR |= (1 << 4);         /* TCIE = 1 (transfer complete interrupt) */

    /* DMA2 Stream0 NDTR: Number of data items to transfer */
    DMA2->Stream[0].NDTR = 16;              /* Transfer 16 samples */

    /* DMA2 Stream0 PAR: Peripheral address (ADC1->DR) */
    DMA2->Stream[0].PAR = (uint32_t)&ADC1->DR;

    /* DMA2 Stream0 M0AR: Memory address (adc_buf) */
    DMA2->Stream[0].M0AR = (uint32_t)adc_buf;

    /* DMA2 Stream0 FCR: FIFO configuration */
    DMA2->Stream[0].FCR = 0;
    DMA2->Stream[0].FCR |= (0 << 2);        /* FTH = 00 (1/4 FIFO) */
    DMA2->Stream[0].FCR &= ~(1 << 2);       /* DMDIS = 0 (direct mode enabled) */

    /* Enable interrupt on transfer complete */
    DMA2->Stream[0].CR |= (1 << 4);         /* TCIE = 1 (transfer complete interrupt enable) */

    /* Enable DMA2 Stream0 */
    DMA2->Stream[0].CR |= (1 << 0);         /* EN = 1 */
}

/* ============================================================================
   USART2 INITIALIZATION (115200 baud)
   ============================================================================ */

void USART2_Init(void)
{
    /**
     * USART2 Configuration:
     *   - Baud rate: 115200 bps
     *   - Data bits: 8
     *   - Stop bits: 1
     *   - Parity: None
     *   - APB1 clock: 42 MHz
     *   - BRR = APB1_Clock / (16 * BaudRate)
     *        = 42,000,000 / (16 * 115,200)
     *        = 42,000,000 / 1,843,200
     *        ≈ 22.785... ≈ 23 (for ~115,384 baud, close enough)
     *
     *   Actually more accurate:
     *   Integer part (BRR[15:4]): 22, Fraction part (BRR[3:0]): 13
     *   BRR = 0x16D = (22 << 4) | 13
     */

    /* Disable USART2 during configuration */
    USART2->CR1 &= ~(1 << 13);              /* UE = 0 */

    /* Configure baud rate: 42 MHz / (16*115200) = 22.785 -> BRR=0x16D */
    USART2->BRR = 0x016D;                   /* BRR for 115200 baud at 42 MHz */

    /* Configure USART2 CR1 */
    USART2->CR1 = 0;
    USART2->CR1 |= (1 << 13);               /* UE = 1 (enable USART) */
    USART2->CR1 |= (1 << 3);                /* TE = 1 (transmitter enable) */
    USART2->CR1 |= (1 << 2);                /* RE = 1 (receiver enable) */
    USART2->CR1 |= (0 << 5);                /* RXNEIE = 0 (no RX interrupt) */
    USART2->CR1 |= (0 << 6);                /* TXEIE = 0 (no TX interrupt) */
    USART2->CR1 |= (0 << 7);                /* TCIE = 0 (no transfer complete interrupt) */

    /* Configure USART2 CR2 */
    USART2->CR2 = 0;
    USART2->CR2 |= (0 << 12);               /* STOP = 00 (1 stop bit) */

    /* Configure USART2 CR3 */
    USART2->CR3 = 0;
}

/* ============================================================================
   USART2 POLLING TRANSMIT FUNCTIONS
   ============================================================================ */

void USART2_SendChar(char c)
{
    /* Wait until TXE (Transmit Data Register Empty) is set */
    while (!(USART2->SR & (1 << 7)));       /* TXE = 1 */
    USART2->DR = c;                         /* Send character */
}

void USART2_SendString(const char *str)
{
    while (*str) {
        USART2_SendChar(*str++);
    }
}

/* ============================================================================
   DMA2 TRANSFER COMPLETE INTERRUPT HANDLER
   ============================================================================ */

void DMA2_Stream0_IRQHandler(void)
{
    /* Check if transfer complete interrupt flag is set */
    if (DMA2->LISR & (1 << 5)) {            /* TCIF0 = 1 */
        /* Clear the transfer complete flag */
        DMA2->LIFCR |= (1 << 5);            /* CTCIF0 = 1 */

        /* Toggle debug LED on PC13 */
        GPIOC->ODR ^= (1 << 13);

        /* Stop DMA */
        DMA2->Stream[0].CR &= ~(1 << 0);    /* EN = 0 */

        /* Format and send the 16 ADC values via UART */
        char buffer[256];
        int len = 0;

        len += snprintf(buffer + len, sizeof(buffer) - len,
                        "Sample #%lu: ", sample_count);

        for (int i = 0; i < 16; i++) {
            len += snprintf(buffer + len, sizeof(buffer) - len,
                            "%04X ", adc_buf[i]);
        }

        len += snprintf(buffer + len, sizeof(buffer) - len, "\r\n");

        /* Send via UART */
        USART2_SendString(buffer);

        /* Increment sample counter */
        sample_count++;

        /* Restart DMA for next 16 samples */
        DMA2->Stream[0].NDTR = 16;          /* Reload count */
        DMA2->Stream[0].CR |= (1 << 0);     /* EN = 1 */

        dma_complete = 1;
    }
}

/* ============================================================================
   ENABLE INTERRUPT IN NVIC
   ============================================================================ */

void NVIC_Enable_DMA2_Stream0(void)
{
    /* DMA2_Stream0 is IRQ 56 */
    /* NVIC_ISER register index: 56 / 32 = 1, bit position: 56 % 32 = 24 */
    volatile uint32_t *NVIC_ISER1 = (volatile uint32_t *)0xE000E104;
    *NVIC_ISER1 |= (1 << 24);
}

/* ============================================================================
   MAIN FUNCTION
   ============================================================================ */

int main(void)
{
    /* Initialize peripherals */
    GPIO_Init();
    TIM2_Init();
    ADC1_Init();
    USART2_Init();
    DMA2_Init();

    /* Enable DMA2 Stream0 interrupt in NVIC */
#ifndef SIMULATION
    NVIC_Enable_DMA2_Stream0();
#else
    /* Enable TIM2 IRQ (IRQn = 28) for simulation-driven sampling */
    volatile uint32_t *NVIC_ISER0 = (volatile uint32_t *)0xE000E100;
    *NVIC_ISER0 |= (1u << 28);
#endif

    /* Send startup message */
    USART2_SendString("\r\n=== STM32F401 ADC DMA UART Firmware ===\r\n");
    USART2_SendString("System Clock: 84 MHz\r\n");
    USART2_SendString("TIM2: 100 Hz\r\n");
    USART2_SendString("ADC1: PA0, DMA2 Stream0, Circular Mode\r\n");
    USART2_SendString("UART: 115200 baud\r\n\r\n");

    /* Enable global interrupts */
    __asm volatile ("cpsie i");             /* PRIMASK = 0 (global interrupt enable) */

    /* Main loop - just wait for DMA interrupts */
    while (1) {
        /* Idle - all work is done in interrupt handler */
        __asm volatile ("wfi");             /* Wait for interrupt */
    }

    return 0;
}

/* ============================================================================
   SUPPORTING FUNCTIONS
   ============================================================================ */

/* Weak symbol for SystemInit - defined in system_stm32f4xx.c */
extern void SystemInit(void);

#ifdef SIMULATION
/* TIM2 update ISR for simulation: synthesize samples and emit every 16 */
void TIM2_IRQHandler(void)
{
    /* Clear update flag */
    TIM2->SR &= ~1U;

    /* Simple sawtooth across 12-bit range */
    uint16_t v = (uint16_t)((sim_index * 257U) & 0x0FFFU);
    adc_buf[sim_index % 16] = v;
    sim_index++;

    if ((sim_index % 16) == 0) {
        GPIOC->ODR ^= (1 << 13);
        char buffer[256];
        int len = 0;
        len += snprintf(buffer + len, sizeof(buffer) - len,
                        "Sample #%lu: ", sample_count);
        for (int i = 0; i < 16; i++) {
            len += snprintf(buffer + len, sizeof(buffer) - len,
                            "%04X ", adc_buf[i]);
        }
        len += snprintf(buffer + len, sizeof(buffer) - len, "\r\n");
        USART2_SendString(buffer);
        sample_count++;
    }
}
#endif
