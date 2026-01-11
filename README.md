# STM32F401CC/F411CE Bare-Metal ADC→DMA→UART Firmware

Complete bare-metal CMSIS register-level firmware for Blackpill board (STM32F401CC or STM32F411CE) using ARM GCC toolchain.

## Features

- **System Clock**: 84 MHz (HSE 8MHz + PLL configuration)
- **TIM2 Timer**: 100 Hz update rate with TRGO output
- **ADC1 Sampling**: External trigger from TIM2 TRGO, samples PA0 (ADC1_IN0)
- **DMA2 Stream0**: Circular mode, transfers 16 ADC samples to RAM buffer
- **UART Interface**: USART2 at 115200 baud, sends formatted ADC data via PA2
- **Debug Output**: LED toggle on PC13 (Blackpill onboard LED) on each DMA complete
- **No HAL**: Pure CMSIS register-level code, no STM32 HAL library

## Project Structure

```
stm32_adc_dma_uart/
├── src/
│   ├── startup_stm32f401_gcc.s    # ARM Cortex-M4 startup code
│   ├── system_stm32f4xx.c         # System clock initialization (PLL configuration)
│   └── main.c                     # Main firmware (all peripherals configured here)
├── inc/                           # Include directory (currently empty)
├── stm32f401_flash.ld             # Linker script (256KB Flash, 64KB RAM)
├── Makefile                       # Build system
└── README.md                      # This file
```

## Hardware Requirements

- **Board**: Blackpill (STM32F401CC or STM32F411CE)
- **Debug Probe**: ST-Link v2 (USB)
- **Oscilloscope** (optional): To verify 100 Hz signal on debug LED

## Pin Configuration

| Pin   | Signal       | Function           | Alternate Function |
|-------|--------------|--------------------|--------------------|
| PA0   | ADC1_IN0     | Analog Input       | Sampled by ADC1    |
| PA2   | USART2_TX    | UART Transmit      | AF7                |
| PA3   | USART2_RX    | UART Receive       | AF7 (not used)     |
| PC13  | LED          | Debug Output       | GPIO Output        |

## System Architecture

```

                   System Clock: 84 MHz                 
             (HSE 8 MHz + PLL: M=4, N=336, P=4)              

                            ▼
    ┌───────────────────────┼───────────────────────┐

APB1: 42 MHz          APB2: 84 MHz          AHB1: 84 MHz
├─ TIM2 (84 MHz)       ├─ ADC1               └─ DMA2
│  └─ 100 Hz TRGO      │  └─ PA0 (ADC_IN0)     └─ Stream0 (ADC DMA)
│                      │  └─ External Trigger
└─ USART2                  (TIM2_TRGO)
   └─ 115200 baud
      └─ PA2 (TX)
```

### Data Flow

1. **TIM2 (100 Hz)** generates TRGO output every 10 ms
2. **ADC1** receives external trigger from TRGO (rising edge)
3. **ADC1** starts single conversion on PA0, stores result in ADC1->DR
4. **DMA2 Stream0** (circular mode) transfers ADC result to RAM buffer
5. After **16 samples collected**, **DMA complete interrupt** fires
6. ISR formats and sends data via **USART2** (blocking/polling TX)
7. **PC13 LED** toggles on each DMA complete
8. **DMA restarts** for next batch of 16 samples


Output files in `build/`:
- `firmware.elf` - Executable (ELF format)
- `firmware.hex` - Intel HEX format
- `firmware.bin` - Raw binary (for flashing)
- `firmware.map` - Symbol map


Example output:
```
=== Firmware Size ===
   text    data     bss     dec     hex filename
  12345     128     512   12985    32a9 build/firmware.elf
```


## Wiring for UART Testing

Connect Blackpill to USB-UART adapter:

| Blackpill | UART Adapter |
|-----------|--------------|
| PA2       | TX (or RX)   |
| PA3       | RX (or TX)   |
| GND       | GND          |
| 3.3V      | 3.3V         |



## Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| System Clock | 84 MHz | PLL from 8 MHz HSE |
| TIM2 Frequency | 100 Hz | 10 ms period |
| ADC Sample Rate | 100 Hz | 100 samples/second |
| DMA Transfer Size | 16 samples | 160 ms total collection |
| UART Baud | 115200 | 115.2 kbit/s |
| Flash Used | ~12 KB | Typical, varies with optimization |
| RAM Used | ~256 B static | + 32 B (16×uint16_t buffer) |


## References

- [STM32F401CC Datasheet](https://www.st.com/resource/en/datasheet/stm32f401cc.pdf)
- [STM32F401 Reference Manual](https://www.st.com/resource/en/reference_manual/stm32f401xbc_dm00096844.pdf)
- [ARM Cortex-M4 Technical Reference Manual](https://developer.arm.com/documentation/ddi0439/d/)
- [CMSIS Documentation](https://arm-software.github.io/CMSIS_5/)

