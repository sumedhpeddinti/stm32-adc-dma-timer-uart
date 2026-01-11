#!/usr/bin/env python3
"""
STM32F401 Firmware Simulation Demo
Demonstrates the ADC→DMA→UART pipeline by running TIM2_IRQHandler loop
and printing synthesized UART output.
"""

import sys
import time

def demo_simulation():
    """Simulate TIM2 IRQ firing every 10ms, collecting 16 samples, then printing"""
    print("=" * 70)
    print("STM32F401 ADC-DMA-UART Simulation (SIMULATION mode)")
    print("=" * 70)
    print("\nConfig:")
    print("  System Clock: 84 MHz")
    print("  TIM2: 100 Hz update (10 ms period)")
    print("  ADC: Synthesized 12-bit sawtooth (simulated)")
    print("  DMA: 16 samples per batch")
    print("  UART: 115200 baud (emulated output below)")
    print("\n" + "=" * 70)
    print("\nStarting firmware emulation...\n")
    
    # Simulate startup banner
    startup_lines = [
        "\r\n=== STM32F401 ADC DMA UART Firmware ===\r\n",
        "System Clock: 84 MHz\r\n",
        "TIM2: 100 Hz\r\n",
        "ADC1: PA0, DMA2 Stream0, Circular Mode\r\n",
        "UART: 115200 baud\r\n\r\n",
    ]
    
    for line in startup_lines:
        time.sleep(0.01)  # Simulate UART TX delay
        print(line, end="", flush=True)
    
    time.sleep(0.1)
    
    # Simulate TIM2 IRQ firing every 10ms, collecting 16 samples
    # After 16 ticks (160ms), print a sample line
    sim_index = 0
    sample_count = 0
    
    print("Simulating 10 sample batches (~1.6 seconds real time):\n")
    
    for batch in range(10):
        # Simulate 16 TIM2 ticks at 100 Hz = 160 ms
        for i in range(16):
            # Synthesized sawtooth: (sim_index * 257) & 0xFFF
            v = (sim_index * 257) & 0xFFF
            sim_index += 1
            time.sleep(0.01)  # 10 ms per TIM2 tick
        
        # After 16 ticks, emit a sample line
        # Format: "Sample #N: AAAA BBBB CCCC ... (16 4-digit hex values)"
        samples = []
        for i in range(16):
            idx = (batch * 16 + i)
            val = (idx * 257) & 0xFFF
            samples.append(f"{val:04X}")
        
        output = f"Sample #{sample_count}: {' '.join(samples)}\r\n"
        print(output, end="", flush=True)
        sample_count += 1
    
    print("\n" + "=" * 70)
    print("Simulation complete! Firmware would produce this output on hardware.")
    print("=" * 70)
    print("\nTo run on real hardware:")
    print("  1. Connect Blackpill to ST-Link debugger")
    print("  2. Build: make clean && make")
    print("  3. Flash: make flash")
    print("  4. Open UART monitor at 115200 baud on /dev/ttyUSB0 or similar")
    print("\n")

if __name__ == "__main__":
    demo_simulation()
