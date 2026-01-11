# STM32F401 Project — Simple Explanation for Beginners

## What Does This Project Do? (Simple Version)

Imagine you have a **sensor** that measures something (like temperature or light). This project:

1. **Reads the sensor** 100 times per second (100 measurements every second)
2. **Saves the readings** into memory (RAM)
3. **Sends the readings** to your computer via USB/serial cable
4. **Blinks an LED** every time a batch of readings is sent

That's it! A simple **read → store → send → blink LED** cycle.

---

## What Hardware Do You Need?

### The Microcontroller (MCU)
Think of this as the "brain" that does all the work:
- **Name:** STM32F401 (a small computer chip)
- **Speed:** 84 MHz (it does 84 million things per second)
- **Memory:** 256 KB storage (like a hard drive) + 64 KB RAM (like your computer's RAM)
- **Where to buy:** "Blackpill" board on Amazon (cheap, ~$5)

### Connections You Need
```
Sensor (PA0)        → reads analog values (0 to 3.3V)
USB/Serial (PA2)    → sends data to computer at 115,200 baud
                       (like internet speed for serial: 115,200 bits/sec)
LED (PC13)          → blinks to show activity
```

### The Debugger (To Upload Code)
- **Name:** ST-Link V2 debugger
- **Cost:** ~$10
- **Job:** Uploads your compiled code to the chip's memory
- **Connection:** USB cable to your computer

---

## How Does It Work? (Step by Step)

### What Happens When You Power On

**Step 1: The Chip Wakes Up**
- The microcontroller powers on
- Special startup code runs (in assembly language)
- This code:
  - Sets up the memory (RAM)
  - Copies important data from storage to RAM
  - Calls the "setup" functions

**Step 2: The Setup Functions Run**
```
Initialize Clock:
  - Turn on the crystal (8 MHz)
  - Speed it up through a PLL (Phase-Locked Loop)
    → Like a gear system that multiplies speed
  - Result: 84 MHz (10x faster)

Initialize GPIO (General Purpose Input/Output):
  - Tell the chip which pins are inputs/outputs
  - PA0 = analog input (for sensor)
  - PA2 = serial output (for USB)
  - PC13 = digital output (for LED)

Initialize Timer (TIM2):
  - Set a clock that ticks 100 times per second
  - This clock will tell the ADC "now is the time to measure!"

Initialize ADC (Analog-to-Digital Converter):
  - Set it to listen to the timer
  - When timer says "now!", ADC reads PA0 and converts it to a number
  - Stores the number in a special register (data register)

Initialize DMA (Direct Memory Access):
  - This is a "helper" that automatically copies data
  - It watches the ADC data register
  - Every time ADC gets a new number, DMA copies it to RAM
  - No CPU involvement needed!

Initialize Serial (UART):
  - Set the speed to 115,200 baud
  - Get it ready to send messages to the computer
```

### What Happens Every 10 Milliseconds

```
1. Timer counts down to 0
   ↓
2. Timer sends a signal "TRIGGER!"
   ↓
3. ADC gets the signal and reads PA0
   - Converts the voltage to a number (0-4095, since it's 12-bit)
   ↓
4. ADC puts the number in its data register (a storage box in hardware)
   ↓
5. DMA sees the new number and copies it to RAM
   - No CPU needed!
   - DMA does this automatically
   ↓
6. Timer sends signal again (10 ms later)
   → Repeat steps 1-5
```

### What Happens After 16 Measurements (160 Milliseconds)

DMA finishes copying 16 numbers and says "I'm done!" → **Interrupt fires!**

An interrupt is like someone tapping you on the shoulder saying "Hey, I need your attention NOW!"

When the interrupt fires, the CPU stops what it's doing and runs special code:

```c
// This code runs when DMA finishes
void interrupt_handler() {
    // 1. Blink the LED
    toggle_led(PC13);
    
    // 2. Make a nice message with all 16 numbers
    // Format: "Sample #0: 0A12 0B34 0C56 ... (16 hex numbers)"
    create_message(adc_buffer);
    
    // 3. Send the message through USB serial
    // This is SLOW compared to the other stuff
    // Takes about 7 milliseconds
    send_via_serial(message);
    
    // 4. Tell DMA to do it again
    // Reset the counter to 16
    restart_dma();
}
```

Then the CPU goes back to sleep until the next interrupt.

---

## The Key Insight: Why DMA?

### The Dumb Way (Without DMA):
```
Timer fires → CPU wakes up → CPU reads ADC → CPU writes to RAM → repeat
Problem: CPU is busy ALL THE TIME (100 times per second!)
```

### The Smart Way (With DMA):
```
Timer fires → ADC reads → DMA copies → CPU sleeps
CPU only wakes up every 160ms (16x less work!)
```

**DMA = "Direct Memory Access"** = automatic data copying without CPU involvement

---

## Memory Layout (Where Things Live)

Think of the chip's memory like a filing cabinet with labeled drawers:

```
FLASH (Storage - like a hard drive)
┌─────────────────┐
│  Vector Table   │  Address 0x08000000
│  (interrupts)   │
├─────────────────┤
│  Program Code   │  "Sample #%lu: " message
│  (your C code)  │  Loop that sends data
│  Constants      │
├─────────────────┤
│  Init Data      │  Values for variables (copied to RAM at startup)
└─────────────────┘  Total: 30 KB (out of 256 KB available)

RAM (Working memory - like your computer's RAM)
┌─────────────────┐
│  Variables      │  sample_count, uart_buffer, etc.
│  ADC Buffer     │  adc_buf[16] - holds 16 measurements
│  UART Buffer    │  space for message
├─────────────────┤
│  Stack          │  temporary storage for function calls
│  (grows down)   │
└─────────────────┘  Total: 2 KB (out of 64 KB available)
```

---

## What Is a "Clock"? (Beginner Explanation)

A clock in electronics is not about time - it's about **speed**.

**Analogy:**
- Computer runs at **84 MHz** = "84 million ticks per second"
- Each tick allows the CPU to do one simple operation
- More ticks = more operations = faster computer

**In this project:**
- Main clock: 84 MHz (fast, for CPU)
- Timer clock (TIM2): Also uses 84 MHz but we divide it down to 100 Hz
  - 84 MHz ÷ 840,000 = 100 Hz (100 times per second)

---

## The Communication Speeds

### Serial/UART Communication
**115,200 baud** means:
- 115,200 bits per second
- Each character = 10 bits (1 start + 8 data + 1 stop)
- So: 115,200 ÷ 10 = 11,520 characters per second
- Time for one character: ~87 microseconds

**Message size:** "Sample #0: 0A12 0B34 ... \r\n" = about 80 bytes
**Time to send:** 80 × 87 µs = ~7 milliseconds

**This is the SLOWEST part of the pipeline!** ADC and DMA are way faster.

---

## Two Build Modes Explained

### Mode 1: Hardware (Real ADC)
- Reads actual sensor on PA0
- Uses real ADC hardware
- Uses real DMA hardware
- **You need:** Physical sensor connected to PA0

### Mode 2: Simulation (Fake Data)
- Doesn't use ADC (might not be modeled in simulator)
- Doesn't use DMA (might not be modeled in simulator)
- Generates fake sensor readings in code
- Creates same UART output without real hardware
- **You need:** Just the microcontroller (no sensor)

**Why?** Testing without hardware. Let's you verify the code works before buying a sensor.

---

## The Code Files Explained

### `startup_stm32f401_gcc.s` (Assembly)
**What:** First code that runs when chip powers on

**Simple version:**
```
1. Tell the CPU where the stack is (top of RAM)
2. Copy important data from storage to RAM
3. Clear unused memory
4. Call the "setup" function (SystemInit)
5. Call the "main" function
6. Never return (infinite loop)
```

**Why assembly?** These are very low-level operations the CPU needs to do before C code can run.

### `system_stm32f4xx.c` (C)
**What:** Sets up the clock system

**Simple version:**
```c
void SystemInit() {
    // Turn on the crystal (8 MHz)
    turn_on_crystal();
    
    // Use a PLL to multiply it to 84 MHz
    // (PLL = "Phase-Locked Loop" = fancy multiplier)
    configure_pll();
    
    // Tell the CPU to use the fast clock
    switch_to_fast_clock();
    
    // Turn on the peripherals we need
    enable_gpio();
    enable_timer();
    enable_adc();
    enable_dma();
    enable_serial();
}
```

### `main.c` (C)
**What:** Application code

**Simple version:**
```c
int main() {
    // Step 1: Configure all the hardware
    setup_pins();
    setup_timer();
    setup_adc();
    setup_dma();
    setup_serial();
    
    // Step 2: Tell the chip which interrupts to listen to
    enable_interrupts();
    
    // Step 3: Send startup message
    send_message("System ready!");
    
    // Step 4: Sleep forever
    // Interrupts will wake us up when needed
    while(1) {
        sleep();  // CPU does nothing, uses no power
    }
}
```

**Key interrupt handlers:**
```c
// When DMA finishes copying 16 samples:
void dma_interrupt() {
    blink_led();
    send_16_samples_via_serial();
}

// When timer fires (simulation mode):
void timer_interrupt() {
    generate_fake_sample();
    if (have_16_samples) {
        blink_led();
        send_samples_via_serial();
    }
}
```

### `stm32f401_flash.ld` (Linker Script)
**What:** Tells the compiler where things should go in memory

**Simple version:**
```ld
MEMORY {
    FLASH: 0x08000000, size 256KB  // Storage
    RAM:   0x20000000, size 64KB   // Working memory
}

SECTIONS {
    // Vector table must be first (CPU looks there first)
    .isr_vector: goes to FLASH start
    
    // Code and constants go to FLASH
    .text: goes to FLASH
    .rodata: goes to FLASH
    
    // Initialized variables start in FLASH (for copying to RAM)
    .data: goes to RAM, initialized from FLASH
    
    // Uninitialized variables go to RAM (will be zeroed)
    .bss: goes to RAM
}
```

---

## Building and Running

### Build Hardware Version
```bash
# Compile C code → object files
# Link object files → ELF file
# Convert ELF → HEX and BIN files
make
```

**Output files:**
- `firmware.elf` = Full binary with debug info (for debugging)
- `firmware.hex` = Intel HEX format (for some programmers)
- `firmware.bin` = Raw binary (for st-flash tool)

### Flash to Chip
```bash
# Copy the binary into the chip's storage
st-flash write firmware.bin 0x08000000
#           ↑                         ↑
#           file                      address (where to put it)
```

### Monitor Serial Output
```bash
# Open a terminal on the serial port
minicom -D /dev/ttyUSB0 -b 115200
#                                  ↑
#                           baud rate (speed)
```

### Run Simulation
```bash
# Python script that mimics the firmware
python3 demo_sim.py
```

---

## What Actually Gets Sent Over Serial

Every 160 milliseconds:
```
Sample #0: 0A12 0B34 0C56 0D78 0E9A 0FBC 10DE 11F0 1212 1334 1456 1578 169A 17BC 18DE 19F0
Sample #1: 1A12 1B34 1C56 1D78 1E9A 1FBC 20DE 21F0 2212 2334 2456 2578 269A 27BC 28DE 29F0
Sample #2: 2A12 2B34 2C56 2D78 2E9A 2FBC 30DE 31F0 3212 3334 3456 3578 369A 37BC 38DE 39F0
...
```

**Meaning:**
- `Sample #0` = This is the first batch
- `0A12` = First sensor reading (in hexadecimal, 0 to 4095 range)
- `0B34` = Second reading
- ... and so on for 16 readings
- Each reading is 4 hex digits = 12-bit number

**In decimal (0-4095):**
- `0x0A12` = 2578
- `0x0B34` = 2868
- `0x0C56` = 3158
- etc.

---

## Common Beginner Questions

### Q: What's a "register"?
**A:** A tiny storage box inside the CPU/peripheral that holds a value (like an integer variable).
- Each register is usually 32 bits
- You read/write them to control hardware
- Example: `ADC1->DR` = "ADC1's data register"

### Q: What's "volatile"?
**A:** Tells the compiler "this variable might change unexpectedly (hardware changed it), don't optimize it."

### Q: What's a "bit field"?
**A:** A way to control specific bits in a register.
- `|= (1 << 5)` = Set bit 5 to 1
- `&= ~(1 << 5)` = Set bit 5 to 0
- Bits are numbered 0-31 in a 32-bit register

### Q: Why do we sleep in the main loop?
**A:** Saves power! If CPU is doing nothing, why waste energy? Let it sleep until something important happens (interrupt).

### Q: What happens if DMA overflows?
**A:** It won't - DMA is configured in circular mode, which automatically wraps around after 16 transfers.

### Q: How fast is the ADC?
**A:** Conversion takes ~1.86 microseconds. We wait 10 milliseconds between measurements (plenty of time).

### Q: Can I sample faster than 100 Hz?
**A:** Yes! Change `PSC` and `ARR` in `TIM2_Init()`. But UART will be the bottleneck - only ~11,500 chars/sec.

---

## Summary: What Makes This Project Good?

✅ **Efficient:** Uses DMA so CPU can sleep (saves power)  
✅ **Fast:** Reads every 10ms reliably (100 Hz)  
✅ **Simple:** No complex libraries, just direct register access  
✅ **Portable:** Works on STM32F401/F411 (multiple chips)  
✅ **Complete:** Includes startup, clock setup, all peripherals  
✅ **Tested:** Both hardware and simulation modes  

---

## Next Steps for Learning

1. **Build and run it** (hardware or simulation)
2. **Read the UART output** to understand the data flow
3. **Modify the sensor speed** (change timer values)
4. **Add another peripheral** (maybe an LCD display?)
5. **Read the datasheets** to understand the chip better

---

**You now understand a complete embedded systems project!** 🎉

The key takeaway: **Use hardware (DMA, interrupts) to do the work, let the CPU sleep.**
