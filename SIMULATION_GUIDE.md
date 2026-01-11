# Running the STM32F401 Firmware in Simulation (No Hardware Required)

## Quick Start: Demo Simulation

To see what the firmware outputs **without any hardware**, run:

```bash
cd /home/sum/Desktop/intern/stm32_adc_dma_uart
python3 renode/demo_sim.py
```

This simulates the TIM2→synthesized samples→UART pipeline and prints what you'd see on real hardware.

---

## Option A: Build for Simulation (Renode)

If you have Renode installed, use the simulation build:

```bash
make clean
make SIM=1
```

Then run in Renode (requires Renode to be installed):

```bash
renode renode/blackpill_sim.resc
```

**Note:** Renode has complex dependencies. The `demo_sim.py` above is simpler and requires no setup.

---

## Option B: Build for Real Hardware (No Renode Needed)

To build for actual STM32F401 hardware (Blackpill):

```bash
make clean
make
```

This produces:
- `build/firmware.elf` — Full binary with debug symbols
- `build/firmware.hex` — Intel HEX format
- `build/firmware.bin` — Raw binary

### Flash to Device

Connect your Blackpill with an ST-Link debugger, then:

```bash
make flash
```

Or manually:

```bash
st-flash write build/firmware.bin 0x08000000
```

### Monitor UART Output

Open a serial monitor at **115200 baud, 8N1**:

```bash
minicom -D /dev/ttyUSB0 -b 115200
# or
screen /dev/ttyUSB0 115200
```

You'll see:

```
=== STM32F401 ADC DMA UART Firmware ===
System Clock: 84 MHz
TIM2: 100 Hz
ADC1: PA0, DMA2 Stream0, Circular Mode
UART: 115200 baud

Sample #0: 0000 0101 0202 0303 0404 0505 0606 0707 0808 0909 0A0A 0B0B 0C0C 0D0D 0E0E 0F0F
Sample #1: 0010 0111 0212 0313 0414 0515 0616 0717 0818 0919 0A1A 0B1B 0C1C 0D1D 0E1E 0F1F
...
```

Every ~160 ms:
- **PC13 LED toggles** (if connected)
- **A line prints** with 16 hex ADC samples
- **Counter increments**

---

## Build Modes

| Mode | Command | Use Case |
|------|---------|----------|
| **Hardware** | `make` | Real STM32F401 on Blackpill with ADC→DMA→UART |
| **Simulation** | `make SIM=1` | Renode emulation (TIM2 synthesizes samples) |
| **Demo** | `python3 renode/demo_sim.py` | No setup—just show output |

---

## Files in `renode/`

- `blackpill_sim.resc` — Renode script (for Renode users)
- `demo_sim.py` — Python demo that simulates firmware output (no Renode needed)

---

## Troubleshooting

**Q: Exit code 127 when running Renode?**  
→ Renode is not installed. Use `demo_sim.py` instead or install Renode separately.

**Q: Build fails with "arm-none-eabi-gcc not found"?**  
→ Install toolchain: `sudo apt-get install gcc-arm-none-eabi`

**Q: No UART output on hardware?**  
→ Check that PA2 (TX) and PA3 (RX) are wired correctly.  
→ Verify baud rate is 115200.

---

Enjoy! 🚀
