#!/usr/bin/env python3
"""
STM32 Firmware UART Simulator using Unicorn Engine
Emulates the STM32F401 on bare Cortex-M4 and prints UART2 output
"""

try:
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_ARM
    from unicorn.arm_const import *
except ImportError:
    print("ERROR: unicorn not installed. Install with: pip install unicorn capstone")
    exit(1)

import struct
import sys
import time

class STM32F401Simulator:
    def __init__(self, elf_path):
        """Initialize the Cortex-M4 emulator with STM32F401 memory map"""
        self.elf_path = elf_path
        self.mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
        
        # STM32F401 memory map
        self.FLASH_BASE = 0x08000000
        self.FLASH_SIZE = 256 * 1024  # 256 KB
        self.RAM_BASE = 0x20000000
        self.RAM_SIZE = 64 * 1024  # 64 KB
        self.PERIPH_BASE = 0x40000000
        
        # Allocate and map memory
        self.mu.mem_map(self.FLASH_BASE, self.FLASH_SIZE)
        self.mu.mem_map(self.RAM_BASE, self.RAM_SIZE)
        self.mu.mem_map(self.PERIPH_BASE, 0x20000)  # Peripheral space
        
        # UART output buffer
        self.uart_output = ""
        self.sample_count = 0
        
    def load_elf(self):
        """Load ELF binary into flash memory"""
        try:
            import elftools.elf.elffile as elffile
        except ImportError:
            print("ERROR: pyelftools not installed. Install with: pip install pyelftools")
            exit(1)
            
        with open(self.elf_path, 'rb') as f:
            elf = elffile.ELFFile(f)
            for segment in elf.iter_segments():
                if segment['p_type'] == 'PT_LOAD':
                    addr = segment['p_paddr']
                    data = segment.data()
                    print(f"Loading {len(data)} bytes to 0x{addr:08X}")
                    self.mu.mem_write(addr, data)
    
    def hook_uart_write(self, address, size, value, user_data):
        """Hook USART2 DR register writes for output"""
        if address == 0x40004404:  # USART2->DR
            char = value & 0xFF
            self.uart_output += chr(char)
            sys.stdout.write(chr(char))
            sys.stdout.flush()
    
    def emulate(self, max_steps=100000):
        """Run emulation for a limited number of steps"""
        # Set stack pointer to top of RAM
        sp = self.RAM_BASE + self.RAM_SIZE - 4
        self.mu.reg_write(UC_ARM_REG_SP, sp)
        
        # Set PC to reset vector
        reset_addr = struct.unpack('<I', self.mu.mem_read(self.FLASH_BASE + 4, 4))[0]
        print(f"Reset vector points to: 0x{reset_addr:08X}")
        
        try:
            self.mu.emu_start(reset_addr | 1, 0, max_steps)  # Thumb mode bit set
        except Exception as e:
            print(f"Emulation stopped: {e}")
    
    def run(self):
        """Main entry point"""
        print(f"Loading ELF: {self.elf_path}")
        self.load_elf()
        print("Starting emulation...")
        self.emulate()
        print("\n=== UART Output ===")
        print(self.uart_output)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 sim_uart.py <elf_file>")
        sys.exit(1)
    
    sim = STM32F401Simulator(sys.argv[1])
    sim.run()
