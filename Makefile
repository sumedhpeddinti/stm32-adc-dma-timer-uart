##############################################################################
# Makefile for STM32F401CC/F411CE Bare-Metal Firmware
# ARM GCC Toolchain
##############################################################################

# Target configuration
TARGET = firmware
MCU = STM32F401xC
CORTEX = cortex-m4

# Toolchain
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size
AR = arm-none-eabi-ar

# Source directories
SRC_DIR = src
INC_DIR = inc
BUILD_DIR ?= build

# Source files
SOURCES = \
	$(SRC_DIR)/startup_stm32f401_gcc.s \
	$(SRC_DIR)/system_stm32f4xx.c \
	$(SRC_DIR)/main.c

# Linker script
LINKER_SCRIPT = stm32f401_flash.ld

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES:.c=.o)))
OBJECTS := $(OBJECTS:.s=.o)

# Output files
ELF_FILE = $(BUILD_DIR)/$(TARGET).elf
HEX_FILE = $(BUILD_DIR)/$(TARGET).hex
BIN_FILE = $(BUILD_DIR)/$(TARGET).bin
MAP_FILE = $(BUILD_DIR)/$(TARGET).map

# Compiler flags
CFLAGS = -mcpu=$(CORTEX)
CFLAGS += -mthumb
CFLAGS += -mfpu=fpv4-sp-d16
CFLAGS += -mfloat-abi=softfp
CFLAGS += -D$(MCU)
CFLAGS += -DUSE_HAL_DRIVER
CFLAGS += -Wall
CFLAGS += -Wextra
CFLAGS += -pedantic
CFLAGS += -O2
CFLAGS += -g3
CFLAGS += -std=c99
CFLAGS += -I$(INC_DIR)
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections

# Simulation build toggle (make SIM=1)
ifeq ($(SIM),1)
CFLAGS += -DSIMULATION
endif

# Assembler flags
ASFLAGS = $(CFLAGS)
ASFLAGS += -x assembler-with-cpp

# Linker flags
LDFLAGS = -mcpu=$(CORTEX)
LDFLAGS += -mthumb
LDFLAGS += -mfpu=fpv4-sp-d16
LDFLAGS += -mfloat-abi=softfp
LDFLAGS += -T$(LINKER_SCRIPT)
LDFLAGS += -Wl,-Map=$(MAP_FILE),--cref
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -lm
LDFLAGS += -lc
LDFLAGS += -lnosys

# Default target
.PHONY: all
all: $(HEX_FILE) $(BIN_FILE) size

# ELF file
$(ELF_FILE): $(OBJECTS) $(LINKER_SCRIPT)
	@echo "[LINK] $@"
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

# HEX file
$(HEX_FILE): $(ELF_FILE)
	@echo "[HEX] $@"
	$(OBJCOPY) -O ihex $< $@

# BIN file
$(BIN_FILE): $(ELF_FILE)
	@echo "[BIN] $@"
	$(OBJCOPY) -O binary $< $@

# C source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(BUILD_DIR)
	@echo "[CC] $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Assembly source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.s
	@mkdir -p $(BUILD_DIR)
	@echo "[AS] $<"
	$(CC) $(ASFLAGS) -c $< -o $@

# Print size
.PHONY: size
size: $(ELF_FILE)
	@echo ""
	@echo "=== Firmware Size ==="
	@$(SIZE) -B $(ELF_FILE)
	@echo ""

# Display disassembly
.PHONY: disasm
disasm: $(ELF_FILE)
	@echo "[DISASM] Generating disassembly..."
	$(OBJDUMP) -d $(ELF_FILE) > $(BUILD_DIR)/disassembly.txt
	@echo "Disassembly saved to: $(BUILD_DIR)/disassembly.txt"

# Clean build artifacts
.PHONY: clean
clean:
	@echo "[CLEAN] Removing build artifacts..."
	rm -rf $(BUILD_DIR)/*
	@echo "Clean complete."

# Flash firmware to device (requires st-flash tool)
.PHONY: flash
flash: $(BIN_FILE)
	@echo "[FLASH] Programming device..."
	st-flash write $(BIN_FILE) 0x08000000
	@echo "Programming complete."

# Flash firmware using STM32CubeProgrammer (if available)
.PHONY: flash_cube
flash_cube: $(BIN_FILE)
	@echo "[FLASH] Programming device using STM32CubeProgrammer..."
	STM32_Programmer_CLI -c port=SWD -d $(BIN_FILE) 0x08000000 -v
	@echo "Programming complete."

# Display help
.PHONY: help
help:
	@echo "STM32F401 Firmware Build System"
	@echo ""
	@echo "Available targets:"
	@echo "  all        - Build firmware (default)"
	@echo "  clean      - Remove build artifacts"
	@echo "  size       - Show firmware size"
	@echo "  disasm     - Generate disassembly listing"
	@echo "  flash      - Flash to device (st-flash)"
	@echo "  flash_cube - Flash to device (STM32CubeProgrammer)"
	@echo "  help       - Show this help message"
	@echo ""
	@echo "Examples:"
	@echo "  make           # Build firmware"
	@echo "  make clean     # Clean build directory"
	@echo "  make flash     # Build and flash (requires st-flash)"
	@echo ""
