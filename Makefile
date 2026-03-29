# BL4818-Servo Firmware Makefile
# Toolchain: SDCC targeting Nuvoton MS51FB9AE (MCS-51 / 8051)

# ── Toolchain ────────────────────────────────────────────────────────────────
CC       = sdcc
AS       = sdas8051
OBJCOPY  = sdobjcopy
PACKIHX  = packihx
SIZE     = size

# ── Target MCU ───────────────────────────────────────────────────────────────
MCU      = mcs51
XRAM_SIZE = 1024
XRAM_LOC  = 0x0000
# Set APROM_SIZE/LDROM_SIZE together when reserving LDROM.
FLASH_TOTAL_SIZE ?= 16384
LDROM_SIZE ?= 0
APROM_SIZE ?= 16384
CODE_SIZE = $(APROM_SIZE)
IRAM_SIZE = 256
STACK_SIZE = 64

# ── Directories ──────────────────────────────────────────────────────────────
SRCDIR   = src
INCDIR   = include
BUILDDIR = build

# ── Sources ──────────────────────────────────────────────────────────────────
EXCLUDE = $(SRCDIR)/pid.c $(SRCDIR)/encoder.c $(SRCDIR)/flash.c
SRCS = $(filter-out $(EXCLUDE),$(wildcard $(SRCDIR)/*.c))
ASM_SRCS = $(SRCDIR)/vectors.asm
C_RELS = $(patsubst $(SRCDIR)/%.c,$(BUILDDIR)/%.rel,$(SRCS))
ASM_RELS = $(patsubst $(SRCDIR)/%.asm,$(BUILDDIR)/%.rel,$(ASM_SRCS))
RELS = $(C_RELS) $(ASM_RELS)

# main.c must be linked first for correct startup
MAIN_REL = $(BUILDDIR)/main.rel
OTHER_C_RELS = $(filter-out $(MAIN_REL),$(C_RELS))
ORDERED_RELS = $(MAIN_REL) $(OTHER_C_RELS) $(ASM_RELS)

# ── Output ───────────────────────────────────────────────────────────────────
TARGET   = bl4818-servo
IHX      = $(BUILDDIR)/$(TARGET).ihx
HEX      = $(BUILDDIR)/$(TARGET).hex
BIN      = $(BUILDDIR)/$(TARGET).bin

ifeq ($(OS),Windows_NT)
MKDIR_BUILD = powershell -NoProfile -Command "if (-not (Test-Path '$(BUILDDIR)')) { New-Item -ItemType Directory '$(BUILDDIR)' | Out-Null }"
RMDIR_BUILD = powershell -NoProfile -Command "if (Test-Path '$(BUILDDIR)') { Remove-Item -Recurse -Force '$(BUILDDIR)' }"
else
MKDIR_BUILD = mkdir -p $(BUILDDIR)
RMDIR_BUILD = rm -rf $(BUILDDIR)
endif

# ── Compiler Flags ───────────────────────────────────────────────────────────
CFLAGS  = -m$(MCU)
CFLAGS += -I$(INCDIR)
CFLAGS += --std-c11
CFLAGS += --opt-code-size
CFLAGS += --stack-auto
CFLAGS += --model-small
CFLAGS += --no-xinit-opt
CFLAGS += -DFLASH_TOTAL_SIZE=$(FLASH_TOTAL_SIZE)
CFLAGS += -DLDROM_SIZE=$(LDROM_SIZE)
CFLAGS += -DAPROM_SIZE=$(APROM_SIZE)

ASFLAGS  = -plosgff

# ── Linker Flags ─────────────────────────────────────────────────────────────
LDFLAGS  = -m$(MCU)
LDFLAGS += --model-small
LDFLAGS += --stack-auto
LDFLAGS += --no-xinit-opt
LDFLAGS += --xram-size $(XRAM_SIZE)
LDFLAGS += --xram-loc $(XRAM_LOC)
LDFLAGS += --code-size $(CODE_SIZE)
LDFLAGS += --code-loc 0x0093
LDFLAGS += --iram-size $(IRAM_SIZE)
LDFLAGS += --stack-size $(STACK_SIZE)
LDFLAGS += -o $(IHX)

# ── Rules ────────────────────────────────────────────────────────────────────
.PHONY: all clean flash size

all: $(IHX) $(BIN)

$(BUILDDIR):
	$(RMDIR_BUILD)
	$(MKDIR_BUILD)

$(BUILDDIR)/%.rel: $(SRCDIR)/%.c $(wildcard $(INCDIR)/*.h) | $(BUILDDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILDDIR)/%.rel: $(SRCDIR)/%.asm | $(BUILDDIR)
	$(AS) $(ASFLAGS) -o $@ $<

$(IHX): $(RELS) | $(BUILDDIR)
	$(CC) $(LDFLAGS) $(ORDERED_RELS)

$(HEX): $(IHX)
	$(PACKIHX) $< > $@

$(BIN): $(IHX)
	$(OBJCOPY) -I ihex -O binary $< $@

clean:
	$(RMDIR_BUILD)

flash: $(IHX)
	@echo "Flashing $(IHX) via Nu-Link..."
	@echo "Ensure Nu-Link programmer is connected."
	@command -v nulink_8051_fwupdate >/dev/null 2>&1 && \
		nulink_8051_fwupdate -a $(IHX) || \
		echo "nulink_8051_fwupdate not found. Install Nuvoton tools or use OpenOCD."

size: $(IHX)
	@echo "=== Memory Usage ==="
	@if [ -f $(BUILDDIR)/main.mem ]; then \
		cat $(BUILDDIR)/main.mem; \
	fi
	@echo "Code size limit: $(CODE_SIZE) bytes"
	@echo "XRAM size limit: $(XRAM_SIZE) bytes"
	@wc -c < $(BIN) 2>/dev/null && echo "bytes (binary)" || \
		echo "(build binary first with 'make $(BIN)')"
