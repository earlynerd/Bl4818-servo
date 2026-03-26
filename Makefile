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
CODE_SIZE = 16384
IRAM_SIZE = 256
STACK_SIZE = 64

# ── Directories ──────────────────────────────────────────────────────────────
SRCDIR   = src
INCDIR   = include
BUILDDIR = build

# ── Sources ──────────────────────────────────────────────────────────────────
SRCS = $(wildcard $(SRCDIR)/*.c)
RELS = $(patsubst $(SRCDIR)/%.c,$(BUILDDIR)/%.rel,$(SRCS))

# main.c must be linked first for correct startup
MAIN_REL = $(BUILDDIR)/main.rel
OTHER_RELS = $(filter-out $(MAIN_REL),$(RELS))
ORDERED_RELS = $(MAIN_REL) $(OTHER_RELS)

# ── Output ───────────────────────────────────────────────────────────────────
TARGET   = bl4818-servo
IHX      = $(BUILDDIR)/$(TARGET).ihx
HEX      = $(BUILDDIR)/$(TARGET).hex
BIN      = $(BUILDDIR)/$(TARGET).bin

# ── Compiler Flags ───────────────────────────────────────────────────────────
CFLAGS  = -m$(MCU)
CFLAGS += -I$(INCDIR)
CFLAGS += --std-c11
CFLAGS += --opt-code-size
CFLAGS += --stack-auto
CFLAGS += --model-small
CFLAGS += --no-xinit-opt

# ── Linker Flags ─────────────────────────────────────────────────────────────
LDFLAGS  = -m$(MCU)
LDFLAGS += --model-small
LDFLAGS += --stack-auto
LDFLAGS += --no-xinit-opt
LDFLAGS += --xram-size $(XRAM_SIZE)
LDFLAGS += --xram-loc $(XRAM_LOC)
LDFLAGS += --code-size $(CODE_SIZE)
LDFLAGS += --iram-size $(IRAM_SIZE)
LDFLAGS += --stack-size $(STACK_SIZE)
LDFLAGS += -o $(IHX)

# ── Rules ────────────────────────────────────────────────────────────────────
.PHONY: all clean flash size

all: $(IHX) $(BIN)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(BUILDDIR)/%.rel: $(SRCDIR)/%.c $(wildcard $(INCDIR)/*.h) | $(BUILDDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(IHX): $(RELS) | $(BUILDDIR)
	$(CC) $(LDFLAGS) $(ORDERED_RELS)

$(HEX): $(IHX)
	$(PACKIHX) $< > $@

$(BIN): $(IHX)
	$(OBJCOPY) -I ihex -O binary $< $@

clean:
	rm -rf $(BUILDDIR)

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
