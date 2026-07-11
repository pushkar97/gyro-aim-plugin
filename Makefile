# Standalone Makefile for the gyro-aim GoldHEN plugin.
#
# Prerequisites (set these environment variables before running `make`):
#   OO_PS4_TOOLCHAIN  - path to a clone of OpenOrbis/OpenOrbis-PS4-Toolchain
#   GOLDHEN_SDK       - path to a clone of GoldHEN/GoldHEN_Plugins_SDK
#
# Build: make
# Debug build (verbose klog): make DEBUG=1
# Output: build/prx/gyro_aim.prx

DEBUG_FLAGS := -D__FINAL__=1
BUILD_TYPE  := _final

ifeq ($(DEBUG),1)
    DEBUG_FLAGS := -D__FINAL__=0
    BUILD_TYPE  := _debug
endif

TOOLCHAIN  := $(OO_PS4_TOOLCHAIN)
GH_SDK     := $(GOLDHEN_SDK)

ifeq ($(TOOLCHAIN),)
$(error OO_PS4_TOOLCHAIN is not set. Clone OpenOrbis/OpenOrbis-PS4-Toolchain and export OO_PS4_TOOLCHAIN=/path/to/it)
endif
ifeq ($(GH_SDK),)
$(error GOLDHEN_SDK is not set. Clone GoldHEN/GoldHEN_Plugins_SDK and export GOLDHEN_SDK=/path/to/it)
endif

OUTPUT_PRX  := gyro_aim
BUILD_DIR   := build
PRX_DIR     := $(BUILD_DIR)/prx$(BUILD_TYPE)
ELF_DIR     := $(BUILD_DIR)/elf$(BUILD_TYPE)
OBJ_DIR     := $(BUILD_DIR)/obj$(BUILD_TYPE)

TARGET_PRX  := $(PRX_DIR)/$(OUTPUT_PRX).prx
TARGET_ELF  := $(ELF_DIR)/$(OUTPUT_PRX).elf

SRC_DIR     := source
INC_DIR     := include

LIBS := -lSceLibcInternal -lGoldHEN_Hook -lkernel -lSceSysmodule -lScePad

EXTRAFLAGS := $(DEBUG_FLAGS) -D__USE_PRINTF__ -Wall -fcolor-diagnostics

CFILES := $(wildcard $(SRC_DIR)/*.c)
OBJS   := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(CFILES))

CFLAGS  := $(EXTRAFLAGS) --target=x86_64-pc-freebsd12-elf -fPIC -funwind-tables -c \
           -isysroot $(TOOLCHAIN) -isystem $(TOOLCHAIN)/include \
           -I$(GH_SDK)/include -I$(INC_DIR)
LDFLAGS := -m elf_x86_64 -pie --script $(TOOLCHAIN)/link.x -e _init --eh-frame-hdr \
           -L$(TOOLCHAIN)/lib -L$(GH_SDK) $(LIBS)

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
    CC   := clang
    LD   := ld.lld
    CDIR := linux
    CREATE_FSELF_BIN := create-fself
endif
ifeq ($(UNAME_S),Darwin)
    # Homebrew's llvm is keg-only and lives under a different prefix on
    # Apple Silicon (/opt/homebrew) vs Intel (/usr/local). Prefer `brew
    # --prefix llvm` when available, falling back to the two conventional
    # locations. lld is a *separate* Homebrew formula from llvm (it doesn't
    # bundle ld.lld), so its prefix is resolved independently.
    LLVM_PREFIX := $(shell brew --prefix llvm 2>/dev/null)
    ifeq ($(LLVM_PREFIX),)
        ifneq ($(wildcard /opt/homebrew/opt/llvm/bin/clang),)
            LLVM_PREFIX := /opt/homebrew/opt/llvm
        else
            LLVM_PREFIX := /usr/local/opt/llvm
        endif
    endif
    LLD_PREFIX := $(shell brew --prefix lld 2>/dev/null)
    ifeq ($(LLD_PREFIX),)
        ifneq ($(wildcard /opt/homebrew/opt/lld/bin/ld.lld),)
            LLD_PREFIX := /opt/homebrew/opt/lld
        else
            LLD_PREFIX := /usr/local/opt/lld
        endif
    endif
    CC   := $(LLVM_PREFIX)/bin/clang
    LD   := $(LLD_PREFIX)/bin/ld.lld
    CDIR := macos
    # The macOS release binary is (inconsistently vs. Linux) named
    # create-fself-macos rather than create-fself.
    CREATE_FSELF_BIN := create-fself-macos
endif

.PHONY: all clean dirs

all: dirs $(TARGET_PRX)

dirs:
	@mkdir -p $(OBJ_DIR) $(ELF_DIR) $(PRX_DIR)

$(TARGET_PRX): $(OBJS)
	$(LD) $(GH_SDK)/build/crtprx.o $(OBJS) -o $(TARGET_ELF) $(LDFLAGS)
	$(TOOLCHAIN)/bin/$(CDIR)/$(CREATE_FSELF_BIN) -in=$(TARGET_ELF) -out=$(TARGET_ELF).oelf --lib=$(TARGET_PRX) --paid 0x3800000000000011

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -rf $(BUILD_DIR)
