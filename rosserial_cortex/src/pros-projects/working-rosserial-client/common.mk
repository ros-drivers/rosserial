# Universal C Makefile for MCU targets
# Top-level template file to configure build

MAKE_COMMAND=make

# Makefile for IFI VeX Cortex Microcontroller (STM32F103VD series)
DEVICE=VexCortex
# Libraries to include in the link (use -L and -l) e.g. -lm, -lmyLib
LIBRARIES=$(wildcard $(ROOT)/firmware/*.a) -lgcc -lm
# Prefix for ARM tools (must be on the path)
MCUPREFIX=arm-none-eabi-
# Flags for the assembler
MCUAFLAGS=-mthumb -mcpu=cortex-m3 -mlittle-endian
# Flags for the compiler
MCUCFLAGS=-mthumb -mcpu=cortex-m3 -mlittle-endian -mfloat-abi=soft
# Flags for the linker
MCULFLAGS=-nostartfiles -Wl,-static -Bfirmware -Wl,-u,VectorTable -Wl,-T -Xlinker firmware/cortex.ld
# Prepares the elf file by converting it to a binary that java can write
MCUPREPARE=$(OBJCOPY) $(OUT) -O binary $(BINDIR)/$(OUTBIN)
# Advanced sizing flags
SIZEFLAGS=
# Uploads program using java
UPLOAD=@java -jar firmware/uniflash.jar vex $(BINDIR)/$(OUTBIN)
# Flashes program using the PROS CLI flash command
FLASH=pros flash -f $(BINDIR)/$(OUTBIN)

# Advanced options
ASMEXT=s
CEXT=c
CPPEXT=cpp
HEXT=h
INCLUDE=-I$(ROOT)/include -I$(ROOT)/src -I$(ROOT)/include/ros_lib
OUTBIN=output.bin
OUTNAME=output.elf

# Flags for programs
AFLAGS:=$(MCUAFLAGS)
ARFLAGS:=$(MCUCFLAGS)
CCFLAGS:=-c -Wall $(MCUCFLAGS) -Os -ffunction-sections -fsigned-char -fomit-frame-pointer -fsingle-precision-constant
CFLAGS:=$(CCFLAGS) -std=gnu99 -Werror=implicit-function-declaration
CPPFLAGS:=$(CCFLAGS) -fno-exceptions -fno-rtti -felide-constructors
LDFLAGS:=-Wall $(MCUCFLAGS) $(MCULFLAGS) -Wl,--gc-sections

# Tools used in program
AR:=$(MCUPREFIX)ar
AS:=$(MCUPREFIX)as
CC:=$(MCUPREFIX)gcc
CPPCC:=$(MCUPREFIX)g++
OBJCOPY:=$(MCUPREFIX)objcopy
