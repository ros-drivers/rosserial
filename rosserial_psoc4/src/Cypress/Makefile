# Makefile for building rosserial PSoC4 example apps
# 
# Set APP= to a name from catkin_ws/src/rosserial/rosserial_psoc4/src/ros_lib/examples
# Run make from within the catkin_ws/src/rosserial/rosserial_psoc4/src/Cypress directory
#  then copy output files to the Windows PSoC Creator environment:
#    exec.ihx to <project name>.cydsn/exec.ihx
#    makehex.bat to <project name>.cydsn/makehex.bat
#  From <project name>.cydsn/ , run makehex.bat
#  Use PSoC Creator "program" function (ctrl-F5) to program the Pioneer Kit board.

# name of example application
APP=HelloWorld

CREATORDIR=./rosserial_psoc4/$(APP).cydsn
$(info Using creator directory $(CREATORDIR))

PROJECT=$(APP)
ODIR=./out/
GENSRCDIR=$(CREATORDIR)/Generated_Source/PSoC4/
COMPONENTLIB=./CyComponentLibrary.a
PROJLIB=$(ODIR)$(PROJECT).a

CC=~/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-gcc
AR=~/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-ar
ASM=~/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-as
OBJCOPY=~/arduino-1.5.2/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-objcopy

PROJLIBCFLAGS= -I. -Wno-main -I$(GENSRCDIR) -mcpu=cortex-m0 -mthumb -Wall -g -D DEBUG -ffunction-sections
PROJLIBASMFLAGS= -I. -I$(GENSRCDIR) -mcpu=cortex-m0 -mthumb -g 
LDFLAGS=-mthumb -march=armv6-m -T $(GENSRCDIR)cm0gcc.ld -g -Wl,-Map,$(ODIR)$(PROJECT).map -Wl,--gc-sections 

PROJLIBSOURCES:=$(wildcard $(GENSRCDIR)*.c) $(GENSRCDIR)CyBootAsmGnu.s
PROJLIBOBJS_C:=$(patsubst $(GENSRCDIR)%.c, $(ODIR)%.o, $(filter $(GENSRCDIR)%.c, $(PROJLIBSOURCES)))
PROJLIBOBJS_C:=$(filter-out $(ODIR)Cm0Start.o, $(PROJLIBOBJS_C))
PROJLIBOBJS_S:=$(patsubst $(GENSRCDIR)%.s, $(ODIR)%.o, $(filter $(GENSRCDIR)%.s, $(PROJLIBSOURCES)))

SOURCEDIR=./libraries/ros_lib/
SOURCES:=$(wildcard $(SOURCEDIR)*.cpp)
OBJECTS:=$(patsubst $(SOURCEDIR)%.cpp, $(ODIR)%.o, $(SOURCES))
OBJECTS:=$(filter-out $(ODIR)init.o, $(OBJECTS))

APPDIR=$(SOURCEDIR)examples/$(APP)/

CXXFLAGS=  -I./libraries/ros_lib/ -Wno-main -mcpu=cortex-m0 -mthumb -Wall -g -D DEBUG -ffunction-sections -fno-rtti -fno-exceptions


all: exec.ihx

$(PROJLIBOBJS_C):$(ODIR)%.o:$(GENSRCDIR)%.c
	#patch capitalization errors in Cypress API code templates
	sed -i 's/\(#include[ \t]\+"\)cyLib.h"/\1CyLib.h"/' $<
	sed -i 's/\(#include[ \t]\+<\)CYLIB.H>/\1CyLib.h>/' $<
	sed -i 's/\(#include[ \t]\+<\)CYDEVICE_TRM.H>/\1cydevice_trm.h>/' $<
	sed -i 's/\(#include[ \t]\+<\)UART_SCB_IRQ.H>/\1UART_SCB_IRQ.h>/' $<
	sed -i 's/\(#include[ \t]\+<\)ADC_SAR_SEQ_IRQ.H>/\1ADC_SAR_SEQ_IRQ.h>/' $<
	$(CC) $(PROJLIBCFLAGS) -c $< -o $@

$(PROJLIBOBJS_S):$(ODIR)%.o:$(GENSRCDIR)%.s
	$(ASM) $(PROJLIBASMFLAGS) -o $@ $< 

$(PROJLIB):  $(PROJLIBOBJS_C) $(PROJLIBOBJS_S)
	$(AR) -rs $@ $^

$(ODIR)Cm0Start.o:$(GENSRCDIR)Cm0Start.c
	$(CC) $(PROJLIBCFLAGS) -c $< -o $@

$(ODIR)init.o:$(SOURCEDIR)init.cpp
	$(CC) -I$(CREATORDIR) -I$(GENSRCDIR) $(CXXFLAGS) -Wa,-alh=$*.lst -c $< -o $@

$(ODIR)$(APP).o:$(APPDIR)$(APP).cpp
	$(CC) -I$(CREATORDIR) -I$(GENSRCDIR) $(CXXFLAGS) -Wa,-alh=$*.lst -c $< -o $@

$(OBJECTS):$(ODIR)%.o:$(SOURCEDIR)%.cpp
	$(CC) $(CXXFLAGS) -Wa,-alh=$(ODIR)$*.lst -c $< -o $@

exec.elf: $(OBJECTS) $(PROJLIB) $(ODIR)Cm0Start.o $(ODIR)init.o $(ODIR)$(APP).o
	$(CC) $(LDFLAGS) -Wl,--start-group  -o $@ $(OBJECTS) $(ODIR)$(APP).o $(ODIR)Cm0Start.o $(ODIR)init.o $(PROJLIB) $(COMPONENTLIB) -Wl,--end-group

exec.ihx: exec.elf
	$(OBJCOPY) -O ihex $< $@
	echo cyhextool -o ./CortexM0/ARM_GCC_441/Debug/$(APP).hex -f exec.ihx -prot ./Generated_Source/PSoC4/protect.hex -id 4C81193 -a EEPROM=90200000:0,PROGRAM=00000000:8000,CONFIG=80000000:1000,PROTECT=90400000:40 -meta 1101 -chipProt 01 | tr "/" "\\\\" > makehex.bat
#	cp exec.ihx $(WINDOWS_SHARE)$(APP).cydsn
#	cp makehex.bat $(WINDOWS_SHARE)$(APP).cydsn

.PHONY: clean

clean:
	rm -rf $(ODIR)*

