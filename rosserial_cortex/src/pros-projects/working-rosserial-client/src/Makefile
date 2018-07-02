# Makefile for compiling PROS projects

# Path to project root (NO trailing slash!)
ROOT=..
# Binary output directory
BINDIR=$(ROOT)/bin

# Nothing below here needs to be modified by typical users

# Include common aspects of this project
-include $(ROOT)/common.mk

ASMSRC:=$(wildcard *.$(ASMEXT))
ASMOBJ:=$(patsubst %.o,$(BINDIR)/%.o,$(ASMSRC:.$(ASMEXT)=.o))
HEADERS:=$(wildcard *.$(HEXT))
### Special section for Cortex projects ###
HEADERS_2:=$(wildcard ../include/*.$(HEXT))
### End special section ###
CSRC=$(wildcard *.$(CEXT))
COBJ:=$(patsubst %.o,$(BINDIR)/%.o,$(CSRC:.$(CEXT)=.o))
CPPSRC:=$(wildcard *.$(CPPEXT))
CPPOBJ:=$(patsubst %.o,$(BINDIR)/%.o,$(CPPSRC:.$(CPPEXT)=.o))
OUT:=$(BINDIR)/$(OUTNAME)

.PHONY: all

# By default, compile program
all: .

# Compiles the program if anything is changed
.: $(ASMOBJ) $(COBJ) $(CPPOBJ)
	@touch .

# Assembly source file management
$(ASMOBJ): $(BINDIR)/%.o: %.$(ASMEXT)
	@echo AS $<
	@$(AS) $(AFLAGS) -o $@ $<

### Special section for Cortex projects ###

# Object management
$(COBJ): $(BINDIR)/%.o: %.$(CEXT) $(HEADERS) $(HEADERS_2)
	@echo CC $(INCLUDE) $<
	@$(CC) $(INCLUDE) $(CFLAGS) -o $@ $<

$(CPPOBJ): $(BINDIR)/%.o: %.$(CPPEXT) $(HEADERS) $(HEADERS_2)
	@echo CPC $(INCLUDE) $<
	@$(CPPCC) $(INCLUDE) $(CPPFLAGS) -o $@ $<

### End special section ###
