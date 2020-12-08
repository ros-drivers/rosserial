# Simply include this file in your project ChibiOS Makefile.
ROSLIBDIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

ROSLIBSRC = $(ROSLIBDIR)/duration.cpp \
            $(ROSLIBDIR)/time.cpp
ROSLIBINC = $(ROSLIBDIR)

ALLCPPSRC += $(ROSLIBSRC)
ALLINC    += $(ROSLIBINC)
