#ifndef _ROSSERIAL_VEX_CORTEX_LOGGER_H_
#define _ROSSERIAL_VEX_CORTEX_LOGGER_H_

#include "API.h"

// Use screen to access uart2 on the terminal with a serial-usb adapter+cable
// on device USB0 at 115200 baud: screen /dev/ttyUSB0 115200

// VEXROS defines are used only locally.

// debugging works over an alternative serial connection.
// 0 for no
// 1 for yes
#define VEXROS_DEBUG_MODE 0

// which serial connection to output debug messages on.
#define VEXROS_DEBUG_OUTPUT_SERIAL uart2

// logging function for debugging via print statements in the PROS environment.
// this macro simply adds formatting ontop of the regular pros frintf function.
// usage: vexroslog("hello, my favorite number is %d", 3);
// remember to include this header for logging in user code!
#define vexroslog(fmtstr, ...) fprintf((VEXROS_DEBUG_OUTPUT_SERIAL ), "[%d]: " " " fmtstr " " "\r\n", millis() ,##__VA_ARGS__)

// will print to stdout, used by the serial client.;
// note: stdin/stdout can both be replaced with uart1 or uart2, note that stdin/stdout being seperate is unique 
// with uart1/uart2, they work as both reading and writing serial connections. 
#define vexroswrite(...) fprintf((stdout), __VA_ARGS__)
#define vexroswritechar(ch) fputc(ch, stdout)
#define vexrosreadchar() fgetc(stdin)

// will only print if debug mode is on (see top of this file).
#define vexroslogdebug(fmtstr, ...) { \
  if(DEBUG_MODE) { \
  vexroslog(fmtstr, ##__VA_ARGS__); \
  } \
}

#endif
