#ifndef _ROSSERIAL_VEX_CORTEX_LOGGER_H_
#define _ROSSERIAL_VEX_CORTEX_LOGGER_H_

#include "API.h"

// VEXROS defines are used only locally.

// debugging works over an alternative serial connection.
// 0 for no
// 1 for yes
#define VEXROS_DEBUG_MODE 0

// which serial connection to output debug messages on.
// To switch the serial connections, switch these values.
// note that stdin/stdout being seperate is unique - stdin/stdout can both be replaced with either uart1 or uart2, 
// because UART1 and UART2 work as both input and output serial connections. 
#define VEXROS_DEBUG_OUTPUT_SERIAL uart2
#define VEXROS_ROSSERIAL_OUTPUT_SERIAL stdout
#define VEXROS_ROSSERIAL_INPUT_SERIAL stdin

// logging function for debugging via print statements in the PROS environment.
// this macro simply adds formatting ontop of the regular pros frintf function.
// usage: vexroslog("hello, my favorite number is %d", 3);
// remember to include this header for logging in user code!
#define vexroslog(fmtstr, ...) fprintf((VEXROS_DEBUG_OUTPUT_SERIAL ), "[%d]: " " " fmtstr " " "\r\n", millis() ,##__VA_ARGS__)

// will print to stdout, used by the serial client.;
#define vexroswrite(...) fprintf((VEXROS_ROSSERIAL_OUTPUT_SERIAL), __VA_ARGS__)
#define vexroswritechar(ch) fputc(ch, VEXROS_ROSSERIAL_OUTPUT_SERIAL)
#define vexrosreadchar() fgetc(VEXROS_ROSSERIAL_INPUT_SERIAL)

// will only print if debug mode is on (see top of this file).
#define vexroslogdebug(fmtstr, ...) { \
  if(DEBUG_MODE) { \
  vexroslog(fmtstr, ##__VA_ARGS__); \
  } \
}

#endif
