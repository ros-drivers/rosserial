#ifndef _ROSSERIAL_VEX_CORTEX_LOGGER_H_
#define _ROSSERIAL_VEX_CORTEX_LOGGER_H_

#include "API.h"

// 0 for no
// 1 for yes
#define DEBUG_MODE 0

// will print to uart2. 
// Use screen to access uart2 on the terminal with a serial-usb adapter+cable
// on device USB0 at 115200 baud: screen /dev/ttyUSB0 115200
#define vexroslog(fmtstr, ...) fprintf((uart2), "[%d]: " " " fmtstr " " "\r\n", millis(), ##__VA_ARGS__)

// will print to stdout, used by the serial client.
#define vexroswrite(...) fprintf((stdout), __VA_ARGS__)
#define vexroswritechar(ch) fputc(ch, stdout);
#define vexrosreadchar() fgetc(stdin)

// will only print if debug mode is on (see top of this file).
#define vexroslogdebug(fmtstr, ...) { \
  if(DEBUG_MODE) { \
  vexroslog(fmtstr, ##__VA_ARGS__); \
  } \
}

#endif
