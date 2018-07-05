#ifndef __CANNON_LOGGER_H__
#define __CANNON_LOGGER_H__

#include "API.h"

// 0 for no
// 1 for yes
#define DEBUG_MODE 0

// apply a macro to all args of a variadic macro, used for logg
// see APPLY_ALL
// #define APPLY0(t, dummy)
// #define APPLY1(t, a) t(a)
// #define APPLY2(t, a, b) t(a) t(b)
// #define APPLY3(t, a, b, c) t(a) t(b) t(c)
// #define APPLY4(t, a, b, c, d) t(a) t(b) t(c) t(d)
// #define APPLY5(t, a, b, c, d, e) t(a) t(b) t(c) t(d) t(e)
// #define APPLY6(t, a, b, c, d, e, f) t(a) t(b) t(c) t(d) t(e) t(f)

// #define NUM_ARGS_H1(dummy, x6, x5, x4, x3, x2, x1, x0, ...) x0
// #define NUM_ARGS(...) NUM_ARGS_H1(dummy, ##__VA_ARGS__, 6, 5, 4, 3, 2, 1, 0)
// #define APPLY_ALL_H3(t, n, ...) APPLY##n(t, __VA_ARGS__)
// #define APPLY_ALL_H2(t, n, ...) APPLY_ALL_H3(t, n, __VA_ARGS__)
// 
// #define APPLY_ALL(t, ...) APPLY_ALL_H2(t, NUM_ARGS(__VA_ARGS__), __VA_ARGS__)

// mine


// will print to uart2. 
// Use screen to access uart2 on the terminal with a serial-usb adapter+cable
// on device USB0 at 115200 baud: screen /dev/ttyUSB0 115200
#define loggy(fmtstr, ...) fprintf((uart2), "[%d]: "       \
                                            " " fmtstr " " \
                                            "\r\n",        \
                                   millis(), ##__VA_ARGS__)
// #define loggynonewline(fmtstr, ...) fprintf((uart2), fmtstr, ##__VA_ARGS__)

// will print to stdout, used by the serial client.
#define writey(...) fprintf((stdout), __VA_ARGS__)
#define writeychar(ch) fputc(ch, stdout);
#define readey() fgetc(stdin)

// will only print if debug mode is on (see top of this file).
#define loggydebug(fmtstr, ...) { \
  if(DEBUG_MODE) { \
  loggy(fmtstr, ##__VA_ARGS__); \
  } \
}

#endif
