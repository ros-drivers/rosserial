#ifndef __CANNON_LOGGER_H__
#define __CANNON_LOGGER_H__

#include "API.h"

// 0 for no
// 1 for yes
#define DEBUG_MODE 0

// stackoverflow code to apply a macro to all args of a variadic macro
// see APPLY_ALL
#define APPLY0(t, dummy)
#define APPLY1(t, a) t(a)
#define APPLY2(t, a, b) t(a) t(b)
#define APPLY3(t, a, b, c) t(a) t(b) t(c)
#define APPLY4(t, a, b, c, d) t(a) t(b) t(c) t(d)
#define APPLY5(t, a, b, c, d, e) t(a) t(b) t(c) t(d) t(e)
#define APPLY6(t, a, b, c, d, e, f) t(a) t(b) t(c) t(d) t(e) t(f)

#define NUM_ARGS_H1(dummy, x6, x5, x4, x3, x2, x1, x0, ...) x0
#define NUM_ARGS(...) NUM_ARGS_H1(dummy, ##__VA_ARGS__, 6, 5, 4, 3, 2, 1, 0)
#define APPLY_ALL_H3(t, n, ...) APPLY##n(t, __VA_ARGS__)
#define APPLY_ALL_H2(t, n, ...) APPLY_ALL_H3(t, n, __VA_ARGS__)

#define APPLY_ALL(t, ...) APPLY_ALL_H2(t, NUM_ARGS(__VA_ARGS__), __VA_ARGS__)

// mine

#define loggy(fmtstr, ...) fprintf((uart2), "[%d]: "       \
                                            " " fmtstr " " \
                                            "\r\n",        \
                                   millis(), ##__VA_ARGS__)
#define loggynonewline(fmtstr, ...) fprintf((uart2), fmtstr, ##__VA_ARGS__)
#define writey(...) fprintf((stdout), __VA_ARGS__)
#define writeychar(ch) fputc(ch, stdout);
#define readey() fgetc(stdin)

#define loggydebug(fmtstr, ...) { \
  if(DEBUG_MODE) { \
  loggy(fmtstr, ##__VA_ARGS__); \
  } \
}

#define loggymemory(variable) loggy(#variable " variable takes up %d bytes of memory at location %p", sizeof(variable), &variable)
#define loggymemorytablerow(variable) loggy(#variable ":\r\t\t%d b,\tloc: \t%p, val: (int) %d (str) %s", sizeof(variable), &variable, variable, variable);

#define loggyfunction(func) loggy(#func " function is at location %p", &func);

// print a formatted table of the variables passed in, and their size/memory.
#define loggymemorytable(...)                      \
  loggy("\n\rMemory Table\n\r.................."); \
  APPLY_ALL(loggymemorytablerow, __VA_ARGS__)      \
  loggy("..................\n\r");

// give a name to the table first. useful for timestamped tables.
#define loggymemorytablenamed(name, ...)                      \
  loggy("\n\rMemory table: " #name "\n\r.................."); \
  APPLY_ALL(loggymemorytablerow, __VA_ARGS__)                 \
  loggy("..................\n\r");

// print a formatted function table of the variables passed in, and their size/memory.
#define loggyfunctiontable(...)                    \
  loggy("\n\rMemory Table\n\r------------------"); \
  APPLY_ALL(loggyfunction, __VA_ARGS__)            \
  loggy("-----------------\n\r");

// give a name to the function table first. useful for timestamped tables.
#define loggyfunctiontablenamed(name, ...)                    \
  loggy("\n\rFunction table: " #name "\n\r----------------"); \
  APPLY_ALL(loggyfunction, __VA_ARGS__)                       \
  loggy("-----------------\n\r");

#endif