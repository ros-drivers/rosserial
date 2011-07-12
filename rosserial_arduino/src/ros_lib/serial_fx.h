/* 
 * These are the serial function hooks for rosserial.
 */

#ifndef serial_fx_H
#define serial_fx_H

#include "WProgram.h"

void fx_open();

int fx_putc(char c);
 
int fx_getc();

#endif
