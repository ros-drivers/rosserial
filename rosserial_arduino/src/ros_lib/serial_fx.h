/* 
 * Redefine these if needed
 */

#ifndef serial_fx_H
#define serial_fx_H

#include "WProgram.h"

void fx_open(){
    Serial.begin(57600);
}


int fx_putc(char c) {
    Serial.write(c);
    return 0;
}
 
int fx_getc(){
    return Serial.read();
}

#endif
