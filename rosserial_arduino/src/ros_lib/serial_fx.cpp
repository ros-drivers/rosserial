/* 
 * These are the serial function hooks for rosserial.
 */

#include "serial_fx.h"

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
