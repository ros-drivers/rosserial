/* 
 * Redefine these if needed
 */

#include "WProgram.h"

void fx_open(){
    Serial.begin(57600);
}

inline int fx_putc(char c) {
    Serial.write(c);
    return 0;
}

inline int fx_getc(){
    return Serial.read();
}
