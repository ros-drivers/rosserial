/* include file for rosserial pubsub example on PSoC4 : led output
 *
 */

#ifndef BLUELED_PSOC4_H
#define BLUELED_PSOC4_H

extern "C" {
#include "device.h"
}


void blueLed_setup() {
  Pin_BlueLED_SetDriveMode(Pin_BlueLED_DM_STRONG);
}

void setBlueLed(bool on) {
  Pin_BlueLED_Write(on ? 0 : 0xFF);
}

#endif

