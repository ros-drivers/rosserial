/* include file for rosserial ADC example on PSoC4 with interrupt-driven conversion
 *
 */

#ifndef ADC_PSOC4_H
#define ADC_PSOC4_H

extern "C" {
#include "device.h"
}

static volatile int16 result[ADC_SAR_SEQ_TOTAL_CHANNELS_NUM];
CY_ISR(ADC_SAR_SEQ_ISR_LOC)
{
    uint32 intr_status;

    /* Read interrupt status registers */
    intr_status = ADC_SAR_SEQ_SAR_INTR_MASKED_REG;
    /* Check for End of Scan interrupt */
    if((intr_status & ADC_SAR_SEQ_EOS_MASK) != 0u)
    {
        unsigned int chan;
        for (chan=0; chan<ADC_SAR_SEQ_TOTAL_CHANNELS_NUM; ++chan)
        {
            /* Read conversion result */
            result[chan] = ADC_SAR_SEQ_GetResult16(chan);

        }    
    }    

    /* Clear handled interrupt */
    ADC_SAR_SEQ_SAR_INTR_REG = intr_status;
}

void adc_setup() {
  /* Init and start sequencing SAR ADC */
  ADC_SAR_SEQ_Start();
  ADC_SAR_SEQ_StartConvert();
  /* Enable interrupt and set interrupt handler to local routine */
  ADC_SAR_SEQ_IRQ_StartEx(ADC_SAR_SEQ_ISR_LOC);
}

int analogRead(int pin) {
  return result[pin];
}

#endif
