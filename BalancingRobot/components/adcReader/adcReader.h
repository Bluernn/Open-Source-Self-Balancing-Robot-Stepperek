#ifndef ADCREADER_H
#define	ADCREADER_H

void ADC_ReaderInit(void);  // ADC and LED initialization
void ADC_TaskStart(void *pvParameter);  // Task to read the ADC

#endif