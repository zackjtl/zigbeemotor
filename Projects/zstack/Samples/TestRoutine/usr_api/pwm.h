#ifndef _PWM_H_
#define _PWM_H_

#include "hal_types.h"
#include "hal_defs.h"

/****************************************************************************
常用宏定義
****************************************************************************/



void PWM_init(void);
void setFreq(uint16 Freq);
void pwmPulse(uint16 PulseFreq);
__interrupt void pwmISR (void);
#endif