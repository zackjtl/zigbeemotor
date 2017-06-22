#include <ioCC2530.h>
#include "pwm.h"


//pwm pins:
//P0.0
//P0.1

uint16 gFreq;


void PWM_init()
{
#if 1
  P0DIR |= BV(3) | BV(4);  // Set P0.4 and P0.5 direction to output
  P0SEL |= BV(3) | BV(4);  // Select P0.4 and P0.5 to the Peripheral function
  PERCFG &= (~(0x40));       // Set Timer1 pin location to alternative 1
  ////PERCFG |= 0x01;
  P2DIR = 0xC0 | (0x3F & P2DIR);  // Make Timer 1 CH2, 3 have first priority on P0 peripheral function.
  
  // Initialize Timer 1
  T1CTL = 0x0C;               // Div = 128, CLR, MODE = Suspended          
  ////T1CCTL1 = 0x0C;             // IM = 0; CMP = Clear output on compare; Mode = Compare
  T1CCTL2 = 0x0C;             // IM = 0; CMP = Clear output on compare; Mode = Compare
  T1CCTL3 = 0x0C;             // IM = 0, CMP = Clear output on compare; Mode = Compare
  T1CNTL = 0;                 // Reset timer to 0;
  T1CCTL0 = 0x4C;                   
  
  T1CC0H = 0x01;             
  T1CC0L = 0x77;                     
  ////T1CC1H = 0x01;             
  ////T1CC1L = 0x77;
  T1CC2H = 0x01;              
  T1CC2L = 0x77;
  T1CC3H = 0x01;              
  T1CC3L = 0x77;    
  
  EA=1;
  IEN1 |= 0x02;               // Enable T1 cpu interrupt  
#else
  //設置pwm端口為輸出
  P1DIR|= BV(0)|BV(1);
  //設置pwm端口為外設端口，非gpio
  P1SEL|= BV(0)|BV(1);
  //由於uart等會佔用我們當前使用的pwm端口，因此需要將uart等重映射到別的端口去。
  PERCFG |= 0x40;             // Move USART1&2 to alternate2 location so that T1 is visible

  // Initialize Timer 1
  T1CTL = 0x0C;               // Div = 128, CLR, MODE = Suspended          
  T1CCTL1 = 0x0C;             // IM = 0; CMP = Clear output on compare; Mode = Compare
  T1CCTL2 = 0x0C;             // IM = 0; CMP = Clear output on compare; Mode = Compare
  ////T1CCTL3 = 0x0C;             // IM = 0, CMP = Clear output on compare; Mode = Compare
  T1CNTL = 0;                 // Reset timer to 0;

  T1CCTL0 = 0x4C;           
  T1CC0H = 0x01;             
  T1CC0L = 0x77;            
          
  T1CC1H = 0x01;             
  T1CC1L = 0x77;
  T1CC2H = 0x01;              
  T1CC2L = 0x77;
  ////T1CC3H = 0x01;              
  ////T1CC3L = 0x77;  

  EA=1;
  IEN1 |= 0x02;               // Enable T1 cpu interrupt
#endif
}


void pwmPulse(uint16 PulseFreq)
{
#if 0
  T1CC2L = (uint8)PulseFreq;
  T1CC2H = (uint8)(PulseFreq >> 8);
  T1CC3L = (uint8)PulseFreq;
  T1CC3H = (uint8)(PulseFreq >> 8);
#else
  T1CC1L = (uint8)PulseFreq;
  T1CC1H = (uint8)(PulseFreq >> 8);
  T1CC2L = (uint8)PulseFreq;
  T1CC2H = (uint8)(PulseFreq >> 8);  
#endif
  // Reset timer
  T1CNTL = 0;
  
  // Start timer in modulo mode.
  T1CTL |= 0x02;   
}

void setFreq(uint16 Freq)
{
  gFreq=Freq;
}

//#pragma register_bank=2
#pragma vector = T1_VECTOR
__interrupt void pwmISR (void) {
    uint8 flags = T1STAT;
    // T1 ch 0
    if (flags & 0x01){          
      pwmPulse(gFreq);
     
    }
    T1STAT = ~ flags;
}
