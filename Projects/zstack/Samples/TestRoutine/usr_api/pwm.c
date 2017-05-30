#include <ioCC2530.h>
#include "pwm.h"


//pwm pins:
//P0.0
//P0.1

uint16 gRed;
uint16 gGreen;
uint16 gBlue;

void PWM_init()
{
#if 1
  P0DIR |= BV(4) | BV(5);  // Set P0.4 and P0.5 direction to output
  P0SEL |= BV(4) | BV(5);  // Select P0.4 and P0.5 to the Peripheral function
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
  ////uint16 r,g,b;
  // stop,注意，不能加這句，加了週期偏差十幾倍，具體原因未查明
  //T1CTL &= BV(0)|BV(1); 
#if 0
  r=375;
  g=1;
  b=1;
#else
  //r=red;
  //g=green;
  //b=blue;
#endif
  // Set up the timer registers
#if 1
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

void setRGB(uint16 red, uint16 green, uint16 blue)
{
  gRed=red;
  gGreen=green;
  gBlue=blue;
}

//#pragma register_bank=2
#pragma vector = T1_VECTOR
__interrupt void pwmISR (void) {
    uint8 flags = T1STAT;
    // T1 ch 0
    if (flags & 0x01){          
      pwmPulse(gRed);
     
    }
    T1STAT = ~ flags;
}
