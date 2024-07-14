Basic GPT7 16bit timer setup with PWM set by ADC values, with serial single-char commands, and Wake-From-Interrupt option.

This code is set up to produce a High Output at 0% to 50% duty-cycle, 
... with an interrupt at the Count Overflow, i.e. falling edge of the cycle
