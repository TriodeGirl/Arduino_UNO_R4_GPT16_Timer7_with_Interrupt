/*  Arduino UNO R4 Minima demo code for core RA4M1 peripheral operations:
 *
 *  NOTE: Pin allocation is as per R4 Minima, the R4 WiFi has many board-pins with different connections.
 *
 *  Susan Parker, my code stuff may be found here: https://github.com/TriodeGirl
 *
 *  Susan Parker - 13/14th July 2024
 *  Basic GPT7 16bit timer setup with PWM set by ADC values.
 *  Single serial char commands
 *  Wake from Interrupt option
 *
 *  This code is set up to produce a High Output at 0% to 50% duty-cycle, 
 *  ... with an interrupt at the Count Overflow, i.e. falling edge of the cycle
 *
 *
 * This code is "AS IS" without warranty or liability. 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed with the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
    
    See the GNU General Public License for more details.
*/

// ARM-developer - Accessing memory-mapped peripherals
// https://developer.arm.com/documentation/102618/0100

// Get the #include file from:
// https://github.com/TriodeGirl/RA4M1_Arduino_UNO-R4_Processor_Direct_Register_Addressing_Defines/blob/main/susan_ra4m1_minima_register_defines.h
// ... change below as appropriate
#include "I:\Arduino Projects\6 - Uno R4 Boards\Code\Includes\susan_ra4m1_minima_register_defines.h" // 

// ==== Local Defines ====

#define PRE_SCALE_DIV64     // Use a divide by 64 prescale
#define PWM_NO_HIGH_PULSE   // Set 50% to 0% PWM. Comment out for c. 49.9% to 0.1% PWM

#define ADC_LEFT_JUST   // Use 16 bit values, 2 LSBs null
#define DAC_LEFT_JUST   // Use 16 bit values, 4 LSBs null

#define ADC_IRQ_CODE    // Call ADC complete interrupt function

// ==== Global Variables ====

uint32_t currentMillis  = 0x0;
uint32_t previousMillis = 0x0;
uint32_t lastMillis = 0x0;

uint32_t secondsCount = 0;
bool minute_flag = false;
static bool wake_from_int_flag = false;

static uint16_t adc_val_A1;
static uint16_t adc_val_A2;
static uint32_t loop_counter = 0;


void setup()
  {
  attachInterrupt(13, timer7interrupt, FALLING);        // This IRQ will be asigned to Slot 05 IELSR05 as 0x001 PORT_IRQ0 - Table 13.4
  *ICU_IELSR05 = IRQ_GPT7_OVF;                          // Assign Slot 05 IELSR05 for GPT7_OVF
  *PFS_P111PFS = 0x00000000;                            // Clear D13 ISEL pin assigned Interrupt Enable
	asm volatile("dsb");                                  // Data bus Synchronization instruction
  *NVIC_IPR05_BY = 0x40;                                // Bounce ICU_IELSR05 irq priority up from 0xC0 to 0x40

#ifdef ADC_IRQ_CODE
  attachInterrupt(15, adcCompleteInterrupt, FALLING);   // This IRQ will be asigned to Slot 06 IELSR06 as 0x002 PORT_IRQ1 - Table 13.4
  *PFS_P000PFS = 0x00000000;                            // Clear A1/D15 ISEL pin assigned Interrupt Enable
  *ICU_IELSR06 = IRQ_ADC140_ADI;                        // Assign Slot 06 IELSR06 for ADC140_ADI
#endif

  Serial.begin(115200);      // The interrupts for the USB serial are already in place before setup() starts
  while (!Serial){};         // Note: USB serial cannot be used for serial comms when running fast IRQs - diagnostics only.
//  while (!Serial && (millis() < 5000);  // Sugested by KurtE - 5 second timeout if using standalone

  setup_adc();
  setup_dac();

  Serial.println("\nGPT Timer Operation");

// Setup fast IRQ timers ABSOLUTLY last!!!  
  setup_timers();

  previousMillis = lastMillis = currentMillis = millis();  // Only keep millis functions for "slow" timer functions i.e. below 10kHz
//  *AGT0_AGTCR = 0;           // disable Millis counter, delay etc. don't want this cutting into Interrupt response time
  }


void loop()  // Some basic functions
  {
  *PFS_P111PFS_BY = 0x04;        // D13 - CLEAR LED
  if(wake_from_int_flag == true)
    asm volatile("wfi");           // Stop here and wait for an interrupt
  *PFS_P111PFS_BY = 0x05;        // D13 - Set LED

  currentMillis = millis();

  if((currentMillis - lastMillis) >= 1)  // Only do an update once per mS
    {
   	lastMillis = currentMillis;
    }

  if((currentMillis - previousMillis) >= 1000) 
    {
   	previousMillis = currentMillis;
    secondsCount++;
    Serial.print(".");          // print a "." once per second - Proof of Life !
    if(secondsCount >= 60)  // 
      {
      secondsCount = 0;
      minute_flag = true;       // Change flag to Blip a LED
      Serial.print("\n");       // print a NewLine once per minute
      minute_flag = false;
      }
    }

  if(Serial.available() > 0)
    {
    char input_char = Serial.read();  // get one byte from serial buffer
    Serial.println(input_char);
    switch(input_char)
      {
      case 's':
        {
        Serial.println("\nEnable Wake from Interrupt");
        secondsCount = 0;
        wake_from_int_flag = true; 
        break;
        }
      case 'r':
        {
        Serial.println("\nDisable Wake from Interrupt");
        secondsCount = 0;
        wake_from_int_flag = false;       // Disable WFI, otherwsie double tap on Reset-Button needed
        break;
        }
      case '?':
        {
        serial_commands();
        break;
        }
      default:
        break;
      }
    }

  }


void serial_commands(void)
  {
  Serial.println("\nGPT Timer Setup");
  Serial.println("s = Enable Wake from Interrupt");
  Serial.println("r = Disable Wake from Interrupt");
  Serial.println("? = This Help List");
  }


void timer7interrupt(void)
  {
//  static uint16_t localValA0 = 0;  // Board input ADC A0 used for DAC output
  static uint16_t localValA1 = 0;  // Board input ADC A1
  static uint16_t localValA2 = 0;  // Board input ADC A1
  static uint16_t localCount = 0;
  static uint8_t test = 0;

  *PFS_P107PFS_BY = 0x05;         // D7 start of Timer Interrupt

// ==== ADC ====

  adc_val_A1 = *ADC140_ADDR00;    //  
  adc_val_A2 = *ADC140_ADDR01;    //  
  *ADC140_ADCSR |= (0x01 << 15);  // Next ADC conversion = c. 300nS

// ==== DAC Monitor ====

#ifdef ADC_LEFT_JUST && DAC_LEFT_JUST
  *DAC12_DADR0 = adc_val_A1;                 // DAC update - both ADC and DAC left justified
#else
  *DAC12_DADR0 = adc_val_A1 >> 2;            // DAC update - change 14 to 12 bits, shift down by 2 
#endif

// ==== Set PWM ====

#ifdef PWM_NO_HIGH_PULSE
#define ADC_PWM_OFFSET 0x3FFF  // Half of total count, max ADC value has NO high pulse.
#else
#define ADC_PWM_OFFSET 0x3FFE  // Half of total count less -1, max ADC value is a single count high pulse.
#endif

#ifdef ADC_LEFT_JUST && DAC_LEFT_JUST
#define ADC_PWM_SHIFT 2        // Using a total counter count of 0x7FFF i.e. 15 bits
  *GPT167_GTCCRC = ((int)adc_val_A1 >> ADC_PWM_SHIFT) + ADC_PWM_OFFSET;    // A compare PWM on pin D8
  *GPT167_GTCCRD = ((int)adc_val_A2 >> ADC_PWM_SHIFT) + ADC_PWM_OFFSET;    // B compare PWM on pin D9
#else
#define ADC_PWM_SHIFT 0        // Using a total counter count of 0x7FFF i.e. 15 bits
  *GPT167_GTCCRC = (adc_val_A1 >> ADC_PWM_SHIFT) + ADC_PWM_OFFSET;    // A compare PWM on pin D8
  *GPT167_GTCCRD = (adc_val_A2 >> ADC_PWM_SHIFT) + ADC_PWM_OFFSET;    // B compare PWM on pin D9
#endif

  loop_counter++;

  *PFS_P107PFS_BY = 0x04;      // Clear D7 Interrupt Monitor pin
  }


#ifdef ADC_IRQ_CODE
void adcCompleteInterrupt(void)
  {
  *PFS_P106PFS_BY = 0x05;      // D6 
  *PFS_P106PFS_BY = 0x04;      //  
  }
#endif


void setup_adc(void)
  {
  *MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD16));  // Enable ADC140 module
// Set ADC Reference source
//  *ADC140_ADHVREFCNT = ADC_HVSEL_VREFH0;           // Set External Aref = analogReference(AR_EXTERNAL);      
  *ADC140_ADHVREFCNT = ADC_HVSEL_AVCC0;            // Set analog reference to AVVC0 supply - when no 4.096V ref      
// Set ADC Mode
  *ADC140_ADCER      = (0x01 << ADCER_ADPCR_1_0);  // 14 bit mode
//  *ADC140_ADCER   |= (0x01 << ADCER_ACE);          // Set ACE bit 5 - A/D Data Register Automatic Clearing Enable
#ifdef ADC_LEFT_JUST
  *ADC140_ADCER   |= (0x01 << ADCER_ADRFMT);       // A/D Data Register Format Select - 1: Flush-left for the A/D data
#endif
// Set ADC Control Register
  *ADC140_ADCSR   |= (0x01 << ADCSR_GBADIE);    // Set b6 - GBADIE Group B Scan End Interrupt Enable
// Set ADC Inputs
//  *ADC140_ADANSA0 |= (0x01 << 9);    // Selected ANSA09
  *ADC140_ADANSA0 |= (0x01 << ADANSA0_ANSA01);    // Selected ANSA01
  *ADC140_ADANSA0 |= (0x01 << ADANSA0_ANSA00);    // Selected ANSA00
//  *ADC140_ADANSA1 |= (0x01 << ADANSA1_ANSA16);    // V-divider net R10(+5V>100K)/R26(13.7K>GND) (P500)
// Set ADC Averaging  
//  *ADC140_ADADC    = 0x83;           // Average mode - 4x
  *ADC140_ADADC    = (ADC_AVG_4 << ADADC_ADC_2_0);  // Average mode - 4x
//  *ADC140_ADADS0  |= (0x01 << 9);    // Enable Averaging for ANSA09
  *ADC140_ADADS0  |= (0x01 << ADADS0_ADS01);    // Enable Averaging for ANSA01
  *ADC140_ADADS0  |= (0x01 << ADADS0_ADS00);    // Enable Averaging for ANSA00
// Start ADC cycle
// *ADC140_ADCSR   |= (0x01 << ADCSR_ADST);   // Start a conversion
  }

void setup_dac(void)       // Note make sure ADC is stopped before setup DAC
  {
  *MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD20));  // Enable DAC12 module
#ifdef DAC_LEFT_JUST
  *DAC12_DADPR    = 0x80;        // DADR0 Format Select Register - Set left-justified format i.e. 16 bit format, 4 LSBs not used
#else
  *DAC12_DADPR    = 0x00;        // DADR0 Format Select Register - Set right-justified format
#endif
//  *DAC12_DAADSCR  = 0x80;        // D/A A/D Synchronous Start Control Register - Enable
  *DAC12_DAADSCR  = 0x00;        // D/A A/D Synchronous Start Control Register - Default
// 36.3.2 Notes on Using the Internal Reference Voltage as the Reference Voltage
  *DAC12_DAVREFCR = 0x00;        // D/A VREF Control Register - Write 0x00 first - see 36.2.5
  *DAC12_DADR0    = 0x0000;      // D/A Data Register 0 
   delayMicroseconds(10);        
  *DAC12_DAVREFCR = 0x01;        // D/A VREF Control Register - Select AVCC0/AVSS0 for Vref
//  *DAC12_DAVREFCR = 0x03;        // D/A VREF Control Register - Select Internal reference voltage/AVSS0
//  *DAC12_DAVREFCR = 0x06;        // D/A VREF Control Register - Select External Vref; set VREFH&L pins used for LEDs
  *DAC12_DACR     = 0x5F;        // D/A Control Register - 
   delayMicroseconds(5);         // 
  *DAC12_DADR0    = 0x0800;      // D/A Data Register 0 
  *PFS_P014PFS   = 0x00000000;   // Make sure all cleared
  *PFS_P014PFS  |= (0x1 << 15);  // Port Mode Control - Used as an analog pin
  }

// ==== GPT - General Purpose Timers ==== //

void setup_timers(void)
  {
  *MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD6));  // Enable GTP16 timer module
                //   76543210
  *GPT167_GTSTP  = 0b10000000; // Stop Timer 7  - Doesn't matter which timer is addressed

// Timer 7 - Sawtooth Mode 
  *GPT167_GTSSR = 0x80000000; // b31 CSTRT Software Source Counter Start Enable
  *GPT167_GTPSR = 0x80000000; // b31 CSTOP Software Source Counter Stop Enable
  *GPT167_GTCSR = 0x80000000; // b31 CCLR  Software Source Counter Clear Enable

  *GPT167_GTUDDTYC = 0x1;
  *GPT167_GTCNT  = 0x0000;     // Set Counter value
  *GPT167_GTCCRA = 0x0000;     // 
  *GPT167_GTCCRB = 0x0000;     // 
  *GPT167_GTCCRC = 0x0000;     // 
  *GPT167_GTCCRD = 0x0000;     // 
  *GPT167_GTPR   = 0x7FFF;     // 
  *GPT167_GTPBR  = 0x7FFF;     // 
  *GPT167_GTBER  = 0x00150000; // PR, CCRB, and CCRA set to single buffer operation 

// b26-b24 - b26 to b24 TPCS[2:0] Timer Prescaler Select
//  0 0 0: PCLKD/1
//  0 0 1: PCLKD/4
//  0 1 0: PCLKD/16
//  0 1 1: PCLKD/64
//  1 0 0: PCLKD/256
//  1 0 1: PCLKD/1024.
  *GPT167_GTCR   = 0x0;           // Set Sawtooth mode, PreScale to PCLKD/1 
#ifdef PRE_SCALE_DIV64
  *GPT167_GTCR   = 0b011 << 24;   // Set PreScale to PCLKD/64 = 22.89Hz with Counter set to 0x7FFF
#endif

// RA4M1 Data-Sheet - 22.2.14 General PWM Timer I/O Control Register (GTIOR)
//                  - Table 22.5 Settings of GTIOA[4:0] and GTIOB[4:0] bits
  *GPT167_GTIOR  = 0x00000000;   // Clear I/O Control Register
//  *GPT167_GTIOR |=  0b01001       ; // GTIOA[4:0] = 01001b - Initial output is low; Low Output at GTCCRA compare match; HIGH at end
//  *GPT167_GTIOR |= (0b01001 << 16); // GTIOB[4:0] = 01001b - Initial output is low; Low Output at GTCCRB compare match; HIGH at end
  *GPT167_GTIOR |=  0b00110       ; // GTIOA[4:0] = 01001b - Initial output is low; High Output at GTCCRA compare match; LOW at end
  *GPT167_GTIOR |= (0b00110 << 16); // GTIOB[4:0] = 01001b - Initial output is low; High Output at GTCCRB compare match; LOW at end
  *GPT167_GTIOR |= 0x01000100; // Set OBE and OAE bits 

  *PFS_P304PFS   = (0b00011 << 24);  // Select PSEL[4:0] for GTIOC7A - See Table 19.10
  *PFS_P304PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P303PFS   = (0b00011 << 24);  // Select PSEL[4:0] for GTIOC7B - See Table 19.10
  *PFS_P303PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function

  *GPT167_GTSTR  = 0b10000000;  // Start Timer 7 - Doesn't matter which timer is addressed
  *GPT167_GTCR   |= 0x1;        // Start Timer 7 with a PreScale of zero
//  *GPT167_GTCLR  = 0b10000000;  // Clear Timer 7 - use to clear multipl times to align counts, not needed for single timer
  }

