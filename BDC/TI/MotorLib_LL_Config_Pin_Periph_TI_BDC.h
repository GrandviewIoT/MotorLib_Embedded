
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                        MotorLib_LL_Config_TI_BDC.h
//
// Motor and platform specific settings for BDC motor control chips.
//
// Contains detailed settings for BDC Controller chips DRV8848, L6293, TNS754410
//
//
// L6206 Xnucleo Usage
// -------------------
//       Vm     Motor 6 V supply         -
//       Gnd    Battery Ground           -
//
//       AIN1  1A   Motor 1   TIM3_CH1   PB4   Arduino D5
//       AIN2  2A   Motor 2   TIM3_CH2   PB5   Arduino D4
//
//       BIN1  1B   Motor 1   TIM2_CH1   PA0   Arduino A0
//       BIN2  2B   Motor 2   TIM2_CH2   PA1   Arduino A1
//
//    A ENABLE / OCD  Motor 1            PA10  Arduino D2
//    B ENABLE / OCD  Motor 2            PC1   Arduino A4
//
//       AOUT1  Motor 1 OUTPUT Left      -
//       AOUT2  Motor 1 OUTPUT Right     -
//       BOUT1  Motor 2 OUTPUT Left      -
//       BOUT2  Motor 2 OUTPUT Right     -
//
//       "A" side Current-Sense          PA6  Arduino D12   ADC12_IN6
//       "B" side Current-Sense          PA4  Arduino A2    ADC12_IN4
//
//        -     Speed Ctl Pot ADC        PB5  Arduino A2    ADC12_IN4   Grove A3
//        -     Fwd/Reverse Slider       PB3  Arduino D3    GPIO rupt   Grove D3
//
//        -     Hall Sensor Left         PA8  Arduino D7    GPIO rupt   Grove D7
//        -     Hall Sensor Right        PA9  Arduino D8    GPIO rupt   Grove D8
//                   White lead = Signal,  Red lead = +3.3   Black lead = Gnd
//
//  Dagu BDC Motor:  Geared, 140 RPM     (gearbox ratio 48:1) Torque: 800 gf-cm
//                   4.5 - 6.0 Volts
//                   No Load:  150ma     Stall Current: 2.75 A at 6V
//                   Measured Motor Resistance (Rload):  5.7 ohms
//
// History:
//   07/22/16 - Carved off into separate file. Duquaine
//
// -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
//
// The MIT License (MIT)
//
// Copyright (c) 2016 Wayne Duquaine / Grandview Systems
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//******************************************************************************

#ifndef _MOTOR_LL_BDC_CONFIG_H_
#define   _MOTOR_LL_BDC_CONFIG_H_

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                        Internal  Platform  Definitions
//
// Do not alter unless you really know what you are doing
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                   MSP430  G2553      Macros  and  Includes  used
//
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(MSP430_G2553_LAUNCHPAD)

#define  TI_CCS_COMPILER            1        // Denote are using TI CCS compiler

#define  MCU_SPEED_16MHZ         16000000         // MSP430 FR5969
#define  ADC_RESOLUTION_10_BIT      1             // with 10 bit ADC resolution
#define  MSP430_MCU                 1             // denote is a MSP430 MCU

#include "msp430.h"                               // pull in MCU defs for MSP430

                // Switch Configuration  J1-4  P2.5  GPIO    Grove J1-4 -> J1-3
#define SWITCH1_CONFIG()      { P2DIR &= ~(BIT5); }  // DIR: 0 = in,  1 = out
#define SWITCH1_READ()        ( P2IN  &= BIT5 )

#define CONFIG_HALL_ENCODER_A   { P4DIR &= ~(BIT1); } // P4.1 J1-5  DIR: 0=in, 1=out
#define CONFIG_HALL_ENCODER_B   { P4DIR &= ~(BIT3); } // P4.3 J1-6
#define CONFIG_HALL_ENCODER_C_Z { P4DIR &= ~(BIT6); } // P4.6 J1-8
#define READ_HALL_ENCODER_A()   ( P4IN  &= BIT1 )
#define READ_HALL_ENCODER_B()   ( P4IN  &= BIT3 )
#define READ_HALL_ENCODER_C_Z() ( P4IN  &= BIT6 )


// GPIO Pin Definitions - all are on Port 2
#define BIN1    BIT1    // P2.1   J1-9  (Boosterpack)
#define BIN2    BIT2    // P2.2   J1-10
#define	AIN1    BIT5    // P2.5   J2-8
#define AIN2    BIT4    // P2.4   J2-9
#define nSLEEP  BIT6    // P2.6   J2-2
#define nFAULT  BIT7    // P2.7   J2-3

#define nSLEEP_CONFIG_GPIO() { P2DIR |=  BIT6;   } // P2.6 J2-2  DIR: 0=in, 1=out
#define nSLEEP_SET_ASLEEP()  { P2OUT &= ~(BIT6); } //    Sleep = Active Low
#define nSLEEP_SET_AWAKE()   { P2OUT |=  BIT6;   }

#define nFAULT_CONFIG_GPIO() { P2DIR &= ~(BIT7); } // P2.7 J2-3  DIR: 0=in, 1=out
#define nFAULT_CLEAR_FAULT() { P2OUT |= BIT6;    }
#define nFAULT_HAS_FAULT     (P2OUT & BIT6) == 0)  //    Fault = Active Low
#define nFAULT_NO_FAULT      (P2OUT & BIT6)

                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
                //    PWM   CONTROL   SIGNALS  for   COMMUTATION         MSP430
                //
                // MSP430 G2553 essentially performs un-natural acts to drive
                // two motors. It re-programs adjacent pairs of pins back
                // and forth between PWM mode and GPIO mode to effect proper
                // AIN1/AIN2 and BIN1/BIN2 sequences.
                //   AIN1 = J2-8  = pin 13 = P2.5/TA1.2  <-- Identical TA1.2
                //                                         | function but mapped
                //   AIN2 = J2-9  = pin 12 = P2.4/TA1.2  <-- to two diff pins
                //
                //   BIN1 = J1-9  = pin  9 = P2.1/TA1.1  <-- Identical TA1.1
                //                                         | function but mapped
                //   BIN2 = J2-10 = pin 10 = P2.2/TA1.1  <-- to two diff pins
                //
                // So it flips: one pin of the pair to PWM mode, and the other
                // pin to GPIO mode, and yea verily it works. A little weird
                // but it works.
                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
#define AIN1_DUTY(dvalue)       { TA1CCR2 = dvalue;    }
                              // Turn on P2.5/TA1.2 PWM output, with Active High
#define AIN1_PWM(dutyval)       { TA1CCR2 = dutyval; \
                                  P2SEL  |= AIN1;      }
                              // Convert P2.5 to GPIO only  (no PWM func enable)
#define AIN1_LOW                { P2SEL &= ~AIN1;     \
                                  P2OUT &= ~AIN1;      }
#define AIN1_HIGH               { P2SEL &= ~AIN1;     \
                                  P2OUT |= AIN1;       }

#define AIN2_DUTY(dvalue)       { TA1CCR2 = dvalue;   }
                              // Turn on P2.4/TA1.2 PWM output, with Active High
#define AIN2_PWM(dutyval)       { TA1CCR2 = dutyval; \
                                  P2SEL  |= AIN2;     }
                              // Convert P2.4 to GPIO only  (no PWM func enable)
#define AIN2_LOW                { P2SEL &= ~AIN2;    \
                                  P2OUT &= ~AIN2;     }
#define AIN2_HIGH               { P2SEL &= ~AIN2;    \
                                  P2OUT |= AIN2;      }


#define BIN1_DUTY(dvalue)       { TA1CCR1 = dvalue;    }
                              // Turn on P2.1/TA1.1 PWM output, with Active High
#define BIN1_PWM(dutyval)       { TA1CCR1 = dutyval; \
                                  P2SEL  |= BIN1;      }
                              // Convert P2.1 to GPIO only  (no PWM func enable)
#define BIN1_LOW                { P2SEL &= ~BIN1;     \
                                  P2OUT &= ~BIN1;      }
#define BIN1_HIGH               { P2SEL &= ~BIN1;     \
                                  P2OUT |= BIN1;       }


#define BIN2_DUTY(dvalue)       { TA1CCR1 = dvalue;    }
                              // Turn on P2.2/TA1.1 PWM output, with Active High
#define BIN2_PWM(dutyval)       { TA1CCR1 = dutyval; \
                                  P2SEL  |= BIN2;      }
                              // Convert P2.2 to GPIO only  (no PWM func enable)
#define BIN2_LOW                { P2SEL &= ~BIN2;     \
                                  P2OUT &= ~BIN2;      }
#define BIN2_HIGH               { P2SEL &= ~BIN2;     \
                                  P2OUT |= BIN2;       }

#endif                    // end MSP430_G2553



//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                   MSP430 FR5969     Macros  and  Includes  used
//
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(MSP430_FR5969_LAUNCHPAD)

#define  TI_CCS_COMPILER            1        // Denote are using TI CCS compiler

#define  MCU_SPEED_16MHZ         16000000         // MSP430 FR5969
#define  ADC_RESOLUTION_10_BIT      1             // with 10 bit ADC resolution
#define  MSP430_MCU                 1             // denote is a MSP430 MCU

#include "msp430.h"                               // pull in MCU defs for MSP430

                // Switch Configuration  J1-4  P2.5  GPIO    Grove J1-4 -> J1-3
#define SWITCH1_CONFIG   { P2DIR &= ~(BIT5); }    // DIR: 0 = in,  1 = out
#define SWITCH1_READ     ( P2IN  &= BIT5 )

                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
                //                   Speed  Control  Input   via   ADC
                // Comment this section out if _not_ using a Potentiomerter as a
                // Speed Control. In such case, use XXXX_xxx API to set speed reference
                //
                // The default speed control is assumed to be attached to
                // Launchpad J1-2 connector using pin P6.0 and ADC channel A15
                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
#define SPEED_CONTROL_ADC_CHANNEL   ADC14INCH_15
#define SPEED_CONTROLPIN            BIT0
#define SPEED_CONTROLPIN_MUX_SEL0   P6SEL0
#define SPEED_CONTROLPIN_MUX_SEL1   P6SEL1

#endif                             //  MSP430_FR5969_LAUNCHPAD



//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                      MSP432     Macros  and  Includes  used
//
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(MSP432_LAUNCHPAD)

#define  TI_CCS_COMPILER            1        // Denote are using TI CCS compiler

#define  MCU_SPEED_48MHZ         48000000         // MSP432
#define  ADC_RESOLUTION_10_BIT      1             // with 10 bit ADC resolution
#define  MSP432_MCU                 1             // denote is a MSP432 MCU

#include "msp.h"                                  // pull in MCU defs for MSP432
                // Switch Configuration
#define SWITCH1_CONFIG      { P3DIR &= ~(BIT3); } // P3.3 J1-4  DIR: 0=in, 1=out
#define SWITCH1_READ        ( P3IN  &= BIT3 )

#define CONFIG_HALL_ENCODER_A   { P4DIR &= ~(BIT1); } // P4.1 J1-5  DIR: 0=in, 1=out
#define CONFIG_HALL_ENCODER_B   { P4DIR &= ~(BIT3); } // P4.3 J1-6
#define CONFIG_HALL_ENCODER_C_Z { P4DIR &= ~(BIT6); } // P4.6 J1-8
#define READ_HALL_ENCODER_A     ( P4IN  &= BIT1 )
#define READ_HALL_ENCODER_B     ( P4IN  &= BIT3 )
#define READ_HALL_ENCODER_C_Z   ( P4IN  &= BIT6 )

#define ASSERT_CS()          (P2OUT &= ~BIT5)     // P2.5 is CS
#define DEASSERT_CS()        (P2OUT |= BIT5)


#define FLCTL_BANK0_RDCTL_WAIT__2    (2 << 12)    // TI Clock setup macros
#define FLCTL_BANK1_RDCTL_WAIT__2    (2 << 12)

                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
                //                   Speed  Control  Input   via   ADC
                // Comment this section out if _not_ using a Potentiomerter as a
                // Speed Control. In such case, use XXXX_xxx API to set speed reference
                //
                // The default speed control is assumed to be attached to
                // Launchpad J1-2 connector using pin P6.0 and ADC channel A15
                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
#define SPEED_CONTROL_ADC_CHANNEL   ADC14INCH_15
#define SPEED_CONTROLPIN            BIT0
#define SPEED_CONTROLPIN_MUX_SEL0   P6SEL0
#define SPEED_CONTROLPIN_MUX_SEL1   P6SEL1

                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
                //    PWM   CONTROL   SIGNALS  for   COMMUTATION         MSP432
                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
#define PWM_AH_DUTY(dvalue)     { TA0CCR4  = dvalue;  }
#define PWM_AH_PWM(dutyval)     { TA0CCR4  = dutyval; \
                                  TA0CCTL4 = OUTMOD_7; }
#define PWM_AH_PWM_INV(dutyval) { TA0CCR4  = dutyval; \
                                  TA0CCTL4 = OUTMOD_3; }
#define PWM_AH_LOW              { TA0CCTL4 = OUTMOD_0; }
#define PWM_AH_HIGH             { TA0CCTL4 = OUTMOD_0 | OUT; }  // slow decay mode causes motor to spin at full tilt anyway


#define PWM_AL_DUTY(dvalue)     { TA0CCR3  = dvalue;  }
#define PWM_AL_PWM(dutyval)     { TA0CCR3  = dutyval; \
                                  TA0CCTL3 = OUTMOD_7; }
#define PWM_AL_PWM_INV(dutyval) { TA0CCR3  = dutyval; \
                                  TA0CCTL3 = OUTMOD_3; }
#define PWM_AL_LOW              { TA0CCTL3 = OUTMOD_0; }
#define PWM_AL_HIGH             { TA0CCTL3 = OUTMOD_0 | OUT; }


#define PWM_BH_DUTY(dvalue)     { TA0CCR1  = dvalue;  }
#define PWM_BH_PWM(dutyval)     { TA0CCR1  = dutyval; \
                                  TA0CCTL1 = OUTMOD_7; }
#define PWM_BH_PWM_INV(dutyval) { TA0CCR1  = dutyval; \
                                  TA0CCTL1 = OUTMOD_3; }
#define PWM_BH_LOW              { TA0CCTL1 = OUTMOD_0; }
#define PWM_BH_HIGH             { TA0CCTL1 = OUTMOD_0 | OUT; }  // slow decay mode causes motor to spin at full tilt anyway


#define PWM_BL_DUTY(dvalue)     { TA2CCR1  = dvalue;  }
#define PWM_BL_PWM(dutyval)     { TA2CCR1  = dutyval; \
                                  TA2CCTL1 = OUTMOD_7; }
#define PWM_BL_PWM_INV(dutyval) { TA2CCR1  = dutyval; \
                                  TA2CCTL1 = OUTMOD_3; }
#define PWM_BL_LOW              { TA2CCTL1 = OUTMOD_0; }
#define PWM_BL_HIGH             { TA2CCTL1 = OUTMOD_0 | OUT; }


#define PWM_CH_DUTY(dvalue)     { TA2CCR3  = dvalue;  }
#define PWM_CH_PWM(dutyval)     { TA2CCR3  = dutyval; \
                                  TA2CCTL3 = OUTMOD_7; }
#define PWM_CH_PWM_INV(dutyval) { TA2CCR3  = dutyval; \
                                  TA2CCTL3 = OUTMOD_3; }
#define PWM_CH_LOW              { TA2CCTL3 = OUTMOD_0; }
#define PWM_CH_HIGH             { TA2CCTL3 = OUTMOD_0 | OUT; }


#define PWM_CL_DUTY(dvalue)     { TA2CCR4  = dvalue;  }
#define PWM_CL_PWM(dutyval)     { TA2CCR4  = dutyval; \
                                  TA2CCTL4 = OUTMOD_7; }
#define PWM_CL_PWM_INV(dutyval) { TA2CCR4  = dutyval; \
                                  TA2CCTL4 = OUTMOD_3; }
#define PWM_CL_LOW              { TA2CCTL4 = OUTMOD_0; }
#define PWM_CL_HIGH             { TA2CCTL4 = OUTMOD_0 | OUT; }

#endif                              // defined(MSP432_LAUNCHPAD)



//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                      Tiva 123G     Macros  and  Includes  used
//
// MCU = TM4C123GH6PM
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(TIVA_123G_LAUNCHPAD)

#define  TI_CCS_COMPILER            1        // Denote are using TI CCS compiler

#define  MCU_SPEED_80MHZ         80000000          // Tiva 123G
#define  ADC_RESOLUTION_12_BIT      1              // with 12 bit ADC resolution
#define  TIVA_MCU                   1              // denote is a Tiva MCU

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
//#include "inc/hw_ints.h"
#include "inc/hw_pwm.h"
#include "inc/hw_ssi.h"
#include "inc/hw_timer.h"
#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"

#include "inc/tm4c123gh6pm.h"

                // Switch Configuration PB1 (and float PB0)
#define SWITCH1_CONFIG  { MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_1); \
                          MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_0); }
#define SWITCH1_READ      MAP_GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_1)

            //-----------------------------------------------
            // DRV8711 GPIO Port and Pin Mapping Definitions
            //-----------------------------------------------
#define  nSLEEP     GPIO_PIN_2      // BDC nSleep = PB2
//#define  RESET      GPIO_PIN_5    // Stepper RESET = PA5
#define  STEP_AIN1  GPIO_PIN_6
#define  DIR_AIN2   GPIO_PIN_7
#define  BIN1       GPIO_PIN_4
#define  BIN2       GPIO_PIN_3

#define  nSTALL     GPIO_PIN_2
#define  nFAULT     GPIO_PIN_0

#define  INIT_DRV8711_CS()     MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_2)

#define  ASSERT_DRV8711_CS()   (GPIO_PORTA_DATA_R |= GPIO_PIN_2)
#define  DEASSERT_DRV8711_CS() (GPIO_PORTA_DATA_R &= ~GPIO_PIN_2)

#define  NSLEEP_HI_ENABLE      GPIO_PORTB_DATA_R |= nSLEEP
#define  NSLEEP_LO_DISABLE     GPIO_PORTB_DATA_R &= ~nSLEEP

//#define  RESET_HI_RESET        GPIO_PORTA_DATA_R |= RESET
//#define  RESET_LO_RUN          GPIO_PORTA_DATA_R &= ~RESET

#define  STEP_AIN1_HIGH        GPIO_PORTA_DATA_R |= STEP_AIN1
#define  STEP_AIN1_LOW         GPIO_PORTA_DATA_R &= ~STEP_AIN1

#define  DIR_AIN2_FORWARD      GPIO_PORTA_DATA_R |= DIR_AIN2
#define  DIR_AIN2_BACKWARD     GPIO_PORTA_DATA_R &= ~DIR_AIN2

#define  BIN1_HI               GPIO_PORTA_DATA_R |= BIN1
#define  BIN1_LO               GPIO_PORTA_DATA_R &= ~BIN1

#define  BIN2_HI               GPIO_PORTA_DATA_R |= BIN2
#define  BIN2_LO               GPIO_PORTA_DATA_R &= ~BIN2

#endif                                 // defined(TIVA_123G_LAUNCHPAD)


#endif                                 // _MOTOR_LL_BDC_CONFIG_H_

//******************************************************************************
