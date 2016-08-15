
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                            MotorLib_LL_SN754410_Tiva.c
//
// Motor control code that is specific to the SN754410 and L293D
// Dual H-Bridge controllers.
//
// The L293D has an identical interface to the SN754410, so we use the same
// code for both.
//
// Controls 2 Brushed DC motors (Dagu) with a TI SN754410 H-Bridge.
// Note: we only use "normal" dual drive mode, not "parallel" drive mode.
//
// 6 Volts is supplied to the DC motors via Boosterpack's Vm pin.
//
// A separate PWM is supplied for each of the motor PWM inputs: ENA1, ENA2 for
// motor 1, and ENA3, ENA4 for motor 2.
// 
// Motor 1 is driven by ENA1 and ENA2, which supplies motor outputs via 1Y and 2Y.
// Inputs 1A and 2A are used to set the Forward and Reverse (CW, CCW) direction
// of Motor 1.
// Motor 2 is driven by ENA3 and ENA4, which supplies motor outputs via 3Y and 4Y.
// Inputs 3A and 4A are used to set the Forward and Reverse (CW, CCW) direction
// of Motor 1.
//
//
// SN7544120 Pin Usage           LP Conn  Tiva          MSP432
// -------------------           -------  -----         ------
//   8   Vcc2   Motor 6 V supply     -      -            -
//  16   Vcc1   Logic +5            J3-1   5v           5v
//  4/5/12/13   Ground              J3-2   Gnd          Gnd
//
//   1   Ena12  Motor 1  PWM        J4-4   M0PWM6 PC4   TA2.1  P5.6
//   9   Ena34  Motor 2  PWM        J4-5   M0PWM7 PC5   TA2.3  P6.6
//   2   1A     Motor 1 Left/CCW    J4-6   GPIO         GPIO   P6.7
//   7   2A     Motor 1 Right/CW    J4-7   GPIO         GPIO   P2.3
//  10   3A     Motor 2 Left/CCW    J4-8   GPIO         GPIO   P5.1
//  15   4A     Motor 2 Right/CW    J4-9   GPIO         GPIO   P3.5
//
//   3   1Y     Motor 1 OUTPUT Left   -
//   6   2Y     Motor 1 OUTPUT Right  -
//  11   3Y     Motor 2 OUTPUT Left   -
//  14   4Y     Motor 2 OUTPUT Right  -
//                                                MSP432
//        -     Speed Ctl Pot ADC   J1-2          A15  P6.0  Grove J1-2->J3-7/27
//        -     Fwd/Reverse Slider  J1-4          GPIO P3.3  Grove J1-4->J1-3
//
//   -    -     Hall Sensor Left    J1-6   GPIO rupt    GPIO   P4.3
//   -    -     Hall Sensor Right   J1-8   GPIO rupt    GPIO   P4.5
//                   White lead = Signal,  Red lead = +3.3   Black lead = Gnd
//
// Dagu BDC Motor:  4.5 - 6.0 Volts     (absolute max = 8.4 v)
//                  No Load:  150ma     Stall Current: 2.75 A at 6V
//                  Measured Motor Resistance (Rload):  5.7 ohms
//
// Hall Sensors:    3.0 - 24.0 Volts    (Open drain)
// Encoder Disk:    8 pole neodymium magnet
//                  625 state changes per wheel revolution
//
// 5V from Launchpad 5V (via USB)  ==> must be tethered to USB cable
// 5V from LM7805 regulator wired to battery pack
//
// 3.3V from Launchpad (via USB)   ==> must be tethered to USB cable
// 3.3V from LM1086-3.3 regulator wired to battery pack
//
// History:
//   04/01/16 - Created as part of Ground 0 Motor Skill set. High level sketch.
//   04/02/16 - Created and tested all base routines (ADC, PWM, GPIO In/Out,
//              SysTick) except Hall encoder pins. Also did motor start/stop/
//          set_direction/go_left/go_right, process_adc_speed_control. Duquaine
//   04/05/16 - Rolled in support for Hall Sensors (attached to wheels).Duquaine
//   04/08/16 - Made ADC and Hall sensor pins in common with DRV8848 project, 
//           and split out motor controller specific (SN754410 vs DRV8848) code. Duquaine
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

#include "BDC_motor_ctl.h"                       // pull in common definitions

           //-----------------------------------------------------
           // Config definitions for Motor hookups on this board
           //-----------------------------------------------------
                              // Setup Control Output GPIOs on J4-6 -> J4-9
                              //      CCW = counter-clockwise (spins left)
                              //      CW  = clockwise         (spins right)
#define A1_CONFIG        { P6DIR |= BIT7; }            // DIR: 1 = out,  0 = in
#define A1_LOW           (P6OUT &= ~BIT7)
#define A1_HIGH_CCW_M1   (P6OUT |= BIT7)

#define A2_CONFIG        { P2DIR |= BIT3; }
#define A2_LOW           (P2OUT &= ~BIT3)
#define A2_HIGH_CW_M1    (P2OUT |= BIT3)

#define A3_CONFIG        { P5DIR |= BIT1; }
#define A3_LOW           (P5OUT &= ~BIT1)
#define A3_HIGH_CCW_M2   (P5OUT |= BIT1)

#define A4_CONFIG        { P3DIR |= BIT5; }
#define A4_LOW           (P3OUT &= ~BIT5)
#define A4_HIGH_CW_M2    (P3OUT |= BIT5)

#define SPEED_MIN           20
#define SPEED_MAX          800
#define SPEED_TIME           5     // slows down speed up/down rate.
                                   // Increase value to slow speed up/down rate

           //--------------------------------------
           //  Device Handler function prototypes
           //--------------------------------------
void  sn754410_motor_brake (MOTOR_BLOCK  *mtr_blk);
void  sn754410_motor_encoder_gpio_init(MOTOR_BLOCK *mtr_blk, int encoder_type);
void  sn754410_motor_speed_adc_init (void);
void  sn754410_motor_adc_ctl_init (void);
void  sn754410_motor_init_ctl_gpio_outputs (void);
void  sn754410_motor_init_ctl_gpio_inputs (void);

void  sn754410_motor_set_direction (MOTOR_BLOCK *mtr_blk, int mtr_direction);
void  sn754410_motor_set_duty_cycle (MOTOR_BLOCK *mtr_blk, long actual_duty_count);
void  sn754410_motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count);
void  sn754410_motor_start (MOTOR_BLOCK *mtr_blk);
void  sn754410_motor_stop (MOTOR_BLOCK *mtr_blk);
void  sn754410_motor_pwm_init (MOTOR_BLOCK *mtr_blk, int pwm_speed);

void  sn754410_motor_gpio_init_inputs (void);
void  sn754410_motor_gpio_init_outputs (void);
void  sn754410_motor_adc_next_conversion (void);


           //-----------------------------------------------------------
           //                  Device Function Table
           //
           //  Defines low-level command handlers that are invoked by
           //  BDC_motor_ctl_lib.c to perform device specific functions
           //-----------------------------------------------------------
  MOTOR_HANDLERS  sn754410_table = {
                                    sn754410_motor_brake,
                                    sn754410_motor_encoder_gpio_init,// init routine for encoder GPIOs
                                    sn754410_motor_speed_adc_init,   // init routine for speed ctl ADC
                                    sn754410_motor_adc_ctl_init,     // control ADCs (Current sense)
                                    sn754410_motor_init_ctl_gpio_inputs,  // control input  GPIOs (FAULT)
                                    sn754410_motor_init_ctl_gpio_outputs, // control output GPIOs (ENABLE)
                                    sn754410_motor_pwm_init,              // motor PWMs
                                    sn754410_motor_set_direction,
                                    sn754410_motor_set_pwm_duty_count,
                                    sn754410_motor_start,
                                    sn754410_motor_stop,
                                    sn754410_motor_adc_next_conversion, // ADC conversion start rtn for speed
                                  };


//******************************************************************************
//  sn754410_motor_brake
//
//               Stop the motor using a "brake" sequence: i.e. set both 1A/2A
//               or 3A/4A to both HIGH or both LOW.
//
//               Setting 1A/2A or 3A/4A both HIGH forces any re-circulating 
//               motor current to ground, and stops the motor faster.
//
//               Setting 1A/2A or 3A/4A both LOW forces any re-circulating 
//               motor current to keep looping through the windings until they
//               dissipate, and lets the motor coast to a stop.
//******************************************************************************
void  sn754410_motor_brake (MOTOR_BLOCK  *mtr_blk)
{
    if (mtr_blk->mtr_num == 1)
       {         // brake Motor 1
         A1_LOW;
         A2_LOW;
       }
      else
       {         // brake Motor 2 
         A3_LOW;
         A4_LOW;
       }
}


//******************************************************************************
//  sn754410_motor_set_direction
//
//      Update Motor 1A/2A/3A/4A settings to set desired CW/CCW direction.
//******************************************************************************
void  sn754410_motor_set_direction (MOTOR_BLOCK *mtr_blk,  int mtr_direction)
{
    int  pwm_duty_count;

    if (mtr_blk->mtr_state != STATE_RUN)
       return;                               // do not apply until motor start

    pwm_duty_count = mtr_blk->mtr_pwm_duty_count;

         //-----------------------------------------------------------------
         // CAUTION: it is importamt to NEVER have _BOTH_ A1 and A2 HIGH
         //          at the same time, and NEVER have _BOTH_ A3 and A4 HIGH
         //          at the same time, because it shorts both ends of
         //          the motor through to the power supply, which can
         //          cause major grief (overheating) with some motors.
         //-----------------------------------------------------------------

    if (mtr_blk->mtr_num == 1)
       {                // change the direction settings for Motor 1
         if (mtr_direction == DIRECTION_FORWARD)   // forward = CW
            {
              A2_HIGH_CW_M1;
              A1_LOW;                              // Motor 1
            }
           else                                    // backward = CCW
            {
              A1_HIGH_CCW_M1;                      // Motor 1
              A2_LOW;
            }
         TA2CCR1 = pwm_duty_count;             // and set CCR1 duty cycle TA2.1
       }                                       //  that drives PWM on ENA1/2

      else
       {                // change the direction settings for Motor 2
         if (mtr_direction == DIRECTION_FORWARD)   // forward = CW
            {
              A3_LOW;                              // Motor 2
              A4_HIGH_CW_M2;
            }
           else                                    // backward = CCW
            {
              A4_LOW;
              A3_HIGH_CCW_M2;                      // Motor 2
            }
         TA2CCR3 = pwm_duty_count;             // and set CCR3 duty cycle TA2.3
       }                                       //  that drives PWM on ENA3/4
}


//******************************************************************************
//  sn754410_motor_set_pwm_duty_count
//              Set the duty cycle to be used for a specific motor.
//              Duty cycle is explicitly passed as PWM ticks, not as percent.
//              The duty cycle sets the motor's speed. (More juice, more speed)
//
//   Parms:
//      motor_num:   number of the motor to update (1 or 2)
//      pwm_duty:    actual duty cycle PWM count value to be loaded in PWM
//******************************************************************************
void  sn754410_motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count)
{
     if (mtr_blk->mtr_state != STATE_RUN)
        return;                                // do not apply till motor start

    pwm_duty_count = mtr_blk->mtr_pwm_duty_count;

    if (mtr_blk->mtr_num == 1)
       {      // Motor 1 CCR parameters
         TA2CCR1 = pwm_duty_count;             // set CCR1 duty cycle   TA2.1
                                               //  that drives PWM on ENA1/2
       }
      else
       {      // Motor 2 CCR parameters
         TA2CCR3 = pwm_duty_count;             // set CCR3 duty cycle   TA2.3
                                               //  that drives PWM on ENA3/4
       }
}


//******************************************************************************
//  sn754410_motor_start
//               Start or re-start the motor after a direction change or a stop.
//           Because a stop will slam off the PWMs, we re-establish Dir and Duty
//******************************************************************************
void  sn754410_motor_start (MOTOR_BLOCK *mtr_blk)
{
       // Invoke set_direction, which will configure PWMs and load duty values.
    sn754410_motor_set_direction (mtr_blk, mtr_blk->mtr_direction);

       // Now turn on PWMs, which will cause the motor to start.
    if (mtr_blk->mtr_num == 1)
       {      // start Motor 1   (enable it's PWM CCR on 1/2ENA)
         TA2CCTL1 = OUTMOD_7;     // restart PWM - CCR1 will use reset/set mode
       }
      else
       {      // start Motor 2   (enable it's PWM CCR on 3/4ENA)
         TA2CCTL3 = OUTMOD_7;     // restart PWM - CCR3 will use reset/set mode
       }
}


//******************************************************************************
//  sn754410_motor_stop
//
//               Stop the motor, using either Fast or Slow decay.
//               Also shuts off the PWMs, by locking them to HIGH or LOW state.
//******************************************************************************
void  sn754410_motor_stop (MOTOR_BLOCK *mtr_blk)
{
    if (mtr_blk->mtr_num == 1)
       {      // stop Motor 1   (disable it's PWM CCR on 1/2ENA)
         TA2CCTL1 = OUTMOD_0;      // shutoff PWM - CCR1
       }
      else
       {      // stop Motor 2   (disable it's PWM CCR on 3/4ENA)
         TA2CCTL3 = OUTMOD_0;      // shutoff PWM - CCR1
       }
}


//******************************************************************************
//  sn754410_motor_pwm_init
//
//         Configure PWMs to driver motor 1/2ENA and 3/4ENA.
//         These use a variable duty cycle to control motor speed.
//
//         Generates a PWM with a frequency of 20,000 Hz (50 usec).
//******************************************************************************
void  sn754410_motor_pwm_init (MOTOR_BLOCK *mtr_blk, int period_ticks)
{
      //------------------------------------------------------------------------
      // configure GPIO pins - P5.6 and P6.6 will be PWM outputs (TA2.1 / TA2.3)
      //------------------------------------------------------------------------
    if (mtr_blk->mtr_num == 1)
       {
         P5DIR    |= BIT6;    // make P5.6 as output
         P5SEL0   |= BIT6;
         P5SEL1   &= BIT6;    // configure P5.6 as Timer A2.1 function
       }

    if (mtr_blk->mtr_num == 2)
       {
         P6DIR    |= BIT6;    // make P6.6 as output
         P6SEL0   |= BIT6;
         P6SEL1   &= BIT6;    // configure P6.6 as Timer A2.3 function
       }

       //--------------------------------------------------
       // Now configure Timer A2 as a PWM on CCR1 and CCR3
       //--------------------------------------------------
    TA2CCR0  = period_ticks;  // CCR0 period: Squarewave 4 * period * 8

    if (mtr_blk->mtr_num == 1)
       {                      // Motor 1 CCR parameters
         TA2CCR1  = 0;        // CCR1 duty cycle set to off       TA2.1
         TA2CCTL1 = OUTMOD_7; // CCR1 will use reset/set mode
       }
    if (mtr_blk->mtr_num == 2)
       {                      // Motor 2 CCR parameters
         TA2CCR3  = 0;        // CCR3 duty cycle set to off       TA2.3
         TA2CCTL3 = OUTMOD_7; // CCR3 will use reset/set mode
       }                      // Motor 2 CCR parameters

    TA2CTL   = TASSEL__SMCLK | MC__UP | TACLR;  // SMCLK, up mode, clear TAR
                              //    bit  mode
                              //    9-8  20    TASSEL, SMCLK=12MHz
                              //    7-6  00    ID, divide by 1
                              //    5-4  11    MC, up-down mode
                              //    2    0     TACLR, no clear
                              //    1    0     TAIE, no interrupt
                              //    0          TAIFG
}


//******************************************************************************
//  sn754410_motor_gpio_init_inputs
//
//         Configure control inputs for Motor GPIOs  - FAULT, ...
//******************************************************************************

void  sn754410_motor_init_ctl_gpio_inputs (void)
{
      // nothing to be done
}


//******************************************************************************
//  sn754410_motor_init_ctl_gpio_outputs
//
//         Configure control outputs for Motor GPUIOs - ENABLE, ...
//******************************************************************************
void  sn754410_motor_init_ctl_gpio_outputs (void)
{
        // These are the 1A thru 4A pins on SN754410 for CW / CCW control
    A1_CONFIG;
    A2_CONFIG;
    A3_CONFIG;
    A4_CONFIG;

#if defined(TEST)
// Test the above to ensure are working         --    WORKS  04/02/16
    A1_HIGH_CCW_M1;
    A2_HIGH_CW_M1;
    A3_HIGH_CCW_M2;
    A4_HIGH_CW_M2;
    A1_LOW;
    A2_LOW;
    A3_LOW;
    A4_LOW;
#endif

          // Rest of motor GPIO output configuration is done in motor_pwm_init
}


//******************************************************************************
//  sn754410_motor_encoder_gpio_init                                      ENCODER
//
//         Configure ADC to read contropl inputrs for current sensors, ...
//******************************************************************************
void  sn754410_motor_encoder_gpio_init (MOTOR_BLOCK *mtr_blk, int encoder_type)
{
    if (mtr_blk->mtr_num == 1)
       {
       }
      else if (mtr_blk->mtr_num == 2)
       {
       }

       //------------------------------------------------------------------
       //                     WHEEL  ENCODER  CONFIG
       // Configure P4.3 and P4.6 (encoders) as open collector inputs with
       // pull-up resistors.
       //------------------------------------------------------------------
    P4DIR  &= ~(uint8_t) (BIT3 | BIT6);   // conf P4.3/P4.6 as input  DIR: 0=in
//  P4REN  |= (BIT3 | BIT6);              // Enable pull-up resistors - uses 10 mA. Must use External pullups
    P4SEL0 &= ~(uint8_t) (BIT3 | BIT6);   // Force to GPIO mode
    P4SEL1 &= ~(uint8_t) (BIT3 | BIT6);
    P4IFG  &= ~(uint8_t) (BIT3 | BIT6);   // Clear assoc Port 4 interrupt flags
    P4IE   |= (BIT3 | BIT6);              // Enable interrupts for P4.3 and P4.6
    P4IES  |= (BIT3 | BIT6);              // Interrupt on high-to-low transition

       // Enable Port 4 interrupts on the NVIC
    NVIC_ISER1 = 1 << ((INT_PORT4 - 16) & 31);
}


//******************************************************************************
//  sn754410_motor_adc_ctl_init
//
//         Configure ADC to read contropl inputrs for current sensors, ...
//******************************************************************************
void  sn754410_motor_adc_ctl_init (void)
{
      // no ADC current sense, etc on sn754410 
}


//******************************************************************************
//  sn754410_motor_speed_adc_init                                          SPEED
//
//         Configure ADC to read potentiometer used for speed control setting.
//         Uses A15  on pin  P6.0
//******************************************************************************
void  sn754410_motor_speed_adc_init (void)
{
       //---------------------------------------------------------
       //                Configure basic ADC
       // Configure ADC14 - Sampling time, S&H=16, ADC14 on
       //---------------------------------------------------------
    ADC14CTL0 = ADC14SHT0_2 | ADC14SHP | ADC14ON;
       // Use sampling timer, 10-bit conversion results
    ADC14CTL1 = ADC14RES_1;            // 3 = 14 bit,  2 = 12-bit,  1 = 10 bit
                                       //     16384        4096         1024

          //---------------------------------------------------------
          // Configure Speed Control Potentiometer ADC input channel.
          //       Speed Ctl Pot ADC        J1-2     P6.0   A15
          //---------------------------------------------------------
    P6SEL1 |= BIT0;                    // Configure P6.0 for ADC
    P6SEL0 |= BIT0;
    ADC14MCTL0 |= ADC14INCH_15;        // use A15 ADC input channel;  Vref=+AVcc

          //------------------------------------------------------------
          // Configure Vref Current Ctl Potentiometer ADC input channel.
          //       VRef  Current Ctl ADC    J1-5     P4.1   A12
          //------------------------------------------------------------
    P4SEL0 |= BIT1;                    // Configure P4.1 for ADC
    P4SEL1 |= BIT1;
    ADC14MCTL1 |= ADC14INCH_12 | ADC14EOS;  // use A12 ADC input channel with
                                       // End-of-Sequence to denote last channel

          // Turn on multi-channel auto-scan for 2 channels of conversion
    ADC14CTL0 = ADC14CONSEQ_1 | ADC14SHT0_2 | ADC14SHP | ADC14ON;

    ADC14IER0  |= ADC14IE0;            // Enable ADC convert complete interrupt

           // Enable ADC interrupt in NVIC module
    NVIC_ISER0 = 1 << ((INT_ADC14 - 16) & 31);
}


//******************************************************************************
//  sn754410_motor_adc_next_conversion
//
//         start a new ADC conversion for Speed control  (called every 50 ms)
//         Uses A15  on pin  P6.0
//******************************************************************************
void  sn754410_motor_adc_next_conversion (void)
{
    ADC14CTL0 |= ADC14ENC | ADC14SC;    // Start a new sampling/conversion
}

//*****************************************************************************
