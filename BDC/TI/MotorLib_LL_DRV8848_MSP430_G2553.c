
// Bringup hacks


  unsigned int G_PWM_PERIOD = 99; // set Timer to run at 20 KHz

      // Note: for slow decay mode, usually need duty = 60 % before motor moves !
      //       for fast decay mode, usually need duty = 40 %    "     "     "
  unsigned int G_PWM_A   = 50;    // PWM Speed control motor A (50 % duty)
  unsigned int G_PWM_B   = 50;    // PWM Speed control motor B (50 % duty)

  unsigned int G_VREF = 0;        // VREF voltage from potentiometer (0-1023)

// copied from Ref Design but not used by this code
  unsigned int G_DECAY_A = 1; // true;       // Decay mode for motor A (fast = false, slow = true)
  unsigned int G_DECAY_B = 1; // true;       // Decay mode for motor B
  unsigned int G_DIR_A   = 0; // false;      // Direction for motor A  (default = forward)
  unsigned int G_DIR_B   = 0; // false;      // Direction for motor B
  unsigned int G_MODE    = 0; // false;      // Motor Mode: 0 = Normal (Dual), 1 = Parallel
  unsigned int G_MODE_TEMP = 0; // false;    // Memory value for G_MODE

  unsigned int G_nSLEEP  = 0;          // DRV8848 SLEEP pin: 0 = SLEEP  ACTIVE_LOW
  unsigned int G_nFAULT  = 0;          // DRV8848 FAULT pin: 0 = FAULT  ACTIVE_LOW


      // GPIO Port 1 Definitions
#define VREF    BIT3    // P1.3


//*******1*********2*********3*********4*********5*********6*********7**********
//
//                     MotorLib_LL_DRV8848_MSP430_G2553.c
//
// Motor control code that is specific to the DRV8848 Dual H-Bridge controller,
// used on the TI DRV8848 Boosterpack board.
//
// Controls 2 Brushed DC motors (Dagu) with a TI DRV8848 H-Bridge.
// Note: we only use "normal" dual drive mode, not "parallel" drive mode.
//
// 6 Volts is supplied to the DC motors via Boosterpack's Vm pin.
//
// Each motor is driven by a pair of PWMs. One PWM is used to drive the speed
// in a specific direction. The other direction's PWM is then locked in either
// a high or low position to control braking.
//
// Motor 1 is driven by AIN1 and AIN2, which supplies motors via AOUT1 / AOUT2.
// Motor 2 is driven by BIN1 and BIN2, which supplies motors via BOUT1 / BOUT2.
//
//
// DRV8848 pinout does NOT match any PWMs on MSP432, so we have to
// to the associated AINx/BINx J1/J2 pins to inputs, and then jumper
// a real PWM to the AINx/BINx input.
//
// drv8848 Xnucleo Usage
// -------------------
//       Vm     Motor 6 V supply     -
//       Gnd    Battery Ground       -
//
//       AIN1  1A   Motor 1  TIM3_CH1   PB4   Arduino D5        TIM3_CH1
//       AIN2  2A   Motor 2  TIM3_CH2   PB5   Arduino D4        TIM3_CH2
//
//       BIN1  1B   Motor 1  TIM2_CH1   PA0   Arduino A0        TIM2_CH1
//       BIN2  2B   Motor 2  TIM2_CH2   PA1   Arduino A1        TIM2_CH2
//
//  A ENABLE / OCD  Motor 1             PA10  Arduino D2
//  B ENABLE / OCD  Motor 2             PC1   Arduino A4
//
//       AOUT1  Motor 1 OUTPUT Left   -
//       AOUT2  Motor 1 OUTPUT Right  -
//       BOUT1  Motor 2 OUTPUT Left   -
//       BOUT2  Motor 2 OUTPUT Right  -
//
//       "A" side Current-Sense         PA6  Arduino D12   ADC12_IN6
//       "B" side Current-Sense         PA4  Arduino A2    ADC12_IN4
//
//        -     Speed Ctl Pot ADC       PB5  Arduino A2    ADC12_IN4   Grove A3
//        -     Fwd/Reverse Slider      PB3  Arduino D3    GPIO rupt   Grove D3
//
//        -     Hall Sensor Left        PA8  Arduino D7    GPIO rupt   Grove D7
//        -     Hall Sensor Right       PA9  Arduino D8    GPIO rupt   Grove D8
//                   White lead = Signal,  Red lead = +3.3   Black lead = Gnd
//
//  Dagu BDC Motor:  4.5 - 6.0 Volts     (absolute max = 8.4 v)
//                   No Load:  150ma     Stall Current: 2.75 A at 6V
//                   Measured Motor Resistance (Rload):  5.7 ohms
//
//  DRV8848:         Dual Brushed DC (BDC) motor drive stage w/integrated FETs
//                   4.0 – 18.0 Volts input
//                   1.0 A RMS,   2.0 A Peak   _per_ H-Bridge
//                   Overcurrent, short-circuit, over-temperature, and
//                   undervoltage reporting is done through the nFAULT pin.
//
// History:
//   05/13/16 - Created as part of Open Source Motor Library support. Duquaine
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

#include "MotorLib_Api.h"                          // pull in common definitions
#include "MotorLib_LL_Config_Pin_Periph_TI_BDC.h"

extern uint16_t  ADC0_Seq0_Conversion_Values[];    // ADC conversion results

    int    g_adc_speed_raw_value = 0;              // speed value from ADC input
                                                   // ??? !!! NOT CURRENTLY USED

    uint32_t  _g_PWM_IO_clock = 0; // Global to hold PWM Max Clock Frequency
    uint32_t  _g_pwm_mdivider = 1; // PWM Master clock : CPU divider ratio
    uint32_t  _g_PWMperiod    = 0; // Global to hold PWM period  (MaxClock/Freq)
    uint32_t  _g_PWMduty      = 0; // Global to hold latest PWM duty cycle
    int       _g_LPWM_rc       = 0;
    int       _g_adc_enabled  = 0;
    int       _g_adc_num_channels = 0;



           //--------------------------------------
           //  Device Handler function prototypes
           //--------------------------------------
int  drv8848_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr);
int  drv8848_motor_adc_ctl_init (void);
int  drv8848_motor_controller_chip_init (void);
int  drv8848_motor_encoder_gpio_init(MOTOR_BLOCK *mtr_blk, int encoder_type);
int  drv8848_motor_init_ctl_gpio_outputs (void);
int  drv8848_motor_init_ctl_gpio_inputs (void);
int  drv8848_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long pwm_ticks);

int  drv8848_motor_run (MOTOR_BLOCK *mtr_blk, int direction);
int  drv8848_motor_set_direction (MOTOR_BLOCK *mtr_blk, int mtr_direction);
int  drv8848_motor_set_duty_cycle (MOTOR_BLOCK *mtr_blk, long actual_duty_count);
int  drv8848_motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count);
int  drv8848_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags);

int  drv8848_motor_gpio_init_inputs (void);
int  drv8848_motor_gpio_init_outputs (void);
int  drv8848_motor_adc_next_conversion (void);


           //------------------------------------------------------------
           //------------------------------------------------------------
           //                Chip Controller Function Table
           //
           //  Defines low-level command handlers that are invoked by
           //  BDC_motor_ctl_lib.c to perform device specific functions
           //------------------------------------------------------------
           //------------------------------------------------------------
 MOTOR_HANDLERS  drv8848_table = {
                     TYPE_MOTOR_BDC,               // motor_type - Motor Type
                     MOTOR_DRIVER,                 // motor_chip - Chip Controller id

                     0L,                           // motor_spi_init - SPI init
                     0L,                           // motor_spi_io - SPI I/O
                     drv8848_motor_speed_adc_init, // motor_adc_speed_ctl_init - init routine for speed ctl ADC
                     drv8848_motor_adc_next_conversion, // motor_adc_speed_ctl_next_conversion - ADC conversion start rtn for speed
                     0L,                           // motor_read_hall_sensors_handler
                     0L,                           // motor_fault_shutdown
                     drv8848_motor_adc_ctl_init,         // motor_init_adc_ctl_inputs - init control ADCs (Current sense)
                     drv8848_motor_controller_chip_init, // motor_init_controller_chip
                     0L,                                 // motor_init_commute_time
                     drv8848_motor_encoder_gpio_init,    // motor_init_encoder_gpio - init routine for encoder GPIOs
                     drv8848_motor_init_ctl_gpio_inputs, // init control input  GPIOs (FAULT)
                     drv8848_motor_init_ctl_gpio_outputs,// init control output GPIOs (ENABLE)
                     drv8848_motor_pwm_init,             // init motor PWMs

                     0L,                           // motor_check_status
                     0L,                           // motor_do_action_handler
                     0L,                           // motor_do_next_commutate
                     0L,                           // motor_get_actual_position
                     0L,                           // motor_get_mark_pos
                     0L,                           // motor_get_motor_status
                     0L,                           // motor_get_period_ticks
                     0L,                           // motor_perform_move
                     drv8848_motor_run,            // motor_run_handler
                     drv8848_motor_set_direction,  // motor_set_direction_handler
                     drv8848_motor_set_pwm_duty_count, // motor_set_pwm_duty_count_handler
                     0L,                           // motor_set_speed_rpm_handler
                     0L,                           // motor_set_step_mode
                     0L,                           // motor_set_torque
                     0L,                           // motor_start_handler (BLDC)
                     drv8848_motor_stop,           // motor_stop_handler
                     0L,                           // motor_wait_complete

                     0L,                           // motor_current_reference_start_stop
                     0L,                           // motor_current_reference_set_value
                     0L,                           // motor_start_pwm_generation (BLDC)
                     0L,                           // motor_stop_pwm_generation  (BLDC)
                     0L,                           // motor_set_bldc_phase_pwm_duty_cycles (BLDC)
                     0L,                           // motor_set_bldc_pwm_enables (BLDC)
                     0L,                           // motor_get_speed_pot_value
                     0L,                           // motor_get_vbus_voltage
                   };


//******************************************************************************
//  drv8848_motor_run
//
//               Run the motor, i.e. issue a start (or re-start after a
//                              direction change or after a stop).
//
//               Because a stop will slam off the PWMs, we re-establish both the
//               Direction and the Duty Cycle.
//******************************************************************************
int  drv8848_motor_run (MOTOR_BLOCK *mtr_blk, int direction)
{
    mtr_blk->mtr_state = MOTOR_STATE_RUN;     // denote we are now in RUN mode

       // Invoke set_direction, which will configure PWMs and load duty values,
       // which will cause the motor to start.
    drv8848_motor_set_direction (mtr_blk, direction);   // 09/03/16 - this is setting direction twice, because prev caller did it too

    return (0);           // denote was successful
}


//******************************************************************************
//  drv8848_motor_set_direction
//
//      Update Motor 1A/2A/3A/4A settings to set desired CW/CCW direction.
//
//      Update the inputs to the DRV8848 based on how the motor is being driven.
//      They are Dependent on direction of motor and decay mode (from GUI vars)
//      Refer to the DRV8848 datasheet for the input truth table.
//      When PWMing it, it actually fluctuates between reverse/forward and decay
//
//             xIN1   xIN2   Xout1  Xout2   Function
//              ----   ----   -----  -----   --------
//               0      0      Z      Z      Coast (Fast Decay)
//               0      1      L      H      Reverse
//              - - - - -     - - - - -
//               1      0      H      L      Forward
//               1      1      L      L      Brake (Slow Decay) - drains to gnd
//
//      Note that when running Slow Decay, one of the xIN bits is always
//      locked HIGH, so we need to invert the duty cycle, because the PWM
//      varies between locked HIGH (Solow Decay) and toggled LOW (PWM 0).
//      Such a config is equivalent to inverting the PWM output pin, so we
//      need to invert the duty cycle count.
//******************************************************************************
int  drv8848_motor_set_direction (MOTOR_BLOCK *mtr_blk,  int direction)
{
    int  pwm_duty_count;

    mtr_blk->mtr_direction = direction;     // ensure desired direction is saved

    if (mtr_blk->mtr_state != MOTOR_STATE_RUN)
        return (0);                           // do not apply until motor start

// STM32 - ??? why doesn't this require INVERT even on FAST ? Or did we just not pass thru here on FAST ?
    if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
       {      // mtr_decay_mode == DECAY_SLOW, _must INVERT_ the PWM duty value
         pwm_duty_count = (mtr_blk->mtr_pwm_period_ticks - mtr_blk->mtr_pwm_duty_count);   // PWM COUNT IS GETTING FUed in HERE  09/03/16
       }                                                                                   // period_ticks = 9215 (TILT) and duty_count = 7371
      else    // mtr_decay_mode == DECAY_FAST, so use normal count
           pwm_duty_count = mtr_blk->mtr_pwm_duty_count;

    if (mtr_blk->mtr_id == 0)
      {                 //----------------------------------------------------
                        //----------------------------------------------------
                        //  change the direction settings for Motor 1     AINx
                        //----------------------------------------------------
                        //----------------------------------------------------
        if (mtr_blk->mtr_direction == DIRECTION_FORWARD)   // forward = CW
           {
                  // Note that we must load the duty cycle _before_
                  // changing TAxCCTLx mode to PWM, to minimize glitches
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {              // Slow Decay
                  AIN1_HIGH;   // run AIN1 as a GPIO = High (lock it as Forward)
                  AIN2_PWM (pwm_duty_count);      // run AIN2 as the PWM
                               //    Alternates between Forward/Brake
                }
               else            // Fast Decay
                {
                  AIN2_LOW;     // run AIN2 as a GPIO = Low (lock it as Coast)
                  AIN1_PWM (pwm_duty_count);      // run AIN1 as the PWM
                                //    Alternates between Coast/Forward
                }
           }
          else                                  // backward/reverse = CCW
           {
             mtr_blk->mtr_direction = DIRECTION_BACKWARD;
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {              // Slow Decay
                  AIN2_HIGH;   // run AIN2 as a GPIO = High (lock it as Brake)
                  AIN1_PWM (pwm_duty_count);      // run AIN1 as the PWM
                               //    Alternates between Brake/Reverse
                }
               else            // Fast Decay
                {
                  AIN1_LOW;    // run AIN1 as a GPIO = Low (lock it as Reverse)
                  AIN2_PWM (pwm_duty_count);      // run AIN2 as the PWM
                               //    Alternates between Reverse/Coast
                }
           }
      }
     else  // mtr_blk->mtr_id == 1
      {                   //--------------------------------------------------
                          //--------------------------------------------------
                          // change the direction settings for Motor 2    BINx
                          //--------------------------------------------------
                          //--------------------------------------------------
        if (mtr_blk->mtr_direction == DIRECTION_FORWARD)   // forward = CW
           {
                  // Note that we must load the duty cycle _before_
                  // changing TAxCCTLx mode to PWM, to minimize glitches
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {              // Slow Decay
                  BIN1_HIGH;   // run BIN1 as a GPIO = High (lock it as Forward)
                  BIN2_PWM (pwm_duty_count);     // run BIN2 as the PWM
                               //    Alternates between Forward/Brake
                }
               else            // Fast Decay
                {
                  BIN2_LOW;     // run BIN2 as a GPIO = Low (lock it as Coast)
                  BIN1_PWM (pwm_duty_count);     // run BIN1 as the PWM
                                //    Alternates between Coast/Forward
                }
           }
          else                                  // backward/reverse = CCW
           {
             mtr_blk->mtr_direction = DIRECTION_BACKWARD;
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {              // Slow Decay
                  BIN2_HIGH;   // run BIN2 as a GPIO = High (lock it as Brake)
                   BIN1_PWM (pwm_duty_count);    // run BIN1 as the PWM
                               //    Alternates between Brake/Reverse
                }
               else            // Fast Decay
                {
                  BIN1_LOW;    // run BIN1 as a GPIO = Low (lock it as Reverse)
                  BIN2_PWM (pwm_duty_count);    // run BIN2 as the PWM
                               //    Alternates between Reverse/Coast
                }
           }
      }

    return (0);           // denote was successful
}


//******************************************************************************
//  drv8848_motor_set_pwm_duty_count
//
//            Set the duty cycle to be used for a specific motor.
//            Here, duty cycle is explicitly passed as PWM ticks, not as percent.
//            The duty cycle sets the motor's speed. (More juice, more speed)
//
//   Parms:
//      motor_num:   number of the motor to update (1 or 2)
//      pwm_duty:    actual duty cycle PWM count value to be loaded in PWM
//******************************************************************************
int  drv8848_motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count)
{
     int  adjusted_duty_count;

     mtr_blk->mtr_pwm_duty_count = pwm_duty_count;  // save actual count (not %)

     if (mtr_blk->mtr_state != MOTOR_STATE_RUN)
        return (0);                             // do not apply till motor start

#if defined(TIVA_MCU)
       //----------------------------------------------------------------------
       // CAUTION: we are running Tiva PWMs in COUNT-DOWN mode, so wee need
       //          to adjust/invert the duty cycle value in relation to period
       //          otherwise a 80% duty value ends up yielding a 20 % output
       //----------------------------------------------------------------------
    adjusted_duty_count = mtr_blk->mtr_pwm_period_ticks - mtr_blk->mtr_pwm_duty_count;
    if (adjusted_duty_count < 0)
       adjusted_duty_count = 0;
#else
    adjusted_duty_count = mtr_blk->mtr_pwm_duty_count;
#endif
             // 05/16/16 - was the above screwing up STM32 ?

// for STM32, need to INVERT the duty value for both SLOW and FAST
//   if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
       {      // mtr_decay_mode == DECAY_SLOW, _must INVERT_ the PWM duty value
         pwm_duty_count = (mtr_blk->mtr_pwm_period_ticks - adjusted_duty_count);
       }
//    else    // mtr_decay_mode == DECAY_FAST, so use normal count
//         pwm_duty_count = adjusted_duty_count;

    if (mtr_blk->mtr_id == 0)
      {                 //---------------------------------------------------
                        //---------------------------------------------------
                        // change the Duty Cycle settings for Motor 1    AINx
                        //---------------------------------------------------
                        //---------------------------------------------------
        if (mtr_blk->mtr_direction == DIRECTION_FORWARD)   // forward = CW
           {
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {              // Slow Decay
                  AIN2_DUTY (pwm_duty_count);   // load INVERTED duty cycle
                }
               else            // Fast Decay
                {
                  AIN1_DUTY (pwm_duty_count);   // load latest duty cycle
                }
           }
          else                                  // backward/reverse = CCW
           {
             mtr_blk->mtr_direction = DIRECTION_BACKWARD;
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {              // Slow Decay
                  AIN1_DUTY (pwm_duty_count);   // load INVERTED duty cycle
                }
               else            // Fast Decay
                {
                  AIN2_DUTY (pwm_duty_count);   // load latest duty cycle
                }
           }
      }
     else   // mtr_id == 1
      {                   //---------------------------------------------------
                          //---------------------------------------------------
                          // change the Duty Cycle settings for Motor 2    BINx
                          //---------------------------------------------------
                          //---------------------------------------------------
        if (mtr_blk->mtr_direction == DIRECTION_FORWARD)   // forward = CW
           {
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {              // Slow Decay
                  BIN2_DUTY (pwm_duty_count);   // load INVERTED duty cycle
                }
               else            // Fast Decay
                {
                  BIN1_DUTY (pwm_duty_count);   // load latest duty cycle
                }
           }
          else                                  // backward/reverse = CCW
           {
             mtr_blk->mtr_direction = DIRECTION_BACKWARD;
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {              // Slow Decay
                  BIN1_DUTY (pwm_duty_count);   // load INVERTED duty cycle
                }
               else            // Fast Decay
                {
                  BIN2_DUTY (pwm_duty_count);   // load latest duty cycle
                }
           }
      }

    return (0);                // denote was successful
}


//******************************************************************************
//  drv8848_motor_stop
//
//                Stop the motor, using either Fast or Slow decay.
//                Also shuts off the PWMs, by locking them to HIGH or LOW state.
//******************************************************************************
int  drv8848_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags)
{
  switch (stop_type)
   {
    case STOP_HARD_BRAKE:
    case STOP_HIZ:
        if (mtr_blk->mtr_id == 0)
           {         // brake Motor 1
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {                 // sets to OUTMOD_0 + OUT => locked HIGH outputs on PWM
                  AIN1_HIGH;      // turning both AINs to HIGH jams AOUT1 and
                  AIN2_HIGH;      // AOUT2 to LOW state which = Slow Decay to gnd
                }
               else
                {                 // sets to OUTMOD_0  => locked LOW outputs on PWM
                  AIN1_LOW;       // turning both AINs to LOW jams AOUT1 and
                  AIN2_LOW;       // AOUT2 to Z (float) state which = Fast Decay
                }
           }
          else
           {         // brake Motor 2
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {                 // sets to OUTMOD_0 + OUT
                  BIN1_HIGH;      // turning both BINs to HIGH jams BOUT1 and
                  BIN2_HIGH;      // BOUT2 to LOW state which = Slow Decay to gnd
                }
               else
                {                 // sets to OUTMOD_0
                  BIN1_LOW;       // turning both BINs to LOW jams BOUT1 and
                  BIN2_LOW;       // BOUT2 to Z (float) state which = Fast Decay
                }
           }
        break;

     case STOP_SOFT_BRAKE:              // aka Coast
             //---------------------------------------------------------------
             //   was:  motor_brake
             // Stop the motor using a "brake" sequence: i.e. set both 1A/2A
             // or 3A/4A to both HIGH or both LOW.
             //
             // Setting 1A/2A or 3A/4A both HIGH forces any re-circulating
             // motor current to ground, and stops the motor faster.
             //
             // Setting 1A/2A or 3A/4A both LOW forces any re-circulating
             // motor current to keep looping through the windings until they
             // dissipate, and lets the motor coast to a stop.
             //---------------------------------------------------------------
        if (mtr_blk->mtr_id == 0)
           {         // brake Motor 1
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {                 // sets to OUTMOD_0 + OUT => locked HIGH output on PWM
                  AIN1_HIGH;      // turning both AINs to HIGH jams AOUT1 and
                  AIN2_HIGH;      // AOUT2 to LOW state which = Slow Decay to gnd
                }
               else
                {                 // sets to OUTMOD_0  => locked LOW output on PWM
                  AIN1_LOW;       // turning both AINs to LOW jams AOUT1 and
                  AIN2_LOW;       // AOUT2 to Z (float) state which = Fast Decay
                }
           }
          else
           {         // brake Motor 2
             if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
                {                 // sets to OUTMOD_0 + OUT
                  BIN1_HIGH;      // turning both BINs to HIGH jams BOUT1 and
                  BIN2_HIGH;      // BOUT2 to LOW state which = Slow Decay to gnd
                }
               else
                {                 // sets to OUTMOD_0
                  BIN1_LOW;       // turning both BINs to LOW jams BOUT1 and
                  BIN2_LOW;       // BOUT2 to Z (float) state which = Fast Decay
                }
           }
        break;
     }                            //  end  switch()

    mtr_blk->mtr_state = MOTOR_STATE_STOPPED;       // update to STOPPED state

    return (0);                   // denote was successful
}


//******************************************************************************
//  drv8848_motor_controller_chip_init
//******************************************************************************
int  drv8848_motor_controller_chip_init (void)
{
      // nothing additional needed to initialize the drv8848 controller chip
}



//******************************************************************************
//  drv8848_motor_pwm_init
//
//         Configure PWMs to driver motor 1/2ENA and 3/4ENA.
//         These use a variable duty cycle to control motor speed.
//
//         Generates a PWM with a frequency of 20,000 Hz (50 usec).
//
//         Note that the PWM for a motor should run at least 10x motor spe8d ???
//
//    Configure 4 outputs for Motor foward/reverse controls: AIN1,AIN2,BIN1,BIN2
//       AIN1  1A   Motor 1  TIM3_CH1   PB4   Arduino D5        TIM3_CH1
//       AIN2  2A   Motor 2  TIM3_CH2   PB5   Arduino D4        TIM3_CH2
//
//       BIN1  1B   Motor 1  TIM2_CH1   PA0   Arduino A0        TIM2_CH1
//       BIN2  2B   Motor 2  TIM2_CH2   PA1   Arduino A1        TIM2_CH2
//******************************************************************************

int  drv8848_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long period_ticks)
{
       //--------------------------------------------------------------------
       // Config AIN1/AIN2/BIN1/BIN2 GPIO pins for PWM mode.
       //--------------------------------------------------------------------

        ///////////////////////////////////////////////
        // Setup PWM Timer for 20 KHz operation
        ///////////////////////////////////////////////
    TA1CTL   = TASSEL_2 | MC_0 | TACLR;
    TA1CCTL1 = OUTMOD_7;           // use CCR in PWM mode
    TA1CCTL2 = OUTMOD_7;
    TA1CCR0  = period_ticks;       // normal Base Period = 20 KHz
    TA1CCR1  = 0;                  // initial Duty Cycle PWM A = 0
    TA1CCR2  = 0;                  // initial Duty Cycle PWM B = 0

       //----------------------------------------------------------------------
       // CRITICAL MOTE:   
       //   For 8848, the PWM Output Enable pins (for AIN1/AIN2/BIN1/BIN2) are
       //   turned on at Motor_Run() time, based on direction and decay type
       //----------------------------------------------------------------------

       //-------------------------------------------------------
       //  Enable the PWM generator modules and their channels
       //-------------------------------------------------------
    TA1CTL = TASSEL_2 | MC_1;  // Enable PWM Timer (but duty=0, and outputs disabled)

       //--------------------------------------------------------------
       // Enable the DRV8848 chip, by turning off SLEEP and FAULT mode
       //--------------------------------------------------------------
       // Setup DRV8848 FAULT and SLEEP GPIO Values
    nFAULT_CLEAR_FAULT();      // Turn off Fault  (Active Low)
    nSLEEP_SET_AWAKE();        // Turn off SLEEP  (Active Low)

    return (0);                // denote was successful
}


//******************************************************************************
//  drv8848_motor_gpio_init_inputs
//
//         Configure control inputs for Motor GPIOs  - FAULT, ...
//          nFault:  Fault = Active Low  J2-3   PE0
//******************************************************************************

int  drv8848_motor_init_ctl_gpio_inputs (void)
{
       // shared inputs and outputs (A ENABLE / OCD and B ENABLE / OCD) were
       // already handled in drv8848_motor_init_ctl_gpio_outputs()

    return (0);           // denote was successful
}


//******************************************************************************
//  drv8848_motor_init_ctl_gpio_outputs
//
//         Configure control outputs for Motor GPUIOs - ENABLE, ...
//
//         Note that on the drv8848, the ENABLE pins are actually shared In/Out
//         pins, with the OCD input coming in through the same pin.
//         (Weird, yes, but thats the way it works. They tried to save on pins)
//******************************************************************************
int  drv8848_motor_init_ctl_gpio_outputs (void)
{
          //-------------------------------------------------------------------
          // Configure ENABLE outputs, and then set the PWM bridges to Enabled
          //
          //      ENABLE:   Each motor has its own ENABLE pin
          //                  Motor 1 - PA10  A ENABLE / OCD   PA10  Arduino D2
          //                  Motor 2 - PC1   B ENABLE / OCD   PC1   Arduino A4
          //-------------------------------------------------------------------

       //----------------------------------------------------------
       // Configure GPIO Port Directions and Peripherals as needed
       //----------------------------------------------------------

       // Port 1
    P1SEL  &= ~( VREF );      // set VREF to ADC mode
    P1SEL2 &= ~( VREF );

    P1DIR  &= ~( VREF );

        // Port 2 - initially reset port 0 to all GPIOs. MotorLib will setup assoc PWM based on Decay and Direction settings
    P2SEL  &= ~(BIN1 | BIN2 | AIN2 | AIN1 | nSLEEP | nFAULT);
    P2SEL2 &= ~(BIN1 | BIN2 | AIN2 | AIN1 | nSLEEP | nFAULT);

    P2DIR  |=  (BIN1 | BIN2 | AIN2 | AIN1 | nSLEEP);  // 1 = OUTPUT
    P2OUT  &= ~(BIN1 | BIN2 | AIN2 | AIN1 | nSLEEP);  // Turn off all pins

    P2DIR  &= ~nFAULT;
    P2OUT  |= nFAULT;
    P2REN  |= nFAULT;

        // Port 3 - set to low power mode by setting all off
    P3DIR |=  (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
    P3OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);



          // Configure the GPIO connected to the "A ENABLE pin as an output

          // Configure the GPIO connected to the "B" ENABLE pin as an output


          //--------------------------------------------------
          //   turn on the PWM bridges - ENABLE - Active HIGH  was LOW
          //--------------------------------------------------
          // Rest of motor GPIO output configuration is done in motor_pwm_init
    return (0);           // denote was successful
}


//******************************************************************************
//  drv8848_motor_encoder_gpio_init                                      ENCODER
//
//         Configure GPIOs for Wheel Encoder pin inputs ...
//              Hall Sensor Left      PA8   Arduino D7    GPIO rupt   Grove D7
//              Hall Sensor Right     PA9   Arduino D8    GPIO rupt   Grove D8
//******************************************************************************
int  drv8848_motor_encoder_gpio_init (MOTOR_BLOCK *mtr_blk, int encoder_type)
{
    if (mtr_blk->mtr_id == 0)
       {       // Motor 1
       }
      else if (mtr_blk->mtr_id == 1)
       {       // Motor 2
       }

       //------------------------------------------------------------------
       //                     WHEEL  ENCODER  CONFIG
       // Configure PE5 and PA5 (encoders) as open collector inputs with
       // pull-up resistors.  Trigger on High-to-Low transition
       //------------------------------------------------------------------
 
        // Enable and set EXTI line 9_5 Interrupt to a low priority
        // Both the Left (PA8) and Right (PA9) wheels share the same EXTI IRQn,
        // so only need to issue once

    return (0);           // denote was successful
}


//******************************************************************************
//  drv8848_motor_adc_ctl_init
//
//         Configure ADC to read contropl inputs for current sensors, ...
//             ADC-6  PA6   ADC12_IN6   "A" side Current-Sense   Arduino D12
//             ADC-4  PA4   ADC12_IN4   "B" side Current-Sense   Arduino A2
//******************************************************************************
int  drv8848_motor_adc_ctl_init (void)
{
    int  status;

        //------------------------------------------------------
        // Configure the GPIOs to be used for ADC.
        // Configure the selected ADC Channels as analog inputs
        //------------------------------------------------------
         //-----------------------------
        // Configure basic ADC module and Channel
        //-----------------------------
 
        ///////////////////////////////////////////////
        // Setup ADC10 to read in current limit VREF
        ///////////////////////////////////////////////
    G_VREF    = 0;
    ADC10CTL0 = ADC10SHT_3 | ADC10ON;
    ADC10CTL1 = INCH_3;
    ADC10AE0 |= VREF;
    return (0);
}


//******************************************************************************
//  drv8848_motor_speed_adc_init                                          SPEED
//
//         Configure ADC to read potentiometer used for speed control setting.
//        -     Speed Ctl Pot ADC    J1-2    PB5  AIN11   Grove J3-7/27 -> J1-2
//******************************************************************************
int  drv8848_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr)
{
    uint8_t   status = 1;

    return (0);           // denote was successful
}


//******************************************************************************
//  drv8848_motor_adc_next_conversion
//
//         start a new ADC conversion for Speed control  (called every 50 ms)
//         Uses A15  on pin  P6.0
//******************************************************************************
int  drv8848_motor_adc_next_conversion (void)
{
    int    rc;

       //------------------------------------------------
       // if this is first time, enable the ADC and DMA
       //------------------------------------------------
    if (_g_adc_enabled == 0)
       {
         _g_adc_enabled = 1;           // denote we started the ADC and its DMA
       }

    return (0);           // denote was successful
}

//******************************************************************************
