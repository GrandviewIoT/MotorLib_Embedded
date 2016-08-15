
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                      MotorLib_LL_DRV8848_MSP430_FR5969.c
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
// L6206 Xnucleo Usage
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

#include "MotorLib_Api.h"                         // pull in common definitions

extern uint16_t  ADC0_Seq0_Conversion_Values[];   // ADC conversion results

    uint32_t  _g_PWM_IO_clock = 0; // Global to hold PWM Max Clock Frequency
    uint32_t  _g_pwm_mdivider = 1; // PWM Master clock : CPU divider ratio
    uint32_t  _g_PWMperiod    = 0; // Global to hold PWM period  (MaxClock/Freq)
    uint32_t  _g_PWMduty      = 0; // Global to hold latest PWM duty cycle
    int       _g_LPWM_rc       = 0;
    int       _g_adc_enabled  = 0;
    int       _g_adc_num_channels = 0;

           //-------------------------------------------------------
           // Required HAL data structures for GPIO, ADC, PWM/Timer
           //-------------------------------------------------------
    ADC_HandleTypeDef       ADC_InitStruct;
    ADC_ChannelConfTypeDef  ADC_ChanConfig;
    DMA_HandleTypeDef       DMA_Handle;           // DMA module typedef handle
    GPIO_InitTypeDef        GPIO_InitStruct;
    TIM_HandleTypeDef       hTimPwm1A;            // PWM timer for  AIN1  1A
    TIM_HandleTypeDef       hTimPwm2A;            // PWM timer for  AIN2  2A
    TIM_HandleTypeDef       hTimPwm1B;            // PWM timer for  BIN1  1B
    TIM_HandleTypeDef       hTimPwm2B;            // PWM timer for  BIN2  2B
    TIM_OC_InitTypeDef      TimPwm_OC_Config;     // PWM High/Low/PWM mode defs
    TIM_OC_InitTypeDef      PWM_ChanConfigOC;     // PWM Channel

           //-----------------------------------------------------
           // Config definitions for Motor hookups on this board
           //-----------------------------------------------------
                                // Setup PWM Control Outputs
                                //      CCW = counter-clockwise (spins left)
                                //      CW  = clockwise         (spins right)
#define AOCD_READ_PIN    { HAL_GPIO_ReadPin(AENABLE_OCD_PORT,AENABLE_OCD_PIN); }
#define BOCD_READ_PIN    { HAL_GPIO_ReadPin(BENABLE_OCD_PORT,BENABLE_OCD_PIN); }

#define AIN1_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm1A, AIN1_PWM_CHANNEL, dvalue); }
                                // Turn on PWM output, with normal Active High
//                          TimPwm_OC_Config.Pulse = duty value;
#define AIN1_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                            TimPwm_OC_Config.Pulse  = dutyval;  \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm1A, &TimPwm_OC_Config, AIN1_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm1A.Instance, AIN1_PWM_CHANNEL, TIM_CCx_ENABLE); }
                                // Turn off PWM output, with output = 0 / Low
#define AIN1_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_INACTIVE; \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm1A, &TimPwm_OC_Config, AIN1_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm1A.Instance, AIN1_PWM_CHANNEL, TIM_CCx_ENABLE); }
                                // Turn off PWM output, with output = 1 / High
#define AIN1_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_ACTIVE; \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm1A, &TimPwm_OC_Config, AIN1_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm1A.Instance, AIN1_PWM_CHANNEL, TIM_CCx_ENABLE); }

#define AIN2_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm2A, AIN2_PWM_CHANNEL, dvalue); }
#define AIN2_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                            TimPwm_OC_Config.Pulse  = dutyval;  \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm2A, &TimPwm_OC_Config, AIN2_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm2A.Instance, AIN2_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define AIN2_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_INACTIVE; \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm2A, &TimPwm_OC_Config, AIN2_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm2A.Instance, AIN2_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define AIN2_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_ACTIVE; \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm2A, &TimPwm_OC_Config, AIN2_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm2A.Instance, AIN2_PWM_CHANNEL, TIM_CCx_ENABLE); }

#define BIN1_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm1B, BIN1_PWM_CHANNEL, dvalue); }
#define BIN1_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                            TimPwm_OC_Config.Pulse  = dutyval;  \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm1B, &TimPwm_OC_Config, BIN1_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm1B.Instance, BIN1_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define BIN1_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_INACTIVE; \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm1B, &TimPwm_OC_Config, BIN1_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm1B.Instance, BIN1_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define BIN1_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_ACTIVE; \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm1B, &TimPwm_OC_Config, BIN1_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm1B.Instance, BIN1_PWM_CHANNEL, TIM_CCx_ENABLE); }

#define BIN2_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm2B, BIN2_PWM_CHANNEL, dvalue); }
#define BIN2_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                            TimPwm_OC_Config.Pulse  = dutyval;  \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm2B, &TimPwm_OC_Config, BIN2_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm2B.Instance, BIN2_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define BIN2_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_INACTIVE; \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm2B, &TimPwm_OC_Config, BIN2_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm2B.Instance, BIN2_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define BIN2_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_ACTIVE; \
                            _g_LPWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm2B, &TimPwm_OC_Config, BIN2_PWM_CHANNEL); \
                            TIM_CCxChannelCmd (hTimPwm2B.Instance, BIN2_PWM_CHANNEL, TIM_CCx_ENABLE); }


           //--------------------------------------
           //  Device Handler function prototypes
           //--------------------------------------
int  l6206_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr);
int  l6206_motor_adc_ctl_init (void);
int  l6206_motor_controller_chip_init (void);
int  l6206_motor_encoder_gpio_init(MOTOR_BLOCK *mtr_blk, int encoder_type);
int  l6206_motor_init_ctl_gpio_outputs (void);
int  l6206_motor_init_ctl_gpio_inputs (void);
int  l6206_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long pwm_ticks);

int  l6206_motor_run (MOTOR_BLOCK *mtr_blk, int direction);
int  l6206_motor_set_direction (MOTOR_BLOCK *mtr_blk, int mtr_direction);
int  l6206_motor_set_duty_cycle (MOTOR_BLOCK *mtr_blk, long actual_duty_count);
int  l6206_motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count);
int  l6206_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags);

int  l6206_motor_gpio_init_inputs (void);
int  l6206_motor_gpio_init_outputs (void);
int  l6206_motor_adc_next_conversion (void);


           //------------------------------------------------------------
           //------------------------------------------------------------
           //                Chip Controller Function Table
           //
           //  Defines low-level command handlers that are invoked by
           //  BDC_motor_ctl_lib.c to perform device specific functions
           //------------------------------------------------------------
           //------------------------------------------------------------
 MOTOR_HANDLERS  drv6206_table = {
                     TYPE_MOTOR_BDC,               // Motor Type
                     MOTOR_DRIVER,                 // Chip Controller id
                     0L,                           // SPI init
                     0L,                           // SPI I/O
                     l6206_motor_speed_adc_init,   // init routine for speed ctl ADC
                     l6206_motor_adc_next_conversion, // ADC conversion start rtn for speed
                     0L,                           // motor_read_hall_sensors_handler
                     0L,                           // motor_fault_shutdown
                     l6206_motor_adc_ctl_init,     // control ADCs (Current sense)
                     l6206_motor_controller_chip_init, // motor_init_controller_chip
                     0L,                               // motor_init_commute_time
                     l6206_motor_encoder_gpio_init,    // init routine for encoder GPIOs
                     l6206_motor_init_ctl_gpio_inputs, // control input  GPIOs (FAULT)
                     l6206_motor_init_ctl_gpio_outputs,// control output GPIOs (ENABLE)
                     l6206_motor_pwm_init,             // motor PWMs

                     0L,                           // motor_check_status
                     0L,                           // motor_do_action_handler
                     0L,                           // motor_do_next_commutate
                     0L,                           // motor_get_actual_position
                     0L,                           // motor_get_mark_pos
                     0L,                           // motor_get_motor_status
                     0L,                           // motor_get_period_ticks
                     0L,                           // motor_perform_move
                     l6206_motor_run,              // motor_run_handler
                     l6206_motor_set_direction,    // motor_set_direction_handler
                     l6206_motor_set_pwm_duty_count, // motor_set_pwm_duty_count_handler
                     0L,                           // motor_set_speed_rpm_handler
                     0L,                           // motor_set_step_mode
                     0L,                           // motor_set_torque
                     0L,                           // motor_start_handler (BLDC)
                     l6206_motor_stop,             // motor_stop_handler
                     0L,                           // motor_wait_complete
                     0L,                           // motor_current_reference_start_stop
                     0L,                           // motor_current_reference_set_value
                     0L,                           // motor_start_pwm_generation (BLDC)
                     0L,                           // motor_stop_pwm_generation  (BLDC)
                     0L,                           // motor_set_bldc_phase_pwm_duty_cycles (BLDC)
                     0L,                           // motor_set_bldc_pwm_enables (BLDC)
                   };


//******************************************************************************
//  l6206_motor_run
//
//               Run the motor, i.e. issue a start (or re-start after a
//                              direction change or after a stop).
//
//               Because a stop will slam off the PWMs, we re-establish both the
//               Direction and the Duty Cycle.
//******************************************************************************
int  l6206_motor_run (MOTOR_BLOCK *mtr_blk, int direction)
{
    mtr_blk->mtr_state = MOTOR_STATE_RUN;     // denote we are now in RUN mode

       // Invoke set_direction, which will configure PWMs and load duty values,
       // which will cause the motor to start.
    l6206_motor_set_direction (mtr_blk, direction);

    return (0);           // denote was successful
}


//******************************************************************************
//  l6206_motor_set_direction
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
int  l6206_motor_set_direction (MOTOR_BLOCK *mtr_blk,  int direction)
{
    int  pwm_duty_count;

    mtr_blk->mtr_direction = direction;     // ensure desired direction is saved

    if (mtr_blk->mtr_state != MOTOR_STATE_RUN)
        return (0);                           // do not apply until motor start

// STM32 - ??? why doesn't this require INVERT even on FAST ? Or did we just not pass thru here on FAST ?
    if (mtr_blk->mtr_decay_mode == DECAY_SLOW)
       {      // mtr_decay_mode == DECAY_SLOW, _must INVERT_ the PWM duty value
         pwm_duty_count = (mtr_blk->mtr_pwm_period_ticks - mtr_blk->mtr_pwm_duty_count);
       }
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
//  l6206_motor_set_pwm_duty_count
//
//            Set the duty cycle to be used for a specific motor.
//            Here, duty cycle is explicitly passed as PWM ticks, not as percent.
//            The duty cycle sets the motor's speed. (More juice, more speed)
//
//   Parms:
//      motor_num:   number of the motor to update (1 or 2)
//      pwm_duty:    actual duty cycle PWM count value to be loaded in PWM
//******************************************************************************
int  l6206_motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count)
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
//  l6206_motor_stop
//
//                Stop the motor, using either Fast or Slow decay.
//                Also shuts off the PWMs, by locking them to HIGH or LOW state.
//******************************************************************************
int  l6206_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags)
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
//  l6206_motor_controller_chip_init
//******************************************************************************
int  l6206_motor_controller_chip_init (void)
{
      // nothing additional needed to initialize the L6206 controller chip
}



//******************************************************************************
//  l6206_motor_pwm_init
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

int  l6206_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long period_ticks)
{
    TIM_HandleTypeDef         *pHTim;
    TIM_MasterConfigTypeDef   sMasterConfig;        // PWM Module
    uint32_t                  prescalar;
    uint32_t                  channel;

    __TIM3_CLK_ENABLE();            // Turn on clocks used by PWM timers
    __TIM2_CLK_ENABLE();

       // clear out the initialization structures
    memset (&hTimPwm1A, 0, sizeof(hTimPwm1A));
    memset (&hTimPwm2A, 0, sizeof(hTimPwm1A));
    memset (&hTimPwm1B, 0, sizeof(hTimPwm1A));
    memset (&hTimPwm2B, 0, sizeof(hTimPwm1A));
    memset (&PWM_ChanConfigOC, 0, sizeof(PWM_ChanConfigOC));

       //--------------------------------------------------------------------
       // Config AIN1/AIN2/BIN1/BIN2 GPIO pins for PWM mode.
       //--------------------------------------------------------------------
    GPIO_InitStruct.Pin       = AIN1_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = AIN1_PWM_MUX_CONFIG; // set Pin Mux Alt Function code
    HAL_GPIO_Init (AIN1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = AIN2_PIN;
    GPIO_InitStruct.Alternate = AIN2_PWM_MUX_CONFIG; // set Pin Mux Alt Function code
    HAL_GPIO_Init (AIN2_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = BIN1_PIN;
    GPIO_InitStruct.Alternate = BIN1_PWM_MUX_CONFIG; // set Pin Mux Alt Function code
    HAL_GPIO_Init (BIN1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = BIN2_PIN;
    GPIO_InitStruct.Alternate = BIN2_PWM_MUX_CONFIG; // set Pin Mux Alt Function code
    HAL_GPIO_Init (BIN2_PORT, &GPIO_InitStruct);

       //--------------------------------------------------------------------
       // Config AIN1/AIN2 (PB4/PB5) pins for PWM mode.  TIM3_CH1 / TIM3_CH2
       //--------------------------------------------------------------------
    pHTim = &hTimPwm1A;                     // PWM Timer struct to use for AIN1
    pHTim->Instance           = AIN1_PWM_TIMER;
    pHTim->Channel            = AIN1_PWM_CHANNEL;
    pHTim->Init.Prescaler     = TIMER_PRESCALER - 1;
    pHTim->Init.CounterMode   = TIM_COUNTERMODE_UP;
    pHTim->Init.Period        = period_ticks;                // set PWM period
    pHTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init (pHTim);
    channel                     = AIN1_PWM_CHANNEL;
    PWM_ChanConfigOC.OCMode     = TIM_OCMODE_PWM1;
    PWM_ChanConfigOC.Pulse      = 0;
    PWM_ChanConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    PWM_ChanConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel (pHTim, &PWM_ChanConfigOC, channel);

    pHTim = &hTimPwm2A;                     // PWM Timer struct to use for AIN2
    pHTim->Instance           = AIN2_PWM_TIMER;
    pHTim->Channel            = AIN2_PWM_CHANNEL;
    pHTim->Init.Prescaler     = TIMER_PRESCALER - 1;
    pHTim->Init.CounterMode   = TIM_COUNTERMODE_UP;
    pHTim->Init.Period        = period_ticks;                // set PWM period
    pHTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init (pHTim);
    channel                 = AIN2_PWM_CHANNEL;
    HAL_TIM_PWM_ConfigChannel (pHTim, &PWM_ChanConfigOC, channel);

       //--------------------------------------------------------------------
       // Config BIN1/BIN2 (PA0/PA1) pins for PWM mode.  TIM2_CH1 / TIM2_CH2
       //--------------------------------------------------------------------
       //--------------------------------------------------------------------
    pHTim = &hTimPwm1B;                     // PWM Timer struct to use for BIN1
    pHTim->Instance           = BIN1_PWM_TIMER;
    pHTim->Channel            = BIN1_PWM_CHANNEL;
    pHTim->Init.Prescaler     = TIMER_PRESCALER - 1;
    pHTim->Init.CounterMode   = TIM_COUNTERMODE_UP;
    pHTim->Init.Period        = period_ticks;                // set PWM period
    pHTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init (pHTim);
    channel                     = BIN1_PWM_CHANNEL;
    PWM_ChanConfigOC.OCMode     = TIM_OCMODE_PWM1;
    PWM_ChanConfigOC.Pulse      = 0;
    PWM_ChanConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    PWM_ChanConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel (pHTim, &PWM_ChanConfigOC, channel);

    pHTim = &hTimPwm2B;                     // PWM Timer struct to use for BIN2
    pHTim->Instance           = BIN2_PWM_TIMER;
    pHTim->Channel            = BIN2_PWM_CHANNEL;
    pHTim->Init.Prescaler     = TIMER_PRESCALER - 1;
    pHTim->Init.CounterMode   = TIM_COUNTERMODE_UP;
    pHTim->Init.Period        = period_ticks;                // set PWM period
    pHTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init (pHTim);
    channel                 = BIN2_PWM_CHANNEL;
    HAL_TIM_PWM_ConfigChannel (pHTim, &PWM_ChanConfigOC, channel);

       //-------------------------------------------------------
       //  Enable the PWM generator modules and their channels
       //-------------------------------------------------------
    HAL_TIM_PWM_Start (&hTimPwm1A, AIN1_PWM_CHANNEL);     // TIM3 startup
    HAL_TIM_PWM_Start (&hTimPwm2A, AIN2_PWM_CHANNEL);
    HAL_TIM_PWM_Start (&hTimPwm1B, BIN1_PWM_CHANNEL);     // TIM2 startup
    HAL_TIM_PWM_Start (&hTimPwm2B, BIN2_PWM_CHANNEL);

    return (0);           // denote was successful
}


//******************************************************************************
//  l6206_motor_gpio_init_inputs
//
//         Configure control inputs for Motor GPIOs  - FAULT, ...
//          nFault:  Fault = Active Low  J2-3   PE0
//******************************************************************************

int  l6206_motor_init_ctl_gpio_inputs (void)
{
       // shared inputs and outputs (A ENABLE / OCD and B ENABLE / OCD) were
       // already handled in l6206_motor_init_ctl_gpio_outputs()

    return (0);           // denote was successful
}


//******************************************************************************
//  l6206_motor_init_ctl_gpio_outputs
//
//         Configure control outputs for Motor GPUIOs - ENABLE, ...
//
//         Note that on the L6206, the ENABLE pins are actually shared In/Out
//         pins, with the OCD input coming in through the same pin.
//         (Weird, yes, but thats the way it works. They tried to save on pins)
//******************************************************************************
int  l6206_motor_init_ctl_gpio_outputs (void)
{
          //-------------------------------------------------------------------
          // Configure ENABLE outputs, and then set the PWM bridges to Enabled
          //
          //      ENABLE:   Each motor has its own ENABLE pin
          //                  Motor 1 - PA10  A ENABLE / OCD   PA10  Arduino D2
          //                  Motor 2 - PC1   B ENABLE / OCD   PC1   Arduino A4
          //-------------------------------------------------------------------
          // Configure the GPIO connected to the "A ENABLE pin as an output
    GPIO_InitStruct.Pin   = AENABLE_OCD_PIN;            // Motor 1 ENABLE / OCD
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init (AENABLE_OCD_PORT, &GPIO_InitStruct);

          // Configure the GPIO connected to the "B" ENABLE pin as an output
    GPIO_InitStruct.Pin   = BENABLE_OCD_PIN;            // Motor 2 ENABLE / OCD
    HAL_GPIO_Init (BENABLE_OCD_PORT, &GPIO_InitStruct);

          //--------------------------------------------------
          //   turn on the PWM bridges - ENABLE - Active HIGH  was LOW
          //--------------------------------------------------
    __disable_irq();
    HAL_GPIO_WritePin (AENABLE_OCD_PORT, AENABLE_OCD_PIN, GPIO_PIN_SET); // Motor 1  was GPIO_PIN_RESET
    __HAL_GPIO_EXTI_CLEAR_IT (AENABLE_OCD_PIN);
    HAL_GPIO_WritePin (BENABLE_OCD_PORT, BENABLE_OCD_PIN, GPIO_PIN_SET); // Motor 2  was GPIO_PIN_RESET
    __HAL_GPIO_EXTI_CLEAR_IT (BENABLE_OCD_PIN);
    __enable_irq();

         // Setup NVIC for the "A" side (Motor 1) OCD GPIO input interrupt
    HAL_NVIC_SetPriority (A_OCD_EXTI_IRQn, A_OCD_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ (A_OCD_EXTI_IRQn);                          // PA10 EXTI

         // Setup NVIC for the "B" side (Motor 2) OCD GPIO input interrupt
    HAL_NVIC_SetPriority (B_OCD_EXTI_IRQn, B_OCD_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ (B_OCD_EXTI_IRQn);                          // PC1 EXTI

          // Rest of motor GPIO output configuration is done in motor_pwm_init
    return (0);           // denote was successful
}


//******************************************************************************
//  l6206_motor_encoder_gpio_init                                      ENCODER
//
//         Configure GPIOs for Wheel Encoder pin inputs ...
//              Hall Sensor Left      PA8   Arduino D7    GPIO rupt   Grove D7
//              Hall Sensor Right     PA9   Arduino D8    GPIO rupt   Grove D8
//******************************************************************************
int  l6206_motor_encoder_gpio_init (MOTOR_BLOCK *mtr_blk, int encoder_type)
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
       // Configure Encoder Left Wheel PA8 pin as input interrupt
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin  = HALL_ENCODER_LEFT_PIN;             // PA8  Arduino D7
    HAL_GPIO_Init (HALL_ENCODER_LEFT_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = HALL_ENCODER_RIGHT_PIN;            // PA9  Arduino D8
    HAL_GPIO_Init (HALL_ENCODER_RIGHT_PORT, &GPIO_InitStruct);

        // Enable and set EXTI line 9_5 Interrupt to a low priority
        // Both the Left (PA8) and Right (PA9) wheels share the same EXTI IRQn,
        // so only need to issue once
    HAL_NVIC_SetPriority (HALL_ENCODER_LEFT_EXTI_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ (HALL_ENCODER_LEFT_EXTI_IRQn);

    return (0);           // denote was successful
}


//******************************************************************************
//  l6206_motor_adc_ctl_init
//
//         Configure ADC to read contropl inputs for current sensors, ...
//             ADC-6  PA6   ADC12_IN6   "A" side Current-Sense   Arduino D12
//             ADC-4  PA4   ADC12_IN4   "B" side Current-Sense   Arduino A2
//******************************************************************************
int  l6206_motor_adc_ctl_init (void)
{
    int  status;

        //------------------------------------------------------
        // Configure the GPIOs to be used for ADC.
        // Configure the selected ADC Channels as analog inputs
        //------------------------------------------------------
    GPIO_InitStruct.Pin  = CURRENT_SENSE_A_PIN;     // PA6  ADC12_IN6  Ardu D12
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init (CURRENT_SENSE_A_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = CURRENT_SENSE_B_PIN;     // PA4  ADC12_IN4  Ardu A2
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init (CURRENT_SENSE_B_PORT, &GPIO_InitStruct);

    __ADC1_CLK_ENABLE();                      // Turn on clocks to ADC module 1

         // Enable the ADC module's associated DMA module's clocks
    __HAL_RCC_DMA2_CLK_ENABLE();              // Enable DMA2 clock      F4 / F7

        //-----------------------------
        // Configure basic ADC module
        //-----------------------------
    ADC_InitStruct.Instance                   = ADC1;    // uses ADC1 module
    ADC_InitStruct.Init.DMAContinuousRequests = ENABLE;  // Use DMA
    ADC_InitStruct.Init.EOCSelection          = DISABLE; // EOC_SINGLE_CONV; DMA will interrupt, not ADC EOC.
    ADC_InitStruct.Init.ScanConvMode          = ENABLE;  // scan multiple channels
    ADC_InitStruct.Init.ContinuousConvMode    = DISABLE;
    ADC_InitStruct.Init.DiscontinuousConvMode = DISABLE;
    ADC_InitStruct.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4; /* (must not exceed 36MHz) */
    ADC_InitStruct.Init.Resolution            = ADC_RESOLUTION12b;    // 12 bits 0-4096
    ADC_InitStruct.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    ADC_InitStruct.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    ADC_InitStruct.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    ADC_InitStruct.Init.NbrOfConversion       = 2;      // 2 channels to convert
    HAL_ADC_Init (&ADC_InitStruct);

        //---------------------------------------------
        // Configure the ADC Channels to be converted
        //---------------------------------------------
    ADC_ChanConfig.Channel      = CURRENT_SENSE_A_CHANNEL; // "A" side current sense
    ADC_ChanConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    ADC_ChanConfig.Rank         = 1;                   // 1st channel to convert
    status = HAL_ADC_ConfigChannel (&ADC_InitStruct, &ADC_ChanConfig);

    ADC_ChanConfig.Channel      = CURRENT_SENSE_B_CHANNEL; // "B" side current sense
    ADC_ChanConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    ADC_ChanConfig.Rank         = 2;                   // 2nd channel to convert
    status = HAL_ADC_ConfigChannel (&ADC_InitStruct, &ADC_ChanConfig);

        //-----------------------------------------------------------------
        //                        DMA   CONFIG
        //
        // Initialize the DMA associated with the ADC module.
        //-----------------------------------------------------------------
    DMA_Handle.Init.Channel    = DMA_CHANNEL_ADC1;
    DMA_Handle.Instance        = DMA_INSTANCE;
    DMA_Handle.Init.Direction  = DMA_PERIPH_TO_MEMORY;  // ADC -> RAM
    DMA_Handle.Init.PeriphInc  = DMA_PINC_DISABLE;
    DMA_Handle.Init.MemInc     = DMA_MINC_ENABLE;       // step buf ptr
    DMA_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  // for 16 bit results
    DMA_Handle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;  // ditto
    DMA_Handle.Init.Mode       = DMA_CIRCULAR;     // Treat as circular buf
    DMA_Handle.Init.Priority   = DMA_PRIORITY_HIGH;
    DMA_Handle.Init.FIFOMode   = DMA_FIFOMODE_DISABLE;  // F4_46 has FIFO
    DMA_Handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    DMA_Handle.Init.MemBurst    = DMA_MBURST_SINGLE;
    DMA_Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;

    status = HAL_DMA_Init (&DMA_Handle);

        // Associate the initialized DMA handle to the the ADC handle
    __HAL_LINKDMA (&ADC_InitStruct, DMA_Handle, DMA_Handle);

              //----------------------------------------------------------
              //          Configure the NVIC for DMA interrupts
              // Configure NVIC for DMA transfer complete interrupt.
              //----------------------------------------------------------
    HAL_NVIC_SetPriority (DMA_STREAM_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ (DMA_STREAM_IRQ);

    _g_adc_num_channels = 2;

    return (0);
}


//******************************************************************************
//  l6206_motor_speed_adc_init                                          SPEED
//
//         Configure ADC to read potentiometer used for speed control setting.
//        -     Speed Ctl Pot ADC    J1-2    PB5  AIN11   Grove J3-7/27 -> J1-2
//******************************************************************************
int  l6206_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr)
{
    uint8_t   status = 1;

        // Configure the GPIO to be used for Speed Control ADC
    GPIO_InitStruct.Pin  = SPEED_CONTROL_PIN;        // PB0  ADC12_IN8  Ardu A3
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init (SPEED_CONTROL_PORT, &GPIO_InitStruct);

        // configure another channel
    ADC_ChanConfig.Channel      = SPEED_CONTROL_CHANNEL;  // Speed Potentiometer
    ADC_ChanConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    ADC_ChanConfig.Rank         = 3;                   // 3rd channel to convert
    status = HAL_ADC_ConfigChannel (&ADC_InitStruct, &ADC_ChanConfig);

    _g_adc_num_channels = 3;

       //---------------------------------------------------
       // Update ADC_InitStruct.Init.NbrOfConversion = 3;
       //---------------------------------------------------
    ADC_InitStruct.Instance->SQR1 &= ~(ADC_SQR1_L);     // clear the # conv bits
           // note that the macro ADC_SQR1 subtracts one from the count _and_
           // shifts it by 20 bits to put the value into the ADC_SQR1_L position
    ADC_InitStruct.Instance->SQR1 |=  ADC_SQR1(_g_adc_num_channels); // F4, F7, L4

    return (0);           // denote was successful
}


//******************************************************************************
//  l6206_motor_adc_next_conversion
//
//         start a new ADC conversion for Speed control  (called every 50 ms)
//         Uses A15  on pin  P6.0
//******************************************************************************
int  l6206_motor_adc_next_conversion (void)
{
    int    rc;

       //------------------------------------------------
       // if this is first time, enable the ADC and DMA
       //------------------------------------------------
    if (_g_adc_enabled == 0)
       {
         _g_adc_enabled = 1;           // denote we started the ADC and its DMA
       }

       //--------------------------------------------------------------
	   // Trigger a new round of ADC conversions.
       // The following starts the ADC and auto-initiates Conversions
       //--------------------------------------------------------------
    rc = HAL_ADC_Start_DMA (&ADC_InitStruct,
                            (uint32_t*) &ADC0_Seq0_Conversion_Values[0],
                            _g_adc_num_channels);

    return (0);           // denote was successful
}

//******************************************************************************
