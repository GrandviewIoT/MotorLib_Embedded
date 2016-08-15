// To Start Motor you MUST press the BLUE User button on STM32

volatile int  pause_true = 1;   // TEST flag


//*******1*********2*********3*********4*********5*********6*********7**********
//
//                                  main_BLDC.c
//
// Controls multiple BLDC motors, including forward/reverse and speed control.
//
// Has been tested with the TI DRV8305 BLDC Boosterpack.'
//
// Works with the following BLDC controllers, based upon the device selected
// in motor_config.h file:
//     - TI DRV8305 BLDC Gate Driver used on the TI DRV8305 Boosterpack.
//     - ST L6xxx  BLDC Gate Driver. used on the STM32 Xnucleo XXXX board.
//
// 24 Volts is supplied to DC motors via BLDC Controller.
// Motors are driven by PWMs, which are used to modulate the speed.
// Forward and reverse (CW, CCW) are controlled through 1A/2A amd 3A/4A pins.
//
//
// Common Pin Usage              LP Conn  Tiva          MSP432
// -------------------           -------  -----         ------
//   8   Vcc2   Motor 6 V supply     -      -            -
//  16   Vcc1   Logic +5            J3-1   5v           5v
//  4/5/12/13   Ground              J3-2   Gnd          Gnd
//
//   -    -     Speed Ctl Pot ADC   J1-2   ADC          A15    P6.0
//   -    -     Fwd/Reverse Slider  J1-4   GPIO         GPIO   P3.3
//
//   -    -     Hall Sensor Left    J1-6   GPIO rupt    GPIO   P4.3  was P6.5
//   -    -     Hall Sensor Right   J1-8   GPIO rupt    GPIO   P4.6  was P6.4
//              Hall sensors erquire 10K pull-ups between Vcc and signal pin.
//              Must use external pullups, because TI internal pullups are only
//              meant for loads of 2-8mA, and most Hall sensors need 10-12 mA.
//              For Hall sensors requiring 5V, use a 10K/6.8K voltage divider,
//              because MSP432 GPIOs are _NOT_ 5V tolerant.    10K/3.3K   ?
//
// SN7544120 Pin Usage            LP Conn  Tiva         MSP432
// -------------------            -------  -----        ------
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
//
// DRV8848 BP Pin Usage           LP Conn  Tiva         MSP432
// -------------------            -------  -----        ------
//   2   1A     Motor 1 Left/CCW    J4-6   GPIO         GPIO   P6.7
//
//
// Dagu BDC Motor:  4.5 - 6.0 Volts     (absolute max = 8.4 v)
//                  No Load:  150ma     Stall Current: 2.75 A at 6V
//                  Measured Motor Resistance (Rload):  5.7 ohms
//
// Hall Sensors:    3.0 - 24.0 Volts    (Open drain, requiring 10K pullups)
// Encoder Disk:    8 pole neodymium magnet
//                  625 state changes per wheel revolution
//
// CAUTION:  MSP432 is _NOT_ 5 volt tolerant !    Max input = 4.0 V on GPIOs
//
// 5V from Launchpad 5V (via USB)  ==> must be tethered to USB cable
// 5V from LM7805 regulator wired to battery pack
//
// 3.3V from Launchpad (via USB)   ==> must be tethered to USB cable
// 3.3V from LM1086-3.3 regulator wired to battery pack
//
// History:
//   05/16/16 - Created as part of Motor Control open source.     Duquaine
//   07/30/16 - Got baseline L6230 code running reliably. Duquaine
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

#include "MotorLib_Api.h"                        // pull in common definitions

void  MC_EXT_button_SixStep (void);
void  EXTI15_10_IRQHandler (void);
void  HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin);
void  USART2_IRQHandler (void);

void  MotorLib_SixStep_StartMotor (uint8_t motor_id, int start_type, int flags);
void  MotorLib_SixStep_StopMotor (uint8_t motor_id, int stop_type, int flags);

    int  motor_mode = BEMF_SENSORLESS;    // HALL_SENSORED
                                          // VARIABLES
    int       do_operational_test = 1;
    int       use_duty_cycle      = 0;         // 1 = use RPS,   0 = duty cycle

    int       motor_app_state     = MOTOR_STATE_STOPPED;
    int       start_stop_pressed  = 1;              // force an initial start w/o button wait

    int       rc = 0;
    int       switch_input         = -1;
    int       prev_switch_input    = -1;
    int       speed_duty_value     = 0;  // computed speed duty value in 0-100 %
    int       speed_prev_adc_value = 0;
    int       speed_adc_complete_callback_value = 0; // speed value in ADC ticks
    int       speed_adc_complete_callback_flag  = 0;
    int       ramp_complete_callback_flag       = 0;

    UART_HandleTypeDef  huart2;          // optional UART VCP connection

           // defines for motor_app_state
#define  MOTOR_STATE_STOPPED   0         // motor is stopped/reset
#define  MOTOR_STATE_STARTUP   1         // motr is going thru startup (Block commutation)
#define  MOTOR_STATE_RUN       2         // motor is up to speed (BEMF commutation)
#define  MOTOR_STATE_FAULT     3         // motor is recovering due to fault condition


           //------------------------------
           //   main() invoked subroutines
           //------------------------------
void  process_hall_sensors(void);                      // RE-THINK THIS ---
void  motor_adc_process_speed_control (int speed_ADC_value);
void  gpio_init_inputs (void);
void  process_direction_switch (int mtr_direction, int do_restart);
int   mcu_clock_init_48mhz (int  use_crystal);
void  systick_init (void);
void  operational_test (void);              // optional - checkout board
void  ramp_done_callback (int motor_num);
void  speed_adc_conversion_done_callback (int motor_id);
int   startup_bemf_sensorless (uint8_t motor_id);
int   startup_hall_sensored (uint8_t motor_id);

void  process_adc_speed_control (int speed_ADC_value);

//int  drv8305_motor_block_commutate (uint8_t motor_id,  int step_num,
//                                    int pwm_duty_count);            // TEST HACK
void BSP_X_NUCLEO_LED_BUTTON_INIT(void);
void BSP_X_NUCLEO_FAULT_LED_ON(void);
void BSP_X_NUCLEO_FAULT_LED_OFF(void);
void MX_USART2_UART_Init (void);
uint32_t  Get_UART_Data();


//******************************************************************************
//  ramp_done_callback
//
//                    Callback by PWM ramp function, once the RAMP is complete.
//******************************************************************************
void  ramp_done_callback (int motor_id)
{
   ramp_complete_callback_flag++;           // denote ramp is complete
}


//******************************************************************************
//  speed_adc_conversion_done_callback
//
//                   Callback by Motor ADC handler, once the ADC reading for
//                   Speed Control is complete.
//
//                   Since ADC's like to be "twichty" in the low order 2 bits,
//                   do a hysterisis check, and only update the speed value
//                   if it changed by more than 4 ticks.
//
// FIX
//    SHOULD place Speed ADC on a separate trigger to sample at
//    much lower rate than current/voltage ADCs (once per 50-100 ms vs 1+ / ms).
//
// If separate sequencer is not available, then put code in ADC
// ISR to only invoke the update once every 50-100 ms, to avoid excess overhead
//******************************************************************************
void  speed_adc_conversion_done_callback (int adc_value)
{
   speed_adc_complete_callback_value = adc_value;

      // perform hysterisis check to see if value changed enough to do an update
   if ((adc_value+4) > speed_prev_adc_value || (adc_value-4) < speed_prev_adc_value)
      {     // yes, tell main() we have a new speed value
        speed_adc_complete_callback_flag++;  // denote new ADC value is complete
        speed_prev_adc_value = adc_value;
      }
}



//******************************************************************************
//  main
//         Overall Logic:
//          - Initialize ADC
//          - Initialize GPIOs for outputs  (4 total for Motor 1A/2A/3A/4A)
//          - Initialize GPIOs for inputs   (Button and Hall Sensors)
//          - Initialize PWMs for Motor
//
//         Main Loop
//          - Check slider button for forward/reverse setting change.
//            Update Motor 1A/2A/3A/4A settings as required.
//          - Check if ADC value has changed. Ratio it 0:4096 = 0:100 %
//            and change PWM duty values as required.
//          - Check Hall Sensors and update current speed/velocity value,
//            as well as approx distance travelled fwd/backwd.
//******************************************************************************
void  main (void)
 {
    board_clock_init (MCU_SPEED, 0);

    board_systick_init();     // set up 1 ms systick Timer period

       /* Enable the FPU for floating point operation */
/// MAP_FPU_enableModule();
/// MAP_FPU_enableLazyStacking();

#if LATER
          // Configure inputs for Fwd/Reverse slider button input
          //     Fwd/Reverse Slider   J1-4    PB1  GPIO    Grove J1-3    -> J1-4
          // initialize GPIO switch used for motor direction
    SWITCH1_CONFIG;      // configure Forward/Reverse slider switch J1-4 P3.3

//#if defined(TEST)
// Test the above to ensure is working           --    WORKS  04/02/16
    rc = SWITCH1_READ;   // hardwire to Gnd
    rc = SWITCH1_READ;   // hardwire to +3.3
#endif

    BSP_X_NUCLEO_LED_BUTTON_INIT(); // Initialize User LED and Push Button

    MX_USART2_UART_Init();          // Initialize UART VCP connection to PC


// -- FUTURE -- add bi-directional vs uni-directional flag --
// -- FUTURE -- Some of parameter stuff needs to be be put in motor_config.h file (Posts/Pins/Mappings)
    MotorLib_Init_Motor (0, PWM_SPEED, MOTOR_POLE_PAIRS); // setup motor 1 PWM for 20 kHz period (50 usec)


//---------  TEMP DEBUG HACK  BEGIN   07/26/16

          // MotorLib_Init_Motor() call above does all the MC_Lib init() stuff. So turn on interrupts
          // and wait for Timer and EXT interrupts just like STM32's sample BLDC code.
    board_enable_global_interrupts();    // init is complete. turn on interrupts
    while (1)
      {   // all work is done in the ISRs   This cures the HardFault error, but is now
          // hanging in B SysTick_Handler loop, because SysTick_Handler is not defined in main code
      }

//---------  TEMP DEBUG HACK  END     07/26/16


    MotorLib_Init_Motor (1, PWM_SPEED, MOTOR_POLE_PAIRS); // setup motor 2 PWM for 20 kHz period (50 usec)

    MotorLib_Encoder_Init (0, ENCODER_HALL_RIGHT_WHEEL);
    MotorLib_Encoder_Init (1, ENCODER_HALL_LEFT_WHEEL);

// ??? should this be by motor - e.g. what if 3-D printer w/multiple-axis ?  What does PRO-MOTION do ?
// Is thie following the cause of the HardFault error, or is it the Encode_Init EXTI setups ?
    MotorLib_Adc_Init_Speed_Control (15, speed_adc_conversion_done_callback); // arbitrary ADC channel

    board_enable_global_interrupts();    // init is complete. turn on interrupts
// blows up right after this point on a HardFault error    07/26/16

       //----------------------------------------------------------
       // During initial board checkout, run HW operational tests
       //----------------------------------------------------------
    if (do_operational_test)
       {
         motor_HW_test (0, 25);   // run test, with PWMs at 25 %
         operational_test();
       }

    MotorLib_Set_Direction (0, DIRECTION_CW);  //  default start in clockwise direction

       //------------------------------------------------------
       //         Main loop to run and control BLDC motor
       //------------------------------------------------------
    while (1)
      {
             //----------------------------------------------------------------
             // Check if speed potentiometer setting has been changed.
             // If so, use its new value to set desired speed for the motor.
             //----------------------------------------------------------------
        if (speed_adc_complete_callback_flag)
           {    // process a new speed value, read in from ADC
             speed_adc_complete_callback_flag = 0;        // clear for next pass
             process_adc_speed_control (speed_adc_complete_callback_value); // process new value
           }

             //-----------------------------------------------------------------
             // Check if direction has been changed, on slider switch P6.0.
             // If so, use its new value to set desired direction for the motor
             //-----------------------------------------------------------------
#if FIX_LATER
        switch_input = SWITCH1_READ;      // get latest setting of slider switch
#endif
        if (switch_input != prev_switch_input)
           {      // the switch has been changed. process the new direction
             if (switch_input == 0)
                process_direction_switch (DIRECTION_CW, 1);
                else process_direction_switch (DIRECTION_CCW, 1);
             prev_switch_input = switch_input;     // save new direction setting
           }

             //-------------------------------------------------------------
             // Based on Motor state, perform any needed actions
             //-------------------------------------------------------------
        switch (motor_app_state)
          {
            case MOTOR_STATE_STOPPED:         // motor is stopped/reset
                    if (start_stop_pressed == 1)
                       {    // Start has been requested. Invoke startup logic
                        if (motor_mode == HALL_SENSORED)
                            {     // startup the motor using HAL encoders to detect position
//                            MotorLib_Set_Direction (0, DIRECTION_CW);   // ??? shouldn't this be on startup() call above ?
                              rc =  MotorLib_Startup_Hall_Sensored (0);   // do I need to also supply a max duty or speed ?
                            }
                           else if (motor_mode == ENCODER_SENSORED)  // is this feasible ?
                            {
                            }
                           else {       // motor mode must == BEMF_SENSORLESS
                                  rc = startup_bemf_sensorless (0); // do I need to also supply a max duty or speed ?
                                }
                         start_stop_pressed = 2;   // set flag to 2 to avoid multiple actions per button press
                         motor_app_state = MOTOR_STATE_STARTUP;
                       }
                    break;

            case MOTOR_STATE_STARTUP:         // motor is going thru startup (Block commutation)
                    if (motor_check_is_started(0))
                       motor_app_state = MOTOR_STATE_RUN;  // we have completed startup
                    break;

            case MOTOR_STATE_RUN:             // motor is up to speed (BEMF commutation)
                    if (start_stop_pressed == 1)
                       {     // stop has been requested
                         MotorLib_Stop (0, STOP_SOFT_BRAKE);
                         start_stop_pressed = 2;   // set flag to 2 to avoid multiple actions per button press
                         motor_app_state = MOTOR_STATE_STOPPED;
                       }
                    break;

            case MOTOR_STATE_FAULT:           // motor is recovering due to fault condition

                    break;
          }                                   // end  switch()

             //----------------------------------------------------------------
             // process any HALL sensor inputs, update RPS (rev/sec) as needed.
             //----------------------------------------------------------------
        process_hall_sensors();               // or do in rupt handler ? ? ?
      }                                       //  end  while()
}


//******************************************************************************
//  motor_adc_process_speed_control
//
//           Check if ADC value has changed. If so, ratio it 0:16384 = 0:100 %
//           and then change PWM duty values as required.
//           This uses integers ("cave man calculus"/cmc) to compute the values.
//******************************************************************************
void  process_adc_speed_control (int speed_ADC_value)
{
    float     scaled_value_flt;
    int       duty_per_cent_value;
    uint16_t  rpm_speed;

        // scale the value to be 0-100 %, from ADC (10-bit) range 0-1023
        //                                  or      12-bit  range 0-4095
    scaled_value_flt    = ((float) speed_ADC_value * 100.0) / ADC_SCALING_VALUE;
    duty_per_cent_value = (int) (scaled_value_flt + 0.5);    // round up result
    if (duty_per_cent_value > 100)
       duty_per_cent_value = 100;

    if (use_duty_cycle)
       {       //---------------------------------------------------------------
               // Use duty value for speed setting (good - BDC, crappy - BLDC)
               // If the 0 to 100 % value has changed, update to new speed value
               //---------------------------------------------------------------
         if (duty_per_cent_value != speed_duty_value)
            {    // ADC duty value has changed: update to new duty speed setting
              MotorLib_Set_Speed (0, (uint16_t) duty_per_cent_value, SPEED_PERCENT_DUTY);
                 // and save it as new speed value
              speed_duty_value = duty_per_cent_value;  // save
            }
       }
      else
       {       //------------------------------------------------------------
               //          Use RPM value instead for speed.
               //
               // multiply by 40 to yield 0 to 4000, which is pretty close to
               // max range of motor's rated speed, which is 4000 RPM.
               //------------------------------------------------------------
         rpm_speed = (duty_per_cent_value * 40);
         MotorLib_Set_Speed_Rpm (0, rpm_speed);
       }
}


//******************************************************************************
//  process_direction_switch
//
//           Check if motor direction (forward/reverse) switch has changed.
//           If so, stop both motors, change the direction as required,
//           then optionally, restart the motors in the new direction.
//
//     Parm
//        mtr_direction:   0 = Forward,   1 = Backward
//******************************************************************************
void  process_direction_switch (int mtr_direction,  int do_restart)
{
    MotorLib_Stop (0, STOP_SOFT_BRAKE);        // stop motors when chg direction

        // Update motor CW/CCW settings to correspond to new setting.
    MotorLib_Set_Direction (0, mtr_direction);

    if (do_restart)
       MotorLib_Run (0, mtr_direction);    // re-start motors with new direction
}


//******************************************************************************
//  operational_test
//
//        Scope test to verify MSP432 outputs properly working.
//        Step through one statement at a time and monitor on
//        the associated MSP432 Launchpad pins on an oscilloscope.
//        Run this _BEFORE_ you hook up the MSP432 to Breadboard or Boosterpack.
//
// This operational test runs through all the motor APIs to ensure correct
// operation of the motor PWM and GPIO pins.
//     Using SN754410 motor driver, you would check the:
//           ENA1/2, 1A, 2A pins (motor 1)  and  ENA3/4, 3A, 4A pins (motor 2)
//     Using DRV8848 BoosterPack, you would check the:
//           AIN1, AIN2 pins (motor 1)   and   BIN1, BIN2 pins (motor 2)
//******************************************************************************
void  operational_test (void)
{
    int   step_delay;
    int   step_num;
    int   num_revs;
    int   pwm_duty_count;
    long  period_ticks;

    period_ticks   = MotorLib_Get_Period_Ticks (0);
    pwm_duty_count = period_ticks / 4;           // run it with a 25% duty cycle
    MotorLib_Set_Pwm_Duty_Count (0, pwm_duty_count); // set duty cycle/speed/velocity  to use

       //-----------------------------------------------------------------------
       // call low level commutation routine to automatically spin up the motor,
       // using built-in commutation table. Start off slow, then speed it up
       //-----------------------------------------------------------------------
    MotorLib_Set_Direction (0, DIRECTION_CW);  //  default start in clockwise direction
    num_revs   = 20;                  // run a total of 10 rounds
    step_delay = 110;                 // start off with step delay = 0.1 seconds
    MotorLib_Run (0, DIRECTION_CW);   // start the motor
    while (num_revs > 0  &&  step_delay > 0)
      {
           // inner loop
         for (step_num = 1;  step_num <= 6;  step_num++)
           {
// 05/26/16 - on initial pass, this causes first CW step to be 2 (increments by 1 before loads it) instead of 1 !!! ==> skipping initial commutate cycle !
             rc = motor_next_commutate (0);      // commutate to next step
//while (pause_true)  // DEBUG SCOPE TRACE HOOK
  ;
             board_delay_ms (step_delay);       // wait for a bit
             pause_true = 1;                 // force it to wait after each commutate
           }
//         step_delay -= 10;     // reduce the step delay for next pass
         num_revs--;
      }
    MotorLib_Stop (0, STOP_SOFT_BRAKE);

#if defined(SCOPE_ONLY_NO_MOTOR_TEST)
       //-------------------------------------------------------------------
       // call low level commutation routine to manually spin up the motor.
       // Start off slow, then speed it up
       //------------------------------------------------------------------
    MotorLib_Set_Direction (0, DIRECTION_CW);  //  default start in clockwise direction
    num_revs   = 20;        // run a total of 10 rounds
    step_delay = 110;       // start off with step delay = 0.1 seconds
    MotorLib_Run (0, DIRECTION_CW);        // start the motor
    while (num_revs > 0  &&  step_delay > 0)
      {
           // inner loop
         for (step_num = 1;  step_num <= 6;  step_num++)
           {
             rc = drv8305_motor_block_commutate (0,
                                            step_num,  // commutate to next step
                                            pwm_duty_count);
while (pause_true)  // DEBUG SCOPE TRACE HOOK
  ;
             board_delay_ms (step_delay);       // wait for a bit
             pause_true = 1;                 // force it to wait after each commutate
           }
         step_delay -= 10;     // reduce the step delay for next pass
         num_revs--;
      }
    MotorLib_Stop (0, STOP_SOFT_BRAKE);

    board_delay_ms (3000);        // pause for 3 seconds

    MotorLib_Set_Direction (0, DIRECTION_CCW); //  set motor to spin in other direction
          // and repeat the same set of tests
    num_revs   = 10;           // run a total of 20 revs
    step_delay = 110;          // start off with step delay = 0.1 seconds
    MotorLib_Run (0, DIRECTION_CCW);           // restart the motor
    while (num_revs > 0  &&  step_delay > 0)
      {
           // inner loop
         for (step_num = 6;  step_num >= 1;  step_num--)
           {     // directly call low level driver or now
             rc = drv8305_motor_block_commutate (0,
                                            step_num,  // commutate to next step
                                            pwm_duty_count);
while (pause_true)  // DEBUG SCOPE TRACE HOOK
  ;
             board_delay_ms (step_delay);                 // wait for a bit
             pause_true = 1;                 // force it to wait after each commutate
           }
         step_delay -= 10;           // reduce the step delay for next pass
         num_revs--;
      }
    MotorLib_Stop (0, STOP_SOFT_BRAKE);
#endif                               // defined(SCOPE_ONLY_NO_MOTOR_TEST)

}



//******************************************************************************
//  startup_hall_sensored
//
//          Startup the BLDC motor - this does initial spins of motor
//          until it has reached point of adequate BEMF feedback
//******************************************************************************

int  startup_hall_sensored (uint8_t motor_id)
{
    int   src;

    src = MotorLib_Startup_Hall_Sensored (motor_id);

    return (src);
}


//******************************************************************************
//  startup_bemf_sensorless
//
//          Startup the BLDC motor - this does initial spins of motor
//          until it has reached point of adequate BEMF feedback
//******************************************************************************

int  startup_bemf_sensorless (uint8_t motor_id)
{
    int   src;

    src = MotorLib_Startup_Bemf_Sensorless (motor_id);

    return (src);
}




/**************************************************************************
*  BSP_X_NUCLEO_FAULT_LED_ON
*
*                  Turns selected LED On.
*/

void  BSP_X_NUCLEO_FAULT_LED_ON (void)
{
    HAL_GPIO_WritePin (GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
}


/**************************************************************************
*  BSP_X_NUCLEO_FAULT_LED_OFF
*
*                  Turns selected LED Off.
*/

void  BSP_X_NUCLEO_FAULT_LED_OFF (void)
{
    HAL_GPIO_WritePin (GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
}


//------------------------------------------------------------------------------
//  User LED and USER BUTTON init function
//------------------------------------------------------------------------------

void  BSP_X_NUCLEO_LED_BUTTON_INIT (void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

       // Configure USER LED GPIO pin :  e.g. PA5
    GPIO_InitStruct.Pin   = USER_LED_GPIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init (USER_LED_GPIO_PORT, &GPIO_InitStruct);

       // Configure USER BUTTON GPIO pin : e.g. PC13
    GPIO_InitStruct.Pin  = USER_BUTTON_GPIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init (USER_BUTTON_GPIO_PORT, &GPIO_InitStruct);

       // EXTI interrupt init - setup IRQ for USER BUTTON press
       //   Set Priority of External Line Interrupt for USER Button press interrupt
    HAL_NVIC_SetPriority (USER_BUTTON_IRQn, 0, 0);
       //   Enable the External Line Interrupt for USER Button press interrupt
    HAL_NVIC_EnableIRQ (USER_BUTTON_IRQn);
}


//------------------------------------------------------------------------------
//  USART2 init function
//------------------------------------------------------------------------------
void  MX_USART2_UART_Init (void)
{

  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = 19200;
  huart2.Init.WordLength   = UART_WORDLENGTH_9B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_ODD;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init (&huart2);

}


/*************************************************************************
*  HAL_UART_MspInit
*                           Initializes UART
*/

void  HAL_UART_MspInit (UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (huart->Instance==USART2)
   {
         /* Peripheral clock enable */
    __USART2_CLK_ENABLE();

         /** USART2 GPIO Configuration
         *     PA2     ------> USART2_TX
         *     PA3     ------> USART2_RX
         */
    GPIO_InitStruct.Pin   = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

         /* System interrupt init*/
    HAL_NVIC_SetPriority (USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ (USART2_IRQn);
   }

}


/*******************************************************************************
*  Get_UART_Data
*                   Get the UART value from DR register
*/  
//PS  uint32_t  Get_UART_Data()
//PS  {
//PS     return (UART.Instance->DR);
//PS  }



/*******************************************************************************
*                         Handles USART2 global interrupt.
*                                                                  USART2_IRQn
*/
void  USART2_IRQHandler (void)
{
  HAL_UART_IRQHandler (&huart2);

#ifdef UART_COMM
  UART_Set_Value();
#endif

}


/**************************************************************************
*                         CHANGE THIS IN FUTURE  !!!
*
*  MC_EXT_button_SixStep
*
*        GPIO EXT Callback - Start or Stop the motor through the Blue push button on STM32Nucleo
*                       Invoked from HAL_GPIO_EXTI_Callback()  ISR CALLBACK
*/
void   MC_StartMotor (uint8_t motor_id, int start_type, int flags);
void   MC_StopMotor (uint8_t motor_id, int stop_type, int flags);
extern uint8_t   Enable_start_button;
       int       motor_start_press = 0;   // 0 = off, 1 = on

void  MC_EXT_button_SixStep (void)        // WVD - GOES THRU HERE ON BUTTON PUSH
{
//  if (Enable_start_button == TRUE)      // is always set to false. not being reset by STM32 MC code
       {    // Alternating press of button either starts or stops motor
         if (motor_start_press == 0)
            {
              MotorLib_SixStep_StartMotor (0, 0, 0);     // ??? TEMP TEST HACK  // WVD GOES THRU HERE on push
              Enable_start_button = FALSE;
              motor_start_press = 1;       // set for next pass
            }
           else
            {
              MotorLib_SixStep_StopMotor (0, 0, 0);      // ??? TEMP TEST HACK
              Enable_start_button = FALSE;
              motor_start_press = 0;       // set for next pass
            }
      }
}


/*******************************************************************************
*                   Handle  EXTI Line[15:10] interrupts.   USER BUTTON press
*                                                           EXTI15_10_IRQn
*/
void  EXTI15_10_IRQHandler (void)
{
    HAL_GPIO_EXTI_IRQHandler (USER_BUTTON_GPIN);
}



/*******************************************************************************
*                                                       BOARD SPECIFIC - F4_01
*                        External Line Callback  ISR          FLAG/FAULT pin
*
* @param[in] GPIO_Pin pin number
* @retval None
*******************************************************************************/
extern void  Encoder_GPIO_EXTI_Callback (uint16_t GPIO_Pin);
       void  L6230_FlagInterruptHandler(void);

void  HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
     // 07/29/16 - this is NOT being called. Encoder_GPIO_EXTI_Callback() is being invoked instead.
  if (GPIO_Pin == GPIO_PIN_10)                  // Fault_Flag  pin PA10 / D2
     {
       L6230_FlagInterruptHandler();   // is FAULT ONLY on L6474 STEPPER ONLY ??? - No such FAULT Rupt code in L6230 Ref Code !
     }                                 // Never being called by L6230  07/29/16

  if (GPIO_Pin == 0x2000)              // USER Button on STM32 F4_01 = 0b0010 0000 0000 0000
     MC_EXT_button_SixStep();          // this _IS_ being called on both code sets. 07/29/16

// else Encoder_GPIO_EXTI_Callback (GPIO_Pin);  // in BLDC_motor_ctl_ISRs_xx.c
}


//******************************************************************************
