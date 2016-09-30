
// CAUTION:  L6206 - requires a minimum of 8V, so the 6V DAGU motors WILL NOT WORK with L6206 !
//           ^^^^^              ^^^^^^^^^^^^^         ^^^^^^^^^^^^^^      ^^^^^^^^

// (1) ENCODERS: not seeing any signal on encoder signal line, even on Oscscope.
//           ==> pullups on MSP432 are not strong enough.
//           ==> need to run 5V to pullups and power, divide down for MSP432 GPIO
//           ==> need to do standalone diagnostics of Encoders.
//
// (2) DRV8848   Current Control VRef ADC is not properly being read.   04/16/16
//           ==> problem in sequencing/logic of ADC14 for multi-channel.
//
// (3) Need to revisit motor_stop() always reseting dty_cycle to 0. Is that consistent with other Motor Driver packages ?
//     What does SPN4 do, and what does TI' BDC (MSDP430) do ?   Ditto Arduino Ada-lib  (Margolois)
//
// (4) Getting glitching during main() while() when switch motor direction on
//     MSP430.  Restarts with motors full out, and have to ramp ADC up/Down
//     to get things to reset.  Also ADC range appears reduced in FORWARD mode
//     in such cases.
//    Also, need to tape bottom of Grove so it does not touch/short FR5969 JTAG.

                            // DEBUG Intercepts - Tiva PWMs stop on debug halts
volatile int  wait_0 = 1;   // PWM Loop Testy       Motor 1
volatile int  wait_1 = 0;   // Forward Slow Decay   Motor 1
volatile int  wait_2 = 0;   // Reverse Slow Decay   Motor 1
volatile int  wait_3 = 0;   // Peak 90% Slow Decay  Motor 1
volatile int  wait_4 = 0;   // Forward Fast Decay   Motor 1
volatile int  wait_5 = 0;   // Reverse Fast Decay   Motor 1
volatile int  wait_6 = 0;   // Forward Slow Decay   Motor 2          -------
volatile int  wait_7 = 0;   // Reverse Slow Decay   Motor 2
volatile int  wait_8 = 0;   // Forward RAMP UP Slow Decay   Motor 1  -------
volatile int  wait_9 = 0;   // Forward RAMP DOWN Slow Decay Motor 1

//*******1*********2*********3*********4*********5*********6*********7**********
//
//                                  main_BDC.c
//
// Controls 2 Brushed DC motors, including forward/reverse and speed control.
//
// Has been tested with the SparkFun "Shadow" Chassis (ROB-13301),
// with Dagu 200 rpm gear motors (ROB-13302), 65 mm wheels (ROB-13259),
// and Hall sensor wheel encoders (ROB-12629).
// This is a complete, yet fairly inexpensive platform (USD $ 33).
//
// Works with the following BDC controllers, based upon the device selected
// in motor_config.h file:
//     - L6239 H-Bridge IC,
//     - TI SN754410 H-Bridge IC,
//     - DRV8848 Dual H-Bridge controller. used on the TI DRV8848 Boosterpack.
//     - L6206   Dual H-Bridge controller. used on the STM32 Xnucleo dual brush
//               expansion boards.
//
// 6 Volts is supplied to DC motors via BDC Controller.
// Motors are driven by PWMs, which are used to modulate the speed.
// Forward and reverse (CW, CCW) are controlled through 1A/2A amd 3A/4A pins.
//
// Note: the (somewhat) heavy Dragu motors and wheels require the following:
//            - SLOW decay: duty cycle of at least 30 %
//            - FAST decay: duty cycle of at least 60 %
//       before the motors are able to move the robot.
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
//   04/01/16 - Created as part of Open Source Motor Control project.
//   04/02/16 - Created and tested all base routines (ADC, PWM, GPIO In/Out,
//              SysTick) except Hall encoder pins. Also did motor start/stop/
//          set_direction/go_left/go_right, process_adc_speed_control. Duquaine
//   04/05/16 - Rolled in support for Hall Sensors (attached to wheels).Duquaine
//   04/08/16 - Split off motor ctlr (SN754410 vs DRV8848)  specific code.Duquaine
//   04/16/16 - Basic forward/reverse + speed control working on MSP432.Duquaine
//   05/09/16 - Got fully working on MSP432 Launchpad with DRV8848.
//   05/12/16 - Got fully working on Tiva 123G Launchpad with DRV8848.
//   05/14/16 - Got fully working on STM32-F4-46 with XNucleo L6206.
//   05/16/16 - Got fully working on MSP430-FR5959 Launchpad with DRV8848.
//   09/03/16 - Finally got working on MSP430-G2553 Launchpad with DRV8848,
//              after working around a CCS compiler optimizer bug.
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


    int       mtr_1_direction_local = DIRECTION_FORWARD;  // current direction
    int       mtr_2_direction_local = DIRECTION_FORWARD;

                                           // VARIABLES
    int       rc;
//  int       speed_duty_value    = 0;     // speed duty value in ticks
    int       ramp_complete_callback_flag = 0;
    int       do_operational_test = 1;

           //------------------------------
           //   main() invoked subroutines
           //------------------------------
void  process_hall_sensors(void);                      // RE-THINK THIS ---
void  motor_adc_process_speed_control (void);            // RE-THINK THIS ---
void  gpio_init_inputs (void);
void  process_direction_switch (int mtr_direction, int do_restart);
int   mcu_clock_init_48mhz (int  use_crystal);
void  systick_init (void);
void  operational_test (void);              // optional - checkout board
void  ramp_done_callback (int motor_num);


//******************************************************************************
//  ramp_done_callback
//
//                    Callback by PWM ramp function, once the RAMP is complete.
//******************************************************************************
void  ramp_done_callback (int motor_num)
{
   ramp_complete_callback_flag++;                // denote ramp is complete
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
    int   switch_input;

       //--------------------------------------------------------------
       // initialize the board's key clocks, and turn on SYSTICK timer.
       // Note that setting the MCU_SPEED parm to a value of 0 denotes
       // "run the chip at it fastest speed".
       //--------------------------------------------------------------
    board_clock_init (MCU_SPEED, 0);

       /* Enable the FPU for floating point operation */
/// MAP_FPU_enableModule();
/// MAP_FPU_enableLazyStacking();

       //--------------------------------------------------
       // initialize motors
       //--------------------------------------------------
//  motor_lib_init (MCU_SPEED);          // initialize base motor library

// -- FUTURE -- add bi-directional vs uni-directional flag --
// -- FUTURE -- Some of parameter stuff needs to be be put in contro_config.hn file (Posts/Pins/Mappings)

    MotorLib_Init_Motor (0, PWM_SPEED, 1); // setup motor 1 PWM for 20 kHz period (50 usec)
// 08/28/16 even after remove memset, it still fucks up syack at end of 2nd  MotorLib_Init_Motor()
//          See if eliminating that helps get rid of stack clobber.  else impies mis-match in motor_block[] size (one thinks is 1 entry, other thinks is 2 entries)
//  MotorLib_Init_Motor (1, 2, PWM_SPEED); // setup motor 2 PWM for 20 kHz period (50 usec)  08/28/16 - loops multiple times thru here !!!
// 09/27/16  blows up (invalid stk ptr) at the tail end of the above call !
    MotorLib_Adc_Init_Speed_Control (15, 0L);   // arbitrary ADC channel
    MotorLib_Encoder_Init (0, ENCODER_HALL_RIGHT_WHEEL);
    MotorLib_Encoder_Init (1, ENCODER_HALL_LEFT_WHEEL);

    board_enable_global_interrupts();    // init is complete. turn on interrupts

       //----------------------------------------------------------
       // During initial board checkout, run the operational tests
       //----------------------------------------------------------
    if (do_operational_test)
       operational_test();

       //-------------------------------------------------------
       //           Setup Motors for normal use.
       //
       // We setup motors to move forward, and enable them, with
       // the speed of the motor PWMs initially set to 0.
       // The speed is modified by reading the Speed ADC value,
       // which is done in the main while() loop below.
       //-------------------------------------------------------
    MotorLib_Set_Direction (0, DIRECTION_FORWARD);
    MotorLib_Set_Direction (1, DIRECTION_FORWARD);
    MotorLib_Set_Decay (0, DECAY_SLOW);    // use SLOW decay (faster speeds)
    MotorLib_Set_Decay (0, DECAY_SLOW);
    MotorLib_Set_Speed_Duty_Cycle (0, 0);        // reset duty cycle to = 0
    MotorLib_Set_Speed_Duty_Cycle (1, 0);

    MotorLib_Run (0, DIRECTION_FORWARD); // enable motors with the new direction
    MotorLib_Run (1, DIRECTION_FORWARD);

       //------------------------------------------------------
       // Main loop to run and control motors
       //------------------------------------------------------
    while (1)
      {
             // check if direction has been changed, on slider switch P6.0
//      switch_input = SWITCH1_READ;  // get curr setting of slider switch P6.0
        if (switch_input == 0)
           process_direction_switch (DIRECTION_FORWARD, 1);
           else process_direction_switch (DIRECTION_BACKWARD, 1);

             //----------------------------------------------------------
             // check if speed potentiometer setting has been changed.
             // If so, use its new value to speed up or slow down motors
             //----------------------------------------------------------
        motor_adc_process_speed_control();           // -- RE-THINK THIS ---

             //----------------------------------------------------------------
             // process any HALL sensor inputs, update RPS (rev/sec) as needed.
             //----------------------------------------------------------------
        process_hall_sensors();             // or do in rupt handler ? ? ?
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
    if (mtr_direction  != mtr_1_direction_local
      && mtr_direction != mtr_2_direction_local)
       {    //------------------------------------------------------------
                // need to change the motor direction for both motors.
                // Update motor CW/CCW settings to correspond to new setting.
                //------------------------------------------------------------
             MotorLib_Stop (0,STOP_SOFT_BRAKE);  // stop motors when chg direction
             MotorLib_Stop (1, STOP_SOFT_BRAKE);    //   WORKS 04/02/16
// CAUTION: the above STOPs reset the PWM duty cycle to 0. Is that consistent with other Motor packages ?

             MotorLib_Set_Direction (0, mtr_direction);
             MotorLib_Set_Direction (1, mtr_direction);
             if (do_restart)
                { MotorLib_Run (0, mtr_direction);  // re-start motors with new direction
                  MotorLib_Run (1, mtr_direction);  //   WORKS 04/02/16
                }
       }
      else if (mtr_direction  != mtr_1_direction_local)
              {        // need to change direction for Motor 1 only
                MotorLib_Stop (0, STOP_SOFT_BRAKE);      // stop motor when change direction
                MotorLib_Set_Direction (0, mtr_direction);
                if (do_restart)
                   MotorLib_Run (0, mtr_direction);  // re-start motor with new direction
              }
      else if (mtr_direction  != mtr_2_direction_local)
              {        // need to change direction for Motor 2 only
                MotorLib_Stop (1, STOP_SOFT_BRAKE);      // stop motor when change direction
                MotorLib_Set_Direction (1, mtr_direction);
                if (do_restart)
                   MotorLib_Run (1, mtr_direction);  // re-start motor with new direction
              }
   mtr_1_direction_local = mtr_direction;
   mtr_2_direction_local = mtr_direction;
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
    int  decay_mode;               //  FAST = 0,   SLOW = 1

    MotorLib_Set_Decay (0, DECAY_SLOW);    // spins fast at 60 % with this setting
//  MotorLib_Set_Direction (0, DIRECTION_FORWARD);   // AIN2_PWM,  AIN1_HIGH
    MotorLib_Set_Speed_Duty_Cycle (0, 80); //   J4-2       J4-1
    MotorLib_Run (0, DIRECTION_FORWARD);   // enable motors with specified direction  - with SLOW decay, motors run flat out at 100 % !!!
    while (wait_0)
      ;                                    // keep running till told to bail out
    board_delay_ms (2000);                 // run for 2 seconds

#if defined(LATER)
       //------------------------------------------------------
       // Motor 1 - PWM TEST   both slow/fast decay variations
       //------------------------------------------------------
    MotorLib_Set_Decay (0, DECAY_SLOW);   // spins fast at 60 % with this setting
    MotorLib_Set_Direction (0, DIRECTION_FORWARD);   // AIN2_PWM,  AIN1_HIGH
    MotorLib_Set_Speed_Duty_Cycle (0, 50);                 //   J4-2       J4-1
    MotorLib_Run (0, DIRECTION_FORWARD);  // enable motors with specified direction  - with SLOW decay, motors run flat out at 100 % !!!
    board_delay_ms (2000);                      // run for 2 seconds

    MotorLib_Set_Speed_Duty_Cycle (0, 0);      // shut off PWM as re-configure

    MotorLib_Set_Decay (0, DECAY_FAST);  //  But just barely starts spinning at 60 % (Slowly) with this setting
    MotorLib_Set_Direction (0, DIRECTION_FORWARD);   // AIN1_PWM,  AIN2_LOW
    MotorLib_Set_Speed_Duty_Cycle (0, 65);                 //   J4-1       J4-2
    MotorLib_Run (0, DIRECTION_FORWARD);  // enable motors with specified direction  - with SLOW decay, motors run flat out at 100 % !!!
    board_delay_ms (2000);                      // run for 2 seconds

    MotorLib_Set_Speed_Duty_Cycle (0, 0);      // shut off PWM as re-configure

       // Motor 1 - REVERSE   both slow/fast decay variations
    MotorLib_Set_Decay (0, DECAY_SLOW);  // spins fast at 60% with this setting
    MotorLib_Set_Direction (0, DIRECTION_BACKWARD);   // AIN1_PWM,  AIN2_HIGH
    MotorLib_Set_Speed_Duty_Cycle (0, 50);                 //   J4-1       J4-2
    MotorLib_Run (0, DIRECTION_BACKWARD);  // enable motors with specified direction  - with SLOW decay, motors run flat out at 100 % !!!
    board_delay_ms (2000);                      // run for 2 seconds

    MotorLib_Set_Speed_Duty_Cycle (0, 0);      // shut off PWM as re-configure

    MotorLib_Set_Decay (0, DECAY_FAST);  //  But just barely starts spinning at 60 % (Slowly) with this setting
    MotorLib_Set_Direction (0, DIRECTION_BACKWARD);   // AIN2_PWM,  AIN1_LOW
    MotorLib_Set_Speed_Duty_Cycle (0, 65);                 //   J4-2       J4-1
    MotorLib_Run (0, DIRECTION_BACKWARD);  // enable motors with specified direction  - with SLOW decay, motors run flat out at 100 % !!!
    board_delay_ms (2000);                      // run for 2 seconds

    MotorLib_Set_Speed_Duty_Cycle (0, 0);      // shut off PWM as re-configure

       //------------------------------------------------------
       // Motor 2 - PWM TEST   both slow/fast decay variations
       //------------------------------------------------------
    MotorLib_Set_Decay (1, DECAY_SLOW);  // spins fast at 60% with this setting
    MotorLib_Set_Direction (1, DIRECTION_FORWARD);   // BIN2_PWM,  BIN1_HIGH
    MotorLib_Set_Speed_Duty_Cycle (1, 50);                 //   J4-5       J4-4
    board_delay_ms (2000);                      // run for 2 seconds

    MotorLib_Set_Speed_Duty_Cycle (1, 0);      // shut off PWM as re-configure

    MotorLib_Set_Decay (1, DECAY_FAST);  //  But just barely starts spinning at 60 % (Slowly) with this setting
    MotorLib_Set_Direction (1, DIRECTION_FORWARD);   // BIN1_PWM,  BIN2_LOW
    MotorLib_Set_Speed_Duty_Cycle (1, 65);                 //   J4-4       J4-5
    board_delay_ms (2000);                      // run for 2 seconds

    MotorLib_Set_Speed_Duty_Cycle (1, 0);      // shut off PWM as re-configure

       // Motor 2 - REVERSE   both slow/fast decay variations
    MotorLib_Set_Decay (1, DECAY_SLOW);  // spins fast at 60% with this setting
    MotorLib_Set_Direction (1, DIRECTION_REVERSE); // BIN1_PWM,  BIN2_HIGH
    MotorLib_Set_Speed_Duty_Cycle (1, 50);                  //   J4-4       J4-5
    board_delay_ms (2000);                      // run for 2 seconds

    MotorLib_Set_Speed_Duty_Cycle (1, 0);      // shut off PWM as re-configure

    MotorLib_Set_Decay (1, DECAY_FAST); //  But just barely starts spinning at 60 % (Slowly) with this setting
    MotorLib_Set_Direction (1, DIRECTION_REVERSE);   // BIN2_PWM,  BIN1_LOW
    MotorLib_Set_Speed_Duty_Cycle (1, 65);           //   J4-5       J4-4
    board_delay_ms (2000);                        // run for 2 seconds

    MotorLib_Set_Speed_Duty_Cycle (1, 0);      // shut off PWM as re-configure
#endif

       //------------------------------------------------------
       //             MOTOR  CHECKOUT  TEST
       //
       // initialize in Forward direction, but duty set to 0
       //------------------------------------------------------
    MotorLib_Set_Direction (0, DIRECTION_FORWARD);
    MotorLib_Set_Direction (1, DIRECTION_FORWARD);
    decay_mode = DECAY_SLOW;              // FAST = 0,   SLOW = 1
    MotorLib_Set_Decay (0, decay_mode);   // spins fast at 60% with this setting
    MotorLib_Set_Decay (1, decay_mode);   // spins fast at 60% with this setting
    MotorLib_Set_Speed_Duty_Cycle (0, 0); // reset duty cycle = 0
    MotorLib_Set_Speed_Duty_Cycle (1, 0);
    MotorLib_Run (0, DIRECTION_FORWARD);  // enable motors with specified direction  - with SLOW decay, motors run flat out at 100 % !!!
    MotorLib_Run (1, DIRECTION_FORWARD);

       //------------------------------------------------------
       //             MOTOR 1  CHECKOUT  TEST
       //  -- single step thru each statement under debugger and note effects --
       //------------------------------------------------------
    MotorLib_Set_Speed_Duty_Cycle (0, 20);          // set duty cycle = 20 %
while (wait_1)
  ;            // DEBUG HACK
    MotorLib_Set_Speed_Duty_Cycle (0, 30);          // set duty cycle = 30 %
    MotorLib_Set_Speed_Duty_Cycle (0, 40);          // set duty cycle = 40 %
    MotorLib_Set_Speed_Duty_Cycle (0, 50);          // set duty cycle = 50 %
    MotorLib_Set_Speed_Duty_Cycle (0, 60);          // set duty cycle = 60 % barely moves when decay = FAST
    MotorLib_Set_Speed_Duty_Cycle (0, 70);          // set duty cycle = 70 %
    MotorLib_Set_Speed_Duty_Cycle (0, 80);          // set duty cycle = 80 %
    MotorLib_Set_Speed_Duty_Cycle (0, 90);          // set duty cycle = 90 %
    MotorLib_Stop (0, STOP_HARD_BRAKE);

    MotorLib_Set_Direction (0, DIRECTION_BACKWARD); // spin motor other direction
    MotorLib_Set_Speed_Duty_Cycle (0, 20);          // set duty cycle = 20 %
    MotorLib_Run (1, DIRECTION_BACKWARD);
while (wait_2)
  ;            // DEBUG HACK
    MotorLib_Set_Speed_Duty_Cycle (0, 30);          // set duty cycle = 30 %
    MotorLib_Set_Speed_Duty_Cycle (0, 40);          // set duty cycle = 40 %
    MotorLib_Set_Speed_Duty_Cycle (0, 50);          // set duty cycle = 50 %
    MotorLib_Set_Speed_Duty_Cycle (0, 60);          // set duty cycle = 60 %  nneds at least 60% juice to run !  (slow when decay = FAST)
    MotorLib_Set_Speed_Duty_Cycle (0, 70);          // set duty cycle = 70 %
    MotorLib_Set_Speed_Duty_Cycle (0, 80);          // set duty cycle = 80 %
    MotorLib_Set_Speed_Duty_Cycle (0, 90);          // set duty cycle = 90 %
    MotorLib_Stop (0, STOP_HARD_BRAKE);
while (wait_3)
  ;            // DEBUG HACK

//  MotorLib_Set_Direction (0, DIRECTION_FORWARD); // and flip direction again
    MotorLib_Set_Speed_Duty_Cycle (0, 70);      // set duty cycle = 70 %
    MotorLib_Run (1, DIRECTION_FORWARD);
    process_direction_switch (DIRECTION_BACKWARD,0);   // and flip direction again, leave motors off
    MotorLib_Run (0, DIRECTION_BACKWARD);       // explicitly start just Motor 1 - STM32 may have issue - is not auto-restartring PWM
    MotorLib_Set_Speed_Duty_Cycle (0, 70);      // set duty cycle = 70 %
    MotorLib_Stop (0, STOP_SOFT_BRAKE);         // totally stop the PWM
                          // the Stop also resets Duty_Cycle = 0. Is that consistent with other Robot packages ?

       // retry motor 1 initial test, but with DECAY_FAST
    MotorLib_Set_Direction (0, DIRECTION_FORWARD);
    MotorLib_Set_Decay (0, DECAY_FAST);
    MotorLib_Set_Speed_Duty_Cycle (0, 20);      // set duty cycle = 20 %
    MotorLib_Run (0, DIRECTION_FORWARD);        // restart the motor
while (wait_4)
  ;            // DEBUG HACK
    MotorLib_Set_Speed_Duty_Cycle (0, 30);      // set duty cycle = 30 %
    MotorLib_Set_Speed_Duty_Cycle (0, 40);      // set duty cycle = 40 %
    MotorLib_Set_Speed_Duty_Cycle (0, 50);      // set duty cycle = 50 %
    MotorLib_Set_Speed_Duty_Cycle (0, 60);      // set duty cycle = 60 %   // slowly starts turning
    MotorLib_Set_Speed_Duty_Cycle (0, 70);      // set duty cycle = 70 %
    MotorLib_Set_Speed_Duty_Cycle (0, 80);               // set duty cycle = 80 %
    MotorLib_Set_Speed_Duty_Cycle (0, 90);               // set duty cycle = 90 %
    MotorLib_Stop (0, STOP_SOFT_BRAKE);

    MotorLib_Set_Direction (0, DIRECTION_BACKWARD);
    MotorLib_Set_Decay (0, DECAY_FAST);
    MotorLib_Set_Speed_Duty_Cycle (0, 20);               // set duty cycle = 20 %
    MotorLib_Run (0, DIRECTION_BACKWARD);       // restart the motor
while (wait_5)
  ;            // DEBUG HACK
    MotorLib_Set_Speed_Duty_Cycle (0, 30);               // set duty cycle = 30 %
    MotorLib_Set_Speed_Duty_Cycle (0, 40);               // set duty cycle = 40 %
    MotorLib_Set_Speed_Duty_Cycle (0, 50);               // set duty cycle = 50 %
    MotorLib_Set_Speed_Duty_Cycle (0, 60);               // set duty cycle = 60 %   // slowly starts turning
    MotorLib_Set_Speed_Duty_Cycle (0, 70);               // set duty cycle = 70 %
    MotorLib_Set_Speed_Duty_Cycle (0, 80);               // set duty cycle = 80 %
    MotorLib_Set_Speed_Duty_Cycle (0, 90);               // set duty cycle = 90 %
    MotorLib_Stop (0, STOP_SOFT_BRAKE);

       //------------------------------------------------------
       //             MOTOR 2  CHECKOUT  TEST
       //  -- single step thru each statement under debugger and note effects --
       //------------------------------------------------------
    MotorLib_Set_Decay (1, DECAY_SLOW);         // ensure is in SLOW decay
    process_direction_switch (DIRECTION_FORWARD,0); // reset back to FORWARD, leave motors off
    MotorLib_Set_Speed_Duty_Cycle (1, 20);          // set duty cycle = 20 %
    MotorLib_Run (1, DIRECTION_FORWARD);        // explicitly start just Motor 2
while (wait_6)
  ;            // DEBUG HACK
    MotorLib_Set_Speed_Duty_Cycle (1, 30);          // set duty cycle = 30 %  Slow Decay starts with 30 %
    MotorLib_Set_Speed_Duty_Cycle (1, 40);          // set duty cycle = 40 %
    MotorLib_Set_Speed_Duty_Cycle (1, 50);          // set duty cycle = 50 %
    MotorLib_Set_Speed_Duty_Cycle (1, 60);          // set duty cycle = 60 %  Fast Decay starts with 60 %
    MotorLib_Set_Speed_Duty_Cycle (1, 70);          // set duty cycle = 70 %
    MotorLib_Set_Speed_Duty_Cycle (1, 80);          // set duty cycle = 80 %
    MotorLib_Set_Speed_Duty_Cycle (1, 90);          // set duty cycle = 90 %
    MotorLib_Stop (1, STOP_HARD_BRAKE);

    MotorLib_Set_Direction (1, DIRECTION_BACKWARD); // spin motor other direction
    MotorLib_Set_Speed_Duty_Cycle (1, 20);          // set duty cycle = 20 %
    MotorLib_Run (1, DIRECTION_BACKWARD);       // explicitly start just Motor 2
while (wait_7)
  ;            // DEBUG HACK
    MotorLib_Set_Speed_Duty_Cycle (1, 30);          // set duty cycle = 30 %  Slow Decay starts with 30 %
    MotorLib_Set_Speed_Duty_Cycle (1, 40);          // set duty cycle = 40 %
    MotorLib_Set_Speed_Duty_Cycle (1, 50);          // set duty cycle = 50 %
    MotorLib_Set_Speed_Duty_Cycle (1, 60);          // set duty cycle = 60 %  Fast Decay starts with 60 %
    MotorLib_Set_Speed_Duty_Cycle (1, 70);          // set duty cycle = 70 %
    MotorLib_Set_Speed_Duty_Cycle (1, 80);          // set duty cycle = 80 %
    MotorLib_Set_Speed_Duty_Cycle (1, 90);          // set duty cycle = 90 %
    MotorLib_Stop (1, STOP_HARD_BRAKE);

    MotorLib_Set_Direction (1, DIRECTION_FORWARD);   // and flip direction again
    MotorLib_Set_Speed_Duty_Cycle (1, 70);           // set duty cycle = 70 %
    process_direction_switch (DIRECTION_BACKWARD,1); // and flip direction again -- STM32 CAUSED A FLAKE/GLITCH on output PWM -- with auto-restart
    MotorLib_Set_Speed_Duty_Cycle (1, 70);           // set duty cycle = 70 %    -- STM32 which then gets cured on this next set_duty_cycle --
    MotorLib_Stop (1, STOP_SOFT_BRAKE);              // totally stop the PWM
    MotorLib_Stop (0, STOP_SOFT_BRAKE);              // totally stop the PWM

       //---------------------------------------
       // Test  Motor 1  RAMP_UP and RAMP_DOWN
       //---------------------------------------
    MotorLib_Set_Decay (0, DECAY_SLOW);          // use SLOW decay (faster speeds)
    process_direction_switch (DIRECTION_FORWARD,0);  // reset back to FORWARD, leave motors off
    ramp_complete_callback_flag = 0;                 // clear RAMP complete flag

        // ramp up Motor 1 from 10% to 90% in 3 seconds. Update PWM every 30 ms
    MotorLib_Speed_Ramp (0, RAMP_UP, 10, 90, 3000, 30, &ramp_done_callback);
    while (ramp_complete_callback_flag == 0)
      ;     // loop till done          ==> uses callback
while (wait_8)
  ;            // DEBUG HACK

    board_delay_ms (2000);                    // wait for 2 seconds before next run
    ramp_complete_callback_flag = 0;       // clear RAMP complete flag

        // ramp down Motor 1 from 90% to 10% in 3 seconds. Update PWM every 30 ms
    MotorLib_Speed_Ramp (0, RAMP_DOWN, 90, 10, 3000, 30, NULL_MPTR);
    while (MotorLib_Speed_Ramp_Check_Complete(0) == 0)
      ;     // loop till done          ==> polls for complete, no callback
while (wait_9)
  ;            // DEBUG HACK

    board_delay_ms (2000);                    // wait for 2 seconds before next run
    ramp_complete_callback_flag = 0;       // clear RAMP complete flag

       //---------------------------------------
       // Test  Motor 2  RAMP_UP and RAMP_DOWN
       //---------------------------------------
    MotorLib_Set_Decay (1, DECAY_SLOW);          // use SLOW decay (faster speeds)
        // ramp up Motor 2 from 10% to 90% in 3 seconds. Update PWM every 5 ms
    MotorLib_Speed_Ramp (1, RAMP_UP, 10, 90, 3000, 5, &ramp_done_callback);
    while (ramp_complete_callback_flag == 0)
      ;     // loop till done          ==> uses callback

    board_delay_ms (2000);                    // wait for 2 seconds before next run
    ramp_complete_callback_flag = 0;       // clear RAMP complete flag

        // ramp down Motor 2 from 90% to 10% in 3 seconds. Update PWM every 5 ms
    MotorLib_Speed_Ramp (1, RAMP_DOWN, 90, 10, 3000, 5, NULL_MPTR);
    while (MotorLib_Speed_Ramp_Check_Complete(1) == 0)
      ;     // loop till done          ==> polls for complete, no callback

    board_delay_ms (2000);                    // wait for 2 seconds before next run
    ramp_complete_callback_flag = 0;       // clear RAMP complete flag

    MotorLib_Stop (0, STOP_SOFT_BRAKE); // Ensure both motors are totally stopped
    MotorLib_Stop (1, STOP_SOFT_BRAKE);
}

//******************************************************************************
