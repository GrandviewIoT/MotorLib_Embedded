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
//   04/01/16 - Created as part of Ground 0 Motor Skill set. High level sketch.
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
//******************************************************************************

#include "BDC_motor_ctl.h"                       // pull in common definitions

//extern  int  mtr_1_ramp_pwm_increment;   // TEMP HACK
//extern  int  mtr_2_ramp_pwm_increment;   // TEMP HACK

    MOTOR_BLOCK  motor_1;                  // two, bi-directional motors
    MOTOR_BLOCK  motor_2;

    int       mtr_1_direction_local = DIRECTION_FORWARD;  // current direction
    int       mtr_2_direction_local = DIRECTION_FORWARD;

                                           // VARIABLES
    int       rc;
//  int       speed_duty_value    = 0;     // speed duty value in ticks
    int       ramp_complete_callback_flag = 0;
    int       do_operatioanl_test = 1;

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

    board_clock_init (MCU_SPEED, 0);

    board_systick_init();     // set up 1 ms systick Timer period

       /* Enable the FPU for floating point operation */
/// MAP_FPU_enableModule();
/// MAP_FPU_enableLazyStacking();

    board_gpio_init();        // initialize GPIO switch used for motor direction

       //--------------------------------------------------
       // initialize motors
       //--------------------------------------------------
    motor_lib_init (MCU_SPEED);          // initialize base motor library
// -- FUTURE -- add bi-directional vs uni-directional flag --
// -- FUTURE -- Some of parameter stuff needs to be be put in contro_config.hn file (Posts/Pins/Mappings)
    motor_init (&motor_1, 1, PWM_SPEED); // setup motor 1 PWM for 20 kHz period (50 usec)
    motor_init (&motor_2, 2, PWM_SPEED); // setup motor 2 PWM for 20 kHz period (50 usec)
    motor_adc_init_speed_control (15);    // arbitrary ADC channel
    motor_encoder_init (&motor_1, ENCODER_HALL_RIGHT_WHEEL);
    motor_encoder_init (&motor_2, ENCODER_HALL_LEFT_WHEEL);

    board_enable_global_interrupts();    // init is complete. turn on interrupts

       //----------------------------------------------------------
       // During initial board checkout, run the operational tests
       //----------------------------------------------------------
    if (do_operatioanl_test)
       operational_test();

       //-------------------------------------------------------
       //           Setup Motors for normal use.
       //
       // We setup motors to move forward, and enable them, with
       // the speed of the motor PWMs initially set to 0.
       // The speed is modified by reading the Speed ADC value,
       // which is done in the main while() loop below.
       //-------------------------------------------------------
    motor_set_direction (&motor_1, DIRECTION_FORWARD);
    motor_set_direction (&motor_2, DIRECTION_FORWARD);
    motor_set_decay (&motor_1, DECAY_SLOW);    // use SLOW decay (faster speeds)
    motor_set_decay (&motor_1, DECAY_SLOW);
    motor_set_duty_cycle (&motor_1, 0);        // reset duty cycle to = 0
    motor_set_duty_cycle (&motor_2, 0);

    motor_start (&motor_1);              // enable motors with the new direction
    motor_start (&motor_2);

       //------------------------------------------------------
       // Main loop to run and control motors
       //------------------------------------------------------
    while (1)
      {
             // check if direction has been changed, on slider switch P6.0
        switch_input = SWITCH1_READ;  // get curr setting of slider switch P6.0
        if (switch_input == 0)
           process_direction_switch (DIRECTION_FORWARD, 1);
           else process_direction_switch (DIRECTION_REVERSE, 1);

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
             motor_stop (&motor_1);            // stop motors when chg direction
             motor_stop (&motor_2);                         //   WORKS 04/02/16
// CAUTION: the above STOPs reset the PWM duty cycle to 0. Is that consistent with other Motor packages ?

             motor_set_direction (&motor_1, mtr_direction);
             motor_set_direction (&motor_2, mtr_direction);
             if (do_restart)
                { motor_start (&motor_1);  // re-start motors with new direction
                  motor_start (&motor_2);                   //   WORKS 04/02/16
                }
       }
      else if (mtr_direction  != mtr_1_direction_local)
              {        // need to change direction for Motor 1 only
                motor_stop (&motor_1);      // stop motor when change direction
                motor_set_direction (&motor_1, mtr_direction);
                if (do_restart)
                   motor_start (&motor_1);  // re-start motor with new direction
              }
      else if (mtr_direction  != mtr_2_direction_local)
              {        // need to change direction for Motor 2 only
                motor_stop (&motor_2);      // stop motor when change direction
                motor_set_direction (&motor_2, mtr_direction);
                if (do_restart)
                   motor_start (&motor_2);  // re-start motor with new direction
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

#if defined(LATER)
       //------------------------------------------------------
       // Motor 1 - PWM TEST   both slow/fast decay variations
       //------------------------------------------------------
    motor_set_decay (&motor_1, DECAY_SLOW);   // spins fast at 60 % with this setting
    motor_set_direction (&motor_1, DIRECTION_FORWARD);   // AIN2_PWM,  AIN1_HIGH
    motor_set_duty_cycle (&motor_1, 50);                 //   J4-2       J4-1
    board_delay (2000);                      // run for 2 seconds

    motor_set_duty_cycle (&motor_1, 0);      // shut off PWM as re-configure

    motor_set_decay (&motor_1, DECAY_FAST);  //  But just barely starts spinning at 60 % (Slowly) with this setting
    motor_set_direction (&motor_1, DIRECTION_FORWARD);   // AIN1_PWM,  AIN2_LOW
    motor_set_duty_cycle (&motor_1, 65);                 //   J4-1       J4-2
    board_delay (2000);                      // run for 2 seconds

    motor_set_duty_cycle (&motor_1, 0);      // shut off PWM as re-configure

       // Motor 1 - REVERSE   both slow/fast decay variations
    motor_set_decay (&motor_1, DECAY_SLOW);  // spins fast at 60% with this setting
    motor_set_direction (&motor_1, DIRECTION_REVERSE);   // AIN1_PWM,  AIN2_HIGH
    motor_set_duty_cycle (&motor_1, 50);                 //   J4-1       J4-2
    board_delay (2000);                      // run for 2 seconds

    motor_set_duty_cycle (&motor_1, 0);      // shut off PWM as re-configure

    motor_set_decay (&motor_1, DECAY_FAST);  //  But just barely starts spinning at 60 % (Slowly) with this setting
    motor_set_direction (&motor_1, DIRECTION_REVERSE);   // AIN2_PWM,  AIN1_LOW
    motor_set_duty_cycle (&motor_1, 65);                 //   J4-2       J4-1
    board_delay (2000);                      // run for 2 seconds

    motor_set_duty_cycle (&motor_1, 0);      // shut off PWM as re-configure

       //------------------------------------------------------
       // Motor 2 - PWM TEST   both slow/fast decay variations
       //------------------------------------------------------
    motor_set_decay (&motor_2, DECAY_SLOW);  // spins fast at 60% with this setting
    motor_set_direction (&motor_2, DIRECTION_FORWARD);   // BIN2_PWM,  BIN1_HIGH
    motor_set_duty_cycle (&motor_2, 50);                 //   J4-5       J4-4
    board_delay (2000);                      // run for 2 seconds

    motor_set_duty_cycle (&motor_2, 0);      // shut off PWM as re-configure

    motor_set_decay (&motor_2, DECAY_FAST);  //  But just barely starts spinning at 60 % (Slowly) with this setting
    motor_set_direction (&motor_2, DIRECTION_FORWARD);   // BIN1_PWM,  BIN2_LOW
    motor_set_duty_cycle (&motor_2, 65);                 //   J4-4       J4-5
    board_delay (2000);                      // run for 2 seconds

    motor_set_duty_cycle (&motor_2, 0);      // shut off PWM as re-configure

       // Motor 2 - REVERSE   both slow/fast decay variations
    motor_set_decay (&motor_2, DECAY_SLOW);  // spins fast at 60% with this setting
    motor_set_direction (&motor_2, DIRECTION_REVERSE);   // BIN1_PWM,  BIN2_HIGH
    motor_set_duty_cycle (&motor_2, 50);                 //   J4-4       J4-5
    board_delay (2000);                      // run for 2 seconds

    motor_set_duty_cycle (&motor_2, 0);      // shut off PWM as re-configure

    motor_set_decay (&motor_2, DECAY_FAST);  //  But just barely starts spinning at 60 % (Slowly) with this setting
    motor_set_direction (&motor_2, DIRECTION_REVERSE);   // BIN2_PWM,  BIN1_LOW
    motor_set_duty_cycle (&motor_2, 65);                 //   J4-5       J4-4
    board_delay (2000);                      // run for 2 seconds

    motor_set_duty_cycle (&motor_2, 0);      // shut off PWM as re-configure
#endif

       //------------------------------------------------------
       //             MOTOR  CHECKOUT  TEST
       //
       // initialize in Forward direction, but duty set to 0
       //------------------------------------------------------
    motor_set_direction (&motor_1, DIRECTION_FORWARD);
    motor_set_direction (&motor_2, DIRECTION_FORWARD);
    decay_mode = DECAY_SLOW;                 // FAST = 0,   SLOW = 1
    motor_set_decay (&motor_1, decay_mode);  // spins fast at 60% with this setting
    motor_set_decay (&motor_2, decay_mode);  // spins fast at 60% with this setting
    motor_set_duty_cycle (&motor_1, 0);      // reset duty cycle = 0
    motor_set_duty_cycle (&motor_2, 0);
    motor_start (&motor_1);            // enable motors with specified direction  - with SLOW decay, motors run flat out at 100 % !!!
    motor_start (&motor_2);

       //------------------------------------------------------
       //             MOTOR 1  CHECKOUT  TEST
       //  -- single step thru each statement under debugger and note effects --
       //------------------------------------------------------
    motor_set_duty_cycle (&motor_1, 20);               // set duty cycle = 20 %
while (wait_1)
  ;            // DEBUG HACK
    motor_set_duty_cycle (&motor_1, 30);               // set duty cycle = 30 %
    motor_set_duty_cycle (&motor_1, 40);               // set duty cycle = 40 %
    motor_set_duty_cycle (&motor_1, 50);               // set duty cycle = 50 %
    motor_set_duty_cycle (&motor_1, 60);               // set duty cycle = 60 % barely moves when decay = FAST
    motor_set_duty_cycle (&motor_1, 70);               // set duty cycle = 70 %
    motor_set_duty_cycle (&motor_1, 80);               // set duty cycle = 80 %
    motor_set_duty_cycle (&motor_1, 90);               // set duty cycle = 90 %
    motor_brake (&motor_1);

    motor_set_direction (&motor_1, DIRECTION_REVERSE); // spin motor other direction
    motor_set_duty_cycle (&motor_1, 20);               // set duty cycle = 20 %
while (wait_2)
  ;            // DEBUG HACK
    motor_set_duty_cycle (&motor_1, 30);               // set duty cycle = 30 %
    motor_set_duty_cycle (&motor_1, 40);               // set duty cycle = 40 %
    motor_set_duty_cycle (&motor_1, 50);               // set duty cycle = 50 %
    motor_set_duty_cycle (&motor_1, 60);               // set duty cycle = 60 %  nneds at least 60% juice to run !  (slow when decay = FAST)
    motor_set_duty_cycle (&motor_1, 70);               // set duty cycle = 70 %
    motor_set_duty_cycle (&motor_1, 80);               // set duty cycle = 80 %
    motor_set_duty_cycle (&motor_1, 90);               // set duty cycle = 90 %
    motor_brake (&motor_1);
while (wait_3)
  ;            // DEBUG HACK

    motor_set_direction (&motor_1, DIRECTION_FORWARD); // and flip direction again
    motor_set_duty_cycle (&motor_1, 70);               // set duty cycle = 70 %
    process_direction_switch (DIRECTION_REVERSE,0);    // and flip direction again, leave motors off
    motor_start (&motor_1);                            // explicitly start just Motor 1 - STM32 may have issue - is not auto-restartring PWM
    motor_set_duty_cycle (&motor_1, 70);               // set duty cycle = 70 %
    motor_stop (&motor_1);                             // totally stop the PWM
                          // the Stop also resets Duty_Cycle = 0. Is that consistent with other Robot packages ?

       // retry motor 1 initial test, but with DECAY_FAST
    motor_set_direction (&motor_1, DIRECTION_FORWARD);
    motor_set_decay (&motor_1, DECAY_FAST);
    motor_set_duty_cycle (&motor_1, 20);               // set duty cycle = 20 %
    motor_start (&motor_1);                            // restart the motor
while (wait_4)
  ;            // DEBUG HACK
    motor_set_duty_cycle (&motor_1, 30);               // set duty cycle = 30 %
    motor_set_duty_cycle (&motor_1, 40);               // set duty cycle = 40 %
    motor_set_duty_cycle (&motor_1, 50);               // set duty cycle = 50 %
    motor_set_duty_cycle (&motor_1, 60);               // set duty cycle = 60 %   // slowly starts turning
    motor_set_duty_cycle (&motor_1, 70);               // set duty cycle = 70 %
    motor_set_duty_cycle (&motor_1, 80);               // set duty cycle = 80 %
    motor_set_duty_cycle (&motor_1, 90);               // set duty cycle = 90 %
    motor_stop (&motor_1);

    motor_set_direction (&motor_1, DIRECTION_REVERSE);
    motor_set_decay (&motor_1, DECAY_FAST);
    motor_set_duty_cycle (&motor_1, 20);               // set duty cycle = 20 %
    motor_start (&motor_1);                            // restart the motor
while (wait_5)
  ;            // DEBUG HACK
    motor_set_duty_cycle (&motor_1, 30);               // set duty cycle = 30 %
    motor_set_duty_cycle (&motor_1, 40);               // set duty cycle = 40 %
    motor_set_duty_cycle (&motor_1, 50);               // set duty cycle = 50 %
    motor_set_duty_cycle (&motor_1, 60);               // set duty cycle = 60 %   // slowly starts turning
    motor_set_duty_cycle (&motor_1, 70);               // set duty cycle = 70 %
    motor_set_duty_cycle (&motor_1, 80);               // set duty cycle = 80 %
    motor_set_duty_cycle (&motor_1, 90);               // set duty cycle = 90 %
    motor_stop (&motor_1);

       //------------------------------------------------------
       //             MOTOR 2  CHECKOUT  TEST
       //  -- single step thru each statement under debugger and note effects --
       //------------------------------------------------------
    motor_set_decay (&motor_2, DECAY_SLOW);            // ensure is in SLOW decay
    process_direction_switch (DIRECTION_FORWARD,0);    // reset back to FORWARD, leave motors off
    motor_set_duty_cycle (&motor_2, 20);               // set duty cycle = 20 %
    motor_start (&motor_2);                            // explicitly start just Motor 2
while (wait_6)
  ;            // DEBUG HACK
    motor_set_duty_cycle (&motor_2, 30);               // set duty cycle = 30 %  Slow Decay starts with 30 %
    motor_set_duty_cycle (&motor_2, 40);               // set duty cycle = 40 %
    motor_set_duty_cycle (&motor_2, 50);               // set duty cycle = 50 %
    motor_set_duty_cycle (&motor_2, 60);               // set duty cycle = 60 %  Fast Decay starts with 60 %
    motor_set_duty_cycle (&motor_2, 70);               // set duty cycle = 70 %
    motor_set_duty_cycle (&motor_2, 80);               // set duty cycle = 80 %
    motor_set_duty_cycle (&motor_2, 90);               // set duty cycle = 90 %
    motor_brake (&motor_2);

    motor_set_direction (&motor_2, DIRECTION_REVERSE); // spin motor other direction
    motor_set_duty_cycle (&motor_2, 20);               // set duty cycle = 20 %
while (wait_7)
  ;            // DEBUG HACK
    motor_set_duty_cycle (&motor_2, 30);               // set duty cycle = 30 %  Slow Decay starts with 30 %
    motor_set_duty_cycle (&motor_2, 40);               // set duty cycle = 40 %
    motor_set_duty_cycle (&motor_2, 50);               // set duty cycle = 50 %
    motor_set_duty_cycle (&motor_2, 60);               // set duty cycle = 60 %  Fast Decay starts with 60 %
    motor_set_duty_cycle (&motor_2, 70);               // set duty cycle = 70 %
    motor_set_duty_cycle (&motor_2, 80);               // set duty cycle = 80 %
    motor_set_duty_cycle (&motor_2, 90);               // set duty cycle = 90 %
    motor_brake (&motor_2);

    motor_set_direction (&motor_2, DIRECTION_FORWARD); // and flip direction again
    motor_set_duty_cycle (&motor_2, 70);               // set duty cycle = 70 %
    process_direction_switch (DIRECTION_REVERSE,1);    // and flip direction again -- STM32 CAUSED A FLAKE/GLITCH on output PWM -- with auto-restart
    motor_set_duty_cycle (&motor_2, 70);               // set duty cycle = 70 %    -- STM32 which then gets cured on this next set_duty_cycle --
    motor_stop (&motor_2);                             // totally stop the PWM
    motor_stop (&motor_1);                             // totally stop the PWM

       //---------------------------------------
       // Test  Motor 1  RAMP_UP and RAMP_DOWN
       //---------------------------------------
    motor_set_decay (&motor_1, DECAY_SLOW);          // use SLOW decay (faster speeds)
    process_direction_switch (DIRECTION_FORWARD,0);  // reset back to FORWARD, leave motors off
    ramp_complete_callback_flag = 0;                 // clear RAMP complete flag

        // ramp up Motor 1 from 10% to 90% in 3 seconds. Update PWM every 30 ms
    motor_ramp_duty (&motor_1, RAMP_UP, 10, 90, 3000, 30, &ramp_done_callback);
    while (ramp_complete_callback_flag == 0)
      ;     // loop till done          ==> uses callback
while (wait_8)
  ;            // DEBUG HACK

    board_delay (2000);                    // wait for 2 seconds before next run
    ramp_complete_callback_flag = 0;       // clear RAMP complete flag

        // ramp down Motor 1 from 90% to 10% in 3 seconds. Update PWM every 30 ms
    motor_ramp_duty (&motor_1, RAMP_DOWN, 90, 10, 3000, 30, NULL_MPTR);
    while (motor_ramp_check_complete(&motor_1) == 0)
      ;     // loop till done          ==> polls for complete, no callback
while (wait_9)
  ;            // DEBUG HACK

    board_delay (2000);                    // wait for 2 seconds before next run
    ramp_complete_callback_flag = 0;       // clear RAMP complete flag

       //---------------------------------------
       // Test  Motor 2  RAMP_UP and RAMP_DOWN
       //---------------------------------------
    motor_set_decay (&motor_2, DECAY_SLOW);          // use SLOW decay (faster speeds)
        // ramp up Motor 2 from 10% to 90% in 3 seconds. Update PWM every 5 ms
    motor_ramp_duty (&motor_2, RAMP_UP, 10, 90, 3000, 5, &ramp_done_callback);
    while (ramp_complete_callback_flag == 0)
      ;     // loop till done          ==> uses callback

    board_delay (2000);                    // wait for 2 seconds before next run
    ramp_complete_callback_flag = 0;       // clear RAMP complete flag

        // ramp down Motor 2 from 90% to 10% in 3 seconds. Update PWM every 5 ms
    motor_ramp_duty (&motor_2, RAMP_DOWN, 90, 10, 3000, 5, NULL_MPTR);
    while (motor_ramp_check_complete(&motor_2) == 0)
      ;     // loop till done          ==> polls for complete, no callback

    board_delay (2000);                    // wait for 2 seconds before next run
    ramp_complete_callback_flag = 0;       // clear RAMP complete flag

    motor_stop (&motor_1);             // Ensure both motors are totally stopped
    motor_stop (&motor_2);
}

//******************************************************************************
