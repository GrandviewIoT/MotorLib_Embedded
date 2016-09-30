// (1) ENCODERS: not seeing any signal on encoder signal line, even on Oscscope.
//           ==> pullups on MSP432 are not strong enough.
//           ==> need to run 5V to pullups and power, divide down for MSP432 GPIO
//           ==> need to do standalone diagnostics of Encoders.
//
// (2) DRV8848   Current Control VRef ADC is not properly being read.   04/16/16
//           ==> problem in sequencing/logic of ADC14 for multi-channel.

//*******1*********2*********3*********4*********5*********6*********7**********
//
//                            BDC_motor_ctl_lib.c
//
// BDC Motor Control Libarary - API and high level routines.
//
// This is a portable code library, designed to work across a number of
// processors, including:   TI Tiva Cortex-M
//                          TI MSP432 with Cortex-M core
//                          TI MSP430 F5529, FR5969, FR6989, FR5994
//                          STM32 F3, F4, L4, L7 Cortex-M series
//
// Provides support for the following BDC controllers:
//     - L6239 H-Bridge IC,
//     - TI SN754410 H-Bridge IC,
//     - DRV8848 Dual H-Bridge controller. used on the TI DRV8848 Boosterpack.
//     - L6206   Dual H-Bridge controller. used on the STM32 Xnucleo dual brush
//               expansion boards.
//
// 6 / 9 / 12 Volts is supplied to DC motors via BDC Controller.
// Motors are driven by PWMs, which are used to modulate the speed.
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
//          set_direction/go_left/go_right, adc_process_speed_control. Duquaine
//   04/05/16 - Rolled in support for Hall Sensors (attached to wheels).Duquaine
//   04/08/16 - Split off motor ctlr (SN754410 vs DRV8848)  specific code.Duquaine
//   04/16/16 - Basic forward/reverse + speed control working on MSP432.Duquaine
//******************************************************************************

#include "BDC_motor_ctl.h"                       // pull in common definitions


extern  MOTOR_BLOCK  motor_1;   //  ??? !!!   BAD  DESPERATE  HACK  !!!
extern  MOTOR_BLOCK  motor_2;

                //---------------------------------------------
               // pick up the appropriate Motor Driver support
                //---------------------------------------------
#if defined(MOTOR_DRIVER_DRV8848)
       extern  MOTOR_HANDLERS  drv8848_table;
               MOTOR_HANDLERS  *mtrdrvr = &drv8848_table;
#elif defined(MOTOR_DRIVER_SN754410)
       extern  MOTOR_HANDLERS  sn754410_table;
               MOTOR_HANDLERS  *mtrdrvr = &sn754410_table;
#elif defined(MOTOR_DRIVER_L6293)
       extern  MOTOR_HANDLERS  l6293_table;
               MOTOR_HANDLERS  *mtrdrvr = &l6293_table;
#elif defined(MOTOR_DRIVER_L6206)
       extern  MOTOR_HANDLERS  l6206_table;
               MOTOR_HANDLERS  *mtrdrvr = &l6206_table;
#else
error "You must define a MOTOR_DRIVER entry"
#endif

                //-----------------------------------------------------
                //   values updated by ADC/GPIO ISRs.
                //
                //   These externs reside in BDC_motor_ctl_ISRs_xxx.c
                //-----------------------------------------------------
extern int       g_adc_current_VREF_raw_value; // DRV8848 value for VREF from ADC
extern int       g_adc_speed_raw_value;        // speed value from ADC input
extern uint32_t  encoder_LEFT_rupts;  // count of interrupts this interval
extern uint32_t  encoder_RIGHT_rupts; // counts are reset at each motor_compute_rps_speed()
                                      // call, which occurs every 50 ms

const long    long_100     = 100;           // CONSTANTS

                                            // VARIABLES
    int       adc_timer           = 0;
    int       speed_duty_value    = 0;      // speed duty value in ticks

    int       g_motor_init_complete = 0;
    long      g_mtr_mcu_speed     = 0;      // MCU speed (SMCLK)
    int       period_ticks;                 // # SMCLK ticks in period
    float     period_ticks_ratio;           // scaling factor ticks : %

    int       motor_ramp_active_flag = 0;   // used by SysTick ISR to see any ramp in progress

           //---------------------------------
           // Encoder related variables
           //---------------------------------
    int       encoder_timer           = 0;  // Used by Systick handler
    int       encoder_one_tenth_rupts = 0;  // counts how many 0.1 interrupts
   uint32_t   encoder_start_interval  = 0;  // 1 second measurement interval
   uint32_t   encoder_end_interval    = 0;  //      updated once per second
    int       encoder_LEFT_1_sec_agg  = 0;  // Aggregate counts for 1 sec
    int       encoder_RIGHT_1_sec_agg = 0;
    float     encoder_LEFT_speed_rps  = 0;  // speed in revs / second
    float     encoder_RIGHT_speed_rps = 0;
    float     encoder_pulses_per_revolution = (PULSES_PER_REVOLUTION / 10.0);
    uint32_t  hall_port_rupt_flags = 0;
    float     revs_per_sec     = 0.0;       // revs/sec from Hall sensors


int   motor_ctl_adc_init (void);            // internal routines prototypes
int   motor_ctl_gpio_init (void);
void  motor_adc_process_speed_control (void);
void  ramp_done_callback (int motor_num);
void  motor_ramp_update_check (void);
void  process_hall_sensors (void);


//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                         UTILITY  MOTOR  ROUTINEs          (INTERNAL USE ONLY)
//
//******************************************************************************
//******************************************************************************
//******************************************************************************

//******************************************************************************
//   called  from                   SYSTICK    ISR
//******************************************************************************
void  motor_systick_1ms_update (void)
{
    if ( ! g_motor_init_complete)
       return;                              // bail if motor init not called yet

    adc_timer++;
    if (adc_timer >= 50)                    // call once every 50 ms
       {        // start a new ADC conversion every 50 ms.
                // call processor's ADC converion rtn
         mtrdrvr->motor_do_next_adc_conversion();
         adc_timer = 0;                      // reset counter for next pass
       }

    encoder_timer++;
    if (encoder_timer >= 100)               // call once every 0.1 seconds
       {        // update our RPS speed calculation
         motor_compute_rps_speed();
         encoder_timer = 0;                 // reset for next pass
       }

    if (motor_ramp_active_flag)
       motor_ramp_update_check();           // increment any ramp as needed
}


//******************************************************************************
//  motor_adc_process_speed_control
//
//           Check if ADC value has changed. If so, ratio it 0:16384 = 0:100 %
//           and then change PWM duty values as required.
//           This uses integers ("cave man calculus"/cmc) to compute the values.
//******************************************************************************
void  motor_adc_process_speed_control (void)
{
    float scaled_value_flt;
    int   duty_per_cent_value;

        // scale the value to be 0-100 %, from ADC (10-bit) range 0-1023
        //                                  or      12-bit  range 0-4095
    scaled_value_flt = ((float) g_adc_speed_raw_value * 100.0) / ADC_SCALING_VALUE;
    duty_per_cent_value = (int) (scaled_value_flt + 0.5);  // round up result
    if (duty_per_cent_value > 100)
       duty_per_cent_value = 100;

       // If the 0 to 100 % value has changed, update to new speed value
    if (duty_per_cent_value != speed_duty_value)
       {   // ADC duty value has changed: update to new duty value/speed setting


// ??? !!! MAJOR ISSUE - is extern to main()'s  motor_1 / motor_2

         motor_set_duty_cycle (&motor_1, (uint16_t) duty_per_cent_value);
         motor_set_duty_cycle (&motor_2, (uint16_t) duty_per_cent_value);
            // and save it as new speed value
         speed_duty_value = duty_per_cent_value;
       }
   //  WORKs  04/02/16

}


//******************************************************************************
//  motor_ctl_adc_init                                          INTERNAL ROUTINE
//
//         Configure control inputs for motor ADCs - Current sense, ...
//
//         ADC GPIO port/pin related stuff is in motor_config.h file
//******************************************************************************
int  motor_ctl_adc_init (void)
{
          // call MCU dependent ADC speed ctl init handler
    mtrdrvr->motor_init_adc_ctl_inputs();

    return (0);            // denote succeeded
}


//******************************************************************************
//  motor_ctl_gpio_init                                         INTERNAL ROUTINE
//
//         Configure control inputs for Motor GPIOs  - FAULT, ...
//
//         Cnfigure control outputs for Motor GPUIOs - ENABLE, ...
//******************************************************************************

int  motor_ctl_gpio_init (void)
{
    mtrdrvr->motor_init_gpio_ctl_inputs();  // call MCU dependent GPIO init inputs handler
    mtrdrvr->motor_init_gpio_ctl_outputs(); // call MCU dependent GPIO init outputs handler

    return (0);            // denote succeeded
}


//******************************************************************************
//  motor_compute_rps_speed
//
//             Called once every 0.1 second by SysTick ISR, to update RPS speed.
//
//             May be losing .5 counts per 0.1 interval. May need to add a
//             once per second aggregator to correct the count calc at
//             end of each 1 secondf interval.
//
//             The Hall encoders provides 625 pulses per revolution.
//             They convert the angular displacement signals into a pulse count.
//******************************************************************************

void  motor_compute_rps_speed (void)
{
   float    flt_enc_count;

             // compute RPS for left wheel
   flt_enc_count = encoder_LEFT_rupts;
   encoder_LEFT_speed_rps = flt_enc_count / encoder_pulses_per_revolution;

             // compute RPS for right wheel
   flt_enc_count = encoder_RIGHT_rupts;
   encoder_RIGHT_speed_rps = flt_enc_count / encoder_pulses_per_revolution;

             // update aggregates used for 1 second RPS calcs
   encoder_LEFT_1_sec_agg  += encoder_LEFT_rupts;
   encoder_RIGHT_1_sec_agg += encoder_RIGHT_rupts;
   encoder_one_tenth_rupts++;

             // check if we have hit the 1 second mark
   if (encoder_one_tenth_rupts >= 10)
      {      // yes. re-check/update RPS once every second to handle any
             // possible round off issues from the 0.1 / second interval calcs.
             // compute RPS for left wheel, using 1 full second of data
        flt_enc_count = encoder_LEFT_1_sec_agg;
        encoder_LEFT_speed_rps = flt_enc_count / encoder_pulses_per_revolution;
             // compute RPS for right wheel, using 1 full second of data
        flt_enc_count = encoder_RIGHT_1_sec_agg;
        encoder_RIGHT_speed_rps = flt_enc_count / encoder_pulses_per_revolution;

        encoder_LEFT_1_sec_agg  = 0;      // Reset for next pass
        encoder_RIGHT_1_sec_agg = 0;
        encoder_one_tenth_rupts = 0;
      }
             // reset interrupt counters for next pass
   encoder_LEFT_rupts  = 0;
   encoder_RIGHT_rupts = 0;
}



//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                         GENERIC  MOTOR  APIs  and  ROUTINEs
//
//******************************************************************************
//******************************************************************************
//******************************************************************************

//******************************************************************************
//  motor_lib_init
//
//         Initialize basic motor library facilities.
//******************************************************************************
int   motor_lib_init (long mcu_speed)   // also pass driver type (DRV9948, SN754419, ... ?)
{                                       // or make external .h parm config instead ?
    g_mtr_mcu_speed = mcu_speed;

    motor_ctl_gpio_init(); // Initialize Motor related inputs/outputs (Enable, Fault, ...)
    motor_ctl_adc_init();  // Initialize Motor related ADCs (current sense, ...)

    g_motor_init_complete = 1;

    return (0);            // denote succeeded
}


//******************************************************************************
//  motor_adc_init_speed_control
//
//         Configure ADC to read potentiometer used for speed control setting.
//         Uses A15  on pin  P6.0
//
//         ADC GPIO port/pin related stuff is in motor_config.h file
//******************************************************************************
int  motor_adc_init_speed_control (int ADC_Channel)
{
          // call MCU dependent ADC speed ctl init handler
    mtrdrvr->motor_init_speed_adc (ADC_Channel);

    return (0);            // denote succeeded
}


//******************************************************************************
//  motor_encoder_init
//
//       Configure inputs for Motor Encode (Hall/QEI) Sensors (interrupt driven)
//
//       Encoder_Type = WHEEL_ENCODER_RIGHT/LEFT, HALL_SENSOR, QEI
//       GPIO port/pin related stuff is in motor_config.h file
//******************************************************************************

int  motor_encoder_init (MOTOR_BLOCK *mtr_blk, int Encoder_Type)
{
          // call MCU dependent GPIO encoder init handler
    mtrdrvr->motor_init_encoder_gpio (mtr_blk, Encoder_Type);

    return (0);            // denote succeeded
}


//******************************************************************************
//  motor_init
//
//         Configure Motor, including setting up PWM timers for Motor operation
//******************************************************************************

int  motor_init (MOTOR_BLOCK *mtr_blk, int motor_num, int pwm_speed)
{
    memset (mtr_blk, 0, sizeof(MOTOR_BLOCK));            // ensure is cleared out

// in future, add to table (ensure no competing entry)

    mtr_blk->mtr_num   = motor_num;
    mtr_blk->mtr_state = STATE_STOPPED;           // denote are currently stopped

       // Using the MCU clock, get the correct PWM tick count for 20 KHz PWM
    period_ticks = (g_mtr_mcu_speed / pwm_speed) - 1;
    period_ticks_ratio = ((float) period_ticks) / 100.00;  // create 100% ratio

    mtr_blk->mtr_pwm_period_ticks = period_ticks;          // save our PWM perid

    mtrdrvr->motor_init_pwms (mtr_blk, period_ticks); // call MCU dependent PWM init routine

    return (0);            // denote succeeded
}


//******************************************************************************
//  motor_brake
//               Stop the motor using a "brake" sequence: i.e. set both 1A/2A
//               or 3A/4A to LOW. This forces any re-circulating motor current
//               to ground, and stops the motor faster, rather than than just
//               turning off the PWM and letting it coast to a stop.
//
//               Note that this just brakes the motor.
//               Duty cycle is left on, so issuing motor_set_direction() will
//               immediately re-start the motor in the new direction.
//
//               To stop motor and set duty cycle = 0, use motor_stop()
//******************************************************************************
int  motor_brake (MOTOR_BLOCK  *mtr_blk)
{
    mtrdrvr->motor_brake_handler (mtr_blk);  // call MCU dependent Motor Brake handler

    return (0);            // denote succeeded
}


//*****************************************************************************
//  motor_ramp_check_complete
//
//           Polls to see if the motor ramp up/down is complete.
//           Returns:  1 if complete,  0 if not complete.
//
//           Note that using the callback (to set a complete flag in main)
//                is more efficient.
//*****************************************************************************
int   motor_ramp_check_complete (MOTOR_BLOCK *mtr_blk)
{
    if (mtr_blk->mtr_num == 1)
       { if (mtr_blk->mtr_ramp_pwm_increment == 0)
            return (1);                           // ramp is complete
            else return (0);                      // ramp is not complete
       }
      else {     // process Motor 2
             if (mtr_blk->mtr_ramp_pwm_increment == 0)
                return (1);                       // ramp is complete
                else return (0);                  // ramp is not complete
           }
}


//*****************************************************************************
//  motor_ramp_duty
//
//           Ramps the duty cycle (and hence speed) of a motor up or down.
//
//           We convert the begin/end duty_cycles from a % (percent) to the
//           underlying PWM counts instead, so that we can provide a smoother,
//           more granular ramp.
//
//           We use different Systick increment rates (1 ms, 5 ms, 10 ms, ...)
//           depending upon the PWM count range, and the associated
//           num_milliseconds range.
//
//  Parms:
//    ramp_up_down         - operation to perform: RAMP_UP or RAMP_DOWN
//    begin_duty_cycle_pc  - starting duty cycle percent: 0-100 (%)
//    nd_duty_cycle_pc     - ending   duty cycle percent: 0-100 (%)
//    ramp_duration_ms     - total number of milli-seconds duration start to end
//                           ranging from 1 to 32767 (32.7 seconds)
//    increment_frequency_ms - how often (in SYSTICK ms) the PWM should be
//                           updated. E.g. 5 = every 5 ms, 10 = every 10 ms, ...
//    cb_ptr               - routine to be called back when RAMP is complete.
//                           Optional parm: setting to NULL_MPTR / 0L denotes
//                           no callback. If no callback defined, App must poll
//                           motor_ramp_complete() to determine end of RAMP.
//
//  Return:   0 = successful,    -1 = invalid parameter
//*****************************************************************************

int  motor_ramp_duty (MOTOR_BLOCK *mtr_blk, uint16_t ramp_up_down,
                     uint16_t begin_duty_cycle_pc, uint16_t end_duty_cycle_pc,
                     uint16_t ramp_duration_ms, uint16_t increment_frequency_ms,
                     CALLBACK_RTN  cb_ptr)
{
    long   start_duty_pwm_ticks;
    long   end_duty_pwm_ticks;
    long   total_pwm_ticks;
    long   total_milliseconds;
    long   total_increments;
    long   milli_inc;
    long   pwm_tick_increment;
    int    ramp_end;

    if (ramp_up_down > RAMP_DOWN || begin_duty_cycle_pc > 100
      || end_duty_cycle_pc > 100 || ramp_duration_ms > 65535
      || increment_frequency_ms > 65535)
       return (-1);                                        // invalid parameter

       //--------------------------------------------------------------------
       // use cave man calculus to compute # ticks in the duty percentage:
       //   (duty_cycle_percent * period_ticks)/100 = # ticks in that duty %
       //--------------------------------------------------------------------
    start_duty_pwm_ticks = (((long) begin_duty_cycle_pc) * period_ticks) / 100;
    end_duty_pwm_ticks   = (((long) end_duty_cycle_pc)   * period_ticks) / 100;

    if (ramp_up_down == RAMP_UP)
       {          // RAMP_UP
         total_pwm_ticks = end_duty_pwm_ticks - start_duty_pwm_ticks;
       }
      else {      // RAMP_DOWN
             total_pwm_ticks = start_duty_pwm_ticks - end_duty_pwm_ticks;
           }

    ramp_end = end_duty_pwm_ticks;

       //-----------------------------------------------------------------------
       // Set how often (in SYSTICK ms) we should update the PWM.
       // Use that to compute how many PWM ticks we should add/subtract
       // for each update period.
       //-----------------------------------------------------------------------
    milli_inc = increment_frequency_ms;     // convert to long for calc to minimize roundoff
    total_milliseconds = ramp_duration_ms;

    total_increments = total_milliseconds / milli_inc;

    pwm_tick_increment = total_pwm_ticks / total_increments;

    if (ramp_up_down == RAMP_DOWN)
       pwm_tick_increment = (0 - pwm_tick_increment);  // for RAMP_DOWN, make it negative

    mtr_blk->mtr_ramp_pwm_increment = pwm_tick_increment;
    mtr_blk->mtr_ramp_pwm_end       = ramp_end;
    mtr_blk->mtr_systick_increment  = increment_frequency_ms;
    mtr_blk->mtr_systick_count      = 0;
    mtr_blk->mtr_callback           = cb_ptr;

       // turn on flag to let SYSTICK ISR know we need to be called
    motor_ramp_active_flag = 1;           // turn ON ramp flag

       //--------------------------------------------------------
       // Now start the ramp, by setting initial RAMP duty value
       //--------------------------------------------------------
    motor_set_duty_cycle (mtr_blk, begin_duty_cycle_pc);

    if (mtr_blk->mtr_state != STATE_RUN)
       motor_start (mtr_blk);             // and startup motor if it was stopped

    return (0);                           // denote succeeded
}


//******************************************************************************
//  motor_ramp_update_check
//
//        Called by SYSTICK ISR when it sees that a ramp operation is requested.
//        Checks if it is time to update the PWM RAMP, and if so, do it.
//
//        Note:  mtr_x_ramp_pwm_increment is positive if RAMP_UP,
//                      "           "     is negative if RAMP_DOWN.
//******************************************************************************
void  motor_ramp_update_check (void)
{
    int16_t    new_duty_count;
    int        hit_end_point;

// ISSUE: how get pointers to MOTOR_BLOCKs from SYSTICK  ==> Queue Header
//                              ?? NEED TO FIX ASAP  !!!

    hit_end_point = 0;
    if (motor_1.mtr_ramp_pwm_increment != 0)   // mtr_blk->mtr_ramp_pwm_increment
       { motor_1.mtr_systick_count++;          // update how many Systicks this pass
         if (motor_1.mtr_systick_count >= motor_1.mtr_systick_increment)
            {     // go ahead and step PWM duty count to next step up or down
              new_duty_count = motor_1.mtr_pwm_duty_count + motor_1.mtr_ramp_pwm_increment;
              if (motor_1.mtr_ramp_pwm_increment > 0)
                 {        // is positive, so must be ramping up
                   if (new_duty_count >= motor_1.mtr_ramp_pwm_end)
                      hit_end_point = 1;         // we have hit our last update
                 }
                 else {   // is negative, so must be ramping down
                        if (new_duty_count <= motor_1.mtr_ramp_pwm_end)
                           hit_end_point = 1;     // we have hit our last update
                      }
              if (hit_end_point == 1)
                 { new_duty_count = motor_1.mtr_ramp_pwm_end;    // set final value
                   motor_1.mtr_ramp_pwm_increment = 0;    // no more ramp steps needed
                 }

// MAJOR BAD HACK

              motor_set_pwm_duty_count (&motor_1, new_duty_count); // update actual PWM counts
              motor_1.mtr_systick_count = 0;        // clear count for next pass
              if (hit_end_point == 1 && motor_1.mtr_callback != NULL_MPTR)
                 motor_1.mtr_callback(1);           // tell main() we are done
            }
       }

    if (motor_2.mtr_ramp_pwm_increment != 0)
       { motor_2.mtr_systick_count++;      // update how many Systicks this pass
         if (motor_2.mtr_systick_count >= motor_2.mtr_systick_increment)
            {     // go ahead and step PWM duty count to next step up or down
              new_duty_count = motor_2.mtr_pwm_duty_count + motor_2.mtr_ramp_pwm_increment;
              if (motor_2.mtr_ramp_pwm_increment > 0)
                 {        // is positive, so must be ramping up
                   if (new_duty_count >= motor_2.mtr_ramp_pwm_end)
                      hit_end_point = 1;          // we have hit our last update
                 }
                 else {   // is negative, so must be ramping down
                        if (new_duty_count <= motor_2.mtr_ramp_pwm_end)
                           hit_end_point = 1;     // we have hit our last update
                      }
              if (hit_end_point == 1)
                 { new_duty_count = motor_2.mtr_ramp_pwm_end; // set final value
                   motor_2.mtr_ramp_pwm_increment = 0;    // no more ramp steps needed
                 }

// MAJOR BAD HACK

              motor_set_pwm_duty_count (&motor_2, new_duty_count); // update actual PWM counts
              motor_2.mtr_systick_count = 0;        // clear count for next pass
              if (hit_end_point == 1 && motor_2.mtr_callback != NULL_MPTR)
                 motor_2.mtr_callback(2);           // tell main() we are done
            }
       }

    if (motor_1.mtr_ramp_pwm_increment == 0  && motor_2.mtr_ramp_pwm_increment == 0)
       motor_ramp_active_flag = 0;             // turn off ramp flag
}


//*****************************************************************************
//  motor_go_left
//
//           Have just one motor go left (in CCW direction).
//           CCW direction is "backward/reverse".
//*****************************************************************************
int  motor_go_left (MOTOR_BLOCK *mtr_blk, int re_start)
{
       // ensure motor is stopped, before we change direction
    motor_stop (mtr_blk);

       // set the desired direction
    motor_set_direction (mtr_blk, DIRECTION_FORWARD);

       // should we automatically re-start motor after changing its direction ?
    if (re_start)
       {     // yes, restart the motor
         motor_start (mtr_blk);
       }

    return (0);            // denote succeeded
}


//*****************************************************************************
//  motor_go_right
//
//           Have just one motor go right (in CW direction).
//           CW direction is "forward".
//*****************************************************************************
int  motor_go_right (MOTOR_BLOCK *mtr_blk, int re_start)
{
       // ensure motor is stopped, before we change direction
    motor_stop (mtr_blk);

       // set the desired direction
    motor_set_direction (mtr_blk, DIRECTION_FORWARD);

       // should we automatically re-start motor after changing its direction ?
    if (re_start)
       {     // yes, restart the motor
         motor_start (mtr_blk);
       }

    return (0);            // denote succeeded
}


//*****************************************************************************
//  motor_set_decay
//
//      Update Motor decay settings. These dictate how fast the motor stops.
//
//         DECAY_FAST = short motor re-circulating current to ground, to
//                      drain out motor current as fast as possible.
//                      It effectively acts like a brake.
//         DECAY_SLOW = let motor current re-circulate till it fades out.
//                      This "coasts" the motor to a stop.
//*****************************************************************************
int  motor_set_decay (MOTOR_BLOCK *mtr_blk,  int decay_type)
{
       // set and save the motor decay type
    if (decay_type == DECAY_SLOW)
       mtr_blk->mtr_decay_mode = DECAY_SLOW;
       else mtr_blk->mtr_decay_mode = DECAY_FAST;

    return (0);            // denote succeeded
}


//*****************************************************************************
//  motor_set_direction
//
//      Update Motor PWM and GPIO settings to set desired Clockwise (CW) or
//      CounterClockwise (CCW) direction.
//
//*****************************************************************************
int  motor_set_direction (MOTOR_BLOCK *mtr_blk,  int mtr_direction)
{
    mtr_blk->mtr_direction = mtr_direction;       // save the new direction

    if (mtr_blk->mtr_state == STATE_RUN)
       {     // if running, immediately update to the new Direction by
             // calling the MCU dependent Set Direction handler
         mtrdrvr->motor_set_direction_handler (mtr_blk,mtr_direction);
       }
      else {    // Is STOPPED, so just keep the saved value.
                // It will get applied to the motor during motor_start() call
           }

    return (0);            // denote succeeded
}


//*****************************************************************************
//  motor_set_duty_cycle
//              Set the duty cycle to be used for a specific motor.
//              Requested duty cycle is passed as a percentage value (0-100).
//              The duty cycle sets the motor's speed. (More juice, more speed)
//
//   Parms:
//      motor_num:           number of the motor to update (1 or 2)
//      duty_cycle_percent:  % of duty cycle to apply, ranging from 0 to 100
//*****************************************************************************
int  motor_set_duty_cycle (MOTOR_BLOCK *mtr_blk,  uint16_t duty_cycle_percent)
{
    long  actual_duty_count;

                 // convert to ticks using integer based "cave man calculus"
    actual_duty_count  = duty_cycle_percent;   // convert to long
    actual_duty_count *= period_ticks;         // convert to ticks
    actual_duty_count /= long_100;             // eliminate * 100 percent piece
    if (actual_duty_count > period_ticks)
       actual_duty_count = period_ticks;       // do not exceed max period

    if (actual_duty_count > 2)
       actual_duty_count--;     // deduct count by 1 because base of counter = 0

    mtr_blk->mtr_pwm_duty_count = actual_duty_count; // save latest PWM duty count

    if (mtr_blk->mtr_state == STATE_RUN)
       {     // if running, immediately update to the new Duty Cycle by
             // calling the MCU dependent Set Duty Cycle handler
         mtrdrvr->motor_set_pwm_duty_count_handler (mtr_blk,
                                                    actual_duty_count);
       }
      else {    // Is STOPPED, so just keep the saved value.
                // It will get applied to the motor during motor_start() call
           }

    return (0);            // denote succeeded
}


//*****************************************************************************
//  motor_set_pwm_duty_count
//              Set the duty cycle to be used for a specific motor.
//              Duty cycle is explicitly passed as PWM ticks, not as percent.
//              The duty cycle sets the motor's speed. (More juice, more speed)
//
//   Parms:
//      motor_num:   number of the motor to update (1 or 2)
//      pwm_duty:    actual duty cycle PWM count value to be loaded in PWM
//*****************************************************************************
int  motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int16_t actual_pwm_duty_count)
{
    mtr_blk->mtr_pwm_duty_count = actual_pwm_duty_count;  // save latest PWM duty cycle count

    if (mtr_blk->mtr_state == STATE_RUN)
       {     // if running, immediately update to the new Duty Cycle by
             // calling the MCU dependent Set Duty Cycle handler
         mtrdrvr->motor_set_pwm_duty_count_handler (mtr_blk,
                                                    actual_pwm_duty_count);
       }
      else {    // If stopped, just keep the saved value.
                // It will get applied to the motor during motor_start() call
           }


    return (0);            // denote succeeded
}


//*****************************************************************************
//  motor_start
//               Start or re-start the motor after a direction change or a stop.
//           Because a stop will slam off the PWMs, we re-establish Dir and Duty
//*****************************************************************************
int  motor_start (MOTOR_BLOCK *mtr_blk)
{
    mtr_blk->mtr_state = STATE_RUN;

        // call MCU dependent Motor Start handler
    mtrdrvr->motor_start_handler (mtr_blk);

    return (0);            // denote succeeded
}


//******************************************************************************
//  motor_stop
//              Stop the motor, and ensure it remains stopped by setting PWM = 0
//******************************************************************************
int  motor_stop (MOTOR_BLOCK *mtr_blk)
{
    mtr_blk->mtr_state = STATE_STOPPED;

        // call MCU dependent Motor Stop handler
    mtrdrvr->motor_stop_handler (mtr_blk);

// ??? !!! ???  REVISIT THIS !!!
    mtr_blk->mtr_pwm_duty_count = 0;                  // set PWM duty = 0

        // call MCU dependent Set PWM Duty Count handler
    mtrdrvr->motor_set_pwm_duty_count_handler (mtr_blk, 0);

    return (0);            // denote succeeded
}


//******************************************************************************
//  process_hall_sensors
//
//            Check Hall Sensors and update current speed/velocity value,
//            as well as approx distance travelled fwd/backwd.
//******************************************************************************
void  process_hall_sensors (void)
{
       // compute revs_per_sec (converted to float)
}

//******************************************************************************
