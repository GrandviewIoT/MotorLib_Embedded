//  MotorLib_Motor_Engage   <--- THIS IS A SHITTY NAME - change it !!!!


// (1) ENCODERS: not seeing any signal on encoder signal line, even on Oscscope.
//           ==> pullups on MSP432 are not strong enough.
//           ==> need to run 5V to pullups and power, divide down for MSP432 GPIO
//           ==> need to do standalone diagnostics of Encoders.
//
// (2) DRV8848   Current Control VRef ADC is not properly being read.   04/16/16
//           ==> problem in sequencing/logic of ADC14 for multi-channel.

// 05/17/16 - Add option/API to allow user to specify speed ref point and data
//            without onboard pot - e.g. Cellphone/BLE and to compute speed from
//            encoders
// 06/25/16 - Got major refactoring/restructed version properly working for BDCs
//            and STEPPERs. Is now ommon code base across multiple motor types.WD

// Sinple Motor Control Examples (1) (2), and (3)
//     (1) Ranmp up, run for a period of time, then ramp down.  = appliance like a dryer
//     (2) Ramp up to moderate speed, run for a while then ramp down. = Wash Cyccle
//         Pause  (e.g. drain out waterin tank)
//         Ramp up to a fast speed, run for a while, then stop. = Spin cycle
//         = applicance like a wash machine
//     (3) More sophiticated Motor control: go to a specific position
//         Ramp up, run at specified velocity (speed), watch encoder, as get
//         close to end-point ramp down until hit target point and stop.
//         Instead of running for a fixed period of time, you run, until
//         you reache the desired end point, and watch the encoder instead.

//*******1*********2*********3*********4*********5*********6*********7**********
//
//                               MotorLib_Basic.c
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
// Key differences between Brushed DC (BDC) and Brushless DC (BLDC) motors:
// ------------------------------------------------------------------------
// (1) Brushed DC motors do not require any sensors for spinning (commutating)
//     the motor.  BLDC motors require either Hall sensors, or BEMF sensing
//     through current feedback sensor to detemine rotor position, so that
//     it can properly fire the electronic commutation sequence.
// (2) BDC motors effectively have only one tuning know - duty cycle, which
//     controls both spped and torque, e.g. to get more spped, increase duty
//     cycle. BLDC motors effectively have two tuning knobs:
//       - commutation speed (how fast it fires electronic commutation sequence)
//       - torque control (amount of power applied) via duty cycle.
//     True, if the motor's speed gets bogged down because of heavy load, the
//     duty cycle does need to be increased to compensate, but in general a BLDC
//     motor can be viewed as having two tuning knobs: speed and power (torque).
//       - Speed is controlled via the motor_set_speed_rps() call.
//       - Torque (power) is contro0lled via the motor_set_duty_cycle() call.
//


//
//
// THE FOLLOWING ONLY BELONGS IN THE LOW LEVEL PLATFORM DRIVER FOR THE CHIP
//
// DRV8305 BP Usage                  PWM Tmr  LP Conn  MSP432
// -------------------               -------  -------  ------
//       Vm     Motor 6 V supply       -
//       Gnd    Battery Ground         -
//
//       PWM_AH   Phase A Hi  PWM     TA0.4    J4-1    P2.7
//       PWM_AL   Phase A Lo  PWM     TA0.3    J4-2    P2.6
//       PWM_BH   Phase B Hi  PWM     TA0.1    J4-3    P2.4
//       PWM_BL   Phase B Lo  PWM     TA2.1    J4-4    P5.6
//       PWM_CH   Phase C Hi  PWM     TA2.3    J4-5    P6.6
//       PWM_CL   Phase C Lo  PWM     TA2.4    J4-6    P6.7
//
//       Voltage Sense A      ADC     A14      J3-3     P6.1   BEMF
//       Voltage Sense B      ADC     A13      J3-4     P4.0   BEMF
//       Voltage Sense C      ADC     A11      J3-5     P4.2   BEMF
//       Voltage Sense Vdd    ADC     A9       J3-6     P4.4
//       Current Sense A      ADC     A8       J3-7     P4.5
//       Current Sense B      ADC     A6       J3-8     P4.7
//       Current Sense C      ADC     A1       J3-9     P5.4   -- Last ADC --
//
//       nFAULT      GPIO             J1-3     P3.2     Input
//       PwrGd       GPIO             J2-5     RST  <-- Is this a show stopper ?
//       EnGate      GPIO             J2-8     P5.0     Output
//       Wake        GPIO             J2-9     P5.2     Output
//
//       Speed Ctl Pot ADC            J1-2     P6.0  A15   Grove J1-2 -> J3-7/27
//       Fwd/Reverse Slider           J1-4     P3.3  GPIO  Grove UART connector
//
//       Hall Sensor   -    Phase A   J1-5     P4.1  GPIO rupt
//       Hall Sensor Right  Phase B   J1-6     P4.3  GPIO rupt
//       Hall Sensor Left   Phase C   J1-8     P4.6  GPIO rupt
//                   White lead = Signal,  Red lead = +3.3   Black lead = Gnd
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
//   06/20/16 - Tweaked APIs to make it easier for BDC, Stepper, and BLDC
//              motors to share APIs and share common support.    Duquaine 
//******************************************************************************

#include "MotorLib_Api.h"                        // pull in common definitions

#if defined(MOTOR_DRIVER_L6474)
#include "l6474.h"
void  L6474_StepClockHandler (uint8_t motor_id);
void  L6474_ApplySpeed (MOTOR_BLOCK *mtr_blk, uint8_t pwmId, uint16_t newSpeed);
#endif

                         //------------------------------------
                         //   BLDC Motor Blocks - 1 per motor
                         //------------------------------------
    MOTOR_BLOCK      motor_blk [MAX_NUMBER_OF_MOTORS];

       uint8_t       _g_motor_id = 0;

extern int           _g_PWM_rc  = 0;         // located in BLDC_motor_ctl_lib.c

       CALLBACK_RTN  _g_ADC_callback_ptr;    // located in xxxx_ISRs.c  ???

extern unsigned char  Hall_CW_DIR_sequence[];      // fwd ref

                //---------------------------------------------
               // pick up the appropriate Motor Driver support
                //---------------------------------------------
#if defined(MOTOR_DRIVER_L6206)
       extern  MOTOR_HANDLERS  drv6206_table;                // heavy duty BDC
               MOTOR_HANDLERS  *_g_mtrdrvr = &drv6206_table;
#elif defined(MOTOR_DRIVER_DRV8305)
       extern  MOTOR_HANDLERS  drv8305_table;                // heavy duty BDC
               MOTOR_HANDLERS  *_g_mtrdrvr = &drv8305_table;
#elif defined(MOTOR_DRIVER_DRV8301)
       extern  MOTOR_HANDLERS  drv8301_table;                // light duty BDC
               MOTOR_HANDLERS  *_g_mtrdrvr = &drv8301_table;
#elif defined(MOTOR_DRIVER_L6474)
       extern  MOTOR_HANDLERS  drv6474_table;                // light duty Stepper
               MOTOR_HANDLERS  *_g_mtrdrvr = &drv6474_table;
#elif defined(MOTOR_DRIVER_L6230)
       extern  MOTOR_HANDLERS  l6230_table;                  // light duty BLDC
               MOTOR_HANDLERS  *_g_mtrdrvr = &l6230_table;
#elif defined(MOTOR_DRIVER_L6398)
       extern  MOTOR_HANDLERS  l6398_table;                  // heavy duty BLDC
               MOTOR_HANDLERS  *_g_mtrdrvr = &l6398_table;
#else
error "You must define a MOTOR_DRIVER entry"
#endif
                        // Module Prototype Defs
MOTOR_HANDLERS * L6206_GetMotorHandle(void);
MOTOR_HANDLERS * L6474_GetMotorHandle(void);
MOTOR_HANDLERS * L647x_GetMotorHandle(void);
MOTOR_HANDLERS * L648x_GetMotorHandle(void);
MOTOR_HANDLERS * Powerstep01_GetMotorHandle(void);
MOTOR_HANDLERS * L6208_GetMotorHandle(void);
MOTOR_HANDLERS * L6230_GetMotorHandle(void);
MOTOR_HANDLERS * L6398_GetMotorHandle(void);


/** @defgroup MOTOR_CONTROL_Weak_Private_Functions MOTOR CONTROL Weak Private Functions
 * @{
 */
                         // Get motor handle for L6206 BDC
__weak MOTOR_HANDLERS * L6206_GetMotorHandle(void) {return ((MOTOR_HANDLERS*) 0);}
                         // Get motor handle for L6474 stepper
__weak MOTOR_HANDLERS * L6474_GetMotorHandle(void) {return ((MOTOR_HANDLERS*) 0);}
                         // Get motor handle for L647x stepper
__weak MOTOR_HANDLERS * L647x_GetMotorHandle(void) {return ((MOTOR_HANDLERS*) 0);}
                         // Get motor handle for L648x
__weak MOTOR_HANDLERS * L648x_GetMotorHandle(void) {return ((MOTOR_HANDLERS*) 0);}
                         // Get motor handle for Powerstep stepper
__weak MOTOR_HANDLERS * Powerstep01_GetMotorHandle(void) {return ((MOTOR_HANDLERS*) 0);}
                         // Get motor handle for L6208 BLDC
__weak MOTOR_HANDLERS * L6208_GetMotorHandle(void) {return ((MOTOR_HANDLERS*) 0);}
                         // Get motor handle for L6230 BLDC
__weak MOTOR_HANDLERS * L6230_GetMotorHandle(void) {return ((MOTOR_HANDLERS*) 0);}
                         // Get motor handle for L6398 BLDC
__weak MOTOR_HANDLERS * L6398_GetMotorHandle(void) {return ((MOTOR_HANDLERS*) 0);}


                //-----------------------------------------------------
                //   values updated by ADC/GPIO ISRs.
                //
                //   These externs reside in BDC_motor_ctl_ISRs_xxx.c
                //-----------------------------------------------------
extern int    g_adc_current_VREF_raw_value; // DRV8848 value for VREF from ADC
extern int    g_adc_speed_raw_value;        // speed value from ADC input

const long    long_100     = 100;           // CONSTANTS

                                            // VARIABLES
    int       adc_timer           = 0;

    int       _g_motor_init_complete = 0;
    long      _g_mtr_mcu_speed       = 0;      // MCU speed (SMCLK)
    uint32_t  _g_pwm_period_ticks    = 0;
    float     _g_period_ticks_ratio;           // scaling factor ticks : %

    int       _g_motor_ramp_active_flag = 0;   // used by SysTick ISR to see any ramp in progress

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
    float     revs_per_sec     = 0.0;       // revs/sec from Hall sensors


int   motor_ctl_adc_init (void);            // internal routines prototypes
int   motor_ctl_gpio_init (void);
void  motor_adc_process_speed_control (void);

void  ramp_done_callback (int motor_num);
//void  motor_ramp_update_check (void);
void  process_hall_sensors (void);


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
    if (_g_motor_init_complete == 1)
       return (0);               // we were previously called. no need to repeat

    _g_mtr_mcu_speed = mcu_speed;

       // this assumes that board_init() was called by the app
       // to ensure CPU clocks and base GPIO clocks were setuup
       // should we call board-Init() just in case and have it check a
       // global _g_board_init_complete flag ???
       // PROBABLY

       //-------------------------------------------------------------------
       // Initialize motor chip related GPIO input and output pins (and SPI)
       //-------------------------------------------------------------------
//  motor_ctl_gpio_init(); // Initialize Motor related inputs/outputs (Enable, Fault, ...)
    _g_mtrdrvr->motor_init_gpio_ctl_inputs();  // call MCU dependent GPIO init inputs handler
    _g_mtrdrvr->motor_init_gpio_ctl_outputs(); // call MCU dependent GPIO init outputs handler

       //-------------------------------------------------------------------
       // Initialize motor chip related ADC inputs (motor current, ...)
       //-------------------------------------------------------------------
//  motor_ctl_adc_init();  // Initialize Motor related ADCs (current sense, ...)
          // call MCU dependent ADC speed ctl init handler
    if (_g_mtrdrvr->motor_init_adc_ctl_inputs != 0L)
       _g_mtrdrvr->motor_init_adc_ctl_inputs();

    _g_motor_init_complete = 1; // flag that we have complete basic initilization

    return (0);                 // denote succeeded
}


//******************************************************************************
//  MotorLib_Init_Motor
//
//         Configure Motor, including setting up PWM timers for Motor operation
//******************************************************************************

int  MotorLib_Init_Motor (uint8_t motor_id, int pwm_speed, int pole_pairs)
{
    MOTOR_BLOCK  *mtr_blk;
    long         period_ticks;

    mtr_blk = &motor_blk[motor_id];

    memset (mtr_blk, 0, sizeof(MOTOR_BLOCK));    // ensure is cleared out

// in future, add to table (ensure no competing entry)

    mtr_blk->mtr_id     = motor_id;
    mtr_blk->mtr_cb_sig = MOTOR_DRIVER;
    mtr_blk->mtr_state  = MOTOR_STATE_STOPPED;   // denote are currently stopped
    mtr_blk->mtr_Trapezoidal_Index = 1;          // set startup defaults
    mtr_blk->mtr_hall_sector = Hall_CW_DIR_sequence[1];
    mtr_blk->mtr_direction  = DIRECTION_CW;

    mtr_blk->mtr_pole_pairs = pole_pairs;        // save # pole pairs in motor
// derive and compute any additional stuff based on # pole pairs

       //----------------------------------------------------------------------
       // Call specific Initialize for the motor's chip itself (Reset/SPI)
       //----------------------------------------------------------------------
    _g_mtrdrvr->motor_init_controller_chip();

       //----------------------------------------------------------------------
       // Using the MCU clock, get the correct PWM tick count for 20 KHz PWM
       //----------------------------------------------------------------------
    period_ticks    = (_g_mtr_mcu_speed / pwm_speed) - 1;
    _g_period_ticks_ratio = ((float) period_ticks) / 100.00;  // create 100% ratio

    mtr_blk->mtr_pwm_period_ticks = period_ticks;  // save our PWM perid
    _g_pwm_period_ticks = period_ticks;               // and a global copy/DEBUG

    _g_mtrdrvr->motor_init_pwms_comm (mtr_blk, period_ticks); // call MCU dependent PWM init (and any SPI init) routine

    if (_g_mtrdrvr->motor_init_commute_timer != 0L)           // BLDC
       _g_mtrdrvr->motor_init_commute_timer (mtr_blk, 10000); // or is larger/smaller value needed for 20,000 RPM ?

#if defined(F5529_OPEN_LOOP)
      //------------------------------------------------------------------
      // Configure Timer A PWMs:
      //      PWMH  A/U    TA2.2   P2.5    J4-1
      //      PWML  A/U    TA2.1   P2.4    J4-2
      //      PWMH  B/V    TA0.4   P1.5    J4-3
      //      PWML  B/V    TA0.3   P1.4    J4-4
      //      PWMH  C/W    TA0.2   P1.3    J4-5
      //      PWML  C/W    TA0.1   P1.2    J4-6
      //------------------------------------------------------------------

      // Setup startup PWM default configuration
      // Setup HighSide MOSFETs (TB0.2/4/6) as PWMs (set/reset)
      // setup LowSide  MOSFETs (TB0.1/3/5) as logic 'high' Z (off)
  TA0CCR0  = TIMER_PWM_PERIOD-1;       // CCR0 = PWM Period = 15640
  TA0CCTL4 = OUTMOD_7;                 // PWMH V CCR4 = PWM mode reset/set
  TA0CCTL3 = OUTMOD_0;                 // PWML V CCR3 = OUT mode and Low
  TA0CCTL2 = OUTMOD_7;                 // PWMH W CCR2 = PWM mode reset/set
  TA0CCTL1 = OUTMOD_0;                 // PWML W CCR1 = OUT mode and Low

  Current_PWM_DutyCycle = MIN_PWM_DUTYCYCLE; // Initial Dutycycle = 1023

      // Init HighSide PWM outputs (TA2.2, TA0.4, TA0.2) with initial duty cycle
  TA2CCR2 = Current_PWM_DutyCycle;
  TA0CCR4 = Current_PWM_DutyCycle;
  TA0CCR2 = Current_PWM_DutyCycle;

  // Check with Vbus is good to proceed with motor start
  if (Avg_vBUS < 0xFF)
    {
      // Wait until vBUS increases 3.3V?
    }

        //-----------------------------------------
        //  Do initial read of Hall Sensor inputs
        //-----------------------------------------
    Hall_IN = P1IN;
    Hall_IN = ((Hall_IN & 0x0E) >> 1);

        //-------------------------------------------------------
        // Setup PWMs based on wehere Hall Encoder is at startup
        //-------------------------------------------------------
    PreDriver_Sequence = Hall_DIR_sequence [Hall_IN]; // set initial state based on Hall
    PWM_update (PreDriver_Sequence);

#endif

    return (0);            // denote succeeded
}



//******************************************************************************
//  MotorLib_Adc_Init_Speed_Control
//
//         Configure ADC to read potentiometer used for speed control setting.
//         Uses A15  on pin  P6.0
//
//         ADC GPIO port/pin related stuff is in motor_config.h file
//******************************************************************************
int  MotorLib_Adc_Init_Speed_Control (int ADC_channel, CALLBACK_RTN cb_ptr)
{
          // call MCU dependent ADC speed ctl init handler
    if (_g_mtrdrvr->motor_adc_speed_ctl_init != 0L)
       _g_mtrdrvr->motor_adc_speed_ctl_init (ADC_channel, cb_ptr);

    _g_ADC_callback_ptr = cb_ptr;    // save pointer to any callback function

    return (0);                      // denote succeeded
}


//******************************************************************************
//  MotorLib_Encoder_Init
//
//       Configure inputs for Motor Encode (Hall/QEI) Sensors (interrupt driven)
//
//       Encoder_Type = WHEEL_ENCODER_RIGHT/LEFT, HALL_SENSOR, QEI
//       GPIO port/pin related stuff is in motor_config.h file
//******************************************************************************

int  MotorLib_Encoder_Init (uint8_t motor_id, int Encoder_Type)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

          // call MCU dependent GPIO encoder init handler
    _g_mtrdrvr->motor_init_encoder_gpio (mtr_blk, Encoder_Type);

    return (0);            // denote succeeded
}


//*****************************************************************************
//  motor_read_hall_sensors
//
//         read in the hall sensor values, and convert to standard Hall index
//*****************************************************************************

int   motor_read_hall_sensors (uint8_t motor_id)
{
    int          hall_index;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    if (_g_mtrdrvr->motor_read_hall_sensors_handler != 0L)
       hall_index = _g_mtrdrvr->motor_read_hall_sensors_handler (mtr_blk);
       else hall_index = -1;

    return (hall_index);
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


//*****************************************************************************
//  motor_HW_test
//
//      Allow caller to run a hardware test on the board, to ensure all
//      GPIOs, ADCs, PWMs are functioning correctly
//
//*****************************************************************************
int  motor_HW_test (uint8_t motor_id, int duty_cycle_percent)
{
    int          trc;
    int          actual_duty_count;
    long         period_ticks;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    period_ticks       = mtr_blk->mtr_pwm_period_ticks;
    actual_duty_count  = duty_cycle_percent;   // convert to long
    actual_duty_count *= period_ticks;         // convert to ticks
    actual_duty_count /= long_100;             // eliminate * 100 percent piece
    if (actual_duty_count > period_ticks)
       actual_duty_count = period_ticks;       // do not exceed max period

    trc = _g_mtrdrvr->motor_HW_check (mtr_blk, actual_duty_count);

    return (0);                    // denote worked OK
}










// Need a CONTROL / ISSUE_CMD type interface to engage/disengage Stepper motor, etc
// Is advanced, but seperate and distict from start


//******************************************************************************
//  MotorLib_Motor_Engage                            - THIS IS A SHITTY NAME - change it !!!!
//
//  Issue special control actions to the motor control chip.
//
// @brief  Requests the motor to mark the current position as the home position (ABS_POSITION = 0)
// @brief  Issues the Enable command to the motor driver of the specified device
// @brief  Issue the Disable command to the motor driver of the specified device
//
// @note For brush DC motor, when input of different brigdes are parallelized 
// together, the disabling of one bridge leads to the disabling
// of the second one
//
//  Valid actions:  ENGAGE_MOTOR / DISENGAGE_MOTOR
//******************************************************************************
int  MotorLib_Motor_Engage (uint8_t motor_id, int on_off, int flags)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    if (_g_mtrdrvr != 0L  &&  _g_mtrdrvr->motor_do_action_handler != 0L)
       {
         if (on_off == 0)                  // dis-engage the (stepper) motor ?
            _g_mtrdrvr->motor_do_action_handler (mtr_blk, CMD_DISENGAGE_MOTOR, flags);
            else _g_mtrdrvr->motor_do_action_handler (mtr_blk, CMD_ENGAGE_MOTOR, flags);
       }
      else return (ERROR_INVALID_CONTROL_ACTION);

    return (0); 
}





/******************************************************//**\
 *  MotorLib_Check_Status
 *
 * @brief Checks if at least one device has an alarm flag set
 * by reading flag pin position.
 * The flag pin is shared between all devices.
 * @retval One if at least one device has an alarm flag set ,
 * otherwise zero
 **********************************************************/
int  MotorLib_Check_Status (uint8_t motor_id, int type_status, int flags)
{
    int          value;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    if ((_g_mtrdrvr != 0) && (_g_mtrdrvr->motor_check_status != 0))
       value = _g_mtrdrvr->motor_check_status (mtr_blk, type_status, flags);
       else return (ERROR_UNSUPPORTED_FUNCTION);

    return (value);
}


//******************************************************************************
//  MotorLib_Get_Acceleration
//
//              Returns the current acceleration (in PPS) of the specified motor
//******************************************************************************
uint16_t  MotorLib_Get_Acceleration (uint8_t motor_id)
{                                                  
    return (motor_blk[motor_id].mtr_acceleration_pc);  // is in PPS squared
}            


//******************************************************************************
//  MotorLib_Get_Current_Speed
//
//              Returns the current speed (in PPS or RPM) of the specified motor
//******************************************************************************
uint16_t  MotorLib_Get_Current_Speed (uint8_t motor_id, int speed_type)
{
    if (speed_type == SPEED_PPS)
       return (motor_blk[motor_id].mtr_current_velocity_pc);
       else if (speed_type == SPEED_RPM)
               ;
       else return (ERROR_INVALID_SPEED_TYPE);
}


//******************************************************************************
//  MotorLib_Get_Deceleration
//
//
//              Returns the current deceleration (in PPS) of the specified motor
//******************************************************************************
uint16_t  MotorLib_Get_Deceleration (uint8_t motor_id)
{                                                  
    return (motor_blk[motor_id].mtr_deceleration_pc);
}


//******************************************************************************
//  MotorLib_Get_Distance_Moved
//
//            Returns the distance move in the last MOVE or GOTO operation.
//            To do this, it must convert steps (or rotations) moved
//            into the equivalent distance.
//
//            Used by BLDC motors with Encoders and Stepper Motors.
//            Can also be used by BDC motors that have HALL encoded shafts/wheels
//
//    motor_id   (from 0 to 2)
//    direction  FORWARD or BACKWARD
//    distance   distance to move
//    units      units used for the distance:  MM, CM, INCHES, ROTATIONS
//
//    Returns a float, that will be positive if the move was forward,
//                     or negative if the move was backward.
//******************************************************************************
float  MotorLib_Get_Distance_Moved (uint8_t motor_id, UNITS_t units)
{
    float        distance_moved;
    float        steps_moved;
    float        rotations_moved;
    MOTOR_BLOCK  *mtr_blk;


// CAUTION: for smart controllers like powerSTEP01, we can directly send the command to the chip. It will do the motion profile

    mtr_blk = &motor_blk[motor_id];

// need to have staging variables that record:
//    last move type (ROTATIONS vs STEPS/DISTANCE)
//    last start position in steps  or  rotations
//    last end position in steps    or  rotations

    if (mtr_blk->mtr_move_type == MOVE_N_ROTATIONS)
       {
         rotations_moved = (float) mtr_blk->mtr_move_begin_rotation - mtr_blk->mtr_move_end_rotation;
         distance_moved  = rotations_moved * mtr_blk->mtr_distance_per_rotation_mm;
       }
      else
       {      // all other moves end up being in steps
         steps_moved = (float) mtr_blk->mtr_move_begin_position - mtr_blk->mtr_move_end_position;
         distance_moved  = steps_moved * mtr_blk->mtr_distance_per_rotation_mm;
       }

              //----------------------------------------------------------------
              // convert the distance moved into the apporpriate output measurement.
              //----------------------------------------------------------------
    switch (units)
       { case UNITS_INCH:
                   // convert MM into INCHES
                   //     1 mm = 0.0393701 inches
                distance_moved = distance_moved * 0.0393701;
                break;

         case UNITS_CM:
                   // convert MM into CM
                distance_moved = distance_moved / 10.0;  // are 10 MM in each 1 CM
                break;

         case UNITS_MM:
                   // already in MM, no distance conversion needed.
                break;

         default:
                return (ERROR_INVALID_UNITS);
       }

    return (distance_moved);
}


/******************************************************//**
 * @brief  Returns the mark position  of the specified device
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval Mark register value converted in a 32b signed integer 
 **********************************************************/
int32_t  MotorLib_Get_Mark (uint8_t motor_id)
{
    int32_t      mark;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    mark = 0;
    if (_g_mtrdrvr != 0L  &&  _g_mtrdrvr->motor_get_mark_pos != 0L)
       mark = _g_mtrdrvr->motor_get_mark_pos (mtr_blk);
       else return (ERROR_UNSUPPORTED_FUNCTION);

    return (mark);
}


//******************************************************************************
//  MotorLib_Get_Max_Speed
//
//
//              Returns the current max speed setting for the specified motor
//******************************************************************************
uint16_t  MotorLib_Get_Max_Speed (uint8_t motor_id)
{
// but should it be returned in PPS, RPM, DUTY, ...   need UNITS parm

    return (motor_blk[motor_id].mtr_max_velocity_pc);
}


//******************************************************************************
//  MotorLib_Get_Min_Speed
//
//
//              Returns the current min speed setting for the specified motor
//******************************************************************************
uint16_t  MotorLib_Get_Min_Speed (uint8_t motor_id)
{
// but should it be returned in PPS, RPM, DUTY, ...   need UNITS parm

    return (motor_blk[motor_id].mtr_min_velocity_pc);
}


/******************************************************//**\
 *  MotorLib_Get_Motor_Status
 *
 * @brief Issue a CmdStatus to Motor to get its actual state
  * The flag pin is shared between all devices.
 * @retval One if at least one device has an alarm flag set ,
 * otherwise zero
 **********************************************************/
int32_t  MotorLib_Get_Motor_Status (uint8_t motor_id, int type_status)
{
    int          value;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    if ((_g_mtrdrvr != 0) && (_g_mtrdrvr->motor_check_status != 0))
       value = _g_mtrdrvr->motor_get_motor_status (mtr_blk, type_status);
       else return (ERROR_UNSUPPORTED_FUNCTION);

    return (value);
}


//*****************************************************************************
//  MotorLib_Get_Period_Ticks
//
//              Get the PWM's actual period in MCU ticks
//*****************************************************************************
long  MotorLib_Get_Period_Ticks (uint8_t motor_id)
{
    long         pwm_period_ticks;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    pwm_period_ticks = mtr_blk->mtr_pwm_period_ticks;

    return (pwm_period_ticks);
}


//******************************************************************************
//  MotorLib_Get_Position
//
//                      Get Position
// @brief  Returns the ABS_POSITION of the specified motor
// @param[in] motor_id (from 0 to 2)
// @retval ABS_POSITION register value converted in a 32b signed integer
//******************************************************************************
int32_t  MotorLib_Get_Position (uint8_t motor_id)
{
    int32_t      abs_position;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    abs_position = _g_mtrdrvr->motor_get_actual_position (mtr_blk);

    return (abs_position);
}


//*****************************************************************************
//  MotorLib_Get_Speed_Angular_Velocity
//              This sets the motor's desired speed in terms of Angular velocity.
//              This can be derived bu multiplying RPS * 2 PI * ???
//
//   Parms:
//      motor_num:                number of the motor to update (1 or 2)
//      target_velovity_per_sec:  requested angular velocity per second.
//      actual_velovity_per_sec:  current, measured  angular velocity per second
//*****************************************************************************
void  MotorLib_Get_Speed_Angular_Velocity (uint8_t motor_id,
                                             uint16_t *target_velovity_per_sec,
                                             uint16_t *actual_velovity_per_sec)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    *target_velovity_per_sec = mtr_blk->mtr_angular_velocity_setpt;
    *actual_velovity_per_sec = mtr_blk->mtr_angular_velocity_actual;
}


//*****************************************************************************
//  MotorLib_Get_Speed_Rpm                                     motor_get_speed_rps
//
//              This gets the motor's current speed in (mechanical) revolutions
//              per minute.
//
//   Parms:
//      motor_num:        number of the motor to update (1 or 2)
//      target_rps_speed: requested speed in (mechanical) revolutions per second
//      actual_rps_speed: current, actual measured speed.
//*****************************************************************************
void  MotorLib_Get_Speed_Rpm (uint8_t motor_id, uint16_t *target_rpm_speed,
                                  uint16_t *actual_rpm_speed)
{

// or make this a by SPEED_UNITs call ???   RPM / PPS / DUTY  ???

    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    *target_rpm_speed = mtr_blk->mtr_rpm_setpt;
    *actual_rpm_speed = mtr_blk->mtr_rpm_actual;
}

/******************************************************//**
 *  MotorLib_Goto_Home
 *
 * @brief  Requests the motor to move to the home position (ABS_POSITION = 0)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
int  MotorLib_Goto_Home (uint8_t motor_id)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    _g_mtrdrvr->motor_perform_move (mtr_blk, MOVE_TO_HOME, 0, 0);
} 

  
/******************************************************//**
 *  MotorLib_Goto_Mark
 *
 * @brief  Requests the motor to move to the mark position 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/
int  MotorLib_Goto_Mark (uint8_t motor_id)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    _g_mtrdrvr->motor_perform_move (mtr_blk, MOVE_TO_MARK, 0, 0);
} 



//******************************************************************************
// MotorLib_Goto_Position
// 
//            Requests the motor to move to a specified (absolute) position,
//            based upon where Home (positon 0) is defined as.
//
//            Used by BLDC motors with Encoders, and Stepper Motors.
//
//            For Steppers, even a GoToHome (aka position 0) causes us to
//            generate a step count, because we query where the Stepper's
//            internal absolute position is, and then move the stepper +/- steps
//            to get back to home (position 0).   Ditto for GoToMark.
//
//   motor_id (from 0 to 2)
//   targetPosition - absolute position in steps
//******************************************************************************
void  MotorLib_Goto_Position (uint8_t motor_id, int32_t targetPosition)
{
    int          direction;
    int32_t      currentPos;
    int32_t      steps;
    MOTOR_BLOCK  *mtr_blk;

// CAUTION: for smart controllers like powerSTEP01, we can directly send the command to the chip. It will do the motion profile

    mtr_blk = &motor_blk[motor_id];

    if (mtr_blk->mtr_homed_flag == 0)
       MotorLib_Set_Home (motor_id);        // force a "Logical Home"

       // ensure motor is stopped
    if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE) 
       {
         _g_mtrdrvr->motor_stop_handler (mtr_blk, STOP_HARD_BRAKE, 0);
       }

       //--------------------------------------------------------------
       // Get current Absolute position of the Stepper or BLDC encoder
       //--------------------------------------------------------------
    currentPos = _g_mtrdrvr->motor_get_actual_position (mtr_blk);
    mtr_blk->mtr_reported_position = currentPos;
    mtr_blk->mtr_current_position  = currentPos;

       //---------------------------------------------------------
       // Compute the number of steps to be performed, based upon
       // what our current position is, and where we need to go.
       //---------------------------------------------------------
    steps = targetPosition - mtr_blk->mtr_current_position;
  
    if (steps >= 0) 
       {         // we need to move forward
         mtr_blk->mtr_steps_to_move = steps;
         direction = DIRECTION_FORWARD;
       } 
      else 
       {         // we need to move backward
         mtr_blk->mtr_steps_to_move = -steps;
         direction = DIRECTION_BACKWARD;
       }
  
    mtr_blk->mtr_move_begin_position = currentPos;
    mtr_blk->mtr_move_end_position   = currentPos + steps;

    if (steps != 0) 
       {           // calculate ramp to get to target position
         mtrlib_calculate_speed_profile (motor_id, mtr_blk->mtr_steps_to_move);

                   // denote we are now in RUN state
         mtr_blk->mtr_state     = MOTOR_STATE_RUN;  // set state: we are in RUN mode
         mtr_blk->mtr_move_type = MOVE_TO_POSITION; // are going to a position

                   // set the direction to move, 
         MotorLib_Set_Direction (motor_id, direction);

         _g_mtrdrvr->motor_perform_move (mtr_blk, MOVE_TO_POSITION, // start motor
                                         direction, mtr_blk->mtr_steps_to_move);
       }  
}


//******************************************************************************
//  MotorLib_Move_Distance
//
//            Moves the motor a specified distance forward or
//            backward from its current position.
//
//            Used by BLDC motors with Encoders and Stepper Motors
//
//    motor_id   (from 0 to 2)
//    direction  FORWARD or BACKWARD
//    distance   distance to move
//    units      units used for the distance:  MM, CM, INCHES, ROTATIONS
//******************************************************************************
int  MotorLib_Move_Distance (uint8_t motor_id,  int direction,
                                 float distance,   UNITS_t units)
{
    long         steps_to_move;
    MOTOR_BLOCK  *mtr_blk;

// CAUTION: for smart controllers like powerSTEP01, we can directly send the command to the chip. It will do the motion profile

    mtr_blk = &motor_blk[motor_id];

    if (units != UNITS_ROTATIONS)
       {      //----------------------------------------------------------------
              // convert the distance into the equivalent number of steps.
              // note that the baseline distance is always in millimeters (MM)
              //----------------------------------------------------------------
         switch (units)
            { case UNITS_INCH:
                        // convert INCHES into CM, then drop thru
                        //     1 mm = 0.0393701 inches
                     distance = distance * 25.4;      // are 25.4 MM in 1 inch
                     break;

              case UNITS_CM:
                        // convert CM into MM, then drop thru
                     distance = distance * 10.0;      // are 10 MM in each 1 CM
                     break;

              case UNITS_MM:
                        // already in MM, no distance conversion needed.
                     break;

              default:
                     return (ERROR_INVALID_UNITS);
            }
                        // calculate number of steps needed for that distance
         steps_to_move = mtr_blk->mtr_distance_per_step_mm * distance;
       }

       // for rotations ...
    if (units == UNITS_ROTATIONS)
       {      // setup to count Hall Encoder interrupts
       }
      else MotorLib_Move_Steps (motor_id, direction, steps_to_move);

    return (0);                    // denote worked OK
}


//******************************************************************************
//  MotorLib_Move_Steps
//
//            Moves the motor a specified number of steps forward or
//            backward from its current position.
//
//            Used by BLDC motors with Encoders, and Stepper Motors
//
//    motor_id (from 0 to 2)
//    direction FORWARD or BACKWARD
//    stepCount Number of steps to perform
//******************************************************************************
void  MotorLib_Move_Steps (uint8_t motor_id, int direction,  uint32_t stepCount)
{
    int32_t      current_position;
    MOTOR_BLOCK  *mtr_blk;

// CAUTION: for smart controllers like powerSTEP01, we can directly send the command to the chip. It will do the motion profile

    mtr_blk = &motor_blk[motor_id];

    if (mtr_blk->mtr_homed_flag == 0)                  // force a "Logical Home"
       _g_mtrdrvr->motor_do_action_handler (mtr_blk, CMD_SET_HOME, 0);

       // ensure motor is stopped
    if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE) 
       {
         _g_mtrdrvr->motor_stop_handler (mtr_blk, STOP_HARD_BRAKE, 0);
       }
  
    if (stepCount != 0) 
       {
         mtr_blk->mtr_steps_to_move = stepCount;

             //--------------------------------------------------------------
             // Get current Sbsolute position of the Stepper or BLDC encoder
             //--------------------------------------------------------------
         current_position = _g_mtrdrvr->motor_get_actual_position (mtr_blk);

         mtr_blk->mtr_current_position    = current_position;

         mtr_blk->mtr_move_begin_position = current_position;
         mtr_blk->mtr_move_end_position   = current_position + stepCount;

             // calculate ramp to get to target position
         mtrlib_calculate_speed_profile (motor_id, mtr_blk->mtr_steps_to_move);

             // put us into RUN state
         mtr_blk->mtr_state     = MOTOR_STATE_RUN; // set state: we are in RUN mode
         mtr_blk->mtr_move_type = MOVE_N_STEPS;    // are stepping N relative steps

             // set the direction to move
         MotorLib_Set_Direction (motor_id, direction);

         _g_mtrdrvr->motor_perform_move (mtr_blk, MOVE_N_STEPS,   // start motor
                                         direction, mtr_blk->mtr_steps_to_move);
      }  
}



//******************************************************************************
//  MotorLib_Run
//                           Run                                 replaces motor_start
//
// @brief  Runs the motor. It will accelerate from the min 
// speed up to the max speed by using the device acceleration.
// @param[in] motor_id (from 0 to 2)
// @param[in] direction FORWARD or BACKWARD
//******************************************************************************
void  MotorLib_Run (uint8_t motor_id, int direction)  // have a flag for ramp/variable_speed ?
{                                                  // or just set a flag in speed_ramp call and trigger off the flag
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

       // ensure motor is stopped
    if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE) 
       {
         _g_mtrdrvr->motor_stop_handler (mtr_blk, STOP_HARD_BRAKE, 0);
       }

    mtr_blk->mtr_state     = MOTOR_STATE_RUN;       // set state: we are in RUN mode
    mtr_blk->mtr_move_type = MOVE_FREE_RUN;     // and we are "free" running
    mtr_blk->mtr_operation_performed = OP_RUN;  //    aka running open loop

        // set the direction to move
    MotorLib_Set_Direction (motor_id, direction);

    _g_mtrdrvr->motor_run_handler (mtr_blk, direction);      // start motor
}


/******************************************************//**
 *  MotorLib_Set_Acceleration
 *
 *                     Set Acceleration
 * @brief  Changes the acceleration of the specified device
 * @param[in] motor_id (from 0 to 2)
 * @param[in] newAcc New acceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool  MotorLib_Set_Acceleration (uint8_t motor_id, uint16_t newAcc)
{                                                  
    bool         cmdExecuted;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    cmdExecuted = FALSE;
    if ((newAcc != 0)
     && ((mtr_blk->mtr_motion_state == MOTION_NOT_ACTIVE) || (mtr_blk->mtr_operation_performed == OP_RUN)))
       {
         mtr_blk->mtr_acceleration_pc = newAcc;
         cmdExecuted = TRUE;
       }    
 
    return (cmdExecuted);
}            


/******************************************************//**
 *  MotorLib_Set_Deceleration
 *
 * @brief  Changes the deceleration of the specified device
 * @param[in] motor_id (from 0 to 2)
 * @param[in] newDec New deceleration to apply in pps^2
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is executing 
 * a MOVE or GOTO command (but it can be used during a RUN command)
 **********************************************************/
bool  MotorLib_Set_Deceleration (uint8_t motor_id, uint16_t newDec)
{                                                  
    bool         cmdExecuted;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    cmdExecuted = FALSE;
    if ((newDec != 0)
     && (mtr_blk->mtr_motion_state == MOTION_NOT_ACTIVE || mtr_blk->mtr_operation_performed == OP_RUN))
        {
          mtr_blk->mtr_deceleration_pc = newDec;
          cmdExecuted = TRUE;
        }

    return cmdExecuted;
}


//*****************************************************************************
//  MotorLib_Set_Decay
//
//      Update Motor decay settings. These dictate how fast the motor stops.
//
//         DECAY_FAST = short motor re-circulating current to ground, to
//                      drain out motor current as fast as possible.
//                      It effectively acts like a brake.
//         DECAY_SLOW = let motor current re-circulate till it fades out.
//                      This "coasts" the motor to a stop.
//*****************************************************************************
int  MotorLib_Set_Decay (uint8_t motor_id,  int decay_type)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

       // set and save the motor decay type
    if (decay_type == DECAY_SLOW)
       mtr_blk->mtr_decay_mode = DECAY_SLOW;
       else mtr_blk->mtr_decay_mode = DECAY_FAST;

    return (0);            // denote succeeded
}


//*****************************************************************************
//  MotorLib_Set_Direction
//
//      Update Motor PWM and GPIO settings to set desired Clockwise (CW) or
//      CounterClockwise (CCW) direction.
//
//*****************************************************************************
int  MotorLib_Set_Direction (uint8_t motor_id,  int mtr_direction)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    mtr_blk->mtr_direction = mtr_direction;       // save the new direction

    if (mtr_blk->mtr_state == MOTOR_STATE_RUN)
       {     // if running, immediately update to the new Direction by
             // calling the MCU dependent Set Direction handler
         _g_mtrdrvr->motor_set_direction_handler (mtr_blk, mtr_direction);
       }
      else {    // Is STOPPED, so just keep the saved value.
                // It will get applied to the motor during motor_start() call
           }

    return (0);            // denote succeeded
}


//******************************************************************************
//  MotorLib_Set_Distance_Per_Step
//
//            Sets the amount of distance (in MM, INCHES, ...) that a
//            single step acheives
//
//            Used by BLDC motors with Encoders and Stepper Motors.
//
//    motor_id   (from 0 to 2)
//    direction  FORWARD or BACKWARD
//    distance   distance moved per step
//    units      units used for the distance:  MM, CM, INCHES
//******************************************************************************
int  MotorLib_Set_Distance_Per_Step (uint8_t motor_id,  int direction,
                                        float distance_per_step,  UNITS_t units)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    switch (units)
        { case UNITS_INCH:
                    // convert INCHES into CM, then drop thru
                    //     1 mm = 0.0393701 inches
                 distance_per_step = distance_per_step * 25.4;  // are 25.4 MM in 1 inch
                 break;

          case UNITS_CM:
                    // convert CM into MM, then drop thru
                 distance_per_step = distance_per_step * 10.0;  // are 10 MM in each 1 CM
                 break;

          case UNITS_MM:
                    // already in MM, no distance conversion needed.
                 break;

          default:
                 return (ERROR_INVALID_UNITS);
        }

    mtr_blk->mtr_distance_per_step_mm = distance_per_step;

    return (0);                    // everything OK
}


//******************************************************************************
//  MotorLib_Set_Distance_Per_Rotation
//
//            Sets the amount of distance (in MM, CM, INCHES, ...) that a
//            single shaft rotation achieves.
//
//            Used by BDC motors with HALL shaft/wheel encoders (ROTATIONS)
//
//    motor_id   (from 0 to 2)
//    direction  FORWARD or BACKWARD
//    distance   distance moved per step
//    units      units used for the distance:  MM, CM, INCHES
//******************************************************************************
int  MotorLib_Set_Distance_Per_Rotation (uint8_t motor_id,
                                     float distance_per_rotation, UNITS_t units)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    switch (units)
        { case UNITS_INCH:
                    // convert INCHES into MM, then drop thru
                    //     1 mm = 0.0393701 inches
                 distance_per_rotation = distance_per_rotation * 25.4;  // are 25.4 MM in 1 inch
                 break;

          case UNITS_CM:
                    // convert CM into MM, then drop thru
                 distance_per_rotation = distance_per_rotation * 10.0;  // are 10 MM in each 1 CM
                 break;

          case UNITS_MM:
                    // already in MM, no distance conversion needed.
                 break;

          default:
                 return (ERROR_INVALID_UNITS);
        }

    mtr_blk->mtr_distance_per_rotation_mm = distance_per_rotation;

    return (0);                     // everything OK
}



//******************************************************************************
//  MotorLib_Set_Home
//                          Set Home
//
// @brief  Set current position to be the Home position (ABS pos set to 0)
// @param[in] motor_id (from 0 to 2)
//******************************************************************************
void  MotorLib_Set_Home (uint8_t motor_id)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    _g_mtrdrvr->motor_do_action_handler (mtr_blk, CMD_SET_HOME, 0);
}


//******************************************************************************
//  MotorLib_Set_Mark
//                          Set Mark
// @brief  Sets current position to be the Mark position 
// @param[in] motor_id (from 0 to 2)
// @retval None
//******************************************************************************
void  MotorLib_Set_Mark (uint8_t motor_id)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    _g_mtrdrvr->motor_do_action_handler (mtr_blk, CMD_SET_MARK, 0);
}



//******************************************************************************
//  MotorLib_Set_Maximum_Speed
//
// @brief  Changes the max speed of the specified motor
// @param[in] motor_id (from 0 to 2)
// @param[in] newMaxSpeed New max speed  to apply in pps
// @retval true if the command is successfully executed, else false
// @note The command is not performed is the device is executing 
// a MOVE or GOTO command (but it can be used during a RUN command).
//******************************************************************************
int  MotorLib_Set_Maximum_Speed (uint8_t motor_id, uint16_t new_maximum_speed)
{                                                  
    int          cmdExecuted;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];
  
    cmdExecuted = FALSE;

#if defined(MOTOR_IS_STEPPER) || defined(MOTOR_IS_BLDC)
    if (new_maximum_speed >= L6474_MIN_PWM_FREQ && new_maximum_speed <= L6474_MAX_PWM_FREQ)
      {
         if ((mtr_blk->mtr_min_velocity_pc <= new_maximum_speed)
         &&  ((mtr_blk->mtr_motion_state == MOTION_NOT_ACTIVE) || (mtr_blk->mtr_operation_performed == OP_RUN)))
            {
              mtr_blk->mtr_max_velocity_pc = new_maximum_speed;
              cmdExecuted = TRUE;
            }
      }
     else return (ERROR_MAX_SPEED_TOO_HIGH);
#endif

    return cmdExecuted;
}                                                     



//******************************************************************************
//  MotorLib_Set_Minimum_Speed
//
//     @brief  Changes the min speed of the specified motor
//     @param[in] motor_id (from 0 to 2)
//     @param[in] newMinSpeed New min speed  to apply in pps
//     @retval true if the command is successfully executed, else false
//    @note The command is not performed is the device is executing 
//     a MOVE or GOTO command (but it can be used during a RUN command).
//******************************************************************************
int  MotorLib_Set_Minimum_Speed (uint8_t motor_id, uint16_t new_minimum_speed)
{                                                  
    int          cmdExecuted;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    cmdExecuted = FALSE;

#if defined(MOTOR_IS_STEPPER) || defined(MOTOR_IS_BLDC)
    if (new_minimum_speed >= L6474_MIN_PWM_FREQ && new_minimum_speed <= L6474_MAX_PWM_FREQ)
       {
         if ((new_minimum_speed <= mtr_blk->mtr_max_velocity_pc)
           && ((mtr_blk->mtr_motion_state == MOTION_NOT_ACTIVE) || (mtr_blk->mtr_operation_performed == OP_RUN)))
            {
              mtr_blk->mtr_min_velocity_pc = new_minimum_speed;
              cmdExecuted = TRUE;
            }
       }
      else return (ERROR_MIN_SPEED_TOO_LOW);
#endif

    return cmdExecuted;
}



//*****************************************************************************
//  MotorLib_Set_Pwm_Duty_Count
//              Directly set the duty cycle values to be used for a specific motor.
//              Duty cycle is explicitly passed as PWM ticks, not as percent.
//              For BLDC, the duty cycle sets the motor's torque (power).
//              (More juice, more power)
//
//   Parms:
//      motor_num:   number of the motor to update (1 or 2)
//      pwm_duty:    actual duty cycle PWM count value to be loaded in PWM
//*****************************************************************************
int  MotorLib_Set_Pwm_Duty_Count (uint8_t motor_id, int16_t actual_pwm_duty_count)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    mtr_blk->mtr_pwm_duty_count = actual_pwm_duty_count;  // save latest PWM duty cycle count

    if (mtr_blk->mtr_state == MOTOR_STATE_RUN)
       {     // if running, immediately update to the new Duty Cycle by
             // calling the MCU dependent Set Duty Cycle handler
         _g_mtrdrvr->motor_set_pwm_duty_count_handler (mtr_blk,
                                                       actual_pwm_duty_count);
       }
      else {    // If stopped, just keep the saved value.
                // It will get applied to the motor during motor_start() call
           }


    return (0);            // denote succeeded
}


//*****************************************************************************
//  MotorLib_Set_Speed                                    was motor_set_duty_cycle
//
//              Set the current speed to be used for a specific motor.
//
//              This is primarily used by BDC motors for duty cycle control,
//              or Steppers to set a specific fixed step rate.
//
//   Parms:
//      motor_num:           number of the motor to update (1 or 2)
//      duty_cycle_percent:  % of duty cycle to apply, ranging from 0 to 100
//      speed_units:         value is defined as PPS, RPM, DUTY_CYCLE, ...
//*****************************************************************************
int  MotorLib_Set_Speed (uint8_t motor_id, uint16_t speed_value, int speed_units)
{
    int          rc;
    uint16_t     duty_cycle_percent;
    long         period_ticks;
    long         actual_duty_count;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];
    rc      = 0;                             // assume good results

    mtr_blk->mtr_speed_units = speed_units;
    period_ticks = mtr_blk->mtr_pwm_period_ticks;
    switch (mtr_blk->mtr_speed_units)
      {
       case SPEED_PERCENT_DUTY:
                 //------------------------------------------------------------
                 // speed is in per cent duty cycle: 0.0 to 100.0  (BDC motors)
                 // Requested duty cycle is passed as a percentage value (0-100).
                 // The duty cycle sets the motor's speed. (More juice, more speed)
                 //------------------------------------------------------------
           duty_cycle_percent = speed_value;
                 // convert to ticks using integer based "cave man calculus"
           actual_duty_count  = duty_cycle_percent;   // convert to long
           actual_duty_count *= period_ticks;         // convert to ticks
           actual_duty_count /= long_100;             // eliminate * 100 percent piece
           if (actual_duty_count > period_ticks)
              actual_duty_count = period_ticks;       // do not exceed max period

           if (actual_duty_count > 2)
              actual_duty_count--;     // deduct count by 1 because base of counter = 0

           mtr_blk->mtr_pwm_duty_count = actual_duty_count; // save latest PWM duty count

           if (mtr_blk->mtr_state == MOTOR_STATE_RUN)
              {     // if running, immediately update to the new Duty Cycle by
                    // calling the MCU dependent Set Duty Cycle handler
                _g_mtrdrvr->motor_set_pwm_duty_count_handler (mtr_blk,
                                                    actual_duty_count);
              }
             else {    // Is STOPPED, so just keep the saved value.
                       // It will get applied to the motor during motor_start() call
                  }
           break;

       case SPEED_RPM:
                 //------------------------------------------------------------
                 // speed is in RPM
                 //------------------------------------------------------------
           rc = MotorLib_Set_Speed_Rpm (motor_id, speed_value);
           break;

       case SPEED_PPS:
                 //------------------------------------------------------------
                 // speed is in Pulses Per Second (steps or QEI encoder pulses)
                 //------------------------------------------------------------
           break;
      }

    return (rc);            // return succeed/error indication
}


//*****************************************************************************
//  MotorLib_Set_Speed_Rpm                                     MotorLib_Set_Speed_rps
//
//              This sets the motor's desired speed in (mechanical) revolutions
//              per minute. It causes the commutation timing to either speed up
//              or slow down.
//
//   Parms:
//      motor_num:   number of the motor to update (1 or 2)
//      rpm_speed:   speed in (mechanical) revolutions per minute.
//*****************************************************************************
int  MotorLib_Set_Speed_Rpm (uint8_t motor_id, uint16_t rpm_speed)
{
    long         total_commutations_per_min;
    long         timer_ticks;
    long         commute_ticks;
    MOTOR_BLOCK  *mtr_blk;

// Or is this now just a variation of set_speed, which allows by RPM, PPS, DUTY

    mtr_blk = &motor_blk[motor_id];

    if (rpm_speed > MOTOR_MAX_RPM)
       return (MOTOR_MAX_RPM_EXCEEDED);

         // save the requested speed value as our set point  (for PID)
    mtr_blk->mtr_rpm_setpt = rpm_speed;

// also convert to angular velocity and save it in mtr_blk

       //---------------------------------------------------------------------
       // First, depending upon the # pole pairs, detemine # commutations/sec
       // that is needed. Note it is * 6 because there are 6 steps / commutation
       //---------------------------------------------------------------------
    total_commutations_per_min = rpm_speed * MOTOR_POLE_PAIRS * 6 * 60;

       //-----------------------------------------------------------------------
       // The determine the speed (in timer ticks) that are needed to meet that
       // The commutation timer is spec'd in terms of 100 microsecond ticks,
       // (0.0001 secs) to give better resolution. There are 10000 such ticks/sec
       //-----------------------------------------------------------------------
    timer_ticks = 10000 / total_commutations_per_min;

       //--------------------------------------------------------------
       // The set commutation timer to pop at that frequency, so that
       // we can update the motor commutation steps at that frequency.
       //--------------------------------------------------------------

// probably need intermediate scaling calculation ... to scale to actual timer's period ticks

//  commute_ticks = mtr_blk->mtr_commute_timer_period_ticks / ???
    commute_ticks = timer_ticks;                           // TEMP HACK !!! ???

// pass the converted value to lower layer - is this really needed  ???
// when should it be started/enabled
//    ==> determine if motor_start issued !
    _g_mtrdrvr->motor_set_speed_rpm_handler (mtr_blk, commute_ticks);

    return (0);            // denote succeeded
}


//*****************************************************************************
//  MotorLib_Set_Speed_Angular_Velocity                         NEED ???
//
//              This sets the motor's desired speed in terms of Angular velocity.
//              This can be derived bu multiplying RPS * 2 PI * ???
//
//   Parms:
//      motor_num:          number of the motor to update (1 or 2)
//      velovity_per_sec:   angular velocity per second.
//*****************************************************************************
int  MotorLib_Set_Speed_Angular_Velocity (uint8_t motor_id, 
                                              uint16_t velovity_per_sec)
{
    long         total_commutations_per_sec;
    long         timer_ticks;
    long         commute_ticks;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    if (velovity_per_sec > MOTOR_MAX_RPM)     // FIX
       return (MOTOR_MAX_RPM_EXCEEDED);

         // save the requested speed value as our set point  (for PID)
    mtr_blk->mtr_angular_velocity_setpt = velovity_per_sec;

// also convert to RPS and save it in mtr_blk

// TBD - may be easier to compute in RPS then back convert to AVS

// TBD to fill in rest of commutation timer and start it/enable it

// pass the converted value to lower layer - is this really needed  ???
    _g_mtrdrvr->motor_set_speed_rpm_handler (mtr_blk, commute_ticks);

    return (0);            // denote succeeded
}


//******************************************************************************
//  MotorLib_Set_Step_Mode
//
//                      Set Step Mode
// @brief  Set the stepping mode 
// @param[in] motor_id (from 0 to 2)
// @param[in] stepMode from full step to 1/16 microstep as specified in enum STEPPER_MODE_t
//******************************************************************************
void  MotorLib_Set_Step_Mode (uint8_t motor_id, int stepper_mode)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    _g_mtrdrvr->motor_set_step_mode (mtr_blk, stepper_mode, 0);   // set STEPPER mode
}


/******************************************************//**
 *  MotorLib_Set_Torgue
 *
 * @brief Set the motor torque           ADVANCED - POWERSTEP and FOC
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * @param[in] torqueMode Torque mode as specified in enum motorTorqueMode_t
 * @param[in] torqueValue in % (from 0 to 100)
 * @retval None
 **********************************************************/

int  MotorLib_Set_Torgue (uint8_t motor_id, int torque_mode, int16_t torque_value)
{
    int          value;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    if ((_g_mtrdrvr != 0) && (_g_mtrdrvr->motor_set_torque != 0))
       value = _g_mtrdrvr->motor_set_torque (mtr_blk, torque_mode, torque_value);
       else return (ERROR_UNSUPPORTED_FUNCTION);

    return (value);
}


//*****************************************************************************
//  MotorLib_Speed_Ramp_Check_Complete                  motor_ramp_check_complete
//
//           Polls to see if the motor ramp up/down is complete.
//           Returns:  1 if complete,  0 if not complete.
//
//           Note that using the callback (to set a complete flag in main)
//                is more efficient.
//*****************************************************************************
int   MotorLib_Speed_Ramp_Check_Complete (uint8_t motor_id)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    if (mtr_blk->mtr_id == 0)
       {          // process 1st Motor
         if (mtr_blk->mtr_ramp_pwm_increment == 0)
            return (1);                           // ramp is complete
            else return (0);                      // ramp is not complete
       }
      else {     // process 2nd Motor
             if (mtr_blk->mtr_ramp_pwm_increment == 0)
                return (1);                       // ramp is complete
                else return (0);                  // ramp is not complete
           }
}


//*****************************************************************************
//  MotorLib_Speed_Ramp                                           motor_ramp_duty
//
//           Ramps the duty cycle (or steps/sec), and hence speed of a motor
//           up or down.
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

// change this, such that:  set_speed_min = begin ramp
//                          set_speed_max = end ramp   (HMMMM !)
//
//      MODIFY THIS to DO THE SAME for PPS Steps/second or RPMs/sec
//             
int  MotorLib_Speed_Ramp (uint8_t motor_id, uint16_t ramp_up_down,
                       uint16_t begin_duty_cycle_pc, uint16_t end_duty_cycle_pc,
                       uint16_t ramp_duration_ms, uint16_t increment_frequency_ms,
                       CALLBACK_RTN  cb_ptr)
{
    long        start_duty_pwm_ticks;
    long        end_duty_pwm_ticks;
    long        total_pwm_ticks;
    long        total_milliseconds;
    long        total_increments;
    long        milli_inc;
    long        pwm_tick_increment;
    long        period_ticks;
    int         ramp_end;
    MOTOR_BLOCK *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    if (ramp_up_down > RAMP_DOWN || begin_duty_cycle_pc > 100 
     || end_duty_cycle_pc > 100)
       return (-1);                                        // invalid parameter

       //--------------------------------------------------------------------
       // use cave man calculus to compute # ticks in the duty percentage:
       //   (duty_cycle_percent * period_ticks)/100 = # ticks in that duty %
       //--------------------------------------------------------------------
    period_ticks         = mtr_blk->mtr_pwm_period_ticks;
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
    _g_motor_ramp_active_flag = 1;           // turn ON ramp flag

       //-----------------------------------------------------------------------
       //  Now start the ramp, by calling the MCU dependent Set Duty Cycle
       //  handler, to set the initial RAMP duty value (real duty count, not %)
       //-----------------------------------------------------------------------
    _g_mtrdrvr->motor_set_pwm_duty_count_handler (mtr_blk,
                                                  begin_duty_cycle_pc);

    if (mtr_blk->mtr_state != MOTOR_STATE_RUN)
       {     // and startup motor if it was stopped
         MotorLib_Run (motor_id, mtr_blk->mtr_direction);
       }

    return (0);                           // denote succeeded
}


//******************************************************************************
//  MotorLib_Speed_Ramp_Update                             motor_ramp_update_check
//
//        Called by SYSTICK ISR when it sees that a ramp operation is requested.
//        Checks if it is time to update the PWM RAMP, and if so, do it.
//
//        Note:  mtr_x_ramp_pwm_increment is positive if RAMP_UP,
//                      "           "     is negative if RAMP_DOWN.
//******************************************************************************
void  MotorLib_Speed_Ramp_Update (void)
{
    int16_t      new_duty_count;
    int          hit_end_point;
    MOTOR_BLOCK  *mtr_blk;

//  mtr_blk = &motor_blk[motor_id];

// ISSUE: how get pointers to MOTOR_BLOCKs from SYSTICK  ==> Queue Header
//                              ?? NEED TO FIX ASAP  !!!
    if (_g_motor_id == 0xFF)
       return;                              // motor has not been intialized yet
    mtr_blk = &motor_blk[_g_motor_id];

    hit_end_point = 0;
    if (mtr_blk->mtr_ramp_pwm_increment != 0)   // mtr_blk->mtr_ramp_pwm_increment
       { mtr_blk->mtr_systick_count++;          // update how many Systicks this pass
         if (mtr_blk->mtr_systick_count >= mtr_blk->mtr_systick_increment)
            {     //----------------------------------------------------------
                  // go ahead and step PWM duty count to next step up or down
                  //----------------------------------------------------------
              new_duty_count = mtr_blk->mtr_pwm_duty_count + mtr_blk->mtr_ramp_pwm_increment;
              if (mtr_blk->mtr_ramp_pwm_increment > 0)
                 {        // is positive, so must be ramping up
                   if (new_duty_count >= mtr_blk->mtr_ramp_pwm_end)
                      hit_end_point = 1;          // we have hit our last update
                 }
                 else {   // is negative, so must be ramping down
                        if (new_duty_count <= mtr_blk->mtr_ramp_pwm_end)
                           hit_end_point = 1;     // we have hit our last update
                      }
              if (hit_end_point == 1)
                 { new_duty_count = mtr_blk->mtr_ramp_pwm_end;  // set final value
                   mtr_blk->mtr_ramp_pwm_increment = 0;    // no more ramp steps needed
                 }

// MAJOR BAD HACK

              MotorLib_Set_Pwm_Duty_Count (_g_motor_id, new_duty_count); // update actual PWM counts
              mtr_blk->mtr_systick_count = 0;    // clear count for next pass
              if (hit_end_point == 1 && mtr_blk->mtr_callback != NULL_MPTR)
                 {       //--------------------------------------------------
                         //       Invoke   Ramp  Done   CALLBACK
                         //--------------------------------------------------
                   mtr_blk->mtr_callback(1);     // tell main() we are done
                 }
            }
       }

    if (mtr_blk->mtr_ramp_pwm_increment == 0)
       _g_motor_ramp_active_flag = 0;            // all done, turn off ramp flag
}


//******************************************************************************
//  MotorLib_Stop
//              Stop the motor, and ensure it remains stopped by setting PWM = 0
//              STOP_HARD_BRAKE, STOP_SOFT_BRAKE (coast), STOP_HIZ (Coast).
//
//               A BDC "brake" sequence sets both 1A/2A or 3A/4A to LOW.
//               A Stepper "brake" sequence ...
//               A BLDC "brake" sequence ...
//
//               A STOP_HARD_STOP forces any re-circulating motor current to
//               ground, and stops the motor faster, rather than than just a 
//               STOP_SOFT_BRAKE which turns off the PWM and lets it coast to
//               a stop.
//******************************************************************************
int  MotorLib_Stop (uint8_t motor_id, int stop_type)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    mtr_blk->mtr_state = MOTOR_STATE_STOPPED;

        // call MCU dependent Motor Stop handler.
        // It will set PWW duty value to 0, and disable motor bridge as required
    _g_mtrdrvr->motor_stop_handler (mtr_blk, stop_type, 0);
//      // call MCU dependent Set PWM Duty Count handler
//  _g_mtrdrvr->motor_set_pwm_duty_count_handler (mtr_blk, 0);

// ??? !!! ???  REVISIT THIS !!!
    mtr_blk->mtr_pwm_duty_count = 0;                  // set PWM duty = 0


    return (0);            // denote succeeded
}


/******************************************************//**
 *  MotorLib_Wait_Complete
 *
 * @brief  Locks until the device state becomes Inactive
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES - 1)
 * For L6208: dummy parameter for compatibility with motor.h
 * @retval None
 **********************************************************/

int  MotorLib_Wait_Complete (uint8_t motor_id, int type_wait, int flags)
{
    int          value;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    if ((_g_mtrdrvr != 0) && (_g_mtrdrvr->motor_wait_complete != 0))
       value = _g_mtrdrvr->motor_wait_complete (mtr_blk, type_wait, flags);
       else return (ERROR_UNSUPPORTED_FUNCTION);

    return (value);
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                    ADVANCED  MOTOR  LIBRARY  ROUTINEs
//
//                            STEPPER  MOTORS
//
//******************************************************************************
//******************************************************************************
//******************************************************************************



//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                    ADVANCED  MOTOR  LIBRARY  ROUTINEs
//
//                            BLDC  MOTORS
//
//******************************************************************************
//******************************************************************************
//******************************************************************************


//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                          BLDC  MOTOR  TABLES
//
//                               TABLES
//
//  Should these go in the CHIP Level Driver,  __OR__ be exported in by USER call
//******************************************************************************
//******************************************************************************
//******************************************************************************

          //--------------------------------------------------------------------
          //   TRAPEZOIDAL -> HALL  Value  Mapping  Table    Trapezoidal  6-step
          //--------------------------------------------------------------------
          // Mapping between the Hall sequence 000-111 and the
          // High-Side / Low-Side (HS/LS) PWMs.
          // This is fixed for 120 degree 3-phase BLDC Commutation

       // CLOCKWISE (CW) Direction = 0x00  Hall_Sector    ABC   UVW  Trapez Step
unsigned char  Hall_CW_DIR_sequence[]  = { 0x00,      //  ---   ---              THIS SEQ MAY NOT BE 100 % CORRECT
                                           HALL_010,  //  A/B   U/V    1        -- CHANGED TO FRED EADY / MCHIP VERSION
                                           HALL_011,  //  A/C   U/W    2         which uses the BLY171S
                                           HALL_001,  //  B/C   V/W    3
                                           HALL_101,  //  B/A   V/U    4
                                           HALL_100,  //  C/A   W/U    5
                                           HALL_110,  //  C/B   W/V    6

//  Microchip is:  A/B  A/C  B/C  B/A  C/A  C/B
//                 010  011  001  101  100  110

//                                         HS_V | LS_W,   // Hall position 001
//                                         HS_U | LS_V,   // Hall position 010
//                                         HS_U | LS_W,   // Hall position 011
//                                         HS_W | LS_U,   // Hall position 100
//                                         HS_V | LS_U,   // Hall position 101
//                                         HS_W | LS_V,   // Hall position 110
                                           0x00      };

    // THIS IS REDUNDANT IF WE STEP THRU CW table above BACKWEARDS !!!
    //  _or_  must step thru this table sequentially 1-6 as well.  ONE or the OTHER
            // COUNTER-CLOCKWISE (CW)  Direction = 0x01
unsigned char  Hall_CCW_DIR_sequence[] = { 0x00,
                                           HALL_110,  //  C/B   W/V    1
                                           HALL_100,  //  C/A   W/U    2
                                           HALL_101,  //  B/A   V/U    3
                                           HALL_001,  //  B/C   V/W    4
                                           HALL_011,  //  A/C   U/W    5
                                           HALL_010,  //  A/B   U/V    6

//                                         HS_W | LS_V,   // Hall position 001
//                                         HS_V | LS_U,   // Hall position 010
//                                         HS_W | LS_U,   // Hall position 011
//                                         HS_U | LS_W,   // Hall position 100
//                                         HS_U | LS_V,   // Hall position 101
//                                         HS_V | LS_W,   // Hall position 110
                                           0x00      };
          //--------------------------------------------------------------------
          //         HALL  PWM  Mapping  Table
          //
          // Shows what PWMs are on for each Hall Sector
          //--------------------------------------------------------------------
unsigned char  Hall_Map_Table[]  = {       0x00,      //             Trap Step
                                           HALL_001,  //  B/C   V/W    3
                                           HALL_010,  //  A/B   U/V    1        -- CHANGED TO FRED EADY / MCHIP VERSION
                                           HALL_011,  //  A/C   U/W    2         which uses the BLY171S
                                           HALL_100,  //  C/A   W/U    5
                                           HALL_101,  //  B/A   V/U    4
                                           HALL_110,  //  C/B   W/V    6
                                           0x00     };

          //-------------------------------------------------------
          //  Startup Trapezoidal Table - CW  (clockwise spin)
          //-------------------------------------------------------
          //  contains CW step value, duration, duty/amplititude

          //-------------------------------------------------------
          //  Startup Trapezoidal Table - CCW  (counter-clockwise)
          //-------------------------------------------------------


//******************************************************************************
//  motor_check_is_started
//
//      Checks if the motor has been successfully started
//******************************************************************************
int  motor_check_is_started (uint8_t motor_id)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    if (mtr_blk->mtr_state == MOTOR_STATE_RUN)
       return (1);      // motor is started
    return (0);         // motr start not complete

// ??? !!! in FUTURE: return Negative number if start failed ??? !!!

}


#if defined(MOTOR_IS_BLDC)

//******************************************************************************
//  motor_block_commutate
//
//               Commutate the motor using the block oriented trapezoidal
//               "6 step" commutation table method.
//
//               This technique is used for standard Hall-based "sensored"
//               applications, as well as used by the motor startup logic
//               when performing "sensorless" BEMF spin-up.
//
//               Upper layer caller selects which step number to perform,
//               based upon direction (CW vs CCW) and either the Hall sequence
//               number (sensored) or the next step in the startup sequence
//               (ensorless).
//
//               Sie-step has 6 valid sequences, and two should not occurs (0,7)
//
//  CRITICAL:  Only one High-Side coil switches on at a time (+ Low side to gnd)
//
//  Note: to make logic work across low-end families that cannot sync PWMs
//        different Timer modules (MSP432, MSP430_F5529, ...) we set the LOW
//        side of the Phase to full ON/HIGH, instead of PWMing i9t, which some
//        some implementations.
//******************************************************************************
int  motor_block_commutate (uint8_t motor_id,  int hall_sector_num,
                            int pwm_duty_count)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

    switch (hall_sector_num)                    //                ABC    UVW
     {
       case 1:                                  // Hall 001       B/C    V/W
              PWM_BL_LOW;                       // Ensure unused
              PWM_CH_LOW;                       //   switches are off
              PWM_AH_LOW;                       // Shut off Phase A
              PWM_AL_LOW;                       //   to float it
              PWM_BH_PWM (pwm_duty_count);      // B-Hi is high-side PWM
              PWM_CL_HIGH;                      // C-Lo shunts thru to ground
              break;

       case 2:                                  // Hall 010       A/B    U/V
              PWM_AL_LOW;                       // Ensure unused
              PWM_BH_LOW;                       //   switches are off
              PWM_CH_LOW;                       // Shut off Phase C
              PWM_CL_LOW;                       //   to float it
              PWM_AH_PWM (pwm_duty_count);      // A-Hi is high-side PWM
              PWM_BL_HIGH;                      // B-Lo shunts thru to ground
              break;

       case 3:                                  // Hall 011       A/C    U/W
              PWM_AL_LOW;                       // Ensure unused
              PWM_CH_LOW;                       //   switches are off
              PWM_BH_LOW;                       // Shut off Phase B
              PWM_BL_LOW;                       //   to float it
              PWM_AH_PWM (pwm_duty_count);      // A-Hi is high-side PWM
              PWM_CL_HIGH;                      // C-Lo shunts thru to ground
              break;

       case 4:                                  // Hall 100       C/A    W/U
              PWM_CL_LOW;                       // Ensure unused
              PWM_AH_LOW;                       //   switches are off
              PWM_BH_LOW;                       // Shut off Phase B
              PWM_BL_LOW;                       //   to float it
              PWM_CH_PWM (pwm_duty_count);      // C-Hi is high-side PWM
              PWM_AL_HIGH;                      // A-Lo shunts thru to ground
              break;

       case 5:                                  // Hall 101       B/A    V/U
              PWM_BL_LOW;                       // Ensure unused
              PWM_AH_LOW;                       //   switches are off
              PWM_CH_LOW;                       // Shut off Phase C
              PWM_CL_LOW;                       //   to float it
              PWM_BH_PWM (pwm_duty_count);      // B-Hi is high-side PWM
              PWM_AL_HIGH;                      // A-Lo shunts thru to ground
              break;

       case 6:                                  // Hall 110       C/B    W/V
              PWM_CL_LOW;                       // Ensure unused
              PWM_BH_LOW;                       //   switches are off
              PWM_AH_LOW;                       // Shut off Phase A
              PWM_AL_LOW;                       //   to float it
              PWM_CH_PWM (pwm_duty_count);      // C-Hi is high-side PWM
              PWM_BL_HIGH;                      // B-Lo shunts thru to ground
              break;

       default:
              return (INVALID_STEP_VALUE);      // invalid sequence step (0, 7)
     }

#if DELETE
// Caller is responsible for updating Trap Index
        //----------------------------------------------------------------------
        // Step to next step in Trapezoidal table, which depends upon whther CW or CCW.
        // This maps to the associated Hall Sector number (PWM pattern) to use.
        //----------------------------------------------------------------------
    mtr_blk->mtr_Trapezoidal_Prev = mtr_blk->mtr_Trapezoidal_Index;  // Debug
    if (mtr_blk->mtr_direction == DIRECTION_CW)
       {       // CW - Clockwise. Go forwards thru Trapezoidal commutation table
         step_num++;
         if (step_num > 6)
            step_num = 1;           // wrap back to beginng of commutation table
         mtr_blk->mtr_Trapezoidal_Index = step_num;
         mtr_blk->mtr_hall_sector = Hall_CW_DIR_sequence [step_num];
       }
      else
       {       // CCW - Counter-clockwise. Go backwards thru Trapezoidal table
         step_num--;
         if (step_num < 1)
            step_num = 6;           // wrap back to end of commutation table
         mtr_blk->mtr_Trapezoidal_Index = step_num;
         mtr_blk->mtr_hall_sector = Hall_CW_DIR_sequence [step_num];
       }
#endif

    return (0);                     // denote worked OK
}
#endif                              // defined(MOTOR_IS_BLDC)


//******************************************************************************
//  MotorLib_Startup_Bemf_Sensorless
//
//          Startup the BLDC motor for BEMF - this does initial spins of motor
//          until it has reached point of adequate BEMF feedback.
//
// The following steps need to be performed
//   - setup a startup ramp
//   - set starting value for commutate time and duty, as well as initial CW step, and startup
//   - at each commutate time expiration
//       - get save commute time period for speed calc
//       - step to next ommutate time, duty, and next CW step
//       - check BEMF value, if it has reached ZC point
//   - At end of commutate time, verify BEMF is within range
//     and switchover to Run state, using standard CW table
//******************************************************************************

int  MotorLib_Startup_Bemf_Sensorless (uint8_t motor_id)
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

       // following steps need to be performed
       //   - setup a startup ramp
       //   - set starting value for commutate time and duty, as well as initial CW step, and startup
       //   - at each commutate time expiration
       //       - get save commute time period for speed calc
       //       - step to next ommutate time, duty, and next CW step
       //       - check BEMF value, if it has reached ZC point
       //   - At end of commutate time, verify BEMF is within range
       //     and switchover to Run state, using standard CW table


//             HACK     HACK     HACK     FIX     FIX     FIX
// TBD
//     - issue blind commutation start if BEMF
//     - TEMP HACK - turn on commutation timer and drive motor w/o HALL sensors

        //----------------------------------------------------------------------
        // setup starting sector number for BEMF "blind_startup" initialization
        //----------------------------------------------------------------------
    if (mtr_blk->mtr_direction == DIRECTION_CW)
       mtr_blk->mtr_hall_sector = Hall_CW_DIR_sequence[1]; // set ramp startup begin sector
       else  mtr_blk->mtr_hall_sector = Hall_CW_DIR_sequence[1];   // --- NOTE --- goes backward thru CW table !

    MotorLib_Run (motor_id, mtr_blk->mtr_direction);       // TEMP HACK

// ??? would RPM be better ??? more granularity, else need frational RPM for 1 RPM/sec ???
    MotorLib_Set_Speed_Rpm (motor_id, 1);    // TEMP HACK  Force initial commutation

//             HACK     HACK     HACK     FIX     FIX     FIX

       //---------------------------------------
       // Perform  Motor  RAMP_UP
       //---------------------------------------
//   ramp_complete_callback_flag = 0;                 // clear RAMP complete flag

        // ramp up Motor 1 from 10% to 90% in 3 seconds. Update PWM every 30 ms
//  motor_ramp_duty (mtr_blk, RAMP_UP, 10, 90, 3000, 30, &ramp_done_callback);
//  while (ramp_complete_callback_flag == 0)
      ;     // loop till done          ==> uses callback


    return (0);            // denote succeeded
}


//******************************************************************************
//  MotorLib_Startup_Hall_Sensored
//
//          Startup the BLDC motor using HALL sensors for rotor position sensing
//
// The following steps need to be performed
//   - TBD
//******************************************************************************

int   MotorLib_Startup_Hall_Sensored (uint8_t motor_id)
{
    int          i;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

//             HACK     HACK     HACK     FIX     FIX     FIX
// TBD
//     - issue blind commutation start if BEMF
//     - TEMP HACK - turn on commutation timer and drive motor w/o HALL sensors

       //-----------------------------------------------------------------------
       //                        READ HALL SENSORS
       // Read the Hall sensors to see which sector we are currently in.
       // Use that to set the inital starting point in the CW/CCW commute table
       //-----------------------------------------------------------------------
    mtr_blk->mtr_Hall_Sensor_Curr_Index = motor_read_hall_sensors (motor_id);
    mtr_blk->mtr_hall_sector = mtr_blk->mtr_Hall_Sensor_Curr_Index; // save for start
       // find the matching trapezoidal index number
    for (i = 1; i < 6; i++)
      { if (Hall_CW_DIR_sequence[i] == mtr_blk->mtr_hall_sector)
           mtr_blk->mtr_Trapezoidal_Index = i;
      }

    MotorLib_Run (motor_id, mtr_blk->mtr_direction);       // TEMP HACK

// ??? would RPM be better ??? more granularity, else need frational RPM for 1 RPM/sec ???
    MotorLib_Set_Speed_Rpm (motor_id, 1);    // TEMP HACK  Force initial commutation

//             HACK     HACK     HACK     FIX     FIX     FIX

    return (0);                  // denote succeeded
}


//******************************************************************************
//  motor_next_commutate
//
//               Called by Commutation Timer ISR, to perform next commutation
//               on the motor.
//******************************************************************************
int  motor_next_commutate (uint8_t motor_id)
{
    int          next_sector;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

        //-------------------------------------------------
        // get next HALL sector from Motor's commutation Table
        //-------------------------------------------------
    if (mtr_blk->mtr_direction == DIRECTION_CCW)
       next_sector = Hall_CW_DIR_sequence [mtr_blk->mtr_Trapezoidal_Index];  // -- NOTE -- Using same table, just walking walking backward - OK ? !
       else next_sector = Hall_CW_DIR_sequence [mtr_blk->mtr_Trapezoidal_Index];
    mtr_blk->mtr_hall_sector = next_sector;         // save it

        // call MCU dependent Next Commutate, passing in the next sector/step
        // number for that Motor's commutation
    motor_block_commutate (motor_id, next_sector, mtr_blk->mtr_pwm_duty_count);
//  _g_mtrdrvr->motor_do_next_commutate (mtr_blk,
//                                      next_sector);

    mtr_blk->mtr_Trapezoidal_Prev = mtr_blk->mtr_Trapezoidal_Index;  // Debug
       // setup Trapezoidal step to use on next commutate cycle. Range is 1 to 6
    if (mtr_blk->mtr_direction == DIRECTION_CW)
       {     // going CW - Clockwise, go forward thru table, so increase step #
         mtr_blk->mtr_Trapezoidal_Index++;             // step to next entry in table
         if (mtr_blk->mtr_Trapezoidal_Index > 6)
            mtr_blk->mtr_Trapezoidal_Index = 1;        // wrap back to begin of table
       }
      else
       {     // going CCW - Counter-Clockwise, go backwards, so decrease step #
         mtr_blk->mtr_Trapezoidal_Index--;             // step to next entry in table
         if (mtr_blk->mtr_Trapezoidal_Index < 1)
            mtr_blk->mtr_Trapezoidal_Index = 6;        // wrap back to begin of table
       }
    return (0);               // denote succeeded
}


//******************************************************************************
//                       COMMUTATION TIMER    ISR
//
//******************************************************************************
    int  ms_counter = 1;

void  motor_commutation_update (void)
{
    if (_g_motor_id != 0xFF)
       {
         // need to consult speed varaible to determine when next commutation time is for now.
//    - In a truly sensored world, we would use the Hall interupt to drive this.
//    - In a sensorless world, we would use the pop from the BEMF 30% timer to drive this.

             // TEMPORARY HACK - always commutae at 1 ms/10 intervals = 100 RPM
         ms_counter++;
         if (ms_counter > 10)
            {
              motor_next_commutate (_g_motor_id); // do next commutation on Motor

              ms_counter = 1;                    // reset ctr for new pass
            }
       }
}



//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//               INTERNAL  MOTOR  LIBRARY  ROUTINEs          (INTERNAL USE ONLY)
//
//******************************************************************************
//******************************************************************************
//******************************************************************************


//******************************************************************************
//                                                             INTERNAL USE ONLY
//  mtrlib_compute_rpm_speed                             motor_compute_rps_speed
//
//             Called once every 0.1 second by SysTick ISR, to update RPM speed.
//
//             May be losing .5 counts per 0.1 interval. May need to add a
//             once per second aggregator to correct the count calc at
//             end of each 1 secondf interval.
//
//             The Hall encoders provides 625 pulses per revolution.
//             They convert the angular displacement signals into a pulse count.
//******************************************************************************

void  mtrlib_compute_rpm_speed (void)
{
   float    flt_enc_count;

             // compute RPM for left wheel
   flt_enc_count = encoder_Ph_C_LEFT_rupts;
   encoder_LEFT_speed_rps = flt_enc_count / encoder_pulses_per_revolution;

             // compute RPM for right wheel
   flt_enc_count = encoder_Ph_B_RIGHT_rupts;
   encoder_RIGHT_speed_rps = flt_enc_count / encoder_pulses_per_revolution;

             // update aggregates used for 1 second RPM calcs
   encoder_LEFT_1_sec_agg  += encoder_Ph_C_LEFT_rupts;
   encoder_RIGHT_1_sec_agg += encoder_Ph_B_RIGHT_rupts;
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
   encoder_Ph_C_LEFT_rupts  = 0;
   encoder_Ph_B_RIGHT_rupts = 0;
}


//******************************************************************************
//                                                             INTERNAL USE ONLY
//  mtrlib_calculate_speed_profile
//
//        Calculates the speed profile according to the number of steps to move.
//        It performs the needed "trajectory" calculations for the motor move.
//
//        Used by BLDC motors with Encoders, and Stepper Motors
//
// This takes the configured the acceleration and deceleration values for the
// motor, and computes the number of steps for each part of the "Motion Ramp":
//    - acceleration    - the number of steps to ramp up to the target velocity,
//    - steady velocity - the number of steps at target speed/velocity, and
//    - deceleration    - number of steps for ramp down and stop at target position.
//
// If the total number of steps to perform is large enough:
//    - a _TRAPEZOIDAL_ move is executed (i.e. there is a steady run phase
//      where the motor runs at the target/maximum speed/velocity).
//                     Steady
//                   ----------
//         ramp up  /          \  ramp down
//               __/            \__
//
//    - otherwise, a _TRIANGULAR_ move is performed (no steady run phase,
//      the target/maximum speed/velocity is never quite reached).
//
//                       /\
//            ramp up   /  \   ramp down
//                   __/    \__
//******************************************************************************

int  mtrlib_calculate_speed_profile (uint8_t motor_id, uint32_t tot_num_steps)
{
    uint32_t     num_accelerate_steps; 
    uint32_t     num_decelerate_steps;
    uint32_t     ramp_steps;
    uint16_t     min_speed;
    uint32_t     acc;
    uint32_t     dec;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

       // compute the number of steps to accelerate/ramp up the targeted speed.
       // Note that we divide by 2, because each part is only doing a single
       // edge (either up or down) of the 2-way ramp.
    min_speed             = mtr_blk->mtr_min_velocity_pc;
    num_accelerate_steps  = (mtr_blk->mtr_max_velocity_pc - min_speed);
    num_accelerate_steps *= (mtr_blk->mtr_max_velocity_pc + min_speed);
    num_decelerate_steps  = num_accelerate_steps;
    num_accelerate_steps /= (uint32_t) mtr_blk->mtr_acceleration_pc;
    num_accelerate_steps /= 2;

       // compute the number of steps to decelerate/ramp down and stop
    num_decelerate_steps /= (uint32_t) mtr_blk->mtr_deceleration_pc;
    num_decelerate_steps /= 2;

       // add up the total number of the steps in the ramp up and down
    ramp_steps = num_accelerate_steps + num_decelerate_steps;

       // now set values to use for each step (accel/steady/decel),
       // based on whether the move will be Triangular or Trapezoidal
    if (ramp_steps > tot_num_steps)
       {      //----------------------------
              //      Triangular move
              //----------------------------
              // num_decelerate_steps = (Pos * Dec) / (Dec + Acc)
         acc = mtr_blk->mtr_acceleration_pc;
         dec = mtr_blk->mtr_deceleration_pc;

         num_decelerate_steps = ((uint32_t) dec * tot_num_steps) / (acc + dec);
         if (num_decelerate_steps > 1)
            {
              num_accelerate_steps = num_decelerate_steps - 1;
              if (num_accelerate_steps == 0)
                 {
                   num_accelerate_steps = 1;
                 }      
            }
           else
            {
              num_accelerate_steps = 0;
            }
         mtr_blk->mtr_accel_end_position   = num_accelerate_steps;
         mtr_blk->mtr_decel_begin_position = num_decelerate_steps;
       }
      else
       {      //----------------------------
              //       Trapezoidal move
              //----------------------------
              // accelerate/ramp up phase to mtr_accel_end_position
              // steady phase         from mtr_accel_end_position to mtr_decel_begin_position
              // decelerate/ramp down from mtr_decel_begin_position to mtr_steps_to_move
         mtr_blk->mtr_accel_end_position   = num_accelerate_steps;
         mtr_blk->mtr_decel_begin_position = tot_num_steps - num_decelerate_steps - 1;
       }

    return (0);                      // denote worked OK
}



// Is the following a generic thing, or a device by device thing ?
// Is there any generic support needed, or just ensure changed names
// in the MOTOR BLOCK all match as one ???

/*******************************************************************************
 *                                                                    CRITICAL
 *                     Step Clock Handler
 *
 *
 *                Is _only_ called by the timer ISR
 *
 *  This is invoked by the PWM channel's CCR interrupt.
 *  The CCR channel _fires an interrupt after each step_.
 *
 * Handles the device state machine at each state
 *
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 ******************************************************************************/
extern int   isrFlag;          // WVD TEMP HACK for in-board L6474_StepClockHandler
       long  inactive_state_rupt = 0;


#if defined(MOTOR_DRIVER_L6474)

void  L6474_StepClockHandler (uint8_t motor_id)
{
    uint32_t     relativePos;
    uint32_t     acc;
    uint32_t     deceleration;
    uint16_t     speed;
    uint16_t     maxSpeed;
    bool         speedUpdated;
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk[motor_id];

      /* Set isr flag */
  isrFlag = TRUE;

      // Increment the relative position that we have moved thus far
  mtr_blk->mtr_new_relative_position++;
  relativePos  = mtr_blk->mtr_new_relative_position;

  switch (mtr_blk->mtr_motion_state) 
  {
    case MOTION_ACCELERATING: 
        speed = mtr_blk->mtr_current_velocity_pc;
        acc   = ((uint32_t) mtr_blk->mtr_acceleration_pc << 16);

		   //--------------------------------------------------------
		   // For GOTO and MOVE commands, we change states when the
		   // relative position (relativePos) matches either the
		   // mtr_accel_end_position or mtr_decel_begin_position.
		   //--------------------------------------------------------
        if ((mtr_blk->mtr_operation_performed == SOFT_STOP_CMD)
          || ((mtr_blk->mtr_operation_performed != OP_RUN) 
          &&  (relativePos == mtr_blk->mtr_decel_begin_position)))
           {
             mtr_blk->mtr_motion_state = MOTION_DECELERATING;
             mtr_blk->accu = 0;
           }
          else if ((speed >= mtr_blk->mtr_max_velocity_pc)
                 || ((mtr_blk->mtr_operation_performed != OP_RUN)
                 && (relativePos == mtr_blk->mtr_accel_end_position)))
                 {
                     //---------------------------------------------------------
                     // Reached desired velocity - so now Cruise at steady state
                     //---------------------------------------------------------
                   mtr_blk->mtr_motion_state = MOTION_STEADY;
                 }
          else
           {
             speedUpdated = FALSE;
                /* keep on accelerating */
             if (speed == 0)
                speed = 1;
             mtr_blk->accu += acc / speed;
             while (mtr_blk->accu >= (0X10000L))
               {
                  mtr_blk->accu -= (0X10000L);
                  speed += 1;
                  speedUpdated = TRUE;
               }
          
             if (speedUpdated)
               {
                 if (speed > mtr_blk->mtr_max_velocity_pc)
                    speed = mtr_blk->mtr_max_velocity_pc;

                 mtr_blk->mtr_current_velocity_pc = speed;

                 L6474_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_current_velocity_pc);
               }
          }
        break;

    case MOTION_STEADY: 
      maxSpeed = mtr_blk->mtr_max_velocity_pc;

		   //--------------------------------------------------------
		   // For GOTO and MOVE commands, we change states when the
		   // relative position (relativePos) matches mtr_decel_begin_position.
		   // For RUN commands, we change states when speed > maxSpeed.
		   //--------------------------------------------------------
      if ((mtr_blk->mtr_operation_performed == SOFT_STOP_CMD)
         || ((mtr_blk->mtr_operation_performed != OP_RUN)
         &&  (relativePos >= (mtr_blk->mtr_decel_begin_position)))
         || ((mtr_blk->mtr_operation_performed == OP_RUN)
         &&  (mtr_blk->mtr_current_velocity_pc > maxSpeed)))
       {
         mtr_blk->mtr_motion_state = MOTION_DECELERATING;
         mtr_blk->accu = 0;
       }
      else if ((mtr_blk->mtr_operation_performed == OP_RUN)
             && (mtr_blk->mtr_current_velocity_pc < maxSpeed))
               {
                 mtr_blk->mtr_motion_state = MOTION_ACCELERATING;
                 mtr_blk->accu = 0;
               }
      break;

    case MOTION_DECELERATING:
      speed        = mtr_blk->mtr_current_velocity_pc;
      deceleration = ((uint32_t) mtr_blk->mtr_deceleration_pc << 16);

		   //--------------------------------------------------------
		   // For GOTO and MOVE commands, we change states to STOP
		   // when relative position (relativePos) matches mtr_steps_to_move.
		   //--------------------------------------------------------
      if (((mtr_blk->mtr_operation_performed == SOFT_STOP_CMD) 
        && (speed <=  mtr_blk->mtr_min_velocity_pc))
        || ((mtr_blk->mtr_operation_performed != OP_RUN)
        &&  (relativePos >= mtr_blk->mtr_steps_to_move)))
         {     //------------------------------------------------
               // Motion process is complete.  Shut off the PWM
               //------------------------------------------------
           _g_mtrdrvr->motor_stop_handler (mtr_blk, STOP_HARD_BRAKE, 0);
         }
       else if ((mtr_blk->mtr_operation_performed == OP_RUN)
              && (speed <= mtr_blk->mtr_max_velocity_pc))
         {
          mtr_blk->mtr_motion_state = MOTION_STEADY;
         }
       else
         {
              /* Keep on decelerating */
          if (speed > mtr_blk->mtr_min_velocity_pc)
             {
               speedUpdated = FALSE;
               if (speed == 0)
                  speed = 1;
               mtr_blk->accu += deceleration / speed;

               while (mtr_blk->accu >= (0X10000L))
                 {
                   mtr_blk->accu -= (0X10000L);
                   if (speed > 1)
                      speed -= 1;
                   speedUpdated = TRUE;
                 }
        
               if (speedUpdated)
                  {
                    if (speed < mtr_blk->mtr_min_velocity_pc)
                       speed = mtr_blk->mtr_min_velocity_pc;
                    mtr_blk->mtr_current_velocity_pc = speed;

                    L6474_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_current_velocity_pc);
                  }
             }
         }
      break;

    case MOTION_NOT_ACTIVE: // if one last interrupt is left in pipe, this can happen
      inactive_state_rupt++;
    default:
      break;
   }                               //  end  switch

        /* Clear isr flag */
  isrFlag = FALSE;
}
#endif                             // #if defined(MOTOR_DRIVER_L6474)




//******************************************************************************
//  motor_adc_process_speed_control
//
//           Check if ADC value has changed. If so, ratio it 0:16384 = 0:100 %
//           and then change PWM duty values as required.
//           This uses integers ("cave man calculus"/cmc) to compute the values.
//******************************************************************************
    int       speed_duty_value = 0;                // speed duty value in ticks

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

// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
// ??? !!! MAJOR ISSUE - is extern to main()'s  motor_1 / motor_2

         MotorLib_Set_Speed (0, duty_per_cent_value, SPEED_PERCENT_DUTY); // Motor 1
         MotorLib_Set_Speed (0, duty_per_cent_value, SPEED_PERCENT_DUTY); // Motor 2

            // and save it as new speed value
         speed_duty_value = duty_per_cent_value;
       }
   //  WORKs  04/02/16

}



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
    if ( ! _g_motor_init_complete)
       return;                              // bail if motor init not called yet

    adc_timer++;
    if (adc_timer >= 50)                    // call once every 50 ms
       {        // start a new ADC conversion every 50 ms.
                // call processor's ADC converion rtn
         if (_g_mtrdrvr->motor_adc_speed_ctl_next_conversion != 0L)
            _g_mtrdrvr->motor_adc_speed_ctl_next_conversion();
         adc_timer = 0;                      // reset counter for next pass
       }

    encoder_timer++;
    if (encoder_timer >= 100)               // call once every 0.1 seconds
       {        // update our RPM speed calculation
         mtrlib_compute_rpm_speed();
         encoder_timer = 0;                 // reset for next pass
       }

    if (_g_motor_ramp_active_flag)
       MotorLib_Speed_Ramp_Update();        // increment any ramp as needed
}


//******************************************************************************
//  motor_ctl_adc_init                                          INTERNAL ROUTINE
//
//         Configure control inputs for motor ADCs:  Current sense, etc, ...
//
//         ADC GPIO port/pin related stuff is in motor_config.h file
//******************************************************************************
int  motor_ctl_adc_init (void)
{
          // call MCU dependent ADC speed ctl init handler
    if (_g_mtrdrvr->motor_init_adc_ctl_inputs != 0L)
       _g_mtrdrvr->motor_init_adc_ctl_inputs();

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
    _g_mtrdrvr->motor_init_gpio_ctl_inputs();  // call MCU dependent GPIO init inputs handler
    _g_mtrdrvr->motor_init_gpio_ctl_outputs(); // call MCU dependent GPIO init outputs handler

    return (0);            // denote succeeded
}



//******************************************************************************
