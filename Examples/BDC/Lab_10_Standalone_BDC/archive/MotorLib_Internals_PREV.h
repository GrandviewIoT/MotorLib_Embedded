
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                            MotorLib_Internals.h
//
// Internals structs and definitions used for BLDC motor control library.
//
// Used to controls BLDC motors with several different BLDC Gate Driver ICs
// and integrated motor controllers.
//
// History:
//   05/22/16 - Split off as separate module. Duquaine
//******************************************************************************

#ifndef _BLDC_MOTOR_CTL_INTERNALS_H_
#define   _BLDC_MOTOR_CTL_INTERNALS_H_

#ifndef CALLBACK_RTN
typedef  void         (*CALLBACK_RTN)(int ivar); // pointer to callback function
#endif

                      // valid cmd values for perform_motor_move() / mtr_move_type
#define  MOVE_TO_POSITION   1   // move to a absolute Stepper position, or BLDC encoder position
#define  MOVE_N_STEPS       2   // move N number of relative steps from current position
#define  MOVE_N_ROTATIONS   3   // spin N number of rotations
#define  MOVE_DISTANCE      4   // move a specific linear distance
#define  MOVE_TO_HOME       5   // move motor to Home position
#define  MOVE_TO_MARK       6   // move motor to last "Mark" position
#define  MOVE_FREE_RUN      7   // continuously spin motor at given speed till stop issued

                      // overall type of motor operation performed: MOVE, RUN, CTL
#define  OP_MOVE            1   // MOVE_TO_POSITION, MOVE_N_STEPS ... MOVE_TO_MARK
#define  OP_RUN             2   // MOVE_FREE_RUN
#define  OP_CTL             3

                      // valid values for mtr_motion_state
#define  MOTION_NOT_ACTIVE   1  // no motion in progress.
#define  MOTION_ACCELERATING 2  // we are accelerating up to desired target speed
#define  MOTION_STEADY       3  // we are running steady at target speed
#define  MOTION_DECELERATING 4  // we are decelerating down to a lower speed or stop

                             //-------------------------------------------------
typedef  struct  motorcb {   //      Structure used for Motor Control Info 
                             //-------------------------------------------------
    uint16_t      mtr_cb_sig;           // Mtr Clk Blk signature
    struct motorcb *mtr_cb_qptr;        // pointer when queued
    CALLBACK_RTN  mtr_callback;
        uint8_t   mtr_profile;          // MOTOR_DRV8305, MOTOR_L64&6, ...
        uint8_t   mtr_id;               // 0, 1, 2, 3,... based on above profile

                                        // Motor settings and state info
        uint8_t   mtr_state;            // RUN / STOPPED / FAILED
        uint8_t   mtr_direction;        // CW/CCW current direction
        uint8_t   mtr_motion_state;     // INACTIVE / ACCEL / STEADY / DECEL
        uint8_t   mtr_decay_mode;       // SLOW / FAST
        uint8_t   mtr_speed_units;      // PERCENT_DUTYCYCLE / RPM / PPS
        uint16_t  mtr_pole_pairs;       // from motor_config_BLDC.h
        uint16_t  mtr_pwm_period_ticks; // PWM period ticks count
        uint16_t  mtr_pwm_duty_count;   // current PWM duty count/ticks

                                   // Controller chip command information
        uint8_t   mtr_move_type;           // MOVE / RUN
        uint16_t  mtr_operation_performed; // logical operation performed  commandExecuted
        uint16_t  mtr_commandExecuted;     // physical command issued (DEBUG)

                                   // Position Control   Stepper step ~ to BLDC encoder phase count
        int32_t   mtr_reported_position;   // Position in steps, reported by motor ctlr (ABS_POS)
        int32_t   mtr_current_position;    // Position in steps, where we think the motor is
        int32_t   mtr_new_relative_position;// new position, based on ISR feedback from
                                            //  step indexer (Stepper) or QEI (BLDC encoder)
        int32_t   mtr_move_begin_position; // Starting position for the latest motor MOVE
        int32_t   mtr_move_end_position;   // Ending   position for the latest motor MOVE
        int32_t   mtr_steps_to_move;       // total number of steps to make in latest MOVE
        int32_t   mtr_accel_end_position;  // Step position that marks end of acceleration
        int32_t   mtr_decel_begin_position;// Step position that marks begin of deceleration
        float     mtr_distance_per_step_mm;// how much distance in mm each step produces
        float     mtr_distance_per_rotation_mm; // how much distance in mm each shaft rotation produces
        int32_t   mtr_move_begin_rotation; // Starting position on a move rotation (BDC/Wheel)
        int32_t   mtr_move_end_rotation;   // Ending   position on a move rotation (BDC/Wheel)
        uint32_t  accu;  // accumulator used to store speed increases smaller than 1 pps
        uint8_t   mtr_homed_flag;          // 1 = motor was physically or logically homed.

                                   // Trajectory Control  Stepper pulse step is ~ to BLDC phase count
        uint16_t  mtr_acceleration_pc; // acceleration in pulses per sec squared
        uint16_t  mtr_deceleration_pc; // deceleration in pulses per sec squared
        uint16_t  mtr_jerk_pc;         // jerk in pulses per sec squared  (FUTURE S-Curve)
        uint16_t  mtr_max_velocity_pc; // max speed in pulses per sec - HOW MAP TO BDC ?
        uint16_t  mtr_min_velocity_pc; // min speed in pulses per sec
        uint16_t  mtr_current_velocity_pc;// current speed in pulses per sec

                                   // Speed Control
        uint16_t  mtr_rpm_setpt;                // desired speed in RPS revolutions per second -- PROBEBLY SHOULD GO TO RPM FOR ALL
        uint16_t  mtr_angular_velocity_setpt;   // "          "     AVS  angular velocity per second -- NEED TO RESOLVE
        uint16_t  mtr_rpm_actual;               // measured speed in RPS
        uint16_t  mtr_angular_velocity_actual;  // "           "     AVS
        uint32_t  mtr_commute_time_ticks;       // duty cycle for commutation
        uint32_t  mtr_commute_timer_period_ticks;  // period value of 100 usec commutation timer
        uint16_t  mtr_Speed_Control_Sense;      // raw ADC value
        uint16_t  mtr_Speed_Control_Increments; // conversion to XXX

                                   // Rotor Position Info
        uint16_t  mtr_hall_sector;           // which 60 degree Hall sector it is in (1-6)
        uint16_t  mtr_Hall_Sensor_Curr_Index;// latest Hall sensor values
        uint16_t  mtr_Hall_Sensor_Prev_Index;// previous Hall sensor values
        uint8_t   mtr_BEMF_Floating_Phase;   // Current "Floating" Phase A, B, or C being monitored
        uint16_t  mtr_BEMF_ZC_Threshhold;    // Reference Level for BEMF Zero Cross

                                   // Ramp information
        int16_t   mtr_ramp_pwm_increment;
        uint16_t  mtr_ramp_pwm_end;
        uint16_t  mtr_systick_increment;
        uint16_t  mtr_systick_count;

                                   // Encoder information
        uint8_t   encoder_profile;     // RIGHT_WHEEL_HALL, ...
        float     encoder_pulses_per_revolution;
        float     encoder_speed_rps0;  // speed in revs / second

                                   // SPI latest SPI I/O to/from DRV8305. All
                                       // I/O is 2 byte, with 5 H.O. bits = reg
        uint16_t  mtr_spi_cmd;
        uint16_t  mtr_spi_cmd_request;
        uint16_t  mtr_spi_reply;

                      //---------------------------------------
                      //           BLDC only fields
                      //---------------------------------------
                                         // Run Commutation - Trapezoidal
        uint16_t  mtr_Trapezoidal_Index; // current position in Trapezoidal table (1-6)
        uint16_t  mtr_Trapezoidal_Prev;  // prior position in Trapezoidal table

                                         // Run Mode Control Variables
// or use just a single state based on Trapezoid step, and derive which is floating ?
        uint8_t   mtr_Phase_A_state;         // High PWM, Low PWM, Floating
        uint8_t   mtr_Phase_B_state;         // High PWM, Low PWM, Floating
        uint8_t   mtr_Phase_C_state;         // High PWM, Low PWM, Floating
// will these ever be different from each other, or are they generic (apply to all)
        uint8_t   mtr_Phase_A_duty_cycle_pc; // Phase A duty % 0-100
        uint8_t   mtr_Phase_B_duty_cycle_pc;
        uint8_t   mtr_Phase_C_duty_cycle_pc;
        uint16_t  mtr_Phase_A_duty_count;    // Phase A actual duty PWM count
        uint16_t  mtr_Phase_B_duty_count;
        uint16_t  mtr_Phase_C_duty_count;

                           // Run Commutation - Sinusoidal
        uint16_t  mtr_Sine_Step_A_Index;    // current position in Sine table - Phase A
        uint16_t  mtr_Sine_Step_Increment;  // next step size to use

                            // Commutation Startup (Blind trapezoidal start)
        uint32_t  mtr_Startup_Commute_Begin;// in 1 ms ticks
        uint32_t  mtr_Startup_Commute_End;  // in 1 ms ticks
        uint16_t  mtr_Commute_Step_Total;   // total # steps in startup
        uint16_t  mtr_Commute_Step_Index;   // which step in startup sequence
        uint32_t  mtr_Startup_Step_Delay;   // Commutate timing delay for this step
        uint32_t  mtr_Startup_Step_DutyVal; // Duty Cycle (Amplitude) for this step

                                         // Voltage / Current Sensors
        uint16_t  mtr_Bus_Voltage_Sense;
        uint16_t  mtr_Phase_A_Voltage_Sense; // raw ADC value
        uint16_t  mtr_Phase_B_Voltage_Sense; // raw ADC value
        uint16_t  mtr_Phase_C_Voltage_Sense; // raw ADC value
        uint16_t  mtr_Phase_A_Current_Sense; // raw ADC value
        uint16_t  mtr_Phase_B_Current_Sense; // raw ADC value
        uint16_t  mtr_Phase_C_Current_Sense; // raw ADC value

      }   MOTOR_BLOCK;


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

typedef  int  (*FUNCTION_CALL_VD)(void);      // pointer to low level handler function
typedef  int  (*FUNCTION_CALL_IN)(int ivar);
typedef  int  (*FUNCTION_CALL_I_CB)(int ivar, CALLBACK_RTN cb_ptr);
typedef  int  (*FUNCTION_CALL_I_I_C_C)(int ivar1, int ivar2, char *buf12, char *buf2);
typedef  int  (*FUNCTION_CALL_MB)(MOTOR_BLOCK *mtr_blk);
typedef  int  (*FUNCTION_CALL_MB_I)(MOTOR_BLOCK *mtr_blk, int ivar);
typedef  int  (*FUNCTION_CALL_MB_L)(MOTOR_BLOCK *mtr_blk, long lnvar);
typedef  int  (*FUNCTION_CALL_MB_I_I)(MOTOR_BLOCK *mtr_blk, int ivar1, int ivar2);
typedef  long (*FUNCTION_CALL_L_MB)(MOTOR_BLOCK *mtr_blk);
          
                      // valid values for motor_type
#define  TYPE_MOTOR_BDC     1   // is a BDC motor attached to a BDC motor controller
#define  TYPE_MOTOR_STEPPER 2   // is a Stepper motor attached to a Stepper controller
#define  TYPE_MOTOR_BLDC    3   // is a BLDC motor attached to a BLDC motor controller



typedef  struct  motor_func_cb {   /* Structure used for Device specific Motor Control Handlers */
                     short                 motor_type;     // BDC, Stepper, BLDC
                     short                 motor_chip;     // chip number (L6474 = 6474, ...)
                                              // standard header routines
                     FUNCTION_CALL_VD      motor_spi_init;             // SPI init
                     FUNCTION_CALL_I_I_C_C motor_spi_io;               // SPI I/O
                     FUNCTION_CALL_I_CB    motor_adc_speed_ctl_init;   // init routine for speed ctl ADC
                     FUNCTION_CALL_VD      motor_adc_speed_ctl_next_conversion;// ADC conversion start rtn for speed
                     FUNCTION_CALL_MB      motor_read_hall_sensors_handler;    // motor_read_hall_sensors_handler
                     FUNCTION_CALL_VD      motor_fault_shutdown;       // motor_fault_shutdown
                     FUNCTION_CALL_VD      motor_init_adc_ctl_inputs;  // control ADCs (Current sense)
                     FUNCTION_CALL_VD      motor_init_controller_chip; // initialize controller chip
                     FUNCTION_CALL_MB_L    motor_init_commute_timer;   // motor_init_commute_time
                     FUNCTION_CALL_MB_I    motor_init_encoder_gpio;    // init routine for encoder GPIOs
                     FUNCTION_CALL_VD      motor_init_gpio_ctl_inputs; // control input  GPIOs (FAULT)
                     FUNCTION_CALL_VD      motor_init_gpio_ctl_outputs;// control output GPIOs (ENABLE)
                     FUNCTION_CALL_MB_L    motor_init_pwms_comm;       // motor PWM init

                                              // detailed chip dependent handlers
                     int         (*motor_check_status)(MOTOR_BLOCK *mtr_blk, int type_status, int flags);
                     int         (*motor_do_action_handler)(MOTOR_BLOCK *mtr_blk, int cmd, int flags);
              FUNCTION_CALL_MB_I  motor_do_next_commutate;
                     int32_t     (*motor_get_actual_position)(MOTOR_BLOCK *mtr_blk);              
                     int32_t     (*motor_get_mark_pos)(MOTOR_BLOCK *mtr_blk);
                     int32_t     (*motor_get_motor_status)(MOTOR_BLOCK *mtr_blk, int type_status);
              FUNCTION_CALL_L_MB  motor_get_period_ticks;
                     int         (*motor_perform_move)(MOTOR_BLOCK *mtr_blk, int cmd, int direction, int32_t count);
                     int         (*motor_run_handler)(MOTOR_BLOCK *mtr_blk, int direction);
              FUNCTION_CALL_MB_I  motor_set_direction_handler;
              FUNCTION_CALL_MB_I  motor_set_pwm_duty_count_handler;
              FUNCTION_CALL_MB_L  motor_set_speed_rpm_handler;
                     int         (*motor_set_step_mode)(MOTOR_BLOCK *mtr_blk, int mode, int flags);
                     int         (*motor_set_torque) (MOTOR_BLOCK *mtr_blk, int torque_mode, int16_t torque_value);
                     int         (*motor_stop_handler)(MOTOR_BLOCK *mtr_blk, int stop_type, int flags);
                     int         (*motor_wait_complete)(MOTOR_BLOCK *mtr_blk, int type_wait, int flags);

                     FUNCTION_CALL_MB_I    motor_HW_check;
                   }    MOTOR_HANDLERS;


//---------------------------------------------------------------------------
// INTERNAL #defs                        move to separate incloudable file
//---------------------------------------------------------------------------
#define  PHASE_A      1       // Phase A related defs: current, voltage, duty, ...
#define  PHASE_B      2
#define  PHASE_C      3

             // 6 step pattern ids for Trapezoidal and Forced BEMF commutation
#define  HALL_001     1
#define  HALL_010     2
#define  HALL_011     3
#define  HALL_100     4
#define  HALL_101     5
#define  HALL_110     6

             // 6 step pattern for CW (clockwise) rotation
#define  CW_STEP_1    0
#define  CW_STEP_2    1        // PLACE-HOLDERS - need to be revised !!!
#define  CW_STEP_3    2
#define  CW_STEP_4    3
#define  CW_STEP_5    4
#define  CW_STEP_6    5

             // 6 step pattern for CW (clockwise) rotation
#define  CCW_STEP_1   5
#define  CCW_STEP_2   4
#define  CCW_STEP_3   3
#define  CCW_STEP_4   2
#define  CCW_STEP_5   1
#define  CCW_STEP_6   0


typedef  struct  six_step_prms {   /* Structure used for Six Step Parameters */
                                  uint16_t     motor_stop_handler;
                               }    SIX_STEP_PARMS;

typedef  struct  step_tab  {
                                  uint16_t    commute_step[6];   // 6 step
                           }  BLDC_STEP_TABLE;


             //--------------------------------------------------------
             // Convert motor_config_BLDC.h parms into generic labels
             // used internally by the code
             //--------------------------------------------------------
#if defined(MOTOR_DRIVER_DRV8848)
#define MOTOR_DRIVER        MOTOR_DRIVER_DRV8848  // BDC TI DRV8848 Boosterpack
#elif defined(MOTOR_DRIVER_SN754410)
#define MOTOR_DRIVER        MOTOR_DRIVER_SN754410 // BDC TI SN754410 Dual Bridge IC
#elif defined(MOTOR_DRIVER_L6293)
#define MOTOR_DRIVER        MOTOR_DRIVER_L6293    // BDC L6293 Dual Bridge IC 
#elif defined(MOTOR_DRIVER_L6206)
#define MOTOR_DRIVER        MOTOR_DRIVER_L6206    // BDC ST L6206 Dual Brush DC Xnucleo
#elif defined(MOTOR_DRIVER_L6474)
#define MOTOR_DRIVER        MOTOR_DRIVER_L6474    // STEPPER ST L6474 Xnucleo
#elif defined(MOTOR_DRIVER_DRV8305)
#define MOTOR_DRIVER        MOTOR_DRIVER_DRV8305  // BLDC TI DRV8305 Boosterpack
#elif defined(MOTOR_DRIVER_DRV8301)
#define MOTOR_DRIVER         MOTOR_DRIVER_DRV8301 // BLDC TI DRV8301 Boosterpack
#elif defined(MOTOR_DRIVER_L6230)
#define MOTOR_DRIVER         MOTOR_DRIVER_L6230   // BLDC ST L6230 Xnucleo
#elif defined(MOTOR_DRIVER_L6398)
#define MOTOR_DRIVER         MOTOR_DRIVER_L6398   // BLDC ST L6398 Xnucleo
#else
#error "You must choose a valid MOTOR_DRIVER"
#endif


#if defined(MCU_SPEED_16MHZ)
#define  MCU_SPEED              16000000
#elif defined(MCU_SPEED_24MHZ)
#define  MCU_SPEED              24000000
#elif defined(MCU_SPEED_48MHZ)
#define  MCU_SPEED              48000000
#elif defined(MCU_SPEED_80MHZ)
#define  MCU_SPEED              80000000
#elif defined(MCU_SPEED_84MHZ)
#define  MCU_SPEED              84000000
#elif defined(MCU_SPEED_120MHZ)
#define  MCU_SPEED             120000000
#elif defined(MCU_SPEED_180MHZ)
#define  MCU_SPEED             180000000
#else
#error "You must choose a valid MCU Speed"
#endif


#if defined(PWM_SPEED_10KHZ)
#define  PWM_SPEED              10000
#elif defined(PWM_SPEED_20KHZ)
#define  PWM_SPEED              20000
#elif defined(PWM_SPEED_30KHZ)
#define  PWM_SPEED              30000
#elif defined(PWM_SPEED_40KHZ)
#define  PWM_SPEED              40000
#else
#error "You must choose a valid PWM Speed"
#endif


#if defined(ADC_RESOLUTION_10_BIT)
#define ADC_SCALING_VALUE            1023.0
#elif defined(ADC_RESOLUTION_12_BIT)
#define ADC_SCALING_VALUE            4095.0
#elif defined(ADC_RESOLUTION_14_BIT)
#define ADC_SCALING_VALUE           16383.0
#else
#error "You must choose a valid ADC Resolution"
#endif

                //---------------------------
                //   Internal SPI I/O codes
                //---------------------------
#define  SPI_READ         1
#define  SPI_WRITE        2
#define  SPI_WRITE_READ   3

#define  SPI_IO_RESET     0             // No I/O operation in progress
#define  SPI_IO_BUSY      1             // I/O operation has been started
#define  SPI_IO_COMPLETE  2             // I/O operation has been completed
#define  SPI_IO_ERROR     3             // I/O operation ended with an error

                //--------------------
                //   Common  Externs
                //--------------------
extern const long  long_100;

extern  MOTOR_HANDLERS  *_g_mtrdrvr;    // located in BLDC_motor_ctl_lib.c
extern  MOTOR_BLOCK     *_g_mtr_blk;    // located in BLDC_motor_ctl_DRV8305.c

extern  int        motor_ramp_flag;     // used by SYSTICK ISR for RAMPs
extern  int        encoder_timer;       // Used by Systick handler
extern  uint32_t   encoder_Ph_B_RIGHT_rupts; // count of interrupts this interval
extern  uint32_t   encoder_Ph_C_LEFT_rupts;  // counts are reset at each motor_compute_rps_speed()
                                             // call, which occurs every 50 ms

                //----------------------------------
                //       internal API calls
                //----------------------------------
int  motor_block_commutate (uint8_t motor_id,  int step_num,
                            int pwm_duty_count);
int   motor_read_hall_sensors (uint8_t motor_id);
void  mtrlib_compute_rpm_speed (void);

                //----------------------------------
                //       board_support calls
                //----------------------------------
int   board_clock_init (long  mcu_speed,  int  use_crystal);
void  board_delay_ms (long wait_time);
void  board_enable_global_interrupts (void);
void  board_gpio_init (void);
void  board_systick_init (void);
long  board_system_clock_get_frequency (void);

#endif                          // _BLDC_MOTOR_CTL_INTERNALS_H_

//*****************************************************************************
