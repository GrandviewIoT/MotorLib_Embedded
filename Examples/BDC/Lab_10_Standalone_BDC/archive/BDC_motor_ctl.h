
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                              BDC_motor_ctl.h
//
// Common API call definitions used for BDC motor control library.
//
// Used to controls 2 Brushed DC motors with several different H-Bridge ICs
// and integrated motor controllers.
//
// History:
//   04/08/16 - Split off as separate module, to allow project resuse. Duquaine
//******************************************************************************

#ifndef _BDC_MOTOR_CTL_H_
#define   _BDC_MOTOR_CTL_H_

#include <stdint.h>                    // pull in standard int16_t, etc defs
#include <stdbool.h>
#include "motor_BDC_config.h"          // pull in processor specific config info


                // valid values for motor_set_direction() call
#define  DIRECTION_FORWARD   0                     // default
#define  DIRECTION_REVERSE   1

                // valid values for motor_set_decay() call
#define  DECAY_FAST          0         // brake the motor to a stop   (default)
#define  DECAY_SLOW          1         // coast the motor to a stop

                // valid values for motor_ramp_duty() call
#define  RAMP_UP             0         // ramp up the motor's duty cycle (speed)
#define  RAMP_DOWN           1         // ramp down the motor's duty cycle (speed)

                // valid values for mtr_state
#define  STATE_STOPPED       0         // motor is stopped
#define  STATE_RUN           1         // motor is in running mode

                // Encoder Configuration
#define  ENCODER_HALL_RIGHT_WHEEL  1     // Hall sensor mounted on wheel
#define  ENCODER_HALL_LEFT_WHEEL   2     // Hall sensor mounted on wheel
#define  ENCODER_HALL_ON_MOTOR     3     // hall sensors mounted on motor itself
#define  ENCODER_QEI_ON_MOTOR      4     // QEI encoder mounted on motor

#define  NULL_MPTR       0L
typedef  void            (*CALLBACK_RTN)(int motor_num); // pointer to callback function



//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

typedef  struct  motorcb {   /* Structure used for Motor Control Info */
                           uint16_t   mtr_cb_sig;         // Mtr Clk Blk signature
                       struct motorcb *mtr_cb_qptr;       // pointer when queued
                       CALLBACK_RTN   mtr_callback;
                           uint8_t    mtr_profile;        // MOTOR_1_DRV8848, MOTOR_1_SN754410, ...
                           uint8_t    mtr_num;            // 1, 2, 3, 4, ... based on above profile
                                         // Motor settings and state info
                           uint8_t    mtr_state;          // RUN / STOPPED
                           uint8_t    mtr_direction;      // current direction
                           uint8_t    mtr_decay_mode;     //
                           uint16_t   mtr_pwm_period_ticks; // PWM period ticks count
                           uint16_t   mtr_pwm_duty_count; // current PWM duty count/ticks
                                         // Ramp information
                           int16_t    mtr_ramp_pwm_increment;
                           uint16_t   mtr_ramp_pwm_end;
                           uint16_t   mtr_systick_increment;
                           uint16_t   mtr_systick_count;
                                         // Encoder information
                           uint8_t    encoder_profile;     // RIGHT_WHEEL_HALL, ...
                           float      encoder_pulses_per_revolution;
                           float      encoder_speed_rps0;  // speed in revs / second
                         }   MOTOR_BLOCK;


//-----------------------------------------------------------------------------
//             Motor API calls
//-----------------------------------------------------------------------------
int   motor_lib_init (long  mcu_speed);
int   motor_init (MOTOR_BLOCK *mtr_blk, int motor_num, int pwm_speed);
int   motor_adc_init_speed_control (int ADC_Channel);
int   motor_encoder_init (MOTOR_BLOCK *mtr_blk, int Encoder_Type);  // Encoder_Type = WHEEL_ENCODER_RIGHT/LEFT, HALL_SENSOR, QEI
                                                                    // port/pin related stuff is in motor_config.h file
int   motor_brake (MOTOR_BLOCK *mtr_blk);
void  motor_compute_rps_speed (void);
int   motor_go_left (MOTOR_BLOCK *mtr_blk, int re_start);
int   motor_go_right (MOTOR_BLOCK *mtr_blk, int re_start);
int   motor_ramp_check_complete (MOTOR_BLOCK *mtr_blk);
int   motor_ramp_duty (MOTOR_BLOCK *mtr_blk, uint16_t ramp_up_down,
                     uint16_t begin_duty_cycle_pc, uint16_t end_duty_cycle_pc,
                     uint16_t ramp_duration_ms, uint16_t increment_frequency_ms,
                     CALLBACK_RTN  cb_ptr);
int   motor_set_decay (MOTOR_BLOCK *mtr_blk, int decay_type);
int   motor_set_direction (MOTOR_BLOCK *mtr_blk, int mtr_direction);
int   motor_set_duty_cycle (MOTOR_BLOCK *mtr_blk, uint16_t duty_cycle);
int   motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int16_t pwm_duty_count);  // ??? need as user API call ???
int   motor_start (MOTOR_BLOCK *mtr_blk);
int   motor_stop (MOTOR_BLOCK *mtr_blk);
void  motor_systick_1ms_update (void);
void  motor_adc_init (void);
void  motor_gpio_init (void);


           //--------------------
           //   Common  Externs
           //--------------------
extern const long    long_100;

extern  int       motor_ramp_flag;          // used by SYSTICK ISR for RAMPs
extern  int       mtr_1_direction;          // current direction
extern  int       mtr_1_duty_cycle;         // current dudty cycle motor 1
extern  int       mtr_2_direction;
extern  int       mtr_2_duty_cycle;         // current dudty cycle motor 2
extern  uint32_t  encoder_LEFT_rupts;       // count of interrupts this interval
extern  uint32_t  encoder_RIGHT_rupts;
extern  int       encoder_timer;            // Used by Systick handler
extern  uint32_t  hall_port_rupt_flags;     // DEBUG - Hall GPIO Interrupt ISR

extern  int   period_ticks;                 // # SMCLK ticks in period


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

typedef  int  (*FUNCTION_CALL_VD)(void);      // pointer to low level handler function
typedef  int  (*FUNCTION_CALL_IN)(int ivar);
typedef  int  (*FUNCTION_CALL_MB)(MOTOR_BLOCK *mtr_blk);
typedef  int  (*FUNCTION_CALL_MB_I)(MOTOR_BLOCK *mtr_blk, int ivar);
typedef  int  (*FUNCTION_CALL_MB_L)(MOTOR_BLOCK *mtr_blk, long lnvar);


typedef  struct  motor_func_cb {   /* Structure used for Device specific Motor Control Handlers */
                       FUNCTION_CALL_MB    motor_brake_handler;
                       FUNCTION_CALL_MB_I  motor_init_encoder_gpio;
                       FUNCTION_CALL_IN    motor_init_speed_adc;
                       FUNCTION_CALL_VD    motor_init_adc_ctl_inputs;
                       FUNCTION_CALL_VD    motor_init_gpio_ctl_inputs;
                       FUNCTION_CALL_VD    motor_init_gpio_ctl_outputs;
                       FUNCTION_CALL_MB_L  motor_init_pwms;
                       FUNCTION_CALL_MB_I  motor_set_direction_handler;
                       FUNCTION_CALL_MB_I  motor_set_pwm_duty_count_handler;
                       FUNCTION_CALL_MB    motor_start_handler;
                       FUNCTION_CALL_MB    motor_stop_handler;
                       FUNCTION_CALL_VD    motor_do_next_adc_conversion; // call ADC conversion rtn for speed
                     }    MOTOR_HANDLERS;


//-------------------------------------------
// board_support calls
//-------------------------------------------
int   board_clock_init (long  mcu_speed,  int  use_crystal);
void  board_delay (long wait_time);
void  board_enable_global_interrupts (void);
void  board_gpio_init (void);
void  board_systick_init (void);

#endif                          // _BDC_MOTOR_CTL_H_

//*****************************************************************************
