
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                              MotorLib_Api.h
//
// Common API call definitions used for BLDC motor control library.
//
// Used to controls BLDC motors with several different BLDC Gate Driver ICs
// and integrated motor controllers.
//
// History:
//   05/16/16 - Created. Duquaine
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

#ifndef _MOTORLIB_API_H_
#define   _MOTORLIB_API_H_

#include <stdint.h>               // pull in standard int16_t, etc defs
#include <stdbool.h>
#include "MotorLib_Config.h"      // pull in USER SUPPLIED project and processor
                                  // specific config info. Should reside in
                                  // User's project src or include directory.
                                  // User must add include path to it in their
                                  // Compiler's "include" directories config.
#include "MotorLib_Internals.h"   // pull in rest of BLDC internals/structs


                // valid values for motor_set_direction() call
#define  DIRECTION_FORWARD        1
#define  DIRECTION_BACKWARD       2
//#define  DIRECTION_REVERSE      2

#define  DIRECTION_CW     DIRECTION_FORWARD   // CW = Clockwise - default - BLDC
#define  DIRECTION_CCW    DIRECTION_BACKWARD  // CCW = Counter-Clockwise

                // valid values for Motor Mode
#define  HALL_SENSORED            1
#define  ENCODER_SENSORED         2
#define  BEMF_SENSORLESS          3


                // valid values for mtr_state
#define  MOTOR_STATE_STOPPED      0    // motor is stopped
#define  MOTOR_STATE_STARTING     1    // motor is spinning up
#define  MOTOR_STATE_RUN          2    // motor is in running mode
#define  MOTOR_STATE_FAILED       3    // motor is failed. needs re-start/recovery


                // valid values for motor_set_decay() call
#define  DECAY_FAST               0    // brake the motor to a stop   (default)
#define  DECAY_SLOW               1    // coast the motor to a stop

                // valid values for SPEED
#define  SPEED_PERCENT_DUTY       0    // speed in percent of duty cycle 0.0-100.0 (BDC motors)
#define  SPEED_RPM                1    // speed in revolutions per minute
#define  SPEED_PPS                2    // pulses per second (e.g. steps or encoder QEI pulses)

                // valid values for motor_speed_ramp() call
#define  RAMP_UP                  0    // ramp up the motor's duty cycle (speed)
#define  RAMP_DOWN                1    // ramp down the motor's duty cycle (speed)

                // Encoder Configuration
#define  ENCODER_HALL_RIGHT_WHEEL 1   // Hall sensor mounted on wheel
#define  ENCODER_HALL_LEFT_WHEEL  2   // Hall sensor mounted on wheel
#define  ENCODER_HALL_ON_MOTOR    3   // hall sensors mounted on motor itself
#define  ENCODER_QEI_ON_MOTOR     4   // QEI encoder mounted on motor


                // valid values for UNITS
typedef enum {
                  UNITS_MM        = 1,
                  UNITS_CM        = 2,
                  UNITS_INCH      = 3,
                  UNITS_ROTATIONS = 4
             } UNITS_t;

                      // valid action_type values for motor_control_motor_action()
#define  CMD_SET_HOME         1
#define  CMD_SET_MARK         2
#define  CMD_ENGAGE_MOTOR     3
#define  CMD_DISENGAGE_MOTOR  4

                      // valid stop_type values for motor_stop()
#define  STOP_HARD_BRAKE      1      // stop fast
#define  STOP_SOFT_BRAKE      2      // coast to stop
#define  STOP_HIZ             3      // stop, then disable motor bridge

                    // valid values for STEPPER MODE      (Stepper Motors)
#define  STEP_MODE_FULL       1
#define  STEP_MODE_HALF       2
#define  STEP_MODE_1_4        3
#define  STEP_MODE_1_8        4
#define  STEP_MODE_1_16       5
#define  STEP_MODE_1_32       6
#define  STEP_MODE_1_64       7
#define  STEP_MODE_1_128      8

#define  NULL_MPTR            0L
typedef  void            (*CALLBACK_RTN)(int ivar); // pointer to callback function

//-----------------------------------------------------------------------------
//             Motor API calls
//-----------------------------------------------------------------------------
int      motor_lib_init (long  mcu_speed);
int      MotorLib_Init_Motor (uint8_t motor_id, int pwm_speed, int pole_pairs);

int      MotorLib_Check_Is_Started (uint8_t motor_id);
int      MotorLib_Check_Status (uint8_t motor_id, int type_status, int flags);
int      MotorLib_Motor_Engage (uint8_t motor_id, int on_off, int flags);
uint16_t MotorLib_Get_Acceleration (uint8_t motor_id);
uint16_t  MotorLib_Get_Current_Speed (uint8_t motor_id, int speed_type);
uint16_t MotorLib_Get_Deceleration (uint8_t motor_id);
float    MotorLib_Get_Distance_Moved (uint8_t motor_id, UNITS_t units);
int32_t  MotorLib_Get_Mark (uint8_t motor_id);
uint16_t MotorLib_Get_Max_Speed (uint8_t motor_id);
uint16_t MotorLib_Get_Min_Speed (uint8_t motor_id);
int32_t  MotorLib_Get_Motor_Status (uint8_t motor_id, int type_status);
long     MotorLib_Get_Period_Ticks (uint8_t motor_id);
int32_t  MotorLib_Get_Position (uint8_t motor_id);
uint16_t MotorLib_Get_Speed_Reference_Value (uint8_t motor_id);
void     MotorLib_Get_Speed_Angular_Velocity (uint8_t motor_id,
                                        uint16_t *target_velovity_per_sec,
                                        uint16_t *actual_velovity_per_sec);
void     MotorLib_Get_Speed_Rpm (uint8_t motor_id, uint16_t *target_rpm_speed,
                                     uint16_t *actual_rpm_speed);
uint16_t MotorLib_Get_Vbus_Voltage (uint8_t motor_id);
int      MotorLib_Goto_Home (uint8_t motor_id);
int      MotorLib_Goto_Mark (uint8_t motor_id);
void     MotorLib_Goto_Position (uint8_t motor_id, int32_t targetPosition);
int      MotorLib_Move_Distance (uint8_t motor_id,  int direction,
                                     float distance,   UNITS_t units);
void     MotorLib_Move_Steps (uint8_t motor_id, int direction,  uint32_t stepCount);
void     MotorLib_Run (uint8_t motor_id, int direction);
bool     MotorLib_Set_Acceleration (uint8_t motor_id, uint16_t newAcc);
bool     MotorLib_Set_Deceleration (uint8_t motor_id, uint16_t newDec);
int      MotorLib_Set_Decay (uint8_t motor_id,  int decay_type);
int      MotorLib_Set_Direction (uint8_t motor_id,  int mtr_direction);
int      MotorLib_Set_Distance_Per_Step (uint8_t motor_id,  int direction,
                                             float distance_per_step,  UNITS_t units);
int      MotorLib_Set_Distance_Per_Rotation (uint8_t motor_id,
                                     float distance_per_rotation, UNITS_t units);
void     MotorLib_Set_Home (uint8_t motor_id);
void     MotorLib_Set_Mark (uint8_t motor_id);
int      MotorLib_Set_Maximum_Speed (uint8_t motor_id, uint16_t new_maximum_speed);
int      MotorLib_Set_Minimum_Speed (uint8_t motor_id, uint16_t new_minimum_speed);
int      MotorLib_Set_Pwm_Duty_Count (uint8_t motor_id, int16_t actual_pwm_duty_count);
int      MotorLib_Set_Speed (uint8_t motor_id,  uint16_t speed_value, int speed_units);
#define  MotorLib_Set_Speed_Duty_Cycle(motor_id,speed_value)  MotorLib_Set_Speed(motor_id,speed_value,SPEED_PERCENT_DUTY)
int      MotorLib_Set_Speed_Rpm (uint8_t motor_id, uint16_t rpM_speed);
int      MotorLib_Set_Speed_Angular_Velocity (uint8_t motor_id, uint16_t velovity_per_sec);
void     MotorLib_Set_Step_Mode (uint8_t motor_id, int stepper_mode);
int      MotorLib_Set_Torgue (uint8_t motor_id, int torque_mode, int16_t torque_value);
int      MotorLib_Speed_Ramp_Check_Complete (uint8_t motor_id);
int      MotorLib_Speed_Ramp (uint8_t motor_id, uint16_t ramp_up_down,
                        uint16_t begin_duty_cycle_pc, uint16_t end_duty_cycle_pc,
                        uint16_t ramp_duration_ms, uint16_t increment_frequency_ms,
                        CALLBACK_RTN  cb_ptr);
void     MotorLib_Speed_Ramp_Update (void);
//int      MotorLib_Start (uint8_t motor_id);
int      MotorLib_Stop (uint8_t motor_id, int stop_type);
int      MotorLib_Wait_Complete (uint8_t motor_id, int type_wait, int flags);


int      MotorLib_Fault_Shutdown (void);

int      MotorLib_Adc_Init_Speed_Control (int ADC_Channel, CALLBACK_RTN cb_ptr);
int      MotorLib_Encoder_Init (uint8_t motor_id, int Encoder_Type);  // Encoder_Type = WHEEL_ENCODER_RIGHT/LEFT, HALL_SENSOR, QEI
                                                                 // port/pin related stuff is in motor_config.h file

int      MotorLib_Startup_Bemf_Sensorless (uint8_t motor_id);
int      MotorLib_Startup_Hall_Sensored (uint8_t motor_id);

           //-----------------------------
           //     Debug  Routines
           //-----------------------------
void    MotorLib_Start_DAC (void);
void    MotorLib_Stop_DAC (void);
void    MotorLib_Set_DAC_Value (uint16_t dac_value);

           //-----------------------------
           //     INTERNAL  APIs   ???
           //-----------------------------
void     motor_adc_init (void);    // Internal routines ???
void     motor_gpio_init (void);

void     motor_commutation_update (void);
int      motor_next_commutate (uint8_t motor_id);

int      mtrlib_calculate_speed_profile (uint8_t motor_id, uint32_t tot_num_steps);







           //-----------------------------
           //     OLD APIs
           //-----------------------------
int   motor_check_is_started (uint8_t motor_id);
void  motor_compute_rps_speed (void);

void  motor_systick_1ms_update (void);
int   motor_HW_test (uint8_t motor_id, int duty_percent);

//long  drv8305_motor_get_period_ticks (uint8_t motor_id);







    //----------------------------------------------------------
    //                      ERROR   CODES
    //----------------------------------------------------------
#define  MOTOR_NOT_SUPPORTED         -10  // that motor is not in our commutation tables
#define  MOTOR_MAX_RPM_EXCEEDED      -20  // exceeds the max Revs/minute specified for the motor
#define  INVALID_STEP_VALUE          -30
#define  ERROR_UNSUPPORTED_FUNCTION  -40  // trying to use a function not supported for your motor tyep
                                          // e.g. issuing a step_set_config() for a BDC motor
#define  ERROR_INVALID_UNITS         -41
#define  ERROR_INVALID_SPEED_TYPE    -42  // invalid type specified in MotorLib_Get_Current_Speed() call
#define  ERROR_MAX_SPEED_TOO_HIGH    -42  // Max speed value is > maximum configured in motor_xx.h file
#define  ERROR_MIN_SPEED_TOO_LOW     -43  // Minspeed value is < minimum configured in motor_xx.h file
#define  ERROR_INVALID_CONTROL_ACTION -44 // action type in motor_control_motor_action() is invalid#define  MOTOR_MAX_RPM_EXCEEDED     -45   // value on set_speed, examined the maximum configured for

#endif                          // _MOTORLIB_API_H_

//*****************************************************************************
