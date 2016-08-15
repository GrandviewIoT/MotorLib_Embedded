
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                          MotorLib_LL_L6474_STM32.c
//
// Motor Control Library:  Low Level Device support for L6474, L6470, and
//                         STSPIN220 series of intelligent Stepper Controllers. 
//
// The L6470 and STSPIN220 uses a command set very similar to the L6474, so we
// just use #ifdefs where there are variances.
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
//   08/12/16 - Merge in changes to bring common logic between
//              STEPPER and BLDC back into sync.                  Duquaine
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

#include "MotorLib_Api.h"             // pull in common definitions
#include "Stepper\STM32\l6474.h"      // L6474 command codes, register defs, etc

    int   _g_adc_enabled = 0;
    int   isrFlag        = 0;

    SPI_HandleTypeDef       _g_hspi1_handle;    // SPI 1 support

    TIM_HandleTypeDef       hTimPwm1;           // Handle for Motor 1's PWM

#define  TIMER_PRESCALER    1024

           //-------------------------------------------------------------------
           // This table should all be populated by #define entries from MotorLib_motor_config.h
           //-------------------------------------------------------------------
L6474_Init_t  _g_L6474_Init_Params =
{
    160,                       // Acceleration rate in step/s2. Range: (0..+inf)
    160,                       // Deceleration rate in step/s2. Range: (0..+inf)
    1600,                      // Maximum speed in step/sec. Range: (30..10000].
    800,                       // Minimum speed in step/sec. Range: [30..10000).
    250,                       // Torque regulation current in mA. (TVAL reg)
                               //                      Range: 31.25mA to 4000mA.
    750,                       // Overcurrent threshold (OCD_TH register).
                               //                      Range: 375mA to 6000mA.
    L6474_CONFIG_OC_SD_ENABLE,       // Overcurrent shutwdown (OC_SD field of CONFIG register).
    L6474_CONFIG_EN_TQREG_TVAL_USED, // Torque regulation method (EN_TQREG field of CONFIG register).
    L6474_STEP_SEL_1_16,             // Step selection (STEP_SEL field of STEP_MODE register).
    L6474_SYNC_SEL_1_2,              // Sync selection (SYNC_SEL field of STEP_MODE register).
    L6474_FAST_STEP_12us,            // Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
    L6474_TOFF_FAST_8us,             // Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
    3,                               // Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
    21,                              // Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
    L6474_CONFIG_TOFF_044us,         // Target Swicthing Period (field TOFF of CONFIG register).
    L6474_CONFIG_SR_320V_us,         // Slew rate (POW_SR field of CONFIG register).
    L6474_CONFIG_INT_16MHZ,          // Clock setting (OSC_CLK_SEL field of CONFIG register).
    (L6474_ALARM_EN_OVERCURRENT      |
     L6474_ALARM_EN_THERMAL_SHUTDOWN |
     L6474_ALARM_EN_THERMAL_WARNING  |
     L6474_ALARM_EN_UNDERVOLTAGE     |
     L6474_ALARM_EN_SW_TURN_ON       |
     L6474_ALARM_EN_WRONG_NPERF_CMD)    // Alarm (ALARM_EN register).
};


int  L6474_GetDeviceState (MOTOR_BLOCK *mtr_blk);
int32_t l6474_get_motor_actual_position (MOTOR_BLOCK *mtr_blk);
int     l6474_motor_do_ctl_action (MOTOR_BLOCK *mtr_blk, int cmd, int flags);
int     l6474_perform_motor_move (MOTOR_BLOCK *mtr_blk, int cmd, int direction, int32_t count);
int     l6474_motor_set_step_mode (MOTOR_BLOCK *mtr_blk, int step_mode, int flags);
int     l6474_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags);
void    l6474_StartMovement (MOTOR_BLOCK *mtr_blk, int move_cmd);
int32_t l6474_get_mark (MOTOR_BLOCK *mtr_blk);
int32_t l6474_CmdGetStatus (MOTOR_BLOCK *mtr_blk, int status_type);
int     l6474_WaitWhileActive (MOTOR_BLOCK *mtr_blk, int wait_type, int flags);
int     l6474_motor_init_controller_chip (void);

int     l6474_motor_run (MOTOR_BLOCK *mtr_blk, int direction);
int     l6474_motor_set_direction (MOTOR_BLOCK *mtr_blk, int mtr_direction);
int     l6474_motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count);
void    L6474_StepClockHandler (uint8_t motor_id);


#include "l6474.h"       // TEMP HACK
#define L6474_MAX_PWM_FREQ   (10000)     // Maximum frequency of the PWMs in Hz
#define L6474_MIN_PWM_FREQ   (2)         // Minimum frequency of the PWMs in Hz

                         //    =====   TEMP   HACK    =======
void    L6474_ApplySpeed(MOTOR_BLOCK *mtr_blk, uint8_t pwmId, uint16_t newSpeed);
void    L6474_ComputeSpeedProfile(MOTOR_BLOCK *mtr_blk, uint32_t nbSteps);
int32_t L6474_ConvertPosition(uint32_t abs_position_reg); 
uint8_t L6474_Convert_Ocd_Thresh_to_RegValue(float Tval);
float   L6474_Convert_Ocd_RegValue_to_Threshold(uint8_t Par);
uint8_t L6474_Convert_Tval_Current_to_RegValue(float Tval);
uint8_t L6474_Convert_Tmin_Time_to_RegValue(float Tmin);
float   L6474_Convert_RegValue_to_Tmin_Time(uint8_t Par);
float   L6474_Convert_Tval_RegValue_to_Current(uint8_t Par);
void    L6474_ErrorHandler(uint16_t error);
void    L6474_FlagInterruptHandler(void);                      
void    L6474_SendCommand(MOTOR_BLOCK *mtr_blk, uint8_t param);
void    L6474_SetRegisterToGivenValues(MOTOR_BLOCK *mtr_blk, L6474_Init_t *pInitPrm);
void    L6474_SetRegisterToPredefinedValues(MOTOR_BLOCK *mtr_blk);
void    L6474_Write_SPI_Bytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte);    
void    L6474_SetDeviceParamsToPredefinedValues(MOTOR_BLOCK *mtr_blk);
void    L6474_SetDeviceParamsToGivenValues(MOTOR_BLOCK *mtr_blk, L6474_Init_t *pInitPrm);
void    l6474_start_motor_move(MOTOR_BLOCK *mtr_blk, int move_cmd);
void    L6474_StepClockHandler(uint8_t motor_id);  
                         //    =====   TEMP   HACK    =======

           //-----------------------------------------------------------
           //-----------------------------------------------------------
           //             Motor Contoller Chip Function Table
           //
           //  Defines low-level command handlers that are invoked by
           //  BLDC_motor_ctl_lib.c to perform device specific functions
           //-----------------------------------------------------------
           //-----------------------------------------------------------
int  l6474_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr);
int  l6474_motor_adc_next_conversion (void);
int  l6474_motor_adc_ctl_init (void);
int  l6474_motor_encoder_gpio_init(MOTOR_BLOCK *mtr_blk, int encoder_type);
int  l6474_motor_init_ctl_gpio_outputs (void);
int  l6474_motor_init_ctl_gpio_inputs (void);
int  l6474_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long pwm_ticks);

  MOTOR_HANDLERS * L6474_GetMotorHandle(void);   // return addr of drv6474_table

  MOTOR_HANDLERS  drv6474_table = {
                     TYPE_MOTOR_STEPPER,           // Motor Type
                     MOTOR_DRIVER,                 // Chip number / id
                     0L,                           // SPI init
                     0L,                           // SPI I/O
                     l6474_motor_speed_adc_init,   // init routine for speed ctl ADC
                     l6474_motor_adc_next_conversion, // ADC conversion start rtn for speed
                     0L,                           // motor_read_hall_sensors_handler
                     0L,                           // motor_fault_shutdown
                     l6474_motor_adc_ctl_init,     // control ADCs (Current sense)
                     l6474_motor_init_controller_chip, // motor_init_controller_chip
                     0L,                               // motor_init_commute_time
                     l6474_motor_encoder_gpio_init,    // init routine for encoder GPIOs
                     l6474_motor_init_ctl_gpio_inputs, // control input  GPIOs (FAULT)
                     l6474_motor_init_ctl_gpio_outputs,// control output GPIOs (ENABLE)
                     l6474_motor_pwm_init,             // motor PWMs

                     0L,                           // motor_check_status
                     l6474_motor_do_ctl_action,    // motor_do_action_handler
                     0L,                           // motor_do_next_commutate
                     l6474_get_motor_actual_position,// motor_get_actual_position
                     l6474_get_mark,               // motor_get_mark_pos
                     l6474_CmdGetStatus,           // motor_get_motor_status
                     0L,                           // motor_get_period_ticks
                     l6474_perform_motor_move,     // motor_perform_move
                     l6474_motor_run,              // motor_run_handler
                     l6474_motor_set_direction,    // motor_set_direction_handler
                     l6474_motor_set_pwm_duty_count,// motor_set_pwm_duty_count_handler
                     0L,                           // motor_set_speed_rpm_handler
                     l6474_motor_set_step_mode,    // motor_set_step_mode
                     0L,                           // motor_set_torque
                     0L,                           // motor_start_handler (BLDC)
                     l6474_motor_stop,             // motor_stop_handler
                     l6474_WaitWhileActive,        // motor_wait_complete
                     0L,                           // motor_current_reference_start_stop
                     0L,                           // motor_current_reference_set_value
                     0L,                           // motor_start_pwm_generation (BLDC)
                     0L,                           // motor_stop_pwm_generation  (BLDC)
                     0L,                           // motor_set_bldc_phase_pwm_duty_cycles (BLDC)
                     0L,                           // motor_set_bldc_pwm_enables (BLDC)
                  };


/*******************************************************************************
  * @file    l6474.c
  * @author  IPC Rennes
  * @version V1.6.0
  * @date    February 03, 2016
  * @brief   L6474 driver (fully integrated microstepping motor driver)
  * @note    (C) COPYRIGHT 2014 STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
*  History:
*     06/17/16 - Fix situation where L6474 may report a bogus ABS_POS value
*                immeidately after reset. Always force a logical home.
*******************************************************************************/

#include "l6474.h"

 
/* Private constants ---------------------------------------------------------*/    

                         /// Error while initialising the SPI
#define L6474_ERROR_0   (0x8000)   
                         /// Error: Bad SPI transaction
#define L6474_ERROR_1   (0x8001)
    
                         /// Maximum number of steps
#define MAX_STEPS       (0x7FFFFFFF)
    
    
/* Private variables ---------------------------------------------------------*/

/** @defgroup L6474_Private_Variables L6474 Private Variables
  * @{
  */       

    uint32_t   g_CmdGetStatus   = 0;
    uint32_t   g_CmdGetStatus_2 = 0;
//  uint32_t   inactive_state_rupt = 0;

               /// Function pointer to flag interrupt call back
void (*flagInterruptCallback)(void);
               /// Function pointer to error handler call back
void (*errorHandlerCallback)(uint16_t);

static volatile uint8_t  numberOfDevices = 1;   // default is 1 motor

static uint8_t  spiTxBursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
static uint8_t  spiRxBursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];

static volatile bool    spiPreemtionByIsr   = FALSE;
//static volatile bool    isrFlag             = FALSE;
static uint16_t         l6474DriverInstance = 0;

                      //------------------------------------
                      // L6474 Motor Blocks - 1 per motor
                      //------------------------------------
extern  MOTOR_BLOCK  motor_blk [];           // is defined in MotorLib_Basic.c


/* Private function prototypes -----------------------------------------------*/

/** @defgroup L6474_Private_functions L6474 Private functions
  * @{
  */ 
#if defined(REENABLE_LATER)
void L6474_ApplySpeed(MOTOR_BLOCK *mtr_blk, uint8_t pwmId, uint16_t newSpeed);
void L6474_ComputeSpeedProfile(MOTOR_BLOCK *mtr_blk, uint32_t nbSteps);
int32_t L6474_ConvertPosition(uint32_t abs_position_reg); 
uint8_t L6474_Convert_Ocd_Thresh_to_RegValue(float Tval);
float   L6474_Convert_Ocd_RegValue_to_Threshold(uint8_t Par);
uint8_t L6474_Convert_Tval_Current_to_RegValue(float Tval);
uint8_t L6474_Convert_Tmin_Time_to_RegValue(float Tmin);
float   L6474_Convert_RegValue_to_Tmin_Time(uint8_t Par);
float   L6474_Convert_Tval_RegValue_to_Current(uint8_t Par);
void L6474_ErrorHandler(uint16_t error);
void L6474_FlagInterruptHandler(void);                      
void L6474_SendCommand(MOTOR_BLOCK *mtr_blk, uint8_t param);
void L6474_SetRegisterToGivenValues(MOTOR_BLOCK *mtr_blk, L6474_Init_t *pInitPrm);
void L6474_SetRegisterToPredefinedValues(MOTOR_BLOCK *mtr_blk);
void L6474_Write_SPI_Bytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte);    
void L6474_SetDeviceParamsToPredefinedValues(MOTOR_BLOCK *mtr_blk);
void L6474_SetDeviceParamsToGivenValues(MOTOR_BLOCK *mtr_blk, L6474_Init_t *pInitPrm);
//void L6474_StartMovement(MOTOR_BLOCK *mtr_blk);
void L6474_StepClockHandler(uint8_t motor_id);  
#endif                         // REENABLE_LATER

/** @defgroup L6474_Exported_Variables L6474 Exported Variables
  * @{
  */

#if defined(MUST_BE_REPLACED_BY_MINE_CAUSE_MANY_ROUTINES_NOW_GONE)
                         //--------------------------------------------------
                         //  L6474 motor driver functions pointer structure 
                         //--------------------------------------------------
motorDrv_t   drv6474_table_ORIG =
{
  l6474_get_motor_actual_position,             // motor_get_actual_position
  l6474_motor_engage,                   // motor_motor_engage_handler
  l6474_perform_motor_move,             // motor_perform_move
  l6474_motor_set_step_mode,                 // motor_set_mode
  l6474_motor_stop,                     // motor_stop
  l6474_get_mark,                       // motor_get_mark_pos

  L6474_Init,                  //void (*Init)(void*);
  L6474_ReadId,                //uint16_t (*ReadID)(void);
  L6474_AttachErrorHandler,    //void (*AttachErrorHandler)(void (*callback)(uint16_t));
  L6474_AttachFlagInterrupt,   //void (*AttachFlagInterrupt)(void (*callback)(void));
  0,                           //void (*AttachBusyInterrupt)(void (*callback)(void));
  L6474_FlagInterruptHandler,  //void (*FlagInterruptHandler)(void);
  L6474_GetAcceleration,       //uint16_t (*GetAcceleration)(uint8_t);
  L6474_GetCurrentSpeed,       //uint16_t (*GetCurrentSpeed)(uint8_t);
  L6474_GetDeceleration,       //uint16_t (*GetDeceleration)(uint8_t);
  L6474_GetDeviceState,        //motorState_t(*GetDeviceState)(uint8_t);
  L6474_GetFwVersion,          //uint8_t (*GetFwVersion)(void);
  l6474_get_mark,              //int32_t (*GetMark)(uint8_t);
  L6474_GetMaxSpeed,           //uint16_t (*GetMaxSpeed)(uint8_t);
  L6474_GetMinSpeed,           //uint16_t (*GetMinSpeed)(uint8_t);
  L6474_GetPosition,           //int32_t (*GetPosition)(uint8_t);
  0L,  // L6474_GoHome,                //void (*GoHome)(uint8_t);
  0L,  // L6474_GoMark,                //void (*GoMark)(uint8_t);
  0L,  // L6474_GoTo,                  //void (*GoTo)(uint8_t, int32_t);
  L6474_HardStop,              //void (*HardStop)(uint8_t);
  0L,    // L6474_Move,                  //void (*Move)(uint8_t, DIRECTION_t, uint32_t);
  L6474_ResetAllDevices,       //void (*ResetAllDevices)(void);
  0L,    // L6474_Run,                   //void (*Run)(uint8_t, DIRECTION_t);
  L6474_SetAcceleration,       //bool(*SetAcceleration)(uint8_t ,uint16_t);
  L6474_SetDeceleration,       //bool(*SetDeceleration)(uint8_t , uint16_t);
  0,     // L6474_SetHome,               //void (*SetHome)(uint8_t);
  0,     // L6474_SetMark,               //void (*SetMark)(uint8_t);
  L6474_SetMaxSpeed,           //bool (*SetMaxSpeed)(uint8_t, uint16_t);
  L6474_SetMinSpeed,           //bool (*SetMinSpeed)(uint8_t, uint16_t);
  L6474_SoftStop,              //bool (*SoftStop)(uint8_t);
  L6474_StepClockHandler,      //void (*StepClockHandler)(uint8_t motor_id);
  L6474_WaitWhileActive,       //void (*WaitWhileActive)(uint8_t);
  L6474_CmdDisable,            //void (*CmdDisable)(uint8_t);
  L6474_CmdEnable,             //void (*CmdEnable)(uint8_t);
  L6474_CmdGetParam,           //uint32_t (*CmdGetParam)(uint8_t, uint32_t);
  L6474_CmdGetStatus,          //uint16_t (*CmdGetStatus)(uint8_t);
  L6474_CmdNop,                //void (*CmdNop)(uint8_t);
  L6474_CmdSetParam,           //void (*CmdSetParam)(uint8_t, uint32_t, uint32_t);
  L6474_ReadStatusRegister,    //uint16_t (*ReadStatusRegister)(uint8_t);
  L6474_ReleaseReset,          //void (*ReleaseReset)(void);
  L6474_Reset,                 //void (*Reset)(void);
  L6474_SelectStepMode,        //void (*SelectStepMode)(uint8_t motor_id, STEPPER_MODE_t);
  L6474_SetDirection,          //void (*SetDirection)(uint8_t, DIRECTION_t);
  0,                           //void (*CmdGoToDir)(uint8_t, DIRECTION_t, int32_t);
  0,                           //uint8_t (*CheckBusyHw)(void);
  0,                           //uint8_t (*CheckStatusHw)(void);
  0,                           //void (*CmdGoUntil)(uint8_t, motorAction_t, DIRECTION_t, uint32_t);
  L6474_HizStop,               //void (*CmdHardHiZ)(uint8_t);
  0,                           //void (*CmdReleaseSw)(uint8_t, motorAction_t, DIRECTION_t);
  0,                           //void (*CmdResetDevice)(uint8_t);
  0,                           //void (*CmdResetPos)(uint8_t);
  0,                           //void (*CmdRun)(uint8_t, DIRECTION_t, uint32_t);
  0,                           //void (*CmdSoftHiZ)(uint8_t);
  0,                           //void (*CmdStepClock)(uint8_t, DIRECTION_t);
  0,                           //void (*FetchAndClearAllStatus)(void);
  0,                           //uint16_t (*GetFetchedStatus)(uint8_t);
  0,                           //uint8_t (*GetNbDevices)(void);
  0,                           //bool (*IsDeviceBusy)(uint8_t);
  0,                           //void (*SendQueuedCommands)(void);
  0,                           //void (*QueueCommands)(uint8_t, uint8_t, int32_t);
  0,                           //void (*WaitForAllDevicesNotBusy)(void);
  L6474_ErrorHandler,          //void (*ErrorHandler)(uint16_t);
  0,                           //void (*BusyInterruptHandler)(void);
  0,                           //void (*CmdSoftStop)(uint8_t);
  0,                           //void (*StartStepClock)(uint16_t);
  0,                           //void (*StopStepClock)(void);
  0,                           //void (*SetDualFullBridgeConfig)(uint8_t);
  0,                           //uint32_t (*GetBridgeInputPwmFreq)(uint8_t);
  0,                           //void (*SetBridgeInputPwmFreq)(uint8_t, uint32_t);
  0,                           //void (*SetStopMode)(uint8_t, motorStopMode_t);
  0,                           //motorStopMode_t (*GetStopMode)(uint8_t);
  0,                           //void (*SetDecayMode)(uint8_t, motorDecayMode_t);
  0,                           //motorDecayMode_t (*GetDecayMode)(uint8_t);
  0,                           //STEPPER_MODE_t (*GetStepMode)(uint8_t);
  0,                           //DIRECTION_t (*GetDirection)(uint8_t);
  0,                           //void (*ExitDeviceFromReset)(uint8_t);
  0,                           //void (*SetTorque)(uint8_t, motorTorqueMode_t, uint8_t);
  0,                           //uint8_t (*GetTorque)(uint8_t, motorTorqueMode_t);
  0,                           //void (*SetRefFreq)(uint8_t, uint32_t);
  0,                           //uint32_t (*GetRefFreq)(uint8_t);
  0,                           //void (*SetRefDc)(uint8_t, uint8_t);
  0,                           //uint8_t (*GetRefDc)(uint8_t);
  L6474_SetNumMotors,          //bool (*SetNbDevices)(uint8_t);
  L6474_SetAnalogValue,        //bool (*SetAnalogValue)(uint8_t, uint32_t, float);
  L6474_GetAnalogValue         //float (*GetAnalogValue )(uint8_t, uint32_t);
};
#endif


//******************************************************************************
//  l6474_start_motor_move
//
//                                                                    KEY  KEY
//                            Start Movement
//
//   Initialises the bridge parameters to start the movement
//   and enable the power bridge
//******************************************************************************

void  l6474_start_motor_move (MOTOR_BLOCK *mtr_blk, int move_cmd)
{
    mtr_blk->mtr_move_type = MOVE_CMD;

         //-----------------------------------------------------
         // Send ENABLE cmd via SPI to Enable L6474 powerstage
         //-----------------------------------------------------
    L6474_SendCommand (mtr_blk, L6474_ENABLE);

    if (mtr_blk->mtr_accel_end_position != 0)      // Is this redundant w/upper layer ?
       mtr_blk->mtr_motion_state = MOTION_ACCELERATING;
       else mtr_blk->mtr_motion_state = MOTION_DECELERATING;    

    mtr_blk->accu = 0;
    mtr_blk->mtr_new_relative_position = 0;        // clear for new move

        // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        //     PWMs finally go on at this point   - STM32_F4
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    L6474_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_min_velocity_pc);
}


//******************************************************************************
//  l6474_motor_adc_ctl_init
//
//         Configure ADC to read contropl inputs for current sensors, ...
//             ADC-6  PA6   ADC12_IN6   "A" side Current-Sense   Arduino D12
//             ADC-4  PA4   ADC12_IN4   "B" side Current-Sense   Arduino A2
//******************************************************************************
int  l6474_motor_adc_ctl_init (void)
{
      // see l6206 code for details

    return (0);           // denote was successful
}


//******************************************************************************
//  l6474_motor_speed_adc_init                                          SPEED
//
//         Configure ADC to read potentiometer used for speed control setting.
//        -     Speed Ctl Pot ADC    J1-2    PB5  AIN11   Grove J3-7/27 -> J1-2
//******************************************************************************
int  l6474_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr)
{
    uint8_t   status = 1;

#if defined(SETUP_LATER)
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
#endif

    return (0);           // denote was successful
}


//******************************************************************************
//  l6474_motor_adc_next_conversion
//
//         start a new ADC conversion for Speed control  (called every 50 ms)
//         Uses A15  on pin  P6.0
//******************************************************************************
int  l6474_motor_adc_next_conversion (void)
{
    int    rc;

       //------------------------------------------------
       // if this is first time, enable the ADC and DMA
       //------------------------------------------------
    if (_g_adc_enabled == 0)
       {
         _g_adc_enabled = 1;           // denote we started the ADC and its DMA
       }

#if defined(STUP_LATER)
       //--------------------------------------------------------------
       // Trigger a new round of ADC conversions.
       // The following starts the ADC and auto-initiates Conversions
       //--------------------------------------------------------------
    rc = HAL_ADC_Start_DMA (&ADC_InitStruct,
                            (uint32_t*) &ADC0_Seq0_Conversion_Values[0],
                            _g_adc_num_channels);
#endif

    return (0);           // denote was successful
}


//******************************************************************************
//  l6474_motor_gpio_init_inputs
//
//         Configure control inputs for Motor GPIOs  - FAULT, ...
//
//     STM32_F4 settings:
//            FAULT/FLAG   PA10 / D2            Active_Low
//******************************************************************************

int  l6474_motor_init_ctl_gpio_inputs (void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

        // Configure L6474 - FAULT/FLAG pin                        (PA10 / D2)
    GPIO_InitStruct.Pin   = GPIO_PIN_10;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

        // Set Priority of External Line Interrupt used for the FLAG interrupt
    HAL_NVIC_SetPriority (EXTI15_10_IRQn, 5, 0);

        // Enable the External Line Interrupt used for the FLAG interrupt
    HAL_NVIC_EnableIRQ (EXTI15_10_IRQn);                        // for PA10 EXTI

    return (0);           // denote was successful
}


//******************************************************************************
//  l6474_motor_init_ctl_gpio_outputs
//
//         Configure control outputs for Motor GPIOs - ENABLE, as well as SPI
//
//     STM32_F4 settings:
//            DIR          PA8 / D7
//            STBY/RESET   PA9 / D8
//            SPI1 CS      PB6 / D10
//            SPI1 CLK     PA5 / D13
//            SPI1 MISO    PA6 / D12
//            SPI1 MOSI    PA7 / D11
//******************************************************************************
int  l6474_motor_init_ctl_gpio_outputs (void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    int               spimode,    rc;

           // Configure L6474 - DIR 1 pin for first Motor (0)      (PA8 / D7)
    GPIO_InitStruct.Pin   = GPIO_PIN_8;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

        // Configure L6474 - STBY/RESET pin                        (PA9 / D8)
    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
    L6474_Board_Reset (0);

        //--------------------------------------------------------
        //  setup SPI for access to L6474 - using SPI1   (F4_01)
        //
        // First, configure SPI associated GPIO pins for SPI mode
        //--------------------------------------------------------
    GPIO_InitStruct.Pin       = GPIO_PIN_5;          // setup SCLK  (PA5 / D13)
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_6;          // setup MISO  (PA6 / D12)
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_7;          // setup MOSI  (PA7 / D11)
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_6;          // setup CS    (PB6 / D10)
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP; //   standard GPIO pin
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_MEDIUM;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // force CS High (De-Assert)

         //------------------------------------------------------------
         //                      SPI Module Config
         //
         //          Setup HAL info needed for SPI Configuration
         //------------------------------------------------------------
    __SPI1_CLK_ENABLE();             // Enable SPI peripheral clocks
    __SPI1_FORCE_RESET();            // Force reset of peripheral
    __SPI1_RELEASE_RESET();

    spimode = 3;                     // L6474 uses SPI Mode 3
    _g_hspi1_handle.Instance             = SPI1;      // set assoc SPI HW module
    _g_hspi1_handle.Init.Mode            = SPI_MODE_MASTER;
    _g_hspi1_handle.Init.Direction       = SPI_DIRECTION_2LINES;
    _g_hspi1_handle.Init.DataSize        = SPI_DATASIZE_8BIT;
    if (spimode == 0)
      { _g_hspi1_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
        _g_hspi1_handle.Init.CLKPhase    = SPI_PHASE_1EDGE;
      }
     else if (spimode == 1)
      { _g_hspi1_handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
        _g_hspi1_handle.Init.CLKPhase    = SPI_PHASE_1EDGE;
      }
     else if (spimode == 2)
      { _g_hspi1_handle.Init.CLKPolarity = SPI_POLARITY_LOW;  // CPOL = 0, CPHA = 1
        _g_hspi1_handle.Init.CLKPhase    = SPI_PHASE_2EDGE;
      }
     else if (spimode == 3)
      { _g_hspi1_handle.Init.CLKPolarity = SPI_POLARITY_HIGH; //EASY_SPIN, PRESCALE = 32
        _g_hspi1_handle.Init.CLKPhase    = SPI_PHASE_2EDGE;   //Ada ST7735 LCD SCALE = 8
      }
    _g_hspi1_handle.Init.NSS             = SPI_NSS_SOFT;
    _g_hspi1_handle.Init.FirstBit        = SPI_FIRSTBIT_MSB;
    _g_hspi1_handle.Init.CRCPolynomial   = 7;
    _g_hspi1_handle.Init.CRCCalculation  = SPI_CRCCALCULATION_DISABLED;
    _g_hspi1_handle.Init.TIMode          = SPI_TIMODE_DISABLED;

       //----------------------------------------------------------------------
       // This is literally a pre-scaler value. It only indirectly sets the SPI baud rate.
       // PCLK2 max frequency is 100 MHz.  BAUD = PCLK2/SPI_BaudRatePrescaler.
       //
       // That's just the way ST's "baud rate generation" works. It's just a
       // prescalar off of a system clock. No fine grain, separate, baud generator.
       //----------------------------------------------------------------------
    _g_hspi1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;

    rc = HAL_SPI_Init (&_g_hspi1_handle); // go intialize the SPI module

L6474_Board_SpiInit();  // 08/13/16 - TEMP HACK to ensure SpiHandle in _nucleo_ihm01a1.c is properly intiialized

    __HAL_SPI_ENABLE (&_g_hspi1_handle);  // turn on SPI Enable flag SPI_CR1_SPE in CR1

    return (rc);                          // denote was successful
}


//******************************************************************************
//  l6474_motor_encoder_gpio_init                                      ENCODER
//
//         Configure GPIOs for Wheel Encoder pin inputs ...
//              Hall Sensor Left      PA8   Arduino D7    GPIO rupt   Grove D7
//              Hall Sensor Right     PA9   Arduino D8    GPIO rupt   Grove D8
//******************************************************************************
int  l6474_motor_encoder_gpio_init (MOTOR_BLOCK *mtr_blk, int encoder_type)
{

    return (0);           // denote was successful
}


//******************************************************************************
//  l6474_motor_pwm_init
//
//         Configure PWMs to driver motor 1/2ENA and 3/4ENA.
//         These use a variable duty cycle to control motor speed.
//
//         Generates a PWM with a frequency of 20,000 Hz (50 usec).
//
//         Note that the PWM for a motor should run at least 10x motor spe8d ???
//
//    Configure STEP CLOCK output for Stepper
//       STEP_CLOCK   Motor 1    TIM3_CH2     PC7 / D9
//       STEP_CLOCK   Motor 2    TIM2_CH2
//       STEP_CLOCK   Motor 3    TIM4_CH3
//******************************************************************************

/// GPIO Pin used for L6474 step clock (PWM) pin of device 0  =  PC7  ORIG CODE


int  l6474_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long period_ticks)
{

    GPIO_InitTypeDef         GPIO_InitStruct;
    TIM_OC_InitTypeDef       sConfigOC;
    TIM_MasterConfigTypeDef  sMasterConfig;
    uint32_t                 channel;

     //----------------------------------------------------------------------
     // for now, only worry about Motor 1 (0) PWM  which uses TIM3  Channel 2
     //----------------------------------------------------------------------
     // Other motors will be in more generalize in my generic STM32 "board support"

    __TIM3_CLK_ENABLE();                  // enable TIM3 Clock

       // configuration PWM channel's GPIO pin for PWM output
    GPIO_InitStruct.Pin       = GPIO_PIN_7;                   // (PC7 / D9)
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);

       // Set Interrupt Group Priority of PWM timer's Interrupt
    HAL_NVIC_SetPriority (TIM3_IRQn, 4, 0);

       // Enable the PWM timer's global Interrupt
    HAL_NVIC_EnableIRQ (TIM3_IRQn);

        //-------------------------------------------------
        // now configure the PWM itself    - Motor 1 (0)
        //-------------------------------------------------
    hTimPwm1.Instance           = TIM3;
    hTimPwm1.Init.Prescaler     = TIMER_PRESCALER - 1;
    hTimPwm1.Init.CounterMode   = TIM_COUNTERMODE_UP;
    hTimPwm1.Init.Period        = 0;
    hTimPwm1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init (&hTimPwm1);

    channel              = TIM_CHANNEL_2;   // and configure PWM channel pin
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel (&hTimPwm1, &sConfigOC, channel);

    return (0);           // denote was successful
}



//******************************************************************************
//  l6474_motor_run
//
//               Run the motor, i.e. issue a start (or re-start after a
//                              direction change or after a stop).
//
//               Because a stop will slam off the PWMs, we re-establish both the
//               Direction and the Duty Cycle.
//******************************************************************************
int  l6474_motor_run (MOTOR_BLOCK *mtr_blk, int direction)
{
    mtr_blk->mtr_state = MOTOR_STATE_RUN;     // denote we are now in RUN mode

//  TBD - => free running Stepper (no counts or distance, or REJCT as NOT_SUPPORTED ?

    return (0);           // denote was successful
}


//******************************************************************************
//  l6474_motor_set_direction
//
//      Update Motor DIR setting to set desired CW/CCW direction.
//
//******************************************************************************
int  l6474_motor_set_direction (MOTOR_BLOCK *mtr_blk,  int direction)
{
    int  pwm_duty_count;

    mtr_blk->mtr_direction = direction;     // ensure desired direction is saved

       //-------------------------------------------------------------
       // Set the DIR pin to be either FORWARD or BACKWARD - PA8 / D7
       //-------------------------------------------------------------
    if (direction == DIRECTION_FORWARD)
       HAL_GPIO_WritePin (GPIOA, GPIO_PIN_8,
                         (GPIO_PinState) 0);      // FORWARD:  turn OFF
       else HAL_GPIO_WritePin (GPIOA, GPIO_PIN_8,
                         (GPIO_PinState) 1);      // BACKWARD: turn ON

    return (0);                 // denote worked OK
}


//******************************************************************************
//  l6474_get_motor_actual_position
//                            gets the absolute position of the motor, from
//                            the contreoller (stepper) or encoder QEI (BLDC)
//******************************************************************************
int32_t l6474_get_motor_actual_position (MOTOR_BLOCK *mtr_blk)
{
    int32_t      raw_position;
    int32_t      abs_position;

    raw_position = L6474_CmdGetParam (mtr_blk, L6474_ABS_POS);

    abs_position = L6474_ConvertPosition (raw_position);

    return (abs_position);
}



//******************************************************************************
//  l6474_motor_set_pwm_duty_count
//
//            Set the duty cycle to be used for a specific motor.
//            Here, duty cycle is explicitly passed as PWM ticks, not as percent.
//            The duty cycle sets the motor's speed. (More juice, more speed)
//
//   Parms:
//      motor_num:   number of the motor to update (1 or 2)
//      pwm_duty:    actual duty cycle PWM count value to be loaded in PWM
//******************************************************************************
int  l6474_motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count)
{
     int  adjusted_duty_count;

     mtr_blk->mtr_pwm_duty_count = pwm_duty_count;  // save actual count (not %)

// TBD - Anythings else to do for a stepper >  Does duty even make sense ?
//           or issue REJECT ?

     return (0);                             // do not apply till motor start

}

//******************************************************************************
//  l6474_motor_do_ctl_action
//                             Issue a command to perform some kind of control
//                       action on the motor (SetHome, SetMark, EnableMotor, ...
//******************************************************************************
int  l6474_motor_do_ctl_action (MOTOR_BLOCK *mtr_blk, int cmd, int flags)
{
    uint32_t   mark;

    switch (cmd)
      {
        case CMD_SET_HOME:
                  L6474_CmdSetParam (mtr_blk, L6474_ABS_POS, 0);
                  break;

        case CMD_SET_MARK:
                  mark = L6474_CmdGetParam (mtr_blk, L6474_ABS_POS);
                  L6474_CmdSetParam (mtr_blk, L6474_MARK, mark);
                  break;

        case CMD_ENGAGE_MOTOR:            // Enables the L6474 power stage
                        // Issue Enable command to the L6474 motor.
                        // Turns on the stepper motor, getting it ready to move.
                  L6474_SendCommand (mtr_blk, L6474_ENABLE);
                  break;

        case CMD_DISENGAGE_MOTOR:         // disables the L6474 power stage
                        // Issue Disable command to the L6474 motor
                  L6474_SendCommand (mtr_blk, L6474_DISABLE);
                  break;
      }

    return (0);                      // denote worked OK
}


//******************************************************************************
//  l6474_perform_motor_move
//                             Issue a command to start the motor moving
//******************************************************************************
            /* Motor activation */
int  l6474_perform_motor_move (MOTOR_BLOCK *mtr_blk, int cmd, int direction, int32_t count)
{
    int   move_op;

    switch (cmd)
      {
        case MOVE_TO_POSITION:        // move to a absolute position
        case MOVE_N_STEPS:            // move N number of relative steps
                    move_op = MOVE_CMD;
                    break;

        case MOVE_N_ROTATIONS:        // spin N number of rotations
        case MOVE_DISTANCE:           // move a specific linear distance
                    break;

        case MOVE_TO_HOME:            // move motor to Home position
                    move_op = MOVE_CMD;
                    break;

        case MOVE_TO_MARK:            // move motor to last "Mark" position
                    move_op = MOVE_CMD;
                    break;

        case MOVE_FREE_RUN:           // continuously spin motor at given speed
                    move_op = RUN_CMD;
                    break;
      }

    l6474_start_motor_move (mtr_blk, move_op);     // start the motor

    return (0);                      // denote worked OK
}


//******************************************************************************
//  l6474_motor_stop                                          motor_stop_handler
//
//                             Stop the motor. The stop_type inidcates the
//                             type of stop (Hard, Soft, Hiz, ...)
//******************************************************************************
int  l6474_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags)
{
    switch (stop_type)
      {
        case STOP_HARD_BRAKE:                      // Immediatly stops the motor
                L6474_Board_PwmStop (mtr_blk->mtr_id); // Disable corresponding PWM
                break;

        case STOP_SOFT_BRAKE:       // Stops motor by using motor's deceleration (Coast to stop)
                if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
                   mtr_blk->mtr_operation_performed = SOFT_STOP_CMD;
                // the Step Clock Handler ISR will handle this op code and cause Coast shutdown
                break;

        case STOP_HIZ:       // Immediatly stops motor and disables power bridge
                L6474_Board_PwmStop (mtr_blk->mtr_id); // Disable corresponding PWM
                             // issue cmd to Disable power stage
                L6474_SendCommand (mtr_blk, L6474_DISABLE);
                break;
      }

    g_CmdGetStatus = l6474_CmdGetStatus (mtr_blk, 0); // clear any L6474 FAULT status (WVD added - for good measure)

       // Clear everytrhing to denote we are in the INACTIVE state
    mtr_blk->mtr_motion_state        = MOTION_NOT_ACTIVE;
    mtr_blk->mtr_operation_performed = NO_CMD;
    mtr_blk->mtr_steps_to_move       = MAX_STEPS;  

    return (0);                      // denote worked OK
}


//******************************************************************************
//  l6474_motor_init_controller_chip
//******************************************************************************
int  l6474_motor_init_controller_chip (void)
{
      // specific L6474 motor parms are fed in during l6474_motor_init
// 08/13/16 TEMP DISABLE  L6474_Init (&_g_L6474_Init_Params);    // initialize the chip (via SPI cmds)
}


/**********************************************************
 *                        Init  -  this uses ALL the old ORIG logic !!!
 *                                 called from l6474_motor_init_controller_chip() above
 * @brief Starts a new L6474 instance 
 * @param[in] pInit pointer to the initialization data
 * @retval None
 **********************************************************/
void  L6474_Init (void *pInit)  
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk [l6474DriverInstance];    // this use of l6474DriverInstance is really HOKEY !
                                                   // but that is the way ST set it up. It then gets 
                                                   // incremented after each L6474_Init() call !
                                                   // But it actually is used as the motor_id by the called routines
       //-------------------------------
       // Initialise additional GPIOs  ???
       //------------------------------      // this calls ORIGINAL nucleo_ihm01a1.c logic  08/12/16
    L6474_Board_GpioInit (mtr_blk->mtr_id);  // 08/06/16 - is in stm32f4xx_nucleo_ihm01a1.c
  
    if (L6474_Board_SpiInit() != 0)          // this calls ORIGINAL nucleo_ihm01a1.c logic  08/12/16
      {
        L6474_ErrorHandler (L6474_ERROR_0);  // report Initialization error
      } 

        //---------------------------------------------------------------------
        // Initialise the PWMs used for the Step clock for the specified motor
        //---------------------------------------------------------------------
    L6474_Board_PwmInit (mtr_blk->mtr_id);   // 08/06/16 - is in stm32f4xx_nucleo_ihm01a1.c
                                             // this calls ORIGINAL nucleo_ihm01a1.c logic  08/12/16
        // Standby-reset deactivation - 
        // i.e. turn off RESET and ENABLE the L6474  // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
    L6474_Board_ReleaseReset (mtr_blk->mtr_id);      //  L6464 FAULT light comes ON at this point - STM32_F4
                                                     // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        /* Let a 10 ms delay after reset */
    L6474_Board_Delay (10);                  // 08/06/16 - is in stm32f4xx_nucleo_ihm01a1.c
  
        /* Set all registers and context variables to the predefined values from l6474_target_config.h */
    if (pInit == 0)
       {
          L6474_SetDeviceParamsToPredefinedValues (mtr_blk);
       }
      else
       {    // 08/12/16 - HANGS FOREREVER IN HERE ==> not all of ORIG init logic was performed, or are missing IRQ enable !
         L6474_SetDeviceParamsToGivenValues (mtr_blk, pInit);
       }

         /* Disable the L6474 power stage */
    L6474_SendCommand (mtr_blk, L6474_DISABLE);

       //--------------------------------------------------
       // Issue "Get Status" to clear flags after start up
       //--------------------------------------------------  // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
    g_CmdGetStatus = l6474_CmdGetStatus (mtr_blk, 0);       //  L6474 FAULT light goes OFF after this instruction - STM32_F4
                                                         //  NOTE: PWMs are still off at this point on STM32_F4
  l6474DriverInstance++;                                 //  g_CmdGetStatus = 0x0000FC03 on STH32_F4 !
                                                         // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    g_CmdGetStatus_2 = l6474_CmdGetStatus (mtr_blk, 0);  // issue back to back to see if it clear FAULT reg as it is supposed to
}


/******************************************************//**
 *                  Attach Error Handler
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error Hanlder
 * @retval None
 **********************************************************/
void  L6474_AttachErrorHandler (void (*callback)(uint16_t))
{
    errorHandlerCallback = (void (*)(uint16_t))callback;
}


/******************************************************//**
 *                 Attach Flag Interrupt 
 * @brief  Attaches a user callback to the flag Interrupt
 * The call back will be then called each time the status 
 * flag pin will be pulled down due to the occurrence of 
 * a programmed alarms ( OCD, thermal pre-warning or 
 * shutdown, UVLO, wrong command, non-performable command)
 * @param[in] callback Name of the callback to attach 
 * to the Flag Interrupt
 * @retval None
 **********************************************************/
void  L6474_AttachFlagInterrupt (void (*callback)(void))
{
    flagInterruptCallback = (void (*)())callback;
}


/**********************************************************
 *                        Error Handler
 *
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void  L6474_ErrorHandler (uint16_t error)
{
  if (errorHandlerCallback != 0)
     {
       (void) errorHandlerCallback(error);
     }
    else   
     {
       while (1)
          {
              /* Infinite loop */
          }
     }
}


/**********************************************************
 *
 *                 FAULT / Flag Interrupt Handler
 *
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @retval None
 **********************************************************/
void  L6474_FlagInterruptHandler (void)
{
  if (flagInterruptCallback != 0)
     {
          /* Set isr flag */
       isrFlag = TRUE;
    
       flagInterruptCallback();
    
           /* Reset isr flag */
       isrFlag = FALSE;   
     }
}


/*******************************************************************************
 *                                Apply Speed
 *
 *        Updates the current speed of the device. Causes juice to flow.
 *
 *        Causes PWMs and rupts to engage by calling L6474_Board_PwmNSetFreq()
 *
 * @param[in] motor_id (from 0 to 2)
 * @param[in] newSpeed in pps
 * @retval None
 ******************************************************************************/
void  L6474_ApplySpeed (MOTOR_BLOCK *mtr_blk, uint8_t pwmId, uint16_t newSpeed)
{

  if (newSpeed < L6474_MIN_PWM_FREQ)
     {
       newSpeed = L6474_MIN_PWM_FREQ;  
     }
  if (newSpeed > L6474_MAX_PWM_FREQ)
     {
       newSpeed = L6474_MAX_PWM_FREQ;
     }
  
  mtr_blk->mtr_current_velocity_pc = newSpeed;

  switch (mtr_blk->mtr_id)
  {
    case  0:
      L6474_Board_Pwm1SetFreq (newSpeed);
      break;

    case 1:
      L6474_Board_Pwm2SetFreq (newSpeed);
      break;

    case 2:
      L6474_Board_Pwm3SetFreq (newSpeed);
      break;

    default:
      break;          // ignore / error
  }
}


/*******************************************************************************
 *                             ConvertPosition
 *
 *      Converts the ABS_POSITION register value to a 32b signed integer
 *
 * @param[in] abs_position_reg value of the ABS_POSITION register
 * @retval operation_result 32b signed integer corresponding to the absolute position 
 ******************************************************************************/
int32_t  L6474_ConvertPosition (uint32_t abs_position_reg)
{
    int32_t  operation_result;

    if (abs_position_reg & L6474_ABS_POS_SIGN_BIT_MASK) 
       {
              // Handle a Negative register value
         abs_position_reg = ~abs_position_reg;
         abs_position_reg += 1;

         operation_result = (int32_t) (abs_position_reg & L6474_ABS_POS_VALUE_MASK);
         operation_result = -operation_result;
       } 
      else 
       {
         operation_result = (int32_t) abs_position_reg;
       }
    return (operation_result);
}


/******************************************************//**
 *                        Cmd Get Param
 * @brief  Issues the GetParam command to the L6474 of the specified motor
 * @param[in] motor_id (from 0 to 2)
 * @param[in] param Register adress (L6474_ABS_POS, L6474_MARK,...)
 * @retval Register value
 **********************************************************/
uint32_t  L6474_CmdGetParam (MOTOR_BLOCK *mtr_blk, uint32_t param)
{
  uint32_t  i;
  uint8_t   spiIndex;
  uint32_t  spiRxData;
  uint8_t   maxArgumentNbBytes = 0;
  bool      itDisable = FALSE;  
  
   spiIndex = numberOfDevices - mtr_blk->mtr_id - 1;
   if (spiIndex > 16)
      spiIndex = 0;          // must have had a wraparound
 do
  {
    spiPreemtionByIsr = FALSE;
    if (itDisable)
      {
           // re-enable L6474_Board_EnableIrq if disable in previous iteration
        L6474_Board_EnableIrq();
        itDisable = FALSE;
      }
  
    for (i = 0; i < numberOfDevices; i++)
      {
        spiTxBursts[0][i] = L6474_NOP;    // initially clear buffer to NOPs
        spiTxBursts[1][i] = L6474_NOP;
        spiTxBursts[2][i] = L6474_NOP;
        spiTxBursts[3][i] = L6474_NOP;
        spiRxBursts[1][i] = 0;
        spiRxBursts[2][i] = 0;
        spiRxBursts[3][i] = 0;    
      }
    switch (param)
    {
      case L6474_ABS_POS: ;
      case L6474_MARK:
        spiTxBursts[0][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (param);
        maxArgumentNbBytes = 3;
        break;

      case L6474_EL_POS: ;
      case L6474_CONFIG: ;
      case L6474_STATUS:
        spiTxBursts[1][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (param);
        maxArgumentNbBytes = 2;
        break;

      default:
        spiTxBursts[2][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }
    
        /* Disable interruption before checking */
        /* pre-emption by ISR and SPI transfers*/
    L6474_Board_DisableIrq();
    itDisable = TRUE;
  } while (spiPreemtionByIsr); // check pre-emption by ISR
    
  for (i = L6474_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6474_CMD_ARG_MAX_NB_BYTES;
       i++)
     {
       L6474_Write_SPI_Bytes (&spiTxBursts[i][0],
                              &spiRxBursts[i][0]);
     }
  
  spiRxData = ((uint32_t)spiRxBursts[1][spiIndex] << 16)|
              (spiRxBursts[2][spiIndex] << 8) |
              (spiRxBursts[3][spiIndex]);
  
      /* re-enable L6474_Board_EnableIrq after SPI transfers */
  L6474_Board_EnableIrq();
    
  return (spiRxData);
}


/******************************************************//**
 *                      Cmd Get Status
 * @brief  Issues the GetStatus command to the L6474 of the specified motor
 * @param[in] motor_id (from 0 to 2)
 * @retval Status Register value
 * @note Once the GetStatus command is performed, the flags of the status register
 * are reset. This is not the case when the status register is read with the
 * GetParam command (via the functions L6474ReadStatusRegister or L6474CmdGetParam).
 **********************************************************/
int32_t  l6474_CmdGetStatus (MOTOR_BLOCK *mtr_blk, int status_type)
{
  uint32_t   i;
  uint16_t   status;
  uint8_t    spiIndex;
  bool       itDisable = FALSE;  

   spiIndex = numberOfDevices - mtr_blk->mtr_id - 1;
   if (spiIndex > 16)
      spiIndex = 0;          // must have had a wraparound

  do
  {
    spiPreemtionByIsr = FALSE;
    if (itDisable)
       {
           /* re-enable L6474_Board_EnableIrq if disable in previous iteration */
         L6474_Board_EnableIrq();
         itDisable = FALSE;
       }

    for (i = 0; i < numberOfDevices; i++)
      {
        spiTxBursts[0][i] = L6474_NOP;    // initially clear buffer to NOPs
        spiTxBursts[1][i] = L6474_NOP;
        spiTxBursts[2][i] = L6474_NOP;
        spiRxBursts[1][i] = 0;
        spiRxBursts[2][i] = 0;
      }
    spiTxBursts[0][spiIndex] = L6474_GET_STATUS;

       /* Disable interruption before checking */
       /* pre-emption by ISR and SPI transfers*/
    L6474_Board_DisableIrq();
    itDisable = TRUE;
  } while (spiPreemtionByIsr); // check pre-emption by ISR

  for (i = 0; i < L6474_CMD_ARG_NB_BYTES_GET_STATUS + L6474_RSP_NB_BYTES_GET_STATUS; i++)
    {    // 08/13/16 - Is dropping into ERROR at this point
      L6474_Write_SPI_Bytes (&spiTxBursts[i][0], &spiRxBursts[i][0]);
    }
  status = (spiRxBursts[1][spiIndex] << 8) | (spiRxBursts[2][spiIndex]);
  
     /* re-enable L6474_Board_EnableIrq after SPI transfers*/
  L6474_Board_EnableIrq();
  
  return (status);
}


/******************************************************//**
 * @brief  Issues the Nop command to the L6474 of the specified motor
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6474_CmdNop (MOTOR_BLOCK *mtr_blk)
{
    L6474_SendCommand (mtr_blk, L6474_NOP);
}


/******************************************************//**
 *                      Cmd Set Param
 * @brief  Issues the SetParam command to the L6474 of the specified motor
 * @param[in] motor_id (from 0 to 2)
 * @param[in] param Register adress (L6474_ABS_POS, L6474_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 **********************************************************/
void  L6474_CmdSetParam (MOTOR_BLOCK *mtr_blk,
                         uint32_t param,
                         uint32_t value)
{
  uint32_t   i;
  uint8_t    spiIndex;
  uint8_t    maxArgumentNbBytes = 0;
  bool       itDisable = FALSE;

  spiIndex = numberOfDevices - mtr_blk->mtr_id - 1;
  if (spiIndex > 16)
     spiIndex = 0;          // must have had a wraparound
  do
  {
    spiPreemtionByIsr = FALSE;
    if (itDisable)
       {
            // re-enable L6474_Board_EnableIrq if disable in previous iteration
         L6474_Board_EnableIrq();
         itDisable = FALSE;
       }
    for (i = 0; i < numberOfDevices; i++)
      {
        spiTxBursts[0][i] = L6474_NOP;    // initially clear buffer to NOPs
        spiTxBursts[1][i] = L6474_NOP;
        spiTxBursts[2][i] = L6474_NOP;
        spiTxBursts[3][i] = L6474_NOP;
      }

   switch (param)
   {
    case L6474_ABS_POS: ;
    case L6474_MARK:
        spiTxBursts[0][spiIndex] = param;
        spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 3;
        break;

    case L6474_EL_POS: ;
    case L6474_CONFIG:
        spiTxBursts[1][spiIndex] = param;
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 2;
        break;

    default:
        spiTxBursts[2][spiIndex] = param;
        maxArgumentNbBytes = 1;
    }

    spiTxBursts[3][spiIndex] = (uint8_t)(value);
    
       /* Disable interruption before checking */
       /* pre-emption by ISR and SPI transfers*/
    L6474_Board_DisableIrq();
    itDisable = TRUE;
  } while (spiPreemtionByIsr); // check pre-emption by ISR
 
        /* issue SPI transfer */
  for (i = L6474_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6474_CMD_ARG_MAX_NB_BYTES;
       i++)
     {
       L6474_Write_SPI_Bytes (&spiTxBursts[i][0],&spiRxBursts[i][0]);
     }

        /* re-enable L6474_Board_EnableIrq after SPI transfers */
  L6474_Board_EnableIrq();
}


/******************************************************//**
 * @brief Converts mA in compatible values for OCD_TH register
 * @param[in] Tval
 * @retval OCD_TH values
 **********************************************************/
inline uint8_t  L6474_Convert_Ocd_Thresh_to_RegValue (float Tval)
{
    return ((uint8_t) (((Tval - 375)*0.002666f)+0.5f));
}


/******************************************************//**
 * @brief Converts  OCD_TH register values in mA 
 * @param[in] Par OCD regiser value
 * @retval mA
 **********************************************************/
inline float  L6474_Convert_Ocd_RegValue_to_Threshold (uint8_t Par)
{
    return (((float) (Par + 1))*375.f);
}

/******************************************************//**
 * @brief Converts mA in compatible values for TVAL register 
 * @param[in] Tval
 * @retval TVAL values
 **********************************************************/
inline uint8_t  L6474_Convert_Tval_Current_to_RegValue (float Tval)
{
    return ((uint8_t) (((Tval - 31.25f)*0.032f)+0.5f));
}

/******************************************************//**
 * @brief Converts  TVAL register values in mA 
 * @param[in] Par TVAL regiser value
 * @retval mA
 **********************************************************/
inline float  L6474_Convert_Tval_RegValue_to_Current (uint8_t Par)
{
    return (((float) (Par + 1))*31.25f);
}


/******************************************************//**
 * @brief Convert TON/TOFF values in time (us)
 * @param[in] Par Values from TON_MIN/TOFF_MIN 
 * @retval time in us
 **********************************************************/
inline float  L6474_Convert_RegValue_to_Tmin_Time (uint8_t Par)
{
    return (((float) (Par + 1)) * 0.5f);
}


/******************************************************//**
 * @brief Convert time in us in compatible values 
 * for TON_MIN register
 * @param[in] Tmin
 * @retval TON_MIN values
 **********************************************************/
inline uint8_t  L6474_Convert_Tmin_Time_to_RegValue (float Tmin)
{
    return ((uint8_t) (((Tmin - 0.5f)*2.0f)+0.5f));
}



/******************************************************//**
 *                        Get Analog Value  
 * @brief Issues L6474 Get analog values from register  
 * L6474_ABS_POS, L6474_MARK, L6474_TVAL, L6474_OCD_TH,
 * L6474_TON_MIN, L6474_TOFF_MIN. The raw register value is 
 * converted in alanog values.
 * For other registers, the returned value is the raw value.
 * @param[in] motor_id (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param L6474 register address
 * @retval Register value - 1 to 3 bytes (depends on register)
 *********************************************************/
float  L6474_GetAnalogValue (MOTOR_BLOCK *mtr_blk, uint32_t param)
{
    float     value;
    uint32_t  registerValue;

    registerValue = L6474_CmdGetParam (mtr_blk, param);

    switch (param)
     {
      case L6474_ABS_POS:
      case L6474_MARK:
        value = (float) L6474_ConvertPosition (registerValue);
        break;

      case L6474_TVAL:
        value = L6474_Convert_Tval_RegValue_to_Current (registerValue);    
        break;

      case L6474_OCD_TH:
        value = L6474_Convert_Ocd_RegValue_to_Threshold (registerValue);    
        break;

      case L6474_TON_MIN:
      case L6474_TOFF_MIN:
        value = L6474_Convert_RegValue_to_Tmin_Time (registerValue);
        break;

      default:
        value = (float) registerValue;
    }

  return value;
}



/******************************************************//**
 * @brief Returns the device state
 * @param[in] motor_id (from 0 to 2)
 * @retval State (ACCELERATING, DECELERATING, STEADY or INACTIVE)
 **********************************************************/
int  L6474_GetDeviceState (MOTOR_BLOCK *mtr_blk)
{
    return (mtr_blk->mtr_motion_state);
}


/******************************************************//**
 * @brief Returns the FW version of the library
 * @retval L6474_FW_VERSION
 **********************************************************/
uint8_t  L6474_GetFwVersion (void)
{
    return (L6474_FW_VERSION);
}


/******************************************************//**
 * @brief  Return motor handle (pointer to the L6474 motor driver structure)
 * @retval Pointer to the motorDrv_t structure
 **********************************************************/
MOTOR_HANDLERS  * L6474_GetMotorHandle (void)
{
    return (&drv6474_table);
}


/******************************************************//**
 * @brief  Returns the mark position  of the specified motor
 * @param[in] motor_id (from 0 to 2)
 * @retval Mark register value converted in a 32b signed integer 
 **********************************************************/
int32_t  l6474_get_mark (MOTOR_BLOCK *mtr_blk)
{
    return L6474_ConvertPosition(L6474_CmdGetParam(mtr_blk,L6474_MARK));
}


/******************************************************//**
 *                   Hard Stop
 * @brief  Immediatly stops the motor 
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6474_HardStop (MOTOR_BLOCK *mtr_blk) 
{
       /* Disable corresponding PWM */
    L6474_Board_PwmStop (mtr_blk->mtr_id);

    g_CmdGetStatus = l6474_CmdGetStatus (mtr_blk, 0); // clear any L6474 FAULT status

       /* Set inactive state */
    mtr_blk->mtr_motion_state        = MOTION_NOT_ACTIVE;
    mtr_blk->mtr_operation_performed = NO_CMD;
    mtr_blk->mtr_steps_to_move       = MAX_STEPS;  
}


/******************************************************//**
 * @brief  Immediatly stops the motor and disable the power bridge
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6474_HizStop (MOTOR_BLOCK *mtr_blk) 
{
       /* Disable corresponding PWM */
    L6474_Board_PwmStop (mtr_blk->mtr_id);

    g_CmdGetStatus = l6474_CmdGetStatus (mtr_blk, 0); // clear any L6474 FAULT status

       /* Disable power stage */
    L6474_SendCommand (mtr_blk, L6474_DISABLE);

       /* Set inactive state */
    mtr_blk->mtr_motion_state        = MOTION_NOT_ACTIVE;
    mtr_blk->mtr_operation_performed = NO_CMD;
    mtr_blk->mtr_steps_to_move       = MAX_STEPS;  
}


/******************************************************//**
 *                Read Status Register
 * @brief  Reads the Status Register value
 * @param[in] motor_id (from 0 to 2)
 * @retval Status register valued
 * @note The status register flags are not cleared 
 * at the difference with L6474CmdGetStatus()
 **********************************************************/
uint16_t  L6474_ReadStatusRegister (MOTOR_BLOCK *mtr_blk)
{
    return (L6474_CmdGetParam(mtr_blk,L6474_STATUS));
}


/******************************************************//**
 * @brief Read id
 * @retval Id of the l6474 Driver Instance
 **********************************************************/
uint16_t  L6474_ReadId (void)
{
    return (l6474DriverInstance);
}


/******************************************************//**
 * @brief Resets all L6474 devices
 * @retval None
 **********************************************************/
void  L6474_ResetAllDevices (void)
{
     uint8_t      loop;
     MOTOR_BLOCK  *mtr_blk;

     for (loop = 0; loop < numberOfDevices; loop++)
      {
           /* Stop movement and disable power stage */
        mtr_blk = &motor_blk[loop];
        L6474_HizStop (mtr_blk);
        L6474_Reset (mtr_blk);
        L6474_Board_Delay (10); // Reset pin must be forced low for at least 10 usec
        L6474_Board_ReleaseReset (loop);
        L6474_Board_Delay (10); 
      }
}


/******************************************************//**
 *                     Release Reset
 * @brief  Releases the L6474 reset (pin set to High) of all devices
 * @retval None
 **********************************************************/
void  L6474_ReleaseReset (MOTOR_BLOCK *mtr_blk)
{ 
    L6474_Board_ReleaseReset (mtr_blk->mtr_id); 
}


/******************************************************//**
 * @brief  Resets the L6474 (reset pin set to low) of all devices
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6474_Reset (MOTOR_BLOCK *mtr_blk)
{
    L6474_Board_Reset (mtr_blk->mtr_id); 
}


/******************************************************//**
 *                      Select Step Mode               L6474_SelectStepMode
 * @brief  Set the stepping mode 
 * @param[in] motor_id (from 0 to 2)
 * @param[in] stepMod from full step to 1/16 microstep as specified in enum STEPPER_MODE_t
 * @retval None
 **********************************************************/
int  l6474_motor_set_step_mode (MOTOR_BLOCK *mtr_blk, int step_mode, int flags)
{
  uint8_t            stepModeRegister;
  L6474_STEP_SEL_t   l6474StepMod;
  
  switch (step_mode)
  {
    case STEP_MODE_FULL:
      l6474StepMod = L6474_STEP_SEL_1;
      break;

    case STEP_MODE_HALF:
      l6474StepMod = L6474_STEP_SEL_1_2;
      break;    

    case STEP_MODE_1_4:
      l6474StepMod = L6474_STEP_SEL_1_4;
      break;        

    case STEP_MODE_1_8:
      l6474StepMod = L6474_STEP_SEL_1_8;
      break;       

    case STEP_MODE_1_16:
    default:
      l6474StepMod = L6474_STEP_SEL_1_16;
      break;       
  }

       // Deactivate motor 08/13/16 - CRATERs in here when tries to issue SPI I/O. Drops into ERROR rtn ==> Tmout ?
    L6474_HizStop (mtr_blk);
  
       /* Read Step mode register and clear STEP_SEL field */
    stepModeRegister = (uint8_t) (0xF8 & L6474_CmdGetParam(mtr_blk,L6474_STEP_MODE)) ;
  
       /* Apply new step mode */
    L6474_CmdSetParam (mtr_blk, L6474_STEP_MODE, stepModeRegister | (uint8_t) l6474StepMod);

       /* Reset abs pos register */
    L6474_CmdSetParam (mtr_blk, L6474_ABS_POS, 0);
}


/******************************************************//**
 *                          Send Command
 * @brief  Sends a command without arguments to the L6474 via the SPI
 * @param[in] motor_id (from 0 to 2)
 * @param[in] param Command to send 
 * @retval None
 **********************************************************/
void  L6474_SendCommand (MOTOR_BLOCK *mtr_blk, uint8_t param)
{
  uint32_t  i;
  uint8_t   spiIndex;
  bool      itDisable = FALSE;  

  spiIndex = numberOfDevices - mtr_blk->mtr_id - 1;
  if (spiIndex > 16)
     spiIndex = 0;          // must have had a wraparound

  do
  {
     spiPreemtionByIsr = FALSE;
     if (itDisable)
        {
             // re-enable interrupts if disabled in previous iteration
          L6474_Board_EnableIrq();
          itDisable = FALSE;
        }
  
     for (i = 0; i < numberOfDevices; i++)
       {
         spiTxBursts[3][i] = L6474_NOP;     
       }
     spiTxBursts[3][spiIndex] = param;
    
        // Disable interrupts before checking to avoid
        // pre-emption by ISR and SPI transfer
     L6474_Board_DisableIrq();
     itDisable = TRUE;
  } while (spiPreemtionByIsr);     // check pre-emption by ISR

  L6474_Write_SPI_Bytes (&spiTxBursts[3][0], &spiRxBursts[3][0]); 
  
        // re-enable interrupts after SPI transfers
  L6474_Board_EnableIrq();
}


/******************************************************//**
 *                     Set Analog Value
 *
 * @brief Set registers L6474_ABS_POS, L6474_MARK, L6474_TVAL, L6474_OCD_TH,
 * L6474_TON_MIN, L6474_TOFF_MIN by analog values. These 
 * analog values are automatically converted in bit values
 * @param[in] motor_id (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Register adress (L6474_ABS_POS, L6474_EL_POS, L6474_MARK, 
 * L6474_TVAL, L6474_TON_MIN, L6474_TOFF_MIN, L6474_OCD_TH)
 * @param[in] value Analog value to convert and set into the register
 * @retval TRUE if param and param is valid, FALSE otherwise
 *********************************************************/
bool  L6474_SetAnalogValue (MOTOR_BLOCK *mtr_blk, uint32_t param, float value)
{
  uint32_t  registerValue;
  bool      result = TRUE;

  if ((value < 0) && (param != L6474_ABS_POS) && (param != L6474_MARK)) 
  {
    result = FALSE;
  }
  else
  {
    switch (param)
    {
      case L6474_EL_POS:
        if  ((value !=0)&&
            ((value > (L6474_ELPOS_STEP_MASK | L6474_ELPOS_MICROSTEP_MASK))||
             (value < (1<<(7-(L6474_STEP_MODE_STEP_SEL & L6474_CmdGetParam(0,L6474_STEP_MODE)))))))
        {
          result = FALSE;
        }
        else
        {
          registerValue = ((uint32_t) value)& (L6474_ELPOS_STEP_MASK | L6474_ELPOS_MICROSTEP_MASK);
        }
        break;

      case L6474_ABS_POS:
      case L6474_MARK:
        if ((value >= L6474_MIN_POSITION) &&
            (value <= L6474_MAX_POSITION))
        {
          if (value >= 0)
          {
            registerValue = ((uint32_t) value)& L6474_ABS_POS_VALUE_MASK;
          }
          else
          {
            registerValue = L6474_ABS_POS_VALUE_MASK - (((uint32_t) (-value))& L6474_ABS_POS_VALUE_MASK) + 1;
          }
        }
        else 
        {
          result = FALSE;
        }
        break;

      case L6474_TVAL:
        if (value > L6474_TVAL_MAX_VALUE)
        {
          result = FALSE;
        }
        else
        {
          registerValue = L6474_Convert_Tval_Current_to_RegValue(value);
        }
        break;

      case L6474_OCD_TH:
        if (value > L6474_OCD_TH_MAX_VALUE)
        {
          result = FALSE;
        }
        else 
        {
          registerValue = L6474_Convert_Ocd_Thresh_to_RegValue (value);
        }
        break;

      case L6474_TON_MIN:
      case L6474_TOFF_MIN:
        if (value > L6474_TOFF_TON_MIN_MAX_VALUE)
        {
          result = FALSE;
        }
        else
        {
          registerValue = L6474_Convert_Tmin_Time_to_RegValue(value);
        }
        break;    
      default:
        result = FALSE;
     }

    if (result != FALSE)
       {
         L6474_CmdSetParam (mtr_blk, param, registerValue);
       }
   }
  return result;
}


/******************************************************//**
 *                     Set Direction
 * @brief  Specifies the direction 
 * @param[in] motor_id (from 0 to 2)
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is only applied if the device 
 * is in INACTIVE state
 * @retval None
 **********************************************************/
void  L6474_SetDirection (MOTOR_BLOCK *mtr_blk, int dir)
{
     L6474_Board_SetDirectionGpio (mtr_blk->mtr_id, dir);
}


/******************************************************//**
 *                  Set Device Params To GivenValues
 *
 * @brief  Set the parameters of the device to values of pInitPrm structure
 * @param[in] motor_id (from 0 to 2)
 * @param pInitPrm pointer to a structure containing the initial device parameters 
 * @retval None
 **********************************************************/
void  L6474_SetDeviceParamsToGivenValues (MOTOR_BLOCK *mtr_blk, L6474_Init_t *pInitPrm)
{
  mtr_blk->mtr_acceleration_pc     = pInitPrm->acceleration_step_s2;
  mtr_blk->mtr_deceleration_pc     = pInitPrm->deceleration_step_s2;
  mtr_blk->mtr_max_velocity_pc     = pInitPrm->maximum_speed_step_s;
  mtr_blk->mtr_min_velocity_pc     = pInitPrm->minimum_speed_step_s;
  
  mtr_blk->accu                     = 0;
  mtr_blk->mtr_current_position     = 0;
  mtr_blk->mtr_accel_end_position   = 0;
  mtr_blk->mtr_new_relative_position = 0;
  mtr_blk->mtr_decel_begin_position = 0;
  mtr_blk->mtr_steps_to_move        = 0;
  mtr_blk->mtr_current_velocity_pc  = 0;
  mtr_blk->mtr_operation_performed  = NO_CMD;
  mtr_blk->mtr_direction            = DIRECTION_FORWARD;
  mtr_blk->mtr_motion_state         = MOTION_NOT_ACTIVE;  
 
  L6474_SetRegisterToGivenValues (mtr_blk, pInitPrm);
}


/******************************************************//**
 *             Set Device Params To Predefined Values
 * @brief  Sets the parameters of the device to predefined values
 * from l6474_target_config.h
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6474_SetDeviceParamsToPredefinedValues (MOTOR_BLOCK *mtr_blk)
{
  mtr_blk->mtr_acceleration_pc = L6474_CONF_PARAM_ACC_DEVICE_0;
  mtr_blk->mtr_deceleration_pc = L6474_CONF_PARAM_DEC_DEVICE_0;
  mtr_blk->mtr_max_velocity_pc = L6474_CONF_PARAM_MAX_SPEED_DEVICE_0;
  mtr_blk->mtr_min_velocity_pc = L6474_CONF_PARAM_MIN_SPEED_DEVICE_0;
  
  mtr_blk->accu                      = 0;
  mtr_blk->mtr_current_position      = 0;
  mtr_blk->mtr_accel_end_position    = 0;
  mtr_blk->mtr_new_relative_position = 0;
  mtr_blk->mtr_decel_begin_position  = 0;
  mtr_blk->mtr_steps_to_move         = 0;
  mtr_blk->mtr_current_velocity_pc   = 0;
  mtr_blk->mtr_operation_performed   = NO_CMD;
  mtr_blk->mtr_direction             = DIRECTION_FORWARD;
  mtr_blk->mtr_motion_state          = MOTION_NOT_ACTIVE;
  
  L6474_SetRegisterToPredefinedValues (mtr_blk);
}




/******************************************************//**
 * @brief  Sets the number of Motors to be used 
 * @param[in] nbDevices (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval TRUE if successfull, FALSE if failure, attempt to set a number of 
 * devices greater than MAX_NUMBER_OF_DEVICES
 **********************************************************/
bool  L6474_SetNumMotors (uint8_t num_motors)
{
    if (num_motors <= MAX_NUMBER_OF_DEVICES)
       {
         numberOfDevices = num_motors;
         return TRUE;
       }
      else
       {
         return FALSE;
       }
}


/******************************************************//**
 * @brief  Sets the registers of the L6474 to the given values from pInitPrm
 * @param[in] motor_id (from 0 to 2)
 * @param pInitPrm pointer to a structure containing the initial device parameters 
 * @retval None
 **********************************************************/
void  L6474_SetRegisterToGivenValues (MOTOR_BLOCK *mtr_blk, L6474_Init_t *pInitPrm)
{
  L6474_CmdSetParam (mtr_blk,
                     L6474_ABS_POS,
                     0);
  L6474_CmdSetParam (mtr_blk,
                     L6474_EL_POS,
                     0);
  L6474_CmdSetParam (mtr_blk,
                     L6474_MARK,
                     0);
  L6474_SetAnalogValue (mtr_blk,
                        L6474_TVAL,
                        pInitPrm->torque_regulation_current_mA);
  L6474_CmdSetParam (mtr_blk,
                     L6474_T_FAST,
                     (uint8_t) pInitPrm->maximum_fast_decay_time |
                     (uint8_t) pInitPrm->fall_time);
  L6474_SetAnalogValue (mtr_blk,
                        L6474_TON_MIN,
                        pInitPrm->minimum_ON_time_us);
  L6474_SetAnalogValue (mtr_blk,
                        L6474_TOFF_MIN,
                        pInitPrm->minimum_OFF_time_us);
  L6474_SetAnalogValue (mtr_blk,
                        L6474_OCD_TH,
                        pInitPrm->overcurrent_threshold);
  L6474_CmdSetParam (mtr_blk,
                     L6474_STEP_MODE,
                     (uint8_t) pInitPrm->step_selection |
                     (uint8_t) pInitPrm->sync_selection);

  L6474_CmdSetParam (mtr_blk,
                     L6474_ALARM_EN,
                     pInitPrm->alarm);
//                   0);                   // Has no effect. Red FAULT stays on

  L6474_CmdSetParam (mtr_blk,
                     L6474_CONFIG,
                     (uint16_t) pInitPrm->clock |
                     (uint16_t) pInitPrm->torque_regulation_method |
                     (uint16_t) pInitPrm->overcurrent_shutwdown |
                     (uint16_t) pInitPrm->slew_rate |
                     (uint16_t) pInitPrm->target_swicthing_period);
  
}


/******************************************************//**
 * @brief  Sets the registers of the L6474 to their predefined values 
 * from l6474_target_config.h
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6474_SetRegisterToPredefinedValues (MOTOR_BLOCK *mtr_blk)
{                                 
                           // L6474 FAULT light is on at entry STM32_F4
  L6474_CmdSetParam (mtr_blk,
                     L6474_ABS_POS,
                     0);
  L6474_CmdSetParam (mtr_blk,
                     L6474_EL_POS,
                     0);
  L6474_CmdSetParam (mtr_blk,
                     L6474_MARK,
                     0);
  switch (mtr_blk->mtr_id)
  {
    case 0:
      L6474_CmdSetParam (mtr_blk,
                        L6474_TVAL,
                        L6474_Convert_Tval_Current_to_RegValue(L6474_CONF_PARAM_TVAL_DEVICE_0));
      L6474_CmdSetParam (mtr_blk,
                              L6474_T_FAST,
                              (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_0 |
                              (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_0);
      L6474_CmdSetParam (mtr_blk,
                              L6474_TON_MIN,
                              L6474_Convert_Tmin_Time_to_RegValue(L6474_CONF_PARAM_TON_MIN_DEVICE_0)
                                );
      L6474_CmdSetParam (mtr_blk,
                              L6474_TOFF_MIN,
                              L6474_Convert_Tmin_Time_to_RegValue(L6474_CONF_PARAM_TOFF_MIN_DEVICE_0));
      L6474_CmdSetParam (mtr_blk,
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_0);
      L6474_CmdSetParam (mtr_blk,
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_0 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_0);

      L6474_CmdSetParam (mtr_blk,
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_0);   // TEMP TEST HACK
//                      0);                             // Ditto has no effect

      L6474_CmdSetParam (mtr_blk,
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_0);
      break;

    case 1:
      L6474_CmdSetParam (mtr_blk,
                        L6474_TVAL,
                        L6474_Convert_Tval_Current_to_RegValue (L6474_CONF_PARAM_TVAL_DEVICE_1));
      L6474_CmdSetParam (mtr_blk,
                        L6474_T_FAST,
                        (uint8_t) L6474_CONF_PARAM_TOFF_FAST_DEVICE_1 |
                        (uint8_t) L6474_CONF_PARAM_FAST_STEP_DEVICE_1);
      L6474_CmdSetParam (mtr_blk,
                        L6474_TON_MIN,
                        L6474_Convert_Tmin_Time_to_RegValue (L6474_CONF_PARAM_TON_MIN_DEVICE_1));
      L6474_CmdSetParam (mtr_blk,
                        L6474_TOFF_MIN,
                        L6474_Convert_Tmin_Time_to_RegValue (L6474_CONF_PARAM_TOFF_MIN_DEVICE_1));
      L6474_CmdSetParam (mtr_blk,
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_1);
      L6474_CmdSetParam (mtr_blk,
                        L6474_STEP_MODE,
                        (uint8_t) L6474_CONF_PARAM_STEP_SEL_DEVICE_1 |
                        (uint8_t) L6474_CONF_PARAM_SYNC_SEL_DEVICE_1);
      L6474_CmdSetParam (mtr_blk,
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_1);
      L6474_CmdSetParam (mtr_blk,
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_1);
      break;

    case 2:
      L6474_CmdSetParam (mtr_blk,
                        L6474_TVAL,
                        L6474_Convert_Tval_Current_to_RegValue (L6474_CONF_PARAM_TVAL_DEVICE_2));
      L6474_CmdSetParam (mtr_blk,
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_2 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_2);
      L6474_CmdSetParam (mtr_blk,
                        L6474_TON_MIN,
                        L6474_Convert_Tmin_Time_to_RegValue (L6474_CONF_PARAM_TON_MIN_DEVICE_2));
      L6474_CmdSetParam (mtr_blk,
                        L6474_TOFF_MIN,
                        L6474_Convert_Tmin_Time_to_RegValue (L6474_CONF_PARAM_TOFF_MIN_DEVICE_2));
      L6474_CmdSetParam (mtr_blk,
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_2);
      L6474_CmdSetParam (mtr_blk,
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_2 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_2);
      L6474_CmdSetParam (mtr_blk,
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_2);
      L6474_CmdSetParam (mtr_blk,
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_2);
      break;

    default: ;
  }                            // L6474 FAULT light stll on at exit - STM32_F4
}


/*******************************************************************************
 *                                                                    KEY  KEY
 *                            Start Movement
 *
 *   Initialises the bridge parameters to start the movement
 *   and enable the power bridge
 *
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 ******************************************************************************/
void  l6474_StartMovement (MOTOR_BLOCK *mtr_blk, int move_cmd)  
{
    mtr_blk->mtr_operation_performed = move_cmd;  // save type of move being executed

         // Send ENABLE cmd via SPI to Enable L6474 powerstage
    L6474_SendCommand (mtr_blk, L6474_ENABLE);

    if (mtr_blk->mtr_accel_end_position != 0)
       mtr_blk->mtr_motion_state = MOTION_ACCELERATING;
       else mtr_blk->mtr_motion_state = MOTION_DECELERATING;    

    mtr_blk->accu        = 0;
    mtr_blk->mtr_new_relative_position = 0;
                                                              // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
    L6474_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_min_velocity_pc); //   PWMs finally go on at this point - STM32_F4
                                                              // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
}


/******************************************************//**
 *                      Soft Stop
 * @brief  Stops the motor by using the device deceleration
 * @param[in] motor_id (from 0 to 2)
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is in INACTIVE state.
 **********************************************************/
bool  L6474_SoftStop (MOTOR_BLOCK *mtr_blk)
{    
    bool cmdExecuted = FALSE;
  
    if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
       {
         mtr_blk->mtr_operation_performed = SOFT_STOP_CMD;
         cmdExecuted = TRUE;
       }
    return (cmdExecuted);
}


/******************************************************//**
*                     Wait While Active
*
*      Locks until the device state becomes Inactive (motion complete)
*
* @param[in] motor_id (from 0 to 2)
 **********************************************************/
int  l6474_WaitWhileActive (MOTOR_BLOCK *mtr_blk, int wait_type, int flags)
{
        // Wait while the motor is running and still in motion.
    while (L6474_GetDeviceState(mtr_blk) !=  MOTION_NOT_ACTIVE)
      ;

    return (0);                     // denote completed OK
}


/******************************************************//**
*                     L6474_Write_SPI_Bytes
*
*                Write and receive a byte via SPI
*
* @param[in] pByteToTransmit pointer to the byte to transmit
* @param[in] pReceivedByte   pointer to the received byte
* @retval None
**********************************************************/
void  L6474_Write_SPI_Bytes (uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
{
    int    rc;

           // ??? !!! following logic is in: stm32f4xx_nucleo_ihm01a1.c   ??? !!!
           //         needs to be re-written and moved to board_support
    rc = L6474_Board_SpiWriteBytes (pByteToTransmit, pReceivedByte, numberOfDevices);

    if (rc != 0)
       {
         L6474_ErrorHandler (L6474_ERROR_1);
       }
  
    if (isrFlag)
       {
         spiPreemtionByIsr = TRUE;  // ??? !!! This is OLD LOGIC, not in new rev
       }
}


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

#if defined(DEFINED_IN_L6474_BASIC_C)
void  L6474_StepClockHandler (uint8_t motor_id)
{
    uint32_t  relativePos;
    uint32_t  endAccPos;
    uint32_t  acc;
    uint32_t  deceleration;
    uint16_t  speed;
    uint16_t  maxSpeed;
    bool      speedUpdated;

      /* Set isr flag */
  isrFlag = TRUE;
  
      // Increment the relative position that we have moved thus far
  mtr_blk->mtr_new_relative_position++;
  relativePos  = mtr_blk->mtr_new_relative_position;

  switch (mtr_blk->mtr_motion_state) 
  {
    case ACCELERATING: 
        speed = mtr_blk->mtr_current_velocity_pc;
        acc   = ((uint32_t) mtr_blk->mtr_acceleration_pc << 16);

           //--------------------------------------------------------
           // For GOTO and MOVE commands, we change states when the
           // relative position (relativePos) matches either the
           // mtr_accel_end_position or mtr_decel_begin_position.
           //--------------------------------------------------------
        if ((mtr_blk->mtr_operation_performed == SOFT_STOP_CMD)
          || ((mtr_blk->mtr_operation_performed != RUN_CMD) 
          &&  (relativePos == mtr_blk->mtr_decel_begin_position)))
           {
             mtr_blk->mtr_motion_state = MOTION_DECELERATING;
             mtr_blk->accu = 0;
           }
          else if ((speed >= mtr_blk->mtr_max_velocity_pc)
                 || ((mtr_blk->mtr_operation_performed != RUN_CMD)
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
                    {
                      speed = mtr_blk->mtr_max_velocity_pc;
                    }    
                 mtr_blk->mtr_current_velocity_pc = speed;
                 L6474_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_current_velocity_pc);
               }
          }
        break;

    case STEADY: 
      maxSpeed = mtr_blk->mtr_max_velocity_pc;

           //--------------------------------------------------------
           // For GOTO and MOVE commands, we change states when the
           // relative position (relativePos) matches mtr_decel_begin_position.
           // For RUN commands, we change states when speed > maxSpeed.
           //--------------------------------------------------------
      if ((mtr_blk->mtr_operation_performed == SOFT_STOP_CMD)
         || ((mtr_blk->mtr_operation_performed != RUN_CMD)
         &&  (relativePos >= (mtr_blk->mtr_decel_begin_position)))
         || ((mtr_blk->mtr_operation_performed == RUN_CMD)
         &&  (mtr_blk->mtr_current_velocity_pc > maxSpeed)))
       {
         mtr_blk->mtr_motion_state = MOTION_DECELERATING;
         mtr_blk->accu = 0;
       }
      else if ((mtr_blk->mtr_operation_performed == RUN_CMD)
             && (mtr_blk->mtr_current_velocity_pc < maxSpeed))
               {
                 mtr_blk->mtr_motion_state = MOTION_ACCELERATING;
                 mtr_blk->accu = 0;
               }
      break;

    case DECELERATING: 
      speed        = mtr_blk->mtr_current_velocity_pc;
      deceleration = ((uint32_t) mtr_blk->mtr_deceleration_pc << 16);

           //--------------------------------------------------------
           // For GOTO and MOVE commands, we change states to STOP
           // when relative position (relativePos) matches mtr_steps_to_move.
           //--------------------------------------------------------
      if (((mtr_blk->mtr_operation_performed == SOFT_STOP_CMD) 
        && (speed <=  mtr_blk->mtr_min_velocity_pc))
        || ((mtr_blk->mtr_operation_performed != RUN_CMD)
        &&  (relativePos >= mtr_blk->mtr_steps_to_move)))
         {     //------------------------------------------------
               // Motion process is complete.  Shut off the PWM
               //------------------------------------------------
            L6474_HardStop (mtr_blk);
         }
      else if ((mtr_blk->mtr_operation_performed == RUN_CMD)
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
                 {  
                   speed -= 1;
                 }
              speedUpdated = TRUE;
            }
        
          if (speedUpdated)
             {
               if (speed < mtr_blk->mtr_min_velocity_pc)
                  {
                    speed = mtr_blk->mtr_min_velocity_pc;
                  }  
               mtr_blk->mtr_current_velocity_pc = speed;
               L6474_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_current_velocity_pc);
             }
        }
      }
      break;

    INACTIVE: // if one last interrupt is left in pipe, this can happen
      inactive_state_rupt++;
    default:
      break;
   }                               //  end  switch

        /* Clear isr flag */
  isrFlag = FALSE;
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

//********************************************************************************
//                         file    l6474.h

#if defined(REDUNDANT_IMPORTS)
///Diretion field of STATUS register
typedef enum {
  L6474_STATUS_DIR_FORWARD = (((uint16_t) 0x0001) << 4),
  L6474_STATUS_DIR_REVERSE = (((uint16_t) 0x0000) << 4)
} L6474_STATUS_DIR_t;


/// L6474 command set
typedef enum {
  L6474_NOP           = ((uint8_t) 0x00),
  L6474_SET_PARAM     = ((uint8_t) 0x00),
  L6474_GET_PARAM     = ((uint8_t) 0x20),
  L6474_ENABLE        = ((uint8_t) 0xB8),
  L6474_DISABLE       = ((uint8_t) 0xA8),
  L6474_GET_STATUS    = ((uint8_t) 0xD0),
  L6474_RESERVED_CMD1 = ((uint8_t) 0xEB),
  L6474_RESERVED_CMD2 = ((uint8_t) 0xF8)
} L6474_Commands_t;
#endif




/*****************************************************************************
  *
  *                             PWM  ISR  Callback
  *
  * @param[in] htim PWM handle pointer
  * @retval None
*****************************************************************************/
    extern   MOTOR_BLOCK   motor_blk [];
 
void  HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
{
    MOTOR_BLOCK   *mtr_blk;

  if ((htim->Instance == TIM3) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
     {     // process motor 1
       mtr_blk = &motor_blk[0];
       if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
          {
            L6474_StepClockHandler (0);
          }
     }
  if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
     {     // process motor 2
       mtr_blk = &motor_blk[1];
       if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
          {
            L6474_StepClockHandler (1);
          }
     }

  if ((htim->Instance == TIM4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3))
     {     // process motor 3
       mtr_blk = &motor_blk[2];
       HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_10);
       if ((mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
         && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET))
          {
            L6474_StepClockHandler (2);
          }
      }
}


/*****************************************************************************
  *
  *                        External Line Callback  ISR          FLAG/FAULT pin
  *
  * @param[in] GPIO_Pin pin number
  * @retval None
*****************************************************************************/
void  HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_10)                    // Fault_Flag  pin PA10 / D2 
     {
       L6474_FlagInterruptHandler();
     }
 }

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
//         LOGIC PULLED FROM ST CODE:  stm32f4xx_nucleo.c        BEGIN
#define LEDn                                    1

#define LED2_PIN                                GPIO_PIN_5
#define LED2_GPIO_PORT                          GPIOA
#define LED2_GPIO_CLK_ENABLE()                  __GPIOA_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()                 __GPIOA_CLK_DISABLE()  

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)    do{if((__INDEX__) == 0) LED2_GPIO_CLK_ENABLE(); \
                                             }while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)    do{if((__INDEX__) == 0) LED2_GPIO_CLK_DISABLE(); \
                                              }while(0)
#define BUTTONn                                 1  

/**
  * @brief Key push-button
  */
#define USER_BUTTON_PIN                         GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT                   GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()   
#define USER_BUTTON_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()  
#define USER_BUTTON_EXTI_LINE                   GPIO_PIN_13
#define USER_BUTTON_EXTI_IRQn                   EXTI15_10_IRQn
/* Aliases */
#define KEY_BUTTON_PIN                          USER_BUTTON_PIN
#define KEY_BUTTON_GPIO_PORT                    USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_GPIO_CLK_ENABLE()            USER_BUTTON_GPIO_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()           USER_BUTTON_GPIO_CLK_DISABLE()
#define KEY_BUTTON_EXTI_LINE                    USER_BUTTON_EXTI_LINE
#define KEY_BUTTON_EXTI_IRQn                    USER_BUTTON_EXTI_IRQn


#define BUTTONx_GPIO_CLK_ENABLE(__BUTTON__)    do { if((__BUTTON__) == BUTTON_USER) { USER_BUTTON_GPIO_CLK_ENABLE(); } } while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__BUTTON__)   do { if((__BUTTON__) == BUTTON_USER) { USER_BUTTON_GPIO_CLK_DISABLE(); } } while(0)

#if defined(WVD_SKIP)
typedef enum  {
                   LED2 = 0
              } Led_TypeDef;

typedef enum  {  
                   BUTTON_USER = 0,                
                   BUTTON_KEY  = BUTTON_USER          /* Alias */
              } Button_TypeDef;

typedef enum  {  
                    BUTTON_MODE_GPIO = 0,
                    BUTTON_MODE_EXTI = 1
              } ButtonMode_TypeDef;
#endif

GPIO_TypeDef    *GPIO_PORT[LEDn] = {LED2_GPIO_PORT};

const uint16_t  GPIO_PIN[LEDn]   = {LED2_PIN};

GPIO_TypeDef    *BUTTON_PORT[BUTTONn] = {KEY_BUTTON_GPIO_PORT}; 
const uint16_t  BUTTON_PIN[BUTTONn]   = {KEY_BUTTON_PIN}; 
const uint8_t   BUTTON_IRQn[BUTTONn]  = {KEY_BUTTON_EXTI_IRQn};


/******************************************************************************
  * @brief  Configure Board's on-board LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void   BSP_LED_Init (Led_TypeDef Led)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
  
         /* Enable the GPIO_LED Clock */
    LEDx_GPIO_CLK_ENABLE(Led);
  
         /* Configure the GPIO_LED pin */
    GPIO_InitStruct.Pin   = GPIO_PIN[Led];
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
    HAL_GPIO_Init (GPIO_PORT[Led], &GPIO_InitStruct);
  
    HAL_GPIO_WritePin (GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void  BSP_LED_On (Led_TypeDef Led)
{
    HAL_GPIO_WritePin (GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void  BSP_LED_Off (Led_TypeDef Led)
{
    HAL_GPIO_WritePin (GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED2  
  * @retval None
  */
void  BSP_LED_Toggle (Led_TypeDef Led)
{
    HAL_GPIO_TogglePin (GPIO_PORT[Led], GPIO_PIN[Led]);
}


/**************************************************************************************
  * @brief  Configure on-board Button GPIO and EXTI Line.
  *
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_KEY
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability  
  * @retval None
  */
void  BSP_PB_Init (Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
   GPIO_InitTypeDef GPIO_InitStruct;
  
         /* Enable the BUTTON Clock */
   BUTTONx_GPIO_CLK_ENABLE(Button);
  
   if (ButtonMode == BUTTON_MODE_GPIO)
     {
           /* Configure Button pin as input */
       GPIO_InitStruct.Pin = BUTTON_PIN[Button];
       GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
       GPIO_InitStruct.Pull = GPIO_PULLDOWN;
       GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
       HAL_GPIO_Init (BUTTON_PORT[Button], &GPIO_InitStruct);
     }
  
   if (ButtonMode == BUTTON_MODE_EXTI)
     {
           /* Configure Button pin as input with External interrupt */
       GPIO_InitStruct.Pin = BUTTON_PIN[Button];
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; 
       HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
    
          /* Enable and set Button EXTI Interrupt to the lowest priority */
       HAL_NVIC_SetPriority ((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
       HAL_NVIC_EnableIRQ ((IRQn_Type)(BUTTON_IRQn[Button]));
     }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_KEY  
  * @retval The Button GPIO pin value.
  */
uint32_t  BSP_PB_GetState (Button_TypeDef Button)
{
    return HAL_GPIO_ReadPin (BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

//         LOGIC PULLED FROM ST CODE:  stm32f4xx_nucleo.c        END
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
