// was MotorLib_LL_L6474.c               SPI is _NOT_ used on L6230 !

//*******1*********2*********3*********4*********5*********6*********7**********
//
//                             MotorLib_LL_L6230_STM32.c
//
// Motor Control Library:  Low Level Device support for L6230 BLDC Controller
//                         running on a STM32 Cortex-M processor.
//
// This is a portable code library, designed to work across a number of
// processors, including:   TI Tiva Cortex-M
//                          TI MSP432 with Cortex-M core
//                          TI MSP430 F5529, FR5969, FR6989, FR5994
//                          STM32 F3, F4, L4, L7 Cortex-M series
//
// Provides support for the following BLDC controller:
//     - L6230   BLDC H-Bridge controller. used on the STM32 Xnucleo
//               expansion boards.
//
// 12 / 24 Volts is supplied to BLDC motors via BLDC Controller.
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
// 12V xxxx Motor:
// Anaheim Automation BLY1724xxx motors, with and without encoders
// Technic (Microchip) PMSM motor
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
//   05/16/16 - Created as part of Open Source Motor Library support. Duquaine
//   07/22/16 - Rolled in L6230 specific changes for BLDC support.    Duquaine
//   07/30/16 - Got baseline L6230 code running reliably.             Duquaine
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
#include "l6230.h"                    // L6230 command codes, register defs, etc
#include "MC_SixStep_param.h"         // need for 6Step constants, etc
#include "6Step_Lib.h"
#include "MC_Common.h"

    int   _g_adc_enabled = 0;
    int   isrFlag        = 0;

    int   mc_init_performed = 0;

    SPI_HandleTypeDef       _g_hspi1_handle;    // SPI 1 support

    TIM_HandleTypeDef       hTimPwm1;           // Handle for Motor 1's PWM

    ADC_HandleTypeDef       ADC_InitStruct;

#define  TIMER_PRESCALER    1024

#if (NOT_USED_BLDC)
           //-------------------------------------------------------------------
           // This table should all be populated by #define entries from MotorLib_motor_config.h
           //-------------------------------------------------------------------
// ??? is this even used in BLDC. ??? Appears to be a relic of L6206 support !!!
L6230_Init_t  _g_L6230_Init_Params =
{
    160,                       // Acceleration rate in step/s2. Range: (0..+inf)
    160,                       // Deceleration rate in step/s2. Range: (0..+inf)
    1600,                      // Maximum speed in step/sec. Range: (30..10000].
    800,                       // Minimum speed in step/sec. Range: [30..10000).
    250,                       // Torque regulation current in mA. (TVAL reg)
                               //                      Range: 31.25mA to 4000mA.
    750,                       // Overcurrent threshold (OCD_TH register).
                               //                      Range: 375mA to 6000mA.
    L6230_CONFIG_OC_SD_ENABLE,       // Overcurrent shutwdown (OC_SD field of CONFIG register).
    L6230_CONFIG_EN_TQREG_TVAL_USED, // Torque regulation method (EN_TQREG field of CONFIG register).
    L6230_STEP_SEL_1_16,             // Step selection (STEP_SEL field of STEP_MODE register).
    L6230_SYNC_SEL_1_2,              // Sync selection (SYNC_SEL field of STEP_MODE register).
    L6230_FAST_STEP_12us,            // Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
    L6230_TOFF_FAST_8us,             // Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
    3,                               // Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
    21,                              // Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
    L6230_CONFIG_TOFF_044us,         // Target Swicthing Period (field TOFF of CONFIG register).
    L6230_CONFIG_SR_320V_us,         // Slew rate (POW_SR field of CONFIG register).
    L6230_CONFIG_INT_16MHZ,          // Clock setting (OSC_CLK_SEL field of CONFIG register).
    (L6230_ALARM_EN_OVERCURRENT      |
     L6230_ALARM_EN_THERMAL_SHUTDOWN |
     L6230_ALARM_EN_THERMAL_WARNING  |
     L6230_ALARM_EN_UNDERVOLTAGE     |
     L6230_ALARM_EN_SW_TURN_ON       |
     L6230_ALARM_EN_WRONG_NPERF_CMD)    // Alarm (ALARM_EN register).
};
#endif                                  // (NOT_USED_BLDC)


// VVVVVVVVVVVVVVVVVVVVVVVVVV  direct lif froom L6230 main.c code
void  MX_GPIO_Init(void);
void  MX_ADC1_Init(void);
void  MX_TIM1_Init(void);
void  MX_TIM3_Init(void);
void  MX_TIM4_Init(void);
void  MX_USART2_UART_Init(void);

void  MC_SixStep_ADC_Channel (uint32_t adc_ch);

// ^^^^^^^^^^^^^^^^^^^^^^^^^^  end  direct lift from L6230 code


int     l6230_GetDeviceState (MOTOR_BLOCK *mtr_blk);
int32_t l6230_get_motor_actual_position (MOTOR_BLOCK *mtr_blk);
int     l6230_motor_do_ctl_action (MOTOR_BLOCK *mtr_blk, int cmd, int flags);
int     l6230_perform_motor_move (MOTOR_BLOCK *mtr_blk, int cmd, int direction, int32_t count);
int     l6230_motor_set_step_mode (MOTOR_BLOCK *mtr_blk, int step_mode, int flags);
int     l6230_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags);
void    l6230_StartMovement (MOTOR_BLOCK *mtr_blk, int move_cmd);
int32_t l6230_get_mark (MOTOR_BLOCK *mtr_blk);
int32_t l6230_CmdGetStatus (MOTOR_BLOCK *mtr_blk, int status_type);
int     l6230_WaitWhileActive (MOTOR_BLOCK *mtr_blk, int wait_type, int flags);
int     l6230_motor_init_controller_chip (void);

int     l6230_motor_run (MOTOR_BLOCK *mtr_blk, int direction);
int     l6230_motor_set_direction (MOTOR_BLOCK *mtr_blk, int mtr_direction);
int     l6230_motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count);
void    l6230_StepClockHandler (uint8_t motor_id);


//#include "l6474.h"       // TEMP HACK
#define L6230_MAX_PWM_FREQ   (10000)     // Maximum frequency of the PWMs in Hz
#define L6230_MIN_PWM_FREQ   (2)         // Minimum frequency of the PWMs in Hz

                         //    =====   TEMP   HACK    =======
void    L6230_ApplySpeed(MOTOR_BLOCK *mtr_blk, uint8_t pwmId, uint16_t newSpeed);
void    L6230_ComputeSpeedProfile(MOTOR_BLOCK *mtr_blk, uint32_t nbSteps);
int32_t L6230_ConvertPosition(uint32_t abs_position_reg);
uint8_t L6230_Convert_Ocd_Thresh_to_RegValue(float Tval);
float   L6230_Convert_Ocd_RegValue_to_Threshold(uint8_t Par);
uint8_t L6230_Convert_Tval_Current_to_RegValue(float Tval);
uint8_t L6230_Convert_Tmin_Time_to_RegValue(float Tmin);
float   L6230_Convert_RegValue_to_Tmin_Time(uint8_t Par);
float   L6230_Convert_Tval_RegValue_to_Current(uint8_t Par);
void    L6230_ErrorHandler(uint16_t error);
void    L6230_FlagInterruptHandler(void);
void    L6230_SendCommand(MOTOR_BLOCK *mtr_blk, uint8_t param);
//void    L6230_SetRegisterToGivenValues(MOTOR_BLOCK *mtr_blk, L6230_Init_t *pInitPrm);   // L6230_Init_t issue again - relic from L6206 code ?
//void    L6230_SetDeviceParamsToGivenValues(MOTOR_BLOCK *mtr_blk, L6230_Init_t *pInitPrm); //   ditto
void    L6230_SetRegisterToPredefinedValues(MOTOR_BLOCK *mtr_blk);
void    L6230_Write_SPI_Bytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte);
void    L6230_SetDeviceParamsToPredefinedValues(MOTOR_BLOCK *mtr_blk);
void    l6230_start_motor_move(MOTOR_BLOCK *mtr_blk, int move_cmd);
void    l6230_StepClockHandler(uint8_t motor_id);

void  L6230_AttachErrorHandler (void (*callback)(uint16_t));      // L6206 relic
void  L6230_AttachFlagInterrupt (void (*callback)(void));         // L6206 relic
uint32_t  L6230_CmdGetParam (MOTOR_BLOCK *mtr_blk, uint32_t param); // L6206 relic
uint8_t  L6230_Convert_Ocd_Thresh_to_RegValue (float Tval);       // L6206 relic
void  L6230_Init (void *pInit);                                   // L6206 relic
uint8_t  L6230_GetFwVersion (void);                               // L6206 relic
void  L6230_HardStop (MOTOR_BLOCK *mtr_blk);                      // L6206 relic
void  L6230_HizStop (MOTOR_BLOCK *mtr_blk);                       // L6206 relic
uint16_t  L6230_ReadStatusRegister (MOTOR_BLOCK *mtr_blk);        // L6206 relic
uint16_t  L6230_ReadId (void);                                    // L6206 relic
void  L6230_ResetAllDevices (void);                               // L6206 relic
void  L6230_Reset (MOTOR_BLOCK *mtr_blk);
void  L6230_ReleaseReset (MOTOR_BLOCK *mtr_blk);
void  L6230_SetDirection (MOTOR_BLOCK *mtr_blk, int dir);
bool  L6230_SetNumMotors (uint8_t num_motors);
bool  L6230_SoftStop (MOTOR_BLOCK *mtr_blk);
bool  L6230_SetNumMotors (uint8_t num_motors);

            // NEED EQUIVALENTS OF THESE in sep BOARD file, similar to what did for L6474
            // these take the place of HAL_xxxx.c file for each diff MCU
int L6230_Board_GpioInit (int mtr_id);                            // L6206 relic
int L6230_Board_SpiInit (void);                                   // L6206 relic
int L6230_Board_PwmInit (int mtr_id);                             // L6206 relic
int L6230_Board_Reset (int mtr_id);                               // L6206 relic
int L6230_Board_ReleaseReset (int mtr_id);                        // L6464 relic
int L6230_Board_PwmStop (int mtr_id);                             // L6206 relic
int L6230_Board_SetDirectionGpio (int mtr_id, int dir);  // ??? does this make sense with BLDC ???
int L6230_Board_SpiWriteBytes (char *xmit_buf, char *rcv_buf, int numberOfDevices);

#define MAX_NUMBER_OF_DEVICES      4
#define L6230_FW_VERSION         101

                         //    =====   TEMP   HACK    =======

           //-----------------------------------------------------------
           //-----------------------------------------------------------
           //             Motor Contoller Chip Function Table
           //
           //  Defines low-level command handlers that are invoked by
           //  BLDC_motor_ctl_lib.c to perform device specific functions
           //-----------------------------------------------------------
           //-----------------------------------------------------------
int  l6230_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr);
int  l6230_motor_adc_next_conversion (void);
int  l6230_motor_adc_ctl_init (void);
int  l6230_motor_encoder_gpio_init(MOTOR_BLOCK *mtr_blk, int encoder_type);
int  l6230_motor_init_ctl_gpio_outputs (void);
int  l6230_motor_init_ctl_gpio_inputs (void);
int  l6230_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long pwm_ticks);

  MOTOR_HANDLERS * L6230_GetMotorHandle(void);   // return addr of drv6474_table

  MOTOR_HANDLERS  drv6230_table = {
                        TYPE_MOTOR_STEPPER,           // Motor Type
                        MOTOR_DRIVER,                 // Chip number / id
                        0L,                           // SPI init
                        0L,                           // SPI I/O
                        l6230_motor_speed_adc_init,   // init routine for speed ctl ADC
                        l6230_motor_adc_next_conversion, // ADC conversion start rtn for speed
                        0L,                           // motor_read_hall_sensors_handler
                        0L,                           // motor_fault_shutdown
                        l6230_motor_adc_ctl_init,     // control ADCs (Current sense)
                        l6230_motor_init_controller_chip, // motor_init_controller_chip
                        0L,                               // motor_init_commute_time
                        l6230_motor_encoder_gpio_init,    // init routine for encoder GPIOs
                        l6230_motor_init_ctl_gpio_inputs, // control input  GPIOs (FAULT)
                        l6230_motor_init_ctl_gpio_outputs,// control output GPIOs (ENABLE)
                        l6230_motor_pwm_init,             // motor PWMs

                        0L,                           // motor_check_status
                        l6230_motor_do_ctl_action,    // motor_do_action_handler
                        0L,                           // motor_do_next_commutate
                        l6230_get_motor_actual_position,// motor_get_actual_position
                        l6230_get_mark,               // motor_get_mark_pos
                        l6230_CmdGetStatus,           // motor_get_motor_status
                        0L,                           // motor_get_period_ticks
                        l6230_perform_motor_move,     // motor_perform_move
                        l6230_motor_run,              // motor_run_handler
                        l6230_motor_set_direction,    // motor_set_direction_handler
                        l6230_motor_set_pwm_duty_count,// motor_set_pwm_duty_count_handler
                        0L,                           // motor_set_speed_rpm_handler
                        l6230_motor_set_step_mode,    // motor_set_step_mode
                        0L,                           // motor_set_torque
                        l6230_motor_stop,             // motor_stop_handler
                        l6230_WaitWhileActive         // motor_wait_complete
                      };


/*******************************************************************************
  * @file    l6474.c
  * @author  IPC Rennes
  * @version V1.6.0
  * @date    February 03, 2016
  * @brief   L6230 driver (fully integrated microstepping motor driver)
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
*     06/17/16 - Fix situation where L6230 may report a bogus ABS_POS value
*                immeidately after reset. Always force a logical home.
*******************************************************************************/

//#include "l6474.h"


/* Private constants ---------------------------------------------------------*/

                         /// Error while initialising the SPI
#define L6230_ERROR_0   (0x8000)
                         /// Error: Bad SPI transaction
#define L6230_ERROR_1   (0x8001)

                         /// Maximum number of steps
#define MAX_STEPS       (0x7FFFFFFF)


/* Private variables ---------------------------------------------------------*/

/** @defgroup L6230_Private_Variables L6230 Private Variables
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

static uint8_t  spiTxBursts[6][3];   // are thses used in L6230 logic or is it relic from L6206 code ?
static uint8_t  spiRxBursts[6][3];

static volatile bool    spiPreemtionByIsr   = FALSE;
//static volatile bool    isrFlag             = FALSE;
static uint16_t         l6474DriverInstance = 0;

                      //------------------------------------
                      // L6230 Motor Blocks - 1 per motor
                      //------------------------------------
extern  MOTOR_BLOCK  motor_blk [];           // is defined in MotorLib_Basic.c


/* Private function prototypes -----------------------------------------------*/

/** @defgroup L6230_Private_functions L6230 Private functions
  * @{
  */
#if defined(REENABLE_LATER)
void L6230_ApplySpeed(MOTOR_BLOCK *mtr_blk, uint8_t pwmId, uint16_t newSpeed);
void L6230_ComputeSpeedProfile(MOTOR_BLOCK *mtr_blk, uint32_t nbSteps);
int32_t L6230_ConvertPosition(uint32_t abs_position_reg);
uint8_t L6230_Convert_Ocd_Thresh_to_RegValue(float Tval);
float   L6230_Convert_Ocd_RegValue_to_Threshold(uint8_t Par);
uint8_t L6230_Convert_Tval_Current_to_RegValue(float Tval);
uint8_t L6230_Convert_Tmin_Time_to_RegValue(float Tmin);
float   L6230_Convert_RegValue_to_Tmin_Time(uint8_t Par);
float   L6230_Convert_Tval_RegValue_to_Current(uint8_t Par);
void L6230_ErrorHandler(uint16_t error);
void L6230_FlagInterruptHandler(void);
void L6230_SendCommand(MOTOR_BLOCK *mtr_blk, uint8_t param);
void L6230_SetRegisterToGivenValues(MOTOR_BLOCK *mtr_blk, L6230_Init_t *pInitPrm);
void L6230_SetRegisterToPredefinedValues(MOTOR_BLOCK *mtr_blk);
void L6230_Write_SPI_Bytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte);
void L6230_SetDeviceParamsToPredefinedValues(MOTOR_BLOCK *mtr_blk);
void L6230_SetDeviceParamsToGivenValues(MOTOR_BLOCK *mtr_blk, L6230_Init_t *pInitPrm);
//void L6230_StartMovement(MOTOR_BLOCK *mtr_blk);
void l6230_StepClockHandler(uint8_t motor_id);
#endif                         // REENABLE_LATER

/** @defgroup L6230_Exported_Variables L6230 Exported Variables
  * @{
  */

#if defined(MUST_BE_REPLACED_BY_MINE_CAUSE_MANY_ROUTINES_NOW_GONE)
                         //--------------------------------------------------
                         //  L6230 motor driver functions pointer structure
                         //--------------------------------------------------
motorDrv_t   drv6474_table_ORIG =
{
  l6230_get_motor_actual_position,      // motor_get_actual_position
  l6230_motor_engage,                   // motor_motor_engage_handler
  l6230_perform_motor_move,             // motor_perform_move
  l6230_motor_set_step_mode,            // motor_set_mode
  l6230_motor_stop,                     // motor_stop
  l6230_get_mark,                       // motor_get_mark_pos

  l6230_Init,                  //void (*Init)(void*);
  l6230_ReadId,                //uint16_t (*ReadID)(void);
  l6230_AttachErrorHandler,    //void (*AttachErrorHandler)(void (*callback)(uint16_t));
  l6230_AttachFlagInterrupt,   //void (*AttachFlagInterrupt)(void (*callback)(void));
  0,                           //void (*AttachBusyInterrupt)(void (*callback)(void));
  l6230_FlagInterruptHandler,  //void (*FlagInterruptHandler)(void);
  l6230_GetAcceleration,       //uint16_t (*GetAcceleration)(uint8_t);
  l6230_GetCurrentSpeed,       //uint16_t (*GetCurrentSpeed)(uint8_t);
  l6230_GetDeceleration,       //uint16_t (*GetDeceleration)(uint8_t);
  l6230_GetDeviceState,        //motorState_t(*GetDeviceState)(uint8_t);
  l6230_GetFwVersion,          //uint8_t (*GetFwVersion)(void);
  l6230_get_mark,              //int32_t (*GetMark)(uint8_t);
  l6230_GetMaxSpeed,           //uint16_t (*GetMaxSpeed)(uint8_t);
  l6230_GetMinSpeed,           //uint16_t (*GetMinSpeed)(uint8_t);
  l6230_GetPosition,           //int32_t (*GetPosition)(uint8_t);
  0L,  // L6230_GoHome,                //void (*GoHome)(uint8_t);
  0L,  // L6230_GoMark,                //void (*GoMark)(uint8_t);
  0L,  // L6230_GoTo,                  //void (*GoTo)(uint8_t, int32_t);
  l6230_HardStop,              //void (*HardStop)(uint8_t);
  0L,    // L6230_Move,                  //void (*Move)(uint8_t, DIRECTION_t, uint32_t);
  L6230_ResetAllDevices,       //void (*ResetAllDevices)(void);
  0L,    // L6230_Run,                   //void (*Run)(uint8_t, DIRECTION_t);
  L6230_SetAcceleration,       //bool(*SetAcceleration)(uint8_t ,uint16_t);
  L6230_SetDeceleration,       //bool(*SetDeceleration)(uint8_t , uint16_t);
  0,     // L6230_SetHome,               //void (*SetHome)(uint8_t);
  0,     // L6230_SetMark,               //void (*SetMark)(uint8_t);
  l6230_SetMaxSpeed,           //bool (*SetMaxSpeed)(uint8_t, uint16_t);
  l6230_SetMinSpeed,           //bool (*SetMinSpeed)(uint8_t, uint16_t);
  l6230_SoftStop,              //bool (*SoftStop)(uint8_t);
  l6230_StepClockHandler,      //void (*StepClockHandler)(uint8_t motor_id);
  l6230_WaitWhileActive,       //void (*WaitWhileActive)(uint8_t);
  l6230_CmdDisable,            //void (*CmdDisable)(uint8_t);
  l6230_CmdEnable,             //void (*CmdEnable)(uint8_t);
  l6230_CmdGetParam,           //uint32_t (*CmdGetParam)(uint8_t, uint32_t);
  l6230_CmdGetStatus,          //uint16_t (*CmdGetStatus)(uint8_t);
  l6230_CmdNop,                //void (*CmdNop)(uint8_t);
  l6230_CmdSetParam,           //void (*CmdSetParam)(uint8_t, uint32_t, uint32_t);
  l6230_ReadStatusRegister,    //uint16_t (*ReadStatusRegister)(uint8_t);
  l6230_ReleaseReset,          //void (*ReleaseReset)(void);
  l6230_Reset,                 //void (*Reset)(void);
  l6230_SelectStepMode,        //void (*SelectStepMode)(uint8_t motor_id, STEPPER_MODE_t);
  l6230_SetDirection,          //void (*SetDirection)(uint8_t, DIRECTION_t);
  0,                           //void (*CmdGoToDir)(uint8_t, DIRECTION_t, int32_t);
  0,                           //uint8_t (*CheckBusyHw)(void);
  0,                           //uint8_t (*CheckStatusHw)(void);
  0,                           //void (*CmdGoUntil)(uint8_t, motorAction_t, DIRECTION_t, uint32_t);
  l6230_HizStop,               //void (*CmdHardHiZ)(uint8_t);
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
  L6230_ErrorHandler,          //void (*ErrorHandler)(uint16_t);
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
  L6230_SetNumMotors,          //bool (*SetNbDevices)(uint8_t);
  L6230_SetAnalogValue,        //bool (*SetAnalogValue)(uint8_t, uint32_t, float);
  L6230_GetAnalogValue         //float (*GetAnalogValue )(uint8_t, uint32_t);
};
#endif



    ADC_HandleTypeDef   hadc1;    // direct lift from L6230.c main.c startup code

    TIM_HandleTypeDef   htim1;
    TIM_HandleTypeDef   htim2;
    TIM_HandleTypeDef   htim3;
    TIM_HandleTypeDef   htim4;

    UART_HandleTypeDef  huart2;   //  end  direct lift


//******************************************************************************
//  l6230_start_motor_move
//
//                                                                    KEY  KEY
//                            Start Movement
//
//   Initialises the bridge parameters to start the movement
//   and enable the power bridge
//******************************************************************************

void  l6230_start_motor_move (MOTOR_BLOCK *mtr_blk, int move_cmd)
{

// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

//  mtr_blk->mtr_move_type = MOVE_CMD;

         // Send ENABLE cmd via SPI to Enable L6230 powerstage
//  L6230_SendCommand (mtr_blk, L6230_ENABLE);                  // L6206 relic

    if (mtr_blk->mtr_accel_end_position != 0)      // Is this redundant w/upper layer ?
       mtr_blk->mtr_motion_state = MOTION_ACCELERATING;
      else mtr_blk->mtr_motion_state = MOTION_DECELERATING;

    mtr_blk->accu        = 0;
    mtr_blk->mtr_new_relative_position = 0;        // clear for new move

                                                              // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
    L6230_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_min_velocity_pc); //   PWMs finally go on at this point - STM32_F4
                                                              // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
}


//******************************************************************************
//  l6230_motor_pwm_init
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

int  l6230_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long period_ticks)
{

    GPIO_InitTypeDef         GPIO_InitStruct;
    TIM_OC_InitTypeDef       sConfigOC;
    TIM_MasterConfigTypeDef  sMasterConfig;
    uint32_t                 channel;

    MX_GPIO_Init();    // direct lift from L6230 main.c startup code
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART2_UART_Init();    // end direct lift


return(0);     // 07/29/16  TEMP DEBUG TEST HACK


     //----------------------------------------------------------------------
     // for now, only worry about Motor 1 (0) PWM  which uses TIM3  Channel 2
     //----------------------------------------------------------------------
     // Other mostors will be in more generalize in my generic STM32 "board support"

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
//  l6230_motor_gpio_init_inputs
//
//         Configure control inputs for Motor GPIOs  - FAULT, ...
//          nFault:  Fault = Active Low  J2-3   PE0
//******************************************************************************

int  l6230_motor_init_ctl_gpio_inputs (void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

return(0);     // 07/29/16  TEMP DEBUG TEST HACK  - if this is called _AFTER_ MX_GPIO_Init() ABOVE, IT COULD BE CLOBBERING THINGS

        // Configure L6230 - FAULT/FLAG pin                        (PA10 / D2)
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
//  l6230_motor_init_ctl_gpio_outputs
//
//         Configure control outputs for Motor GPIOs - ENABLE, as well as SPI
//******************************************************************************
int  l6230_motor_init_ctl_gpio_outputs (void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    int               spimode,    rc;

return(0);     // 07/29/16  TEMP DEBUG TEST HACK  - if this is called _AFTER_ MX_GPIO_Init() ABOVE, IT COULD BE CLOBBERING THINGS

        // Configure L6230 - STBY/RESET pin                        (PA9 / D8)
    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
//  L6230_Board_Reset (0);                      // L6206 relic  NEED EQUIV FOR L6230

           // Configure L6230 - DIR 1 pin for first Motor (0)      (PA8 / D7)
    GPIO_InitStruct.Pin   = GPIO_PIN_8;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);



// SPI is _NOT_ used on L6230  !!!


        //--------------------------------------------------------
        //  setup SPI for access to L6230 - using SPI1   (F4_01)
        //
        // First, configure SPI associated GPIO pins for SPI mode
        //--------------------------------------------------------
    __SPI1_CLK_ENABLE();                            // Enable SPI Clock

    GPIO_InitStruct.Pin       = GPIO_PIN_5;         // setup SCLK  (PA5 / D13)
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_6;         // setup MISO  (PA6 / D12)
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_7;         // setup MOSI  (PA7 / D11)
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_6;         // setup CS    (PB6 / D10)
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;  // standard GPIO pin
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_MEDIUM;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

         //------------------------------------------------------------
         //                      SPI Module Config
         //
         //          Setup HAL info needed for SPI Configuration
         //------------------------------------------------------------
    spimode = 3;                     // L6230 uses SPI Mode 3
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
    rc = HAL_SPI_Init (&_g_hspi1_handle);  // go intialize the SPI module

    __HAL_SPI_ENABLE (&_g_hspi1_handle);   // turn on SPI Enable flag SPI_CR1_SPE in CR1


    return (rc);           // denote was successful
}


//******************************************************************************
//  l6230_motor_encoder_gpio_init                                      ENCODER
//
//         Configure GPIOs for Wheel Encoder pin inputs ...
//              Hall Sensor Left      PA8   Arduino D7    GPIO rupt   Grove D7
//              Hall Sensor Right     PA9   Arduino D8    GPIO rupt   Grove D8
//******************************************************************************
int  l6230_motor_encoder_gpio_init (MOTOR_BLOCK *mtr_blk, int encoder_type)
{

    return (0);           // denote was successful
}

//******************************************************************************
//  l6230_motor_adc_ctl_init
//
//         Configure ADC to read contropl inputs for current sensors, ...
//             ADC-6  PA6   ADC12_IN6   "A" side Current-Sense   Arduino D12
//             ADC-4  PA4   ADC12_IN4   "B" side Current-Sense   Arduino A2
//******************************************************************************
int  l6230_motor_adc_ctl_init (void)
{
      // see l6206 code for details

    return (0);           // denote was successful
}


//******************************************************************************
//  l6230_motor_speed_adc_init                                          SPEED
//
//         Configure ADC to read potentiometer used for speed control setting.
//        -     Speed Ctl Pot ADC    J1-2    PB5  AIN11   Grove J3-7/27 -> J1-2
//******************************************************************************
int  l6230_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr)
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
//  l6230_motor_adc_next_conversion
//
//         start a new ADC conversion for Speed control  (called every 50 ms)
//         Uses A15  on pin  P6.0
//******************************************************************************
int  l6230_motor_adc_next_conversion (void)
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
//  l6230_motor_run
//
//               Run the motor, i.e. issue a start (or re-start after a
//                              direction change or after a stop).
//
//               Because a stop will slam off the PWMs, we re-establish both the
//               Direction and the Duty Cycle.
//******************************************************************************
int  l6230_motor_run (MOTOR_BLOCK *mtr_blk, int direction)
{
    mtr_blk->mtr_state = MOTOR_STATE_RUN;     // denote we are now in RUN mode

//  TBD - => free running Stepper (no counts or distance, or REJCT as NOT_SUPPORTED ?

    return (0);           // denote was successful
}


//******************************************************************************
//  l6230_motor_set_direction
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
int  l6230_motor_set_direction (MOTOR_BLOCK *mtr_blk,  int direction)
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
//  l6230_get_motor_actual_position
//                            gets the absolute position of the motor, from
//                            the contreoller (stepper) or encoder QEI (BLDC)
//******************************************************************************
int32_t l6230_get_motor_actual_position (MOTOR_BLOCK *mtr_blk)
{
    int32_t      raw_position;
    int32_t      abs_position;


// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

//  raw_position = L6230_CmdGetParam (mtr_blk, L6230_ABS_POS);   // L6206 relic

    abs_position = L6230_ConvertPosition (raw_position);

    return (abs_position);
}



//******************************************************************************
//  l6230_motor_set_pwm_duty_count
//
//            Set the duty cycle to be used for a specific motor.
//            Here, duty cycle is explicitly passed as PWM ticks, not as percent.
//            The duty cycle sets the motor's speed. (More juice, more speed)
//
//   Parms:
//      motor_num:   number of the motor to update (1 or 2)
//      pwm_duty:    actual duty cycle PWM count value to be loaded in PWM
//******************************************************************************
int  l6230_motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count)
{
     int  adjusted_duty_count;

     mtr_blk->mtr_pwm_duty_count = pwm_duty_count;  // save actual count (not %)

// TBD - Anythings else to do for a stepper >  Does duty even make sense ?
//           or issue REJECT ?

     return (0);                             // do not apply till motor start

}

//******************************************************************************
//  l6230_motor_do_ctl_action
//                             Issue a command to perform some kind of control
//                       action on the motor (SetHome, SetMark, EnableMotor, ...
//******************************************************************************
int  l6230_motor_do_ctl_action (MOTOR_BLOCK *mtr_blk, int cmd, int flags)
{
    uint32_t   mark;


// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

    switch (cmd)
      {
#if defined(DO_MANAULLY_L6230)
        case CMD_SET_HOME:
                  L6230_CmdSetParam (mtr_blk, L6230_ABS_POS, 0);  // L6206 relic
                  break;

        case CMD_SET_MARK:
                  mark = L6230_CmdGetParam (mtr_blk, L6230_ABS_POS);  // L6206 relic
                  L6230_CmdSetParam (mtr_blk, L6230_MARK, mark);      // L6206 relic
                  break;

        case CMD_ENGAGE_MOTOR:            // Enables the L6230 power stage
                        // Issue Enable command to the L6230 motor.
                        // Turns on the stepper motor, getting it ready to move.
                  L6230_SendCommand (mtr_blk, L6230_ENABLE);    // L6206 relic
                  break;

        case CMD_DISENGAGE_MOTOR:         // disables the L6230 power stage
                        // Issue Disable command to the L6230 motor
                  L6230_SendCommand (mtr_blk, L6230_DISABLE);  // L6206 relic
                  break;
#endif
      }

    return (0);                      // denote worked OK
}


//******************************************************************************
//  l6230_perform_motor_move
//                             Issue a command to start the motor moving
//******************************************************************************
            /* Motor activation */
int  l6230_perform_motor_move (MOTOR_BLOCK *mtr_blk, int cmd, int direction, int32_t count)
{
    int   move_op;

// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

#if defined(DOMANUALLY_L6230)
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
#endif

    l6230_start_motor_move (mtr_blk, move_op);     // start the motor

    return (0);                      // denote worked OK
}


//******************************************************************************
//  l6230_motor_stop                                          motor_stop_handler
//
//                             Stop the motor. The stop_type inidcates the
//                             type of stop (Hard, Soft, Hiz, ...)
//******************************************************************************
int  l6230_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags)
{

// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

#if defined(DO_MANUALLY_L6230)
    switch (stop_type)
      {
        case STOP_HARD_BRAKE:                      // Immediatly stops the motor
                L6230_Board_PwmStop (mtr_blk->mtr_id); // Disable corresponding PWM   // L6206 relic
                break;

        case STOP_SOFT_BRAKE:       // Stops motor by using motor's deceleration (Coast to stop)
                if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
                   mtr_blk->mtr_operation_performed = SOFT_STOP_CMD;
                // the Step Clock Handler ISR will handle this op code and cause Coast shutdown
                break;

        case STOP_HIZ:       // Immediatly stops motor and disables power bridge
                L6230_Board_PwmStop (mtr_blk->mtr_id); // Disable corresponding PWM  // L6206 relic
                             // issue cmd to Disable power stage
                L6230_SendCommand (mtr_blk, L6230_DISABLE);       // L6206 relic
                break;
      }
#endif

    g_CmdGetStatus = l6230_CmdGetStatus (mtr_blk, 0); // clear any L6230 FAULT status (WVD added - for good measure)

       // Clear everytrhing to denote we are in the INACTIVE state
    mtr_blk->mtr_motion_state        = MOTION_NOT_ACTIVE;
//  mtr_blk->mtr_operation_performed = NO_CMD;               ??? FIX RESOLVE
    mtr_blk->mtr_steps_to_move       = MAX_STEPS;

    return (0);                      // denote worked OK
}


//******************************************************************************
//  l6230_motor_init_controller_chip
//******************************************************************************
int  l6230_motor_init_controller_chip (void)
{
      // specific L6230 motor parms are fed in during l6230_motor_init

//  L6230_Init (&_g_L6230_Init_Params);    // initialize the chip (via SPI cmds)  // L6206 relic

// for multiple motors, this will be called twice. Hack for BLDC is just do it once via flag on 1st motor
    if (mc_init_performed == 0)
       {
             // Initialize all configured peripherals - from ST SAMPLE CODE
         MX_GPIO_Init();
         MX_ADC1_Init();
         MX_TIM1_Init();
         MX_TIM3_Init();
         MX_TIM4_Init();
         MX_USART2_UART_Init();

             // initialize 6-Step lib ######  - from ST SAMPLE CODE
         MC_SixStep_INIT();

         mc_init_performed = 1;     // denote we completed the init
       }
}


/******************************************************//**
 *                        Init
 *
 * @brief Starts a new L6230 instance
 * @param[in] pInit pointer to the initialization data
 * @retval None
 **********************************************************/
void  L6230_Init (void *pInit)  // L6206 relic
{
    MOTOR_BLOCK  *mtr_blk;

    mtr_blk = &motor_blk [l6474DriverInstance];    // this use of l6474DriverInstance is really HOKEY !
                                                   // but that is the way ST set it up. It then gets
                                                   // incremented after each L6230_Init() call !
                                                   // But it actually is used as the motor_id by the called routines

       //-------------------------------
       // Initialise additional GPIOs  ???
       //------------------------------
    L6230_Board_GpioInit (mtr_blk->mtr_id);  // L6206 relic

    if (L6230_Board_SpiInit() != 0)  // L6206 relic
      {
        L6230_ErrorHandler (L6230_ERROR_0);   // report Initialization error
      }

        //---------------------------------------------------------------------
        // Initialise the PWMs used for the Step clock for the specified motor
        //---------------------------------------------------------------------
    L6230_Board_PwmInit (mtr_blk->mtr_id);  // L6206 relic

        // Standby-reset deactivation -
        // i.e. turn off RESET and ENABLE the L6230  // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
    L6230_Board_ReleaseReset (mtr_blk->mtr_id);      //  L6464 FAULT light comes ON at this point - STM32_F4
                                                     // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   // L6206 relic
        /* Let a 10 ms delay after reset */
    board_delay_ms (10);  // L6206 relic

        /* Set all registers and context variables to the predefined values from l6230_target_config.h */
    if (pInit == 0)
       {
//        L6230_SetDeviceParamsToPredefinedValues (mtr_blk);
       }
      else
       {
//       L6230_SetDeviceParamsToGivenValues (mtr_blk, pInit);
       }

         /* Disable the L6230 power stage */
//  L6230_SendCommand (mtr_blk, L6230_DISABLE);  // L6206 relic

       //--------------------------------------------------
       // Issue "Get Status" to clear flags after start up
       //--------------------------------------------------  // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
    g_CmdGetStatus = l6230_CmdGetStatus (mtr_blk, 0);       //  L6230 FAULT light goes OFF after this instruction - STM32_F4
                                                         //  NOTE: PWMs are still off at this point on STM32_F4
    l6474DriverInstance++;                               //  g_CmdGetStatus = 0x0000FC03 on STH32_F4 !
                                                         // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    g_CmdGetStatus_2 = l6230_CmdGetStatus (mtr_blk, 0);  // issue back to back to see if it clear FAULT reg as it is supposed to
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
void  L6230_AttachErrorHandler (void (*callback)(uint16_t))  // L6206 relic
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
void  L6230_AttachFlagInterrupt (void (*callback)(void))  // L6206 relic
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
void  L6230_ErrorHandler (uint16_t error)
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
void  L6230_FlagInterruptHandler (void)
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
 *        Causes PWMs and rupts to engage by calling L6230_Board_PwmNSetFreq()
 *
 * @param[in] motor_id (from 0 to 2)
 * @param[in] newSpeed in pps
 * @retval None
 ******************************************************************************/
void  L6230_ApplySpeed (MOTOR_BLOCK *mtr_blk, uint8_t pwmId, uint16_t newSpeed)
{

  if (newSpeed < L6230_MIN_PWM_FREQ)
     {
       newSpeed = L6230_MIN_PWM_FREQ;
     }
  if (newSpeed > L6230_MAX_PWM_FREQ)
     {
       newSpeed = L6230_MAX_PWM_FREQ;
     }

  mtr_blk->mtr_current_velocity_pc = newSpeed;

  switch (mtr_blk->mtr_id)
  {
    case  0:
//     L6230_Board_Pwm1SetFreq (newSpeed);  // L6206 relic    ??? NEED L6230 EQUIV
      break;

    case 1:
//    L6230_Board_Pwm2SetFreq (newSpeed);  // L6206 relic
      break;

    case 2:
//    L6230_Board_Pwm3SetFreq (newSpeed);  // L6206 relic
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
int32_t  L6230_ConvertPosition (uint32_t abs_position_reg)
{
    int32_t  operation_result;

// CAUTION: dumb BLDC - need to inquire QEI encoder instead !!!

#if defined(DO_MANUALLY)
    if (abs_position_reg & L6230_ABS_POS_SIGN_BIT_MASK)     // L6206 relic - need generic
       {
              // Handle a Negative register value
         abs_position_reg = ~abs_position_reg;
         abs_position_reg += 1;

         operation_result = (int32_t) (abs_position_reg & L6230_ABS_POS_VALUE_MASK);   // L6206 relic - needs generic
         operation_result = -operation_result;
       }
      else
       {
         operation_result = (int32_t) abs_position_reg;
       }
#endif

    return (operation_result);
}


/******************************************************//**
 *                        Cmd Get Param
 * @brief  Issues the GetParam command to the L6230 of the specified motor
 * @param[in] motor_id (from 0 to 2)
 * @param[in] param Register adress (L6230_ABS_POS, L6230_MARK,...)
 * @retval Register value
 **********************************************************/
uint32_t  L6230_CmdGetParam (MOTOR_BLOCK *mtr_blk, uint32_t param)   // L6206 relic
{
  uint32_t  i;
  uint8_t   spiIndex;
  uint32_t  spiRxData;
  uint8_t   maxArgumentNbBytes = 0;
  bool      itDisable = FALSE;


// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

#if defined(DO_MANUALLY_L6230)
   spiIndex = numberOfDevices - mtr_blk->mtr_id - 1;
   if (spiIndex > 16)
      spiIndex = 0;          // must have had a wraparound
 do
  {
    spiPreemtionByIsr = FALSE;
    if (itDisable)
      {
           // re-enable L6230_Board_EnableIrq if disable in previous iteration
        L6230_Board_EnableIrq();       // L6206 relic need generic
        itDisable = FALSE;
      }

    for (i = 0; i < numberOfDevices; i++)
      {
        spiTxBursts[0][i] = L6230_NOP;    // initially clear buffer to NOPs
        spiTxBursts[1][i] = L6230_NOP;
        spiTxBursts[2][i] = L6230_NOP;      // L6206 relic  ???
        spiTxBursts[3][i] = L6230_NOP;
        spiRxBursts[1][i] = 0;
        spiRxBursts[2][i] = 0;
        spiRxBursts[3][i] = 0;
      }
    switch (param)
    {
      case L6230_ABS_POS: ;    // L6206 relics
      case L6230_MARK:
        spiTxBursts[0][spiIndex] = ((uint8_t)L6230_GET_PARAM )| (param);
        maxArgumentNbBytes = 3;
        break;

      case L6230_EL_POS: ;     // L6206 relics
      case L6230_CONFIG: ;
      case L6230_STATUS:
        spiTxBursts[1][spiIndex] = ((uint8_t)L6230_GET_PARAM )| (param);
        maxArgumentNbBytes = 2;
        break;

      default:
        spiTxBursts[2][spiIndex] = ((uint8_t)L6230_GET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }

        /* Disable interruption before checking */
        /* pre-emption by ISR and SPI transfers*/
    L6230_Board_DisableIrq();                   // L6206 relic - make generic
    itDisable = TRUE;
  } while (spiPreemtionByIsr); // check pre-emption by ISR

  for (i = L6230_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6230_CMD_ARG_MAX_NB_BYTES;
       i++)
     {
       L6230_Write_SPI_Bytes (&spiTxBursts[i][0],
                              &spiRxBursts[i][0]);
     }

  spiRxData = ((uint32_t)spiRxBursts[1][spiIndex] << 16)|
              (spiRxBursts[2][spiIndex] << 8) |
              (spiRxBursts[3][spiIndex]);

      /* re-enable L6230_Board_EnableIrq after SPI transfers */
  L6230_Board_EnableIrq();      // L6206 relic - make generic
#endif

  return (spiRxData);
}


/*******************************************************************************
 *                      Cmd Get Status
 * @brief  Issues the GetStatus command to the L6230 of the specified motor
 * @param[in] motor_id (from 0 to 2)
 * @retval Status Register value
 * @note Once the GetStatus command is performed, the flags of the status register
 * are reset. This is not the case when the status register is read with the
 * GetParam command (via the functions L6230ReadStatusRegister or L6230CmdGetParam).
 ******************************************************************************/
int32_t  l6230_CmdGetStatus (MOTOR_BLOCK *mtr_blk, int status_type)
{
  uint32_t   i;
  uint16_t   status;
  uint8_t    spiIndex;
  bool       itDisable = FALSE;

   spiIndex = numberOfDevices - mtr_blk->mtr_id - 1;
   if (spiIndex > 16)
      spiIndex = 0;          // must have had a wraparound


// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

#if defined(DO_MANUALLY_L6230)
  do
  {
    spiPreemtionByIsr = FALSE;
    if (itDisable)
       {
           /* re-enable L6230_Board_EnableIrq if disable in previous iteration */
//       L6230_Board_EnableIrq();  // L6206 relic   ???
         itDisable = FALSE;
       }

    for (i = 0; i < numberOfDevices; i++)
      {
        spiTxBursts[0][i] = L6230_NOP;    // initially clear buffer to NOPs
        spiTxBursts[1][i] = L6230_NOP;
        spiTxBursts[2][i] = L6230_NOP;  // L6206 relic
        spiRxBursts[1][i] = 0;
        spiRxBursts[2][i] = 0;
      }
    spiTxBursts[0][spiIndex] = L6230_GET_STATUS;

       /* Disable interruption before checking */
       /* pre-emption by ISR and SPI transfers*/
    L6230_Board_DisableIrq();  // L6206 relic
    itDisable = TRUE;
  } while (spiPreemtionByIsr); // check pre-emption by ISR

  for (i = 0; i < L6230_CMD_ARG_NB_BYTES_GET_STATUS + L6230_RSP_NB_BYTES_GET_STATUS; i++)    // L6206 relic
    {
      L6230_Write_SPI_Bytes (&spiTxBursts[i][0], &spiRxBursts[i][0]);
    }
  status = (spiRxBursts[1][spiIndex] << 8) | (spiRxBursts[2][spiIndex]);

     /* re-enable L6230_Board_EnableIrq after SPI transfers*/
  L6230_Board_EnableIrq();
#endif

  return (status);
}


#if defined(DO_MANUALLY)
/******************************************************//**
 * @brief  Issues the Nop command to the L6230 of the specified motor
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6230_CmdNop (MOTOR_BLOCK *mtr_blk)  // L6206 relic
{
    L6230_SendCommand (mtr_blk, L6230_NOP);  // L6206 relic
}
#endif



// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !
#if defined(DO_MANUALLY)
/******************************************************//**
 *                      Cmd Set Param
 * @brief  Issues the SetParam command to the L6230 of the specified motor
 * @param[in] motor_id (from 0 to 2)
 * @param[in] param Register adress (L6230_ABS_POS, L6230_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 **********************************************************/
void  L6230_CmdSetParam (MOTOR_BLOCK *mtr_blk,  // L6206 relic
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
            // re-enable L6230_Board_EnableIrq if disable in previous iteration
         L6230_Board_EnableIrq();  // L6206 relic
         itDisable = FALSE;
       }
    for (i = 0; i < numberOfDevices; i++)
      {
        spiTxBursts[0][i] = L6230_NOP;    // initially clear buffer to NOPs
        spiTxBursts[1][i] = L6230_NOP;
        spiTxBursts[2][i] = L6230_NOP;
        spiTxBursts[3][i] = L6230_NOP;  // L6206 relic
      }

   switch (param)
   {
    case L6230_ABS_POS: ;  // L6206 relic
    case L6230_MARK:
        spiTxBursts[0][spiIndex] = param;
        spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 3;
        break;

    case L6230_EL_POS: ;  // L6206 relic
    case L6230_CONFIG:
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
    L6230_Board_DisableIrq();  // L6206 relic
    itDisable = TRUE;
  } while (spiPreemtionByIsr); // check pre-emption by ISR

        /* issue SPI transfer */
  for (i = L6230_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6230_CMD_ARG_MAX_NB_BYTES;
       i++)
     {
       L6230_Write_SPI_Bytes (&spiTxBursts[i][0],&spiRxBursts[i][0]);
     }

        /* re-enable L6230_Board_EnableIrq after SPI transfers */
  L6230_Board_EnableIrq();  // L6206 relic
}
#endif


/******************************************************//**
 * @brief Converts mA in compatible values for OCD_TH register
 * @param[in] Tval
 * @retval OCD_TH values
 **********************************************************/
uint8_t  L6230_Convert_Ocd_Thresh_to_RegValue (float Tval)
{
    return ((uint8_t) (((Tval - 375) * 0.002666f) + 0.5f));
}


/******************************************************//**
 * @brief Converts  OCD_TH register values in mA
 * @param[in] Par OCD regiser value
 * @retval mA
 **********************************************************/
inline float  L6230_Convert_Ocd_RegValue_to_Threshold (uint8_t Par)
{
    return (((float) (Par + 1))*375.f);
}

/******************************************************//**
 * @brief Converts mA in compatible values for TVAL register
 * @param[in] Tval
 * @retval TVAL values
 **********************************************************/
inline uint8_t  L6230_Convert_Tval_Current_to_RegValue (float Tval)
{
    return ((uint8_t) (((Tval - 31.25f)*0.032f)+0.5f));
}

/******************************************************//**
 * @brief Converts  TVAL register values in mA
 * @param[in] Par TVAL regiser value
 * @retval mA
 **********************************************************/
inline float  L6230_Convert_Tval_RegValue_to_Current (uint8_t Par)
{
    return (((float) (Par + 1))*31.25f);
}


/******************************************************//**
 * @brief Convert TON/TOFF values in time (us)
 * @param[in] Par Values from TON_MIN/TOFF_MIN
 * @retval time in us
 **********************************************************/
inline float  L6230_Convert_RegValue_to_Tmin_Time (uint8_t Par)
{
    return (((float) (Par + 1)) * 0.5f);
}


/******************************************************//**
 * @brief Convert time in us in compatible values
 * for TON_MIN register
 * @param[in] Tmin
 * @retval TON_MIN values
 **********************************************************/
inline uint8_t  L6230_Convert_Tmin_Time_to_RegValue (float Tmin)
{
    return ((uint8_t) (((Tmin - 0.5f)*2.0f)+0.5f));
}



// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

#if defined(DO_MANUALL)
/******************************************************//**
 *                        Get Analog Value
 * @brief Issues L6230 Get analog values from register
 * L6230_ABS_POS, L6230_MARK, L6230_TVAL, L6230_OCD_TH,
 * L6230_TON_MIN, L6230_TOFF_MIN. The raw register value is
 * converted in alanog values.
 * For other registers, the returned value is the raw value.
 * @param[in] motor_id (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param L6230 register address
 * @retval Register value - 1 to 3 bytes (depends on register)
 *********************************************************/
float  L6230_GetAnalogValue (MOTOR_BLOCK *mtr_blk, uint32_t param)  // L6206 relic
{
    float     value;
    uint32_t  registerValue;

    registerValue = L6230_CmdGetParam (mtr_blk, param);

    switch (param)
     {
      case L6230_ABS_POS:  // L6206 relic
      case L6230_MARK:
        value = (float) L6230_ConvertPosition (registerValue);
        break;

      case L6230_TVAL:  // L6206 relic
        value = L6230_Convert_Tval_RegValue_to_Current (registerValue);
        break;

      case L6230_OCD_TH:  // L6206 relic
        value = L6230_Convert_Ocd_RegValue_to_Threshold (registerValue);
        break;

      case L6230_TON_MIN:  // L6206 relic
      case L6230_TOFF_MIN:
        value = L6230_Convert_RegValue_to_Tmin_Time (registerValue);
        break;

      default:
        value = (float) registerValue;
    }

  return value;
}
#endif


/******************************************************//**
 * @brief Returns the device state
 * @param[in] motor_id (from 0 to 2)
 * @retval State (ACCELERATING, DECELERATING, STEADY or INACTIVE)
 **********************************************************/
int  l6230_GetDeviceState (MOTOR_BLOCK *mtr_blk)
{
    return (mtr_blk->mtr_motion_state);
}


/******************************************************//**
 * @brief Returns the FW version of the library
 * @retval L6230_FW_VERSION
 **********************************************************/
uint8_t  L6230_GetFwVersion (void)  // L6206 relic
{
    return (L6230_FW_VERSION);  // L6206 relic
}


/******************************************************//**
 * @brief  Return motor handle (pointer to the L6230 motor driver structure)
 * @retval Pointer to the motorDrv_t structure
 **********************************************************/
MOTOR_HANDLERS  * L6230_GetMotorHandle (void)
{
    return (&drv6230_table);  // L6206 relic
}


/******************************************************//**
 * @brief  Returns the mark position  of the specified motor
 * @param[in] motor_id (from 0 to 2)
 * @retval Mark register value converted in a 32b signed integer
 **********************************************************/
int32_t  l6230_get_mark (MOTOR_BLOCK *mtr_blk)

{
// CAUTION:  dumb BLDC vs smart STEPPER

//  return L6230_ConvertPosition(L6230_CmdGetParam(mtr_blk,L6230_MARK));  // L6206 relic
}


/******************************************************//**
 *                   Hard Stop
 * @brief  Immediatly stops the motor
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6230_HardStop (MOTOR_BLOCK *mtr_blk)  // L6206 relic
{
       /* Disable corresponding PWM */
    L6230_Board_PwmStop (mtr_blk->mtr_id);  // L6206 relic

    g_CmdGetStatus = l6230_CmdGetStatus (mtr_blk, 0); // clear any L6230 FAULT status

       /* Set inactive state */
    mtr_blk->mtr_motion_state        = MOTION_NOT_ACTIVE;
//   mtr_blk->mtr_operation_performed = NO_CMD;             // L6474 relic
    mtr_blk->mtr_steps_to_move       = MAX_STEPS;
}


/******************************************************//**
 * @brief  Immediatly stops the motor and disable the power bridge
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6230_HizStop (MOTOR_BLOCK *mtr_blk)  // L6206 relic
{
       /* Disable corresponding PWM */
    L6230_Board_PwmStop (mtr_blk->mtr_id);  // L6206 relic

//
#if defined(DO_MANUALLY)
    g_CmdGetStatus = l6230_CmdGetStatus (mtr_blk, 0); // clear any L6230 FAULT status

       /* Disable power stage */
    L6230_SendCommand (mtr_blk, L6230_DISABLE);  // L6206 relic

       /* Set inactive state */
    mtr_blk->mtr_motion_state        = MOTION_NOT_ACTIVE;
    mtr_blk->mtr_operation_performed = NO_CMD;
#endif

    mtr_blk->mtr_steps_to_move       = MAX_STEPS;
}


/******************************************************//**
 *                Read Status Register
 * @brief  Reads the Status Register value
 * @param[in] motor_id (from 0 to 2)
 * @retval Status register valued
 * @note The status register flags are not cleared
 * at the difference with L6230CmdGetStatus()
 **********************************************************/
uint16_t  L6230_ReadStatusRegister (MOTOR_BLOCK *mtr_blk)  // L6206 relic
{
// CAUTION:  dumb BLDC chip vs STEPPER - DO_MANUALLY

//   return (L6230_CmdGetParam(mtr_blk,L6230_STATUS));  // L6206 relic

}


/******************************************************//**
 * @brief Read id
 * @retval Id of the l6474 Driver Instance
 **********************************************************/
uint16_t  L6230_ReadId (void)  // L6206 relic
{
    return (l6474DriverInstance);
}


/******************************************************//**
 * @brief Resets all L6230 devices
 * @retval None
 **********************************************************/
void  L6230_ResetAllDevices (void)  // L6206 relic
{
     uint8_t      loop;
     MOTOR_BLOCK  *mtr_blk;

#if defined(NOT_USED_L6230)
     for (loop = 0; loop < numberOfDevices; loop++)
      {
           /* Stop movement and disable power stage */
        mtr_blk = &motor_blk[loop];
        L6230_HizStop (mtr_blk);
        L6230_Reset (mtr_blk);
        board_delay_ms (10); // Reset pin must be forced low for at least 10 usec
        L6230_Board_ReleaseReset (loop);
        board_delay_ms (10);
      }
#endif
}


/******************************************************//**
 *                     Release Reset
 * @brief  Releases the L6230 reset (pin set to High) of all devices
 * @retval None
 **********************************************************/
void  L6230_ReleaseReset (MOTOR_BLOCK *mtr_blk)
{
    L6230_Board_ReleaseReset (mtr_blk->mtr_id);
}


/******************************************************//**
 * @brief  Resets the L6230 (reset pin set to low) of all devices
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6230_Reset (MOTOR_BLOCK *mtr_blk)
{
    L6230_Board_Reset (mtr_blk->mtr_id);
}


/******************************************************//**
 *                      Select Step Mode               L6230_SelectStepMode
 * @brief  Set the stepping mode
 * @param[in] motor_id (from 0 to 2)
 * @param[in] stepMod from full step to 1/16 microstep as specified in enum STEPPER_MODE_t
 * @retval None
 **********************************************************/
int  l6230_motor_set_step_mode (MOTOR_BLOCK *mtr_blk, int step_mode, int flags)
{
    uint8_t            stepModeRegister;
//  L6230_STEP_SEL_t   l6474StepMod;

// CAUTION dumb BLDC vs smart STEPPER ...

#if defined(DO_MANUALLY)
  switch (step_mode)
  {
    case STEP_MODE_FULL:
      l6474StepMod = L6230_STEP_SEL_1;
      break;

    case STEP_MODE_HALF:
      l6474StepMod = L6230_STEP_SEL_1_2;
      break;

    case STEP_MODE_1_4:
      l6474StepMod = L6230_STEP_SEL_1_4;
      break;

    case STEP_MODE_1_8:
      l6474StepMod = L6230_STEP_SEL_1_8;
      break;

    case STEP_MODE_1_16:
    default:
      l6474StepMod = L6230_STEP_SEL_1_16;
      break;
  }

       /* Deactivate motor */
    L6230_HizStop (mtr_blk);

       /* Read Step mode register and clear STEP_SEL field */
    stepModeRegister = (uint8_t) (0xF8 & L6230_CmdGetParam(mtr_blk,L6230_STEP_MODE)) ;

       /* Apply new step mode */
    L6230_CmdSetParam (mtr_blk, L6230_STEP_MODE, stepModeRegister | (uint8_t) l6474StepMod);

       /* Reset abs pos register */
    L6230_CmdSetParam (mtr_blk, L6230_ABS_POS, 0);
#endif

}


/******************************************************//**
 *                          Send Command
 * @brief  Sends a command without arguments to the L6230 via the SPI
 * @param[in] motor_id (from 0 to 2)
 * @param[in] param Command to send
 * @retval None
 **********************************************************/
void  L6230_SendCommand (MOTOR_BLOCK *mtr_blk, uint8_t param)
{
  uint32_t  i;
  uint8_t   spiIndex;
  bool      itDisable = FALSE;

  spiIndex = numberOfDevices - mtr_blk->mtr_id - 1;
  if (spiIndex > 16)
     spiIndex = 0;          // must have had a wraparound

// CAUTION  dumb BLDC chip vs STEPPER chip

#if defined(DO_MANUALLY)
  do
  {
    spiPreemtionByIsr = FALSE;
    if (itDisable)
    {
          /* re-enable L6230_Board_EnableIrq if disable in previous iteration */
      board_enable_global_interrupts();
      itDisable = FALSE;
    }

    for (i = 0; i < numberOfDevices; i++)
    {
      spiTxBursts[3][i] = L6230_NOP;
    }
    spiTxBursts[3][spiIndex] = param;

       /* Disable interruption before checking */
       /* pre-emption by ISR and SPI transfers*/
    board_disable_global_interrupts();
    itDisable = TRUE;
  } while (spiPreemtionByIsr); // check pre-emption by ISR
#endif

  L6230_Write_SPI_Bytes (&spiTxBursts[3][0], &spiRxBursts[3][0]);

      /* re-enable L6230_Board_EnableIrq after SPI transfers*/
  board_enable_global_interrupts();
}


#if defined(DO_MANUALL)
/******************************************************//**
 *                     Set Analog Value
 *
 * @brief Set registers L6230_ABS_POS, L6230_MARK, L6230_TVAL, L6230_OCD_TH,
 * L6230_TON_MIN, L6230_TOFF_MIN by analog values. These
 * analog values are automatically converted in bit values
 * @param[in] motor_id (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Register adress (L6230_ABS_POS, L6230_EL_POS, L6230_MARK,
 * L6230_TVAL, L6230_TON_MIN, L6230_TOFF_MIN, L6230_OCD_TH)
 * @param[in] value Analog value to convert and set into the register
 * @retval TRUE if param and param is valid, FALSE otherwise
 *********************************************************/
bool  L6230_SetAnalogValue (MOTOR_BLOCK *mtr_blk, uint32_t param, float value)
{
  uint32_t  registerValue;
  bool      result = TRUE;

  if ((value < 0) && (param != L6230_ABS_POS) && (param != L6230_MARK))
  {
    result = FALSE;
  }
  else
  {
    switch (param)
    {
      case L6230_EL_POS:
        if  ((value !=0)&&
            ((value > (L6230_ELPOS_STEP_MASK | L6230_ELPOS_MICROSTEP_MASK))||
             (value < (1<<(7-(L6230_STEP_MODE_STEP_SEL & L6230_CmdGetParam(0,L6230_STEP_MODE)))))))
        {
          result = FALSE;
        }
        else
        {
          registerValue = ((uint32_t) value)& (L6230_ELPOS_STEP_MASK | L6230_ELPOS_MICROSTEP_MASK);
        }
        break;

      case L6230_ABS_POS:
      case L6230_MARK:
        if ((value >= L6230_MIN_POSITION) &&
            (value <= L6230_MAX_POSITION))
        {
          if (value >= 0)
          {
            registerValue = ((uint32_t) value)& L6230_ABS_POS_VALUE_MASK;
          }
          else
          {
            registerValue = L6230_ABS_POS_VALUE_MASK - (((uint32_t) (-value))& L6230_ABS_POS_VALUE_MASK) + 1;
          }
        }
        else
        {
          result = FALSE;
        }
        break;

      case L6230_TVAL:
        if (value > L6230_TVAL_MAX_VALUE)
        {
          result = FALSE;
        }
        else
        {
          registerValue = L6230_Convert_Tval_Current_to_RegValue(value);
        }
        break;

      case L6230_OCD_TH:
        if (value > L6230_OCD_TH_MAX_VALUE)
        {
          result = FALSE;
        }
        else
        {
          registerValue = L6230_Convert_Ocd_Thresh_to_RegValue (value);
        }
        break;

      case L6230_TON_MIN:
      case L6230_TOFF_MIN:
        if (value > L6230_TOFF_TON_MIN_MAX_VALUE)
        {
          result = FALSE;
        }
        else
        {
          registerValue = L6230_Convert_Tmin_Time_to_RegValue(value);
        }
        break;
      default:
        result = FALSE;
     }

    if (result != FALSE)
       {
         L6230_CmdSetParam (mtr_blk, param, registerValue);
       }
   }
  return result;
}
#endif


/******************************************************//**
 *                     Set Direction
 * @brief  Specifies the direction
 * @param[in] motor_id (from 0 to 2)
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is only applied if the device
 * is in INACTIVE state
 * @retval None
 **********************************************************/
void  L6230_SetDirection (MOTOR_BLOCK *mtr_blk, int dir)
{
     L6230_Board_SetDirectionGpio (mtr_blk->mtr_id, dir);
}


// CAUTION  dumb BLDC vs STEPPER chip

#if defined(DO_MANUALLY_L6230)
/******************************************************//**
 *                  Set Device Params To GivenValues
 *
 * @brief  Set the parameters of the device to values of pInitPrm structure
 * @param[in] motor_id (from 0 to 2)
 * @param pInitPrm pointer to a structure containing the initial device parameters
 * @retval None
 **********************************************************/
void  L6230_SetDeviceParamsToGivenValues (MOTOR_BLOCK *mtr_blk, L6230_Init_t *pInitPrm)
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

// L6230_SetRegisterToGivenValues (mtr_blk, pInitPrm);
}


/******************************************************//**
 *             Set Device Params To Predefined Values
 * @brief  Sets the parameters of the device to predefined values
 * from l6230_target_config.h
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6230_SetDeviceParamsToPredefinedValues (MOTOR_BLOCK *mtr_blk)
{
  mtr_blk->mtr_acceleration_pc = L6230_CONF_PARAM_ACC_DEVICE_0;
  mtr_blk->mtr_deceleration_pc = L6230_CONF_PARAM_DEC_DEVICE_0;
  mtr_blk->mtr_max_velocity_pc = L6230_CONF_PARAM_MAX_SPEED_DEVICE_0;
  mtr_blk->mtr_min_velocity_pc = L6230_CONF_PARAM_MIN_SPEED_DEVICE_0;

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

  L6230_SetRegisterToPredefinedValues (mtr_blk);
}
#endif                       // defined(DO_MANUALLY_L6230)




/******************************************************//**
 * @brief  Sets the number of Motors to be used
 * @param[in] nbDevices (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval TRUE if successfull, FALSE if failure, attempt to set a number of
 * devices greater than MAX_NUMBER_OF_DEVICES
 **********************************************************/
bool  L6230_SetNumMotors (uint8_t num_motors)
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


#if defined(NOT_USED_BLDC)
/******************************************************//**
 * @brief  Sets the registers of the L6230 to the given values from pInitPrm
 * @param[in] motor_id (from 0 to 2)
 * @param pInitPrm pointer to a structure containing the initial device parameters
 * @retval None
 **********************************************************/
void  L6230_SetRegisterToGivenValues (MOTOR_BLOCK *mtr_blk, L6230_Init_t *pInitPrm)
{

// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

  L6230_CmdSetParam (mtr_blk,
                     L6230_ABS_POS,
                     0);
  L6230_CmdSetParam (mtr_blk,
                     L6230_EL_POS,
                     0);
  L6230_CmdSetParam (mtr_blk,
                     L6230_MARK,
                     0);
  L6230_SetAnalogValue (mtr_blk,
                        L6230_TVAL,
                        pInitPrm->torque_regulation_current_mA);
  L6230_CmdSetParam (mtr_blk,
                     L6230_T_FAST,
                     (uint8_t) pInitPrm->maximum_fast_decay_time |
                     (uint8_t) pInitPrm->fall_time);
  L6230_SetAnalogValue (mtr_blk,
                        L6230_TON_MIN,
                        pInitPrm->minimum_ON_time_us);
  L6230_SetAnalogValue (mtr_blk,
                        L6230_TOFF_MIN,
                        pInitPrm->minimum_OFF_time_us);
  L6230_SetAnalogValue (mtr_blk,
                        L6230_OCD_TH,
                        pInitPrm->overcurrent_threshold);
  L6230_CmdSetParam (mtr_blk,
                     L6230_STEP_MODE,
                     (uint8_t) pInitPrm->step_selection |
                     (uint8_t) pInitPrm->sync_selection);

  L6230_CmdSetParam (mtr_blk,
                     L6230_ALARM_EN,
                     pInitPrm->alarm);
//                   0);                   // Has no effect. Red FAULT stays on

  L6230_CmdSetParam (mtr_blk,
                     L6230_CONFIG,
                     (uint16_t) pInitPrm->clock |
                     (uint16_t) pInitPrm->torque_regulation_method |
                     (uint16_t) pInitPrm->overcurrent_shutwdown |
                     (uint16_t) pInitPrm->slew_rate |
                     (uint16_t) pInitPrm->target_swicthing_period);

}


/******************************************************//**
 * @brief  Sets the registers of the L6230 to their predefined values
 * from l6230_target_config.h
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6230_SetRegisterToPredefinedValues (MOTOR_BLOCK *mtr_blk)
{


// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

                           // L6230 FAULT light is on at entry STM32_F4
  L6230_CmdSetParam (mtr_blk,
                     L6230_ABS_POS,
                     0);
  L6230_CmdSetParam (mtr_blk,
                     L6230_EL_POS,
                     0);
  L6230_CmdSetParam (mtr_blk,
                     L6230_MARK,
                     0);
  switch (mtr_blk->mtr_id)
  {
    case 0:
      L6230_CmdSetParam (mtr_blk,
                        L6230_TVAL,
                        L6230_Convert_Tval_Current_to_RegValue(L6230_CONF_PARAM_TVAL_DEVICE_0));
      L6230_CmdSetParam (mtr_blk,
                              L6230_T_FAST,
                              (uint8_t)L6230_CONF_PARAM_TOFF_FAST_DEVICE_0 |
                              (uint8_t)L6230_CONF_PARAM_FAST_STEP_DEVICE_0);
      L6230_CmdSetParam (mtr_blk,
                              L6230_TON_MIN,
                              L6230_Convert_Tmin_Time_to_RegValue(L6230_CONF_PARAM_TON_MIN_DEVICE_0)
                                );
      L6230_CmdSetParam (mtr_blk,
                              L6230_TOFF_MIN,
                              L6230_Convert_Tmin_Time_to_RegValue(L6230_CONF_PARAM_TOFF_MIN_DEVICE_0));
      L6230_CmdSetParam (mtr_blk,
                        L6230_OCD_TH,
                        L6230_CONF_PARAM_OCD_TH_DEVICE_0);
      L6230_CmdSetParam (mtr_blk,
                        L6230_STEP_MODE,
                        (uint8_t)L6230_CONF_PARAM_STEP_SEL_DEVICE_0 |
                        (uint8_t)L6230_CONF_PARAM_SYNC_SEL_DEVICE_0);

      L6230_CmdSetParam (mtr_blk,
                        L6230_ALARM_EN,
                        L6230_CONF_PARAM_ALARM_EN_DEVICE_0);   // TEMP TEST HACK
//                      0);                       // Ditto has no effect

      L6230_CmdSetParam (mtr_blk,
                        L6230_CONFIG,
                        (uint16_t)L6230_CONF_PARAM_CLOCK_SETTING_DEVICE_0 |
                        (uint16_t)L6230_CONF_PARAM_TQ_REG_DEVICE_0 |
                        (uint16_t)L6230_CONF_PARAM_OC_SD_DEVICE_0 |
                        (uint16_t)L6230_CONF_PARAM_SR_DEVICE_0 |
                        (uint16_t)L6230_CONF_PARAM_TOFF_DEVICE_0);
      break;

    case 1:
      L6230_CmdSetParam (mtr_blk,
                        L6230_TVAL,
                        L6230_Convert_Tval_Current_to_RegValue (L6230_CONF_PARAM_TVAL_DEVICE_1));
      L6230_CmdSetParam (mtr_blk,
                        L6230_T_FAST,
                        (uint8_t) L6230_CONF_PARAM_TOFF_FAST_DEVICE_1 |
                        (uint8_t) L6230_CONF_PARAM_FAST_STEP_DEVICE_1);
      L6230_CmdSetParam (mtr_blk,
                        L6230_TON_MIN,
                        L6230_Convert_Tmin_Time_to_RegValue (L6230_CONF_PARAM_TON_MIN_DEVICE_1));
      L6230_CmdSetParam (mtr_blk,
                        L6230_TOFF_MIN,
                        L6230_Convert_Tmin_Time_to_RegValue (L6230_CONF_PARAM_TOFF_MIN_DEVICE_1));
      L6230_CmdSetParam (mtr_blk,
                        L6230_OCD_TH,
                        L6230_CONF_PARAM_OCD_TH_DEVICE_1);
      L6230_CmdSetParam (mtr_blk,
                        L6230_STEP_MODE,
                        (uint8_t) L6230_CONF_PARAM_STEP_SEL_DEVICE_1 |
                        (uint8_t) L6230_CONF_PARAM_SYNC_SEL_DEVICE_1);
      L6230_CmdSetParam (mtr_blk,
                        L6230_ALARM_EN,
                        L6230_CONF_PARAM_ALARM_EN_DEVICE_1);
      L6230_CmdSetParam (mtr_blk,
                        L6230_CONFIG,
                        (uint16_t)L6230_CONF_PARAM_CLOCK_SETTING_DEVICE_1 |
                        (uint16_t)L6230_CONF_PARAM_TQ_REG_DEVICE_1 |
                        (uint16_t)L6230_CONF_PARAM_OC_SD_DEVICE_1 |
                        (uint16_t)L6230_CONF_PARAM_SR_DEVICE_1 |
                        (uint16_t)L6230_CONF_PARAM_TOFF_DEVICE_1);
      break;

    case 2:
      L6230_CmdSetParam (mtr_blk,
                        L6230_TVAL,
                        L6230_Convert_Tval_Current_to_RegValue (L6230_CONF_PARAM_TVAL_DEVICE_2));
      L6230_CmdSetParam (mtr_blk,
                        L6230_T_FAST,
                        (uint8_t)L6230_CONF_PARAM_TOFF_FAST_DEVICE_2 |
                        (uint8_t)L6230_CONF_PARAM_FAST_STEP_DEVICE_2);
      L6230_CmdSetParam (mtr_blk,
                        L6230_TON_MIN,
                        L6230_Convert_Tmin_Time_to_RegValue (L6230_CONF_PARAM_TON_MIN_DEVICE_2));
      L6230_CmdSetParam (mtr_blk,
                        L6230_TOFF_MIN,
                        L6230_Convert_Tmin_Time_to_RegValue (L6230_CONF_PARAM_TOFF_MIN_DEVICE_2));
      L6230_CmdSetParam (mtr_blk,
                        L6230_OCD_TH,
                        L6230_CONF_PARAM_OCD_TH_DEVICE_2);
      L6230_CmdSetParam (mtr_blk,
                        L6230_STEP_MODE,
                        (uint8_t)L6230_CONF_PARAM_STEP_SEL_DEVICE_2 |
                        (uint8_t)L6230_CONF_PARAM_SYNC_SEL_DEVICE_2);
      L6230_CmdSetParam (mtr_blk,
                        L6230_ALARM_EN,
                        L6230_CONF_PARAM_ALARM_EN_DEVICE_2);
      L6230_CmdSetParam (mtr_blk,
                        L6230_CONFIG,
                        (uint16_t)L6230_CONF_PARAM_CLOCK_SETTING_DEVICE_2 |
                        (uint16_t)L6230_CONF_PARAM_TQ_REG_DEVICE_2 |
                        (uint16_t)L6230_CONF_PARAM_OC_SD_DEVICE_2 |
                        (uint16_t)L6230_CONF_PARAM_SR_DEVICE_2 |
                        (uint16_t)L6230_CONF_PARAM_TOFF_DEVICE_2);
      break;

    default: ;
  }                            // L6230 FAULT light stll on at exit - STM32_F4
}
#endif                         // defined(NOT_USED_BLDC)


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
void  l6230_StartMovement (MOTOR_BLOCK *mtr_blk, int move_cmd)
{
    mtr_blk->mtr_operation_performed = move_cmd;  // save type of move being executed

// CAUTION:  dumb BLDC chip vs STEPPER - DO_MANUALLY

         // Send ENABLE cmd via SPI to Enable L6230 powerstage
//  L6230_SendCommand (mtr_blk, L6230_ENABLE);

    if (mtr_blk->mtr_accel_end_position != 0)
       mtr_blk->mtr_motion_state = MOTION_ACCELERATING;
       else mtr_blk->mtr_motion_state = MOTION_DECELERATING;

    mtr_blk->accu        = 0;
    mtr_blk->mtr_new_relative_position = 0;
                                                              // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
    L6230_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_min_velocity_pc); //   PWMs finally go on at this point - STM32_F4
                                                              // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
}


/******************************************************//**
 *                      Soft Stop
 * @brief  Stops the motor by using the device deceleration
 * @param[in] motor_id (from 0 to 2)
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is in INACTIVE state.
 **********************************************************/
bool  L6230_SoftStop (MOTOR_BLOCK *mtr_blk)
{
    bool cmdExecuted = FALSE;

    if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
       {
//       mtr_blk->mtr_operation_performed = SOFT_STOP_CMD;   // L6274 legacy
         cmdExecuted = TRUE;
       }
    return (cmdExecuted);
}


/******************************************************//**
 *                     Wait While Active
 * @brief  Locks until the device state becomes Inactive
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
int  l6230_WaitWhileActive (MOTOR_BLOCK *mtr_blk, int wait_type, int flags)
{
        /* Wait while the motor is running */
    while (l6230_GetDeviceState(mtr_blk) !=  MOTION_NOT_ACTIVE)
      ;
}


/******************************************************//**
 *                     L6230_Write_SPI_Bytes
 *
 * @brief  Write and receive a byte via SPI
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte pointer to the received byte
 * @retval None
 **********************************************************/
void  L6230_Write_SPI_Bytes (uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
{
    int    rc;

    rc = L6230_Board_SpiWriteBytes (pByteToTransmit, pReceivedByte, numberOfDevices);

    if (rc != 0)
       {
         L6230_ErrorHandler (L6230_ERROR_1);
       }

    if (isrFlag)
       {
         spiPreemtionByIsr = TRUE;
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

void  l6230_StepClockHandler (uint8_t motor_id)
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

#if defined(DEFINED_IN_L6230_BASIC_C)
      // Increment the relative position that we have moved thus far
  mtr_blk->mtr_new_relative_position++;
  relativePos = mtr_blk->mtr_new_relative_position;


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
                 L6230_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_current_velocity_pc);
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
            L6230_HardStop (mtr_blk);
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
               L6230_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_current_velocity_pc);
             }
        }
      }
      break;

    INACTIVE: // if one last interrupt is left in pipe, this can happen
      inactive_state_rupt++;
    default:
      break;
   }                               //  end  switch
#endif

        /* Clear isr flag */
  isrFlag = FALSE;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
//         The following were directyly lifted from L6230.c code and X-NUCLEO-IHM07M1.c

void EnableInput_CH1_E_CH2_E_CH3_D(void);
void EnableInput_CH1_E_CH2_D_CH3_E(void);
void EnableInput_CH1_D_CH2_E_CH3_E(void);
void DisableInput_CH1_D_CH2_D_CH3_D(void);
void Start_PWM_driving(void);
void Stop_PWM_driving(void);
void HF_TIMx_SetDutyCycle_CH1(uint16_t);
void HF_TIMx_SetDutyCycle_CH2(uint16_t);
void HF_TIMx_SetDutyCycle_CH3(uint16_t);
void Current_Reference_Start(void);
void Current_Reference_Stop(void);
void Current_Reference_Setvalue(uint16_t);

void L6230_ECH1CH2_DCH3_IO_Write(void);
void L6230_ECH1CH3_DCH2_IO_Write(void);
void L6230_ECH2CH3_DCH1_IO_Write(void);
void L6230_DCH1CH2CH3_IO_Write(void);
void L6230_Start_PWM_generation(void);
void L6230_Stop_PWM_generation(void);
void L6230_HFTIM_DC_CH1(uint16_t CCR_value);
void L6230_HFTIM_DC_CH2(uint16_t CCR_value);
void L6230_HFTIM_DC_CH3(uint16_t CCR_value);

void BSP_X_NUCLEO_FAULT_LED_ON(void);
void BSP_X_NUCLEO_FAULT_LED_OFF(void);

void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D (void);
void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E (void);
void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D (void);
void MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E (void);
void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E (void);
void MC_SixStep_Start_PWM_driving (void);
void  MC_SixStep_Stop_PWM_driving (void);




/*******************************************************************************
  * @defgroup L6230MotorDriver    L6230MotorDriver
  *
  * @brief API pointer for L6230
*******************************************************************************/
 extern  L6230_MotorDriver_TypeDef    L6230MotorDriver;

L6230_MotorDriver_TypeDef  L6230MotorDriver =
    {
      EnableInput_CH1_E_CH2_E_CH3_D,
      EnableInput_CH1_E_CH2_D_CH3_E,
      EnableInput_CH1_D_CH2_E_CH3_E,
      DisableInput_CH1_D_CH2_D_CH3_D,
      Start_PWM_driving,
      Stop_PWM_driving,
      HF_TIMx_SetDutyCycle_CH1,
      HF_TIMx_SetDutyCycle_CH2,
      HF_TIMx_SetDutyCycle_CH3,
      Current_Reference_Start,
      Current_Reference_Stop,
      Current_Reference_Setvalue,
    };



/**************************************************************************
  * @defgroup EnableInput_CH1_E_CH2_E_CH3_D    EnableInput_CH1_E_CH2_E_CH3_D
  *
  * @brief  Enable Input channel CH1 and CH2 for L6230
  */

void  EnableInput_CH1_E_CH2_E_CH3_D (void)
{
    L6230_ECH1CH2_DCH3_IO_Write();
}


/**************************************************************************
  * @defgroup EnableInput_CH1_E_CH2_D_CH3_E    EnableInput_CH1_E_CH2_D_CH3_E
  *  @{
  * @brief  Enable Input channel CH1 and CH3 for L6230
*/

void  EnableInput_CH1_E_CH2_D_CH3_E (void)
{
    L6230_ECH1CH3_DCH2_IO_Write();
}


/**************************************************************************
  * @defgroup EnableInput_CH1_D_CH2_E_CH3_E    EnableInput_CH1_D_CH2_E_CH3_E
  *
  * @brief  Enable Input channel CH2 and CH3 for L6230
*/

void  EnableInput_CH1_D_CH2_E_CH3_E (void)
{
    L6230_ECH2CH3_DCH1_IO_Write();
}


/**************************************************************************
  * @defgroup DisableInput_CH1_D_CH2_D_CH3_D    DisableInput_CH1_D_CH2_D_CH3_D
  *
  * @brief  Enable Input channel CH2 and CH3 for L6230
*/

void  DisableInput_CH1_D_CH2_D_CH3_D (void)
{
    L6230_DCH1CH2CH3_IO_Write();
}


/**************************************************************************
  * @defgroup Start_PWM_driving    Start_PWM_driving
  *
  * @brief  Enable PWM channels for L6230
  *         Periodically kicked by LF timer started by Start_Motor
*/

void  Start_PWM_driving (void)
{
    L6230_Start_PWM_generation();
}

void  MC_SixStep_Start_PWM_driving (void)
{
    L6230MotorDriver.Start_PWM_driving();
}


/**************************************************************************
  * @defgroup Stop_PWM_driving    Stop_PWM_driving
  *
  * @brief  Disable PWM channels for L6230
*/

void  Stop_PWM_driving (void)
{
    L6230_Stop_PWM_generation();
}

void  MC_SixStep_Stop_PWM_driving (void)
{
    L6230MotorDriver.Stop_PWM_driving();
}


/**************************************************************************
  * @defgroup HF_TIMx_SetDutyCycle_CH1    HF_TIMx_SetDutyCycle_CH1
  *
  * @brief  Set the Duty Cycle value for CH1
*/

void  HF_TIMx_SetDutyCycle_CH1 (uint16_t CCR_value)
{
    L6230_HFTIM_DC_CH1(CCR_value);
}



/**************************************************************************
  * @defgroup HF_TIMx_SetDutyCycle_CH2    HF_TIMx_SetDutyCycle_CH2
  *
  * @brief  Set the Duty Cycle value for CH2
*/

void   HF_TIMx_SetDutyCycle_CH2 (uint16_t CCR_value)
{
    L6230_HFTIM_DC_CH2(CCR_value);
}



/**************************************************************************
  * @defgroup HF_TIMx_SetDutyCycle_CH3    HF_TIMx_SetDutyCycle_CH3
  *
  * @brief  Set the Duty Cycle value for CH3
  * @retval None
*/

void   HF_TIMx_SetDutyCycle_CH3 (uint16_t CCR_value)        // WVD - GOES THRU HERE
{
    L6230_HFTIM_DC_CH3 (CCR_value);
}


/**************************************************************************
  * @defgroup Current_Reference_Start    Current_Reference_Start
  *
  * @brief  Enable the Current Reference generation
*/

void  Current_Reference_Start (void)             // WVD - GOES THRU HERE
{
    START_Ref_Generation();
}


/***************************************************************************
  *                                                   BOARD SPECIFIC - F4_01
  * @brief  Start the current generation
  * @retval None
*/
void  START_Ref_Generation()                        // WVD - GOES THRU HERE
{
     DACx.Instance->CCR1 = 0;
     HAL_TIM_PWM_Start (&DACx, HF_TIMx_CH1);
}


/***************************************************************************
  *                                                   BOARD SPECIFIC - F4_01
  * @brief  Stop the current generation
  * @retval None
*/
void  STOP_Ref_Generation()
{
     DACx.Instance->CCR1 = 0;
     HAL_TIM_PWM_Stop (&DACx, HF_TIMx_CH1);
}


/***************************************************************************
  *                                                   BOARD SPECIFIC - F4_01
  * @brief  Set the current generation value
  * @param  Iref Current reference value
  * @retval None
*/
  void  Set_Ref_Generation (uint16_t Iref)          // WVD - GOES THRU HERE
  {
      DACx.Instance->CCR1 = (uint32_t) ((Iref * DACx.Instance->ARR) / 4096);
  }


/**************************************************************************
  * @defgroup Current_Reference_Stop    Current_Reference_Stop
  *
  * @brief  Disable the Current Reference generation
*/

void  Current_Reference_Stop (void)
{
    STOP_Ref_Generation();
}


/**************************************************************************
  * @defgroup Current_Reference_Setvalue    Current_Reference_Setvalue
  *
  * @brief  Set the value for Current Reference
*/

void  Current_Reference_Setvalue (uint16_t Iref)          // WVD - GOES THRU HERE
{
    Set_Ref_Generation (Iref);
}


           // code from  X-NUCLEO-IHM07M1.c     BEGIN
           //                                                MCU F4_01 DEPENDENT



/*******************************************************************************
  * EnableInput_CH1_E_CH2_E_CH3_D
  *                      Enable Input channel CH1 and CH2 for L6230
  * @retval None
*/

void  MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D (void)
{
   L6230MotorDriver.EnableInput_CH1_E_CH2_E_CH3_D();
}


/*******************************************************************************
  * EnableInput_CH1_E_CH2_D_CH3_E
  *                     Enable Input channel CH1 and CH3 for L6230
  * @retval None
*/

void  MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E (void)
{
  L6230MotorDriver.EnableInput_CH1_E_CH2_D_CH3_E();
}


/*******************************************************************************
  * EnableInput_CH1_D_CH2_E_CH3_E
  *                       Enable Input channel CH2 and CH3 for L6230
  * @retval None
*/

void  MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E (void)
{
  L6230MotorDriver.EnableInput_CH1_D_CH2_E_CH3_E();
}


/*******************************************************************************
  * MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D
  *
  *         Enable Input channel CH2 and CH3 for L6230     BOARD SPECIFIC - F4_01
*/

void  MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D (void)
{
    L6230MotorDriver.DisableInput_CH1_D_CH2_D_CH3_D ();
}



/**************************************************************************
  * @defgroup L6230_ECH1CH2_DCH3_IO_Write    L6230_ECH1CH2_DCH3_IO_Write
  *
  * @brief Enable Input channel CH1 and CH2 for L6230
*/

void  L6230_ECH1CH2_DCH3_IO_Write (void)
{
    HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH1,GPIO_SET);      //EN1 ENABLE
    HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH2,GPIO_SET);      //EN2 DISABLE
    HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH3,GPIO_RESET);    //EN3 ENABLE
}



/**************************************************************************
  * @defgroup L6230_ECH1CH3_DCH2_IO_Write    L6230_ECH1CH3_DCH2_IO_Write
  *
  * @brief Enable Input channel CH1 and CH3 for L6230
*/

void  L6230_ECH1CH3_DCH2_IO_Write (void)
{
    HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH1,GPIO_SET);    // EN1 ENABLE
    HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH2,GPIO_RESET);  // EN2 DISABLE
    HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH3,GPIO_SET);    // EN3 ENABLE
}


/**************************************************************************
  * @defgroup L6230_ECH2CH3_DCH1_IO_Write    L6230_ECH2CH3_DCH1_IO_Write
  *
  * @brief Enable Input channel CH2 and CH3 for L6230
*/

void  L6230_ECH2CH3_DCH1_IO_Write (void)
{
    HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH1,GPIO_RESET);  // EN1 DISABLE
    HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH2,GPIO_SET);    // EN2 ENABLE
    HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH3,GPIO_SET);    // EN3 ENABLE
}


/**************************************************************************
  * @defgroup L6230_DCH1CH2CH3_IO_Write    L6230_DCH1CH2CH3_IO_Write
  *
  * @brief Disable all channels for L6230
*/

void  L6230_DCH1CH2CH3_IO_Write (void)
{
    HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH1,GPIO_RESET);  // EN1 DISABLE
    HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH2,GPIO_RESET);  // EN2 DISABLE
    HAL_GPIO_WritePin (GPIO_PORT_1,GPIO_CH3,GPIO_RESET);  // EN3 DISABLE
}


/**************************************************************************
  * @defgroup L6230_Start_PWM_generation    L6230_Start_PWM_generation
  *
  * @brief  Enable the PWM generation on Input channels for L6230
  *         called via LF TIM rupts started by Start_Motor()
*/

void  L6230_Start_PWM_generation (void)
{
    HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH1);        // TIM1_CH1 ENABLE
    HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH2);        // TIM1_CH2 ENABLE
    HAL_TIM_PWM_Start (&HF_TIMx, HF_TIMx_CH3);        // TIM1_CH3 ENABLE
}


/**************************************************************************
  * @defgroup L6230_Stop_PWM_generation    L6230_Stop_PWM_generation
  *
  * @brief Disable the PWM generation on Input channels for L6230
*/

void  L6230_Stop_PWM_generation (void)
{
    HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH1);         // TIM1_CH1 DISABLE
    HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH2);         // TIM1_CH2 DISABLE
    HAL_TIM_PWM_Stop (&HF_TIMx, HF_TIMx_CH3);         // TIM1_CH3 DISABLE
}


/**************************************************************************
  * @defgroup L6230_HFTIM_DC_CH1    L6230_HFTIM_DC_CH1
  *
  * @brief Set the Duty Cycle value for CH1
*/

void  L6230_HFTIM_DC_CH1 (uint16_t CCRx)
{
    HF_TIMx.Instance->HF_TIMx_CCR1 = CCRx;
}


/**************************************************************************
  * @defgroup L6230_HFTIM_DC_CH2    L6230_HFTIM_DC_CH2
  *
  * @brief Set the Duty Cycle value for CH2
*/

void  L6230_HFTIM_DC_CH2 (uint16_t CCRx)
{
    HF_TIMx.Instance->HF_TIMx_CCR2 = CCRx;
}


/**************************************************************************
  * @defgroup L6230_HFTIM_DC_CH3    L6230_HFTIM_DC_CH3
  *
  * @brief Set the Duty Cycle value for CH3
*/

void  L6230_HFTIM_DC_CH3 (uint16_t CCRx)            // WVD - GOES THRU HERE
{
    HF_TIMx.Instance->HF_TIMx_CCR3 = CCRx;
}


/**************************************************************************
  * @defgroup BSP_X_NUCLEO_FAULT_LED_ON    BSP_X_NUCLEO_FAULT_LED_ON
  *
  * @brief Turns selected LED On.
*/

void  BSP_X_NUCLEO_FAULT_LED_ON (void)
{
    HAL_GPIO_WritePin (GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
}


/**************************************************************************
  * @defgroup BSP_X_NUCLEO_FAULT_LED_OFF    BSP_X_NUCLEO_FAULT_LED_OFF
  *
  * @brief Turns selected LED Off.
*/

void  BSP_X_NUCLEO_FAULT_LED_OFF (void)
{
    HAL_GPIO_WritePin (GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                  end direct lift from L6230.c code and X-NUCLEO-IHM07M1.c





// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
//         The following were directyly lifted from L6230 main.c startup code

//------------------------------------------------------------------------------
//  ADC1 init function
//------------------------------------------------------------------------------
void  MX_ADC1_Init (void)
{
  ADC_ChannelConfTypeDef   sConfig;

     // Configure the global features of the ADC (Clock, Resolution,
     // Data Alignment and number of conversion)
  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution            = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode          = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.EOCSelection          = EOC_SINGLE_CONV;
  HAL_ADC_Init (&hadc1);

     // Configure for the selected ADC regular channel its corresponding rank
     // in the sequencer and its sample time.
  sConfig.Channel      = ADC_CHANNEL_12;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel (&hadc1, &sConfig);
}


//------------------------------------------------------------------------------
//  TIM1 init function
//------------------------------------------------------------------------------
void  MX_TIM1_Init (void)
{
  TIM_SlaveConfigTypeDef          sSlaveConfig;
  TIM_MasterConfigTypeDef         sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef  sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef              sConfigOC;
  TIM_ClearInputConfigTypeDef     sClearInputConfig;

  htim1.Instance           = TIM1;
  htim1.Init.Prescaler     = 0;
  htim1.Init.CounterMode   = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period        = 839;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init (&htim1);

  HAL_TIM_PWM_Init (&htim1);

  sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  HAL_TIM_SlaveConfigSynchronization (&htim1, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization (&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 0;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime (&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 671;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel (&htim1, &sConfigOC, TIM_CHANNEL_3);

  sClearInputConfig.ClearInputSource    = TIM_CLEARINPUTSOURCE_ETR;
  sClearInputConfig.ClearInputPolarity  = TIM_CLEARINPUTPOLARITY_NONINVERTED;
  sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
  sClearInputConfig.ClearInputFilter    = 0;
  HAL_TIM_ConfigOCrefClear (&htim1, &sClearInputConfig, TIM_CHANNEL_1);

  HAL_TIM_ConfigOCrefClear (&htim1, &sClearInputConfig, TIM_CHANNEL_2);

  HAL_TIM_ConfigOCrefClear (&htim1, &sClearInputConfig, TIM_CHANNEL_3);
}



//------------------------------------------------------------------------------
//  TIM3 init function
//------------------------------------------------------------------------------
void  MX_TIM3_Init (void)
{
  TIM_SlaveConfigTypeDef    sSlaveConfig;
  TIM_MasterConfigTypeDef   sMasterConfig;
  TIM_OC_InitTypeDef        sConfigOC;

  htim3.Instance           = TIM3;
  htim3.Init.Prescaler     = 0;
  htim3.Init.CounterMode   = TIM_COUNTERMODE_CENTERALIGNED1;
  htim3.Init.Period        = 839;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init (&htim3);

  HAL_TIM_PWM_Init (&htim3);

  sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  HAL_TIM_SlaveConfigSynchronization (&htim3, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization (&htim3, &sMasterConfig);

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 671;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  HAL_TIM_PWM_ConfigChannel (&htim3, &sConfigOC, TIM_CHANNEL_1);
}


//------------------------------------------------------------------------------
//  TIM4 init function
//------------------------------------------------------------------------------
void  MX_TIM4_Init (void)
{
  TIM_ClockConfigTypeDef   sClockSourceConfig;
  TIM_MasterConfigTypeDef  sMasterConfig;

  htim4.Instance           = TIM4;
  htim4.Init.Prescaler     = 0;
  htim4.Init.CounterMode   = TIM_COUNTERMODE_UP;
  htim4.Init.Period        = 24000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init (&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource (&htim4, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization (&htim4, &sMasterConfig);

}

//------------------------------------------------------------------------------
//  USART2 init function
//------------------------------------------------------------------------------
void  MX_USART2_UART_Init (void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_ODD;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}


//------------------------------------------------------------------------------
// Configure GPIO pins
//------------------------------------------------------------------------------

void  MX_GPIO_Init (void)
{

  GPIO_InitTypeDef   GPIO_InitStruct;

      /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

      /* Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);

      /* Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin   = GPIO_PIN_5;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

      /* Configure GPIO pins : PC4 PC7 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_11
                           | GPIO_PIN_12;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);

      /* Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin   = GPIO_PIN_2;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

      /* EXTI interrupt init */
  HAL_NVIC_SetPriority (EXTI15_10_IRQn, 0, 0);    // setup for USER Button press
  HAL_NVIC_EnableIRQ (EXTI15_10_IRQn);
}


// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                  end direct lift from L6230 main.c startup code



//********************************************************************************
//                         file    l6474.h

#if defined(REDUNDANT_IMPORTS)
///Diretion field of STATUS register
typedef enum {
  L6230_STATUS_DIR_FORWARD = (((uint16_t) 0x0001) << 4),
  L6230_STATUS_DIR_REVERSE = (((uint16_t) 0x0000) << 4)
} L6230_STATUS_DIR_t;


/// L6230 command set
typedef enum {
  L6230_NOP           = ((uint8_t) 0x00),
  L6230_SET_PARAM     = ((uint8_t) 0x00),
  L6230_GET_PARAM     = ((uint8_t) 0x20),
  L6230_ENABLE        = ((uint8_t) 0xB8),
  L6230_DISABLE       = ((uint8_t) 0xA8),
  L6230_GET_STATUS    = ((uint8_t) 0xD0),
  L6230_RESERVED_CMD1 = ((uint8_t) 0xEB),
  L6230_RESERVED_CMD2 = ((uint8_t) 0xF8)
} L6230_Commands_t;
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
            l6230_StepClockHandler (0);
          }
     }
  if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
     {     // process motor 2
       mtr_blk = &motor_blk[1];
       if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
          {
            l6230_StepClockHandler (1);
          }
     }

  if ((htim->Instance == TIM4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3))
     {     // process motor 3
       mtr_blk = &motor_blk[2];
       HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_10);
       if ((mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
         && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET))
          {
            l6230_StepClockHandler (2);
          }
      }
}





                // SPLIT OUT LATER - keep as one for global replaces

//*******1*********2*********3*********4*********5*********6*********7**********
//
//                               MotorLib_Advanced_SixStep.c
//
// Motor Control Libarary - API and high level routines for managing BLDC
//                          motors for (Hall) Sensored Six-Step and
//                          BEMF Sensorless Six-Step trapezoidal processing.
//
//                          Main functions for 6-Step algorithm
//
// History:
//   07/23/16 - Created. Duquaine
//******************************************************************************
//#include "6Step_Lib.h"


/*****************************************************************************
The main function are the following:

  1) MC_SixStep_TABLE(...) -> Set the peripherals (TIMx, GPIO etc.) for each step
  2) MC_SixStep_ARR_step() -> Generate the ARR value for Low Frequency TIM during start-up
  3) MC_SixStep_INIT()     -> Init the main variables for motor driving from MC_SixStep_param.h
  4) MC_SixStep_RESET()    -> Reset all variables used for 6Step control algorithm
  5) MC_SixStep_Ramp_Motor_calc() -> Calculate the acceleration profile step by step for motor during start-up
  6) MC_SixStep_NEXT_step()-> Generate the next step number according with the direction (CW or CCW)
  7) MC_Task_Speed()       -> Speed Loop with PI regulator
  8) MC_Set_Speed(...)     -> Set the new motor speed value
  9) MC_StartMotor()       -> Start the Motor
  10)MC_StopMotor()       -> Stop the Motor
*******************************************************************************/


               //----------------------------------------------------
               //        Critical Data structs used by SixStep logic
               //----------------------------------------------------
    SIXSTEP_Base_InitTypeDef        SIXSTEP_parameters;   /* Main SixStep structure*/
    SIXSTEP_PI_PARAM_InitTypeDef_t  PI_parameters;        /* SixStep PI regulator structure*/

               //----------------------------------------------------
               //        Primary variables used by SixStep logic
               //----------------------------------------------------
    uint16_t  Rotor_poles_pairs;                         /*!<  Number of pole pairs of the motor */
    uint32_t  mech_accel_hz = 0;                         /*!<  Hz -- Mechanical acceleration rate */
    uint32_t  constant_k = 0;                            /*!<  1/3*mech_accel_hz */
    uint32_t  Time_vector_tmp = 0;                       /*!<  Startup variable  */
    uint32_t  Time_vector_prev_tmp = 0 ;                 /*!<  Startup variable  */
    uint32_t  T_single_step = 0;                         /*!<  Startup variable  */
    uint32_t  T_single_step_first_value = 0;             /*!<  Startup variable  */
    int32_t   delta = 0;                                 /*!<  Startup variable  */
    uint16_t  index_array = 1;                           /*!<  Speed filter variable */
    int16_t   speed_tmp_array[FILTER_DEEP];               /*!<  Speed filter variable */
    uint16_t  speed_tmp_buffer[FILTER_DEEP];             /*!<  Potentiometer filter variable */
    uint16_t  HFBuffer[HFBUFFERSIZE];                    /*!<  Buffer for Potentiometer Value Filtering at the High-Frequency ADC conversion */
    uint16_t  HFBufferIndex = 0;                         /*!<  High-Frequency Buffer Index */
    uint8_t   array_completed = FALSE;                   /*!<  Speed filter variable */
    uint8_t   buffer_completed = FALSE;                  /*!<  Potentiometer filter variable */
    uint8_t   UART_FLAG_RECEIVE = FALSE;                 /*!<  UART commmunication flag */
    uint32_t  ARR_LF = 0;                                /*!<  Autoreload LF TIM variable */
    int32_t   Mech_Speed_RPM = 0;                         /*!<  Mechanical motor speed */
    int32_t   El_Speed_Hz = 0;                            /*!<  Electrical motor speed */
    uint16_t  index_adc_chn = 0;                         /*!<  Index of ADC channel selector for measuring */
    uint16_t  index_motor_run = 0;                       /*!<  Tmp variable for DEMO mode */
    uint16_t  test_motor_run = 1;                        /*!<  Tmp variable for DEMO mode */
    uint8_t   Enable_start_button = TRUE;                 /*!<  Start/stop button filter to avoid double command */
    uint16_t  index_ARR_step = 1;
    uint32_t  n_zcr_startup = 0;
    uint16_t  index_startup_motor = 1;
    uint16_t  target_speed = TARGET_SPEED;               /*!<  Target speed for closed loop control */
    uint16_t  shift_n_sqrt = 14;
    uint16_t  cnt_bemf_event = 0;
    uint8_t   startup_bemf_failure = 0;
    uint8_t   speed_fdbk_error = 0;
__IO uint32_t uwTick = 0;                        /*!<  Tick counter - 1msec updated */
    uint8_t   dac_status = DAC_ENABLE;
    uint16_t  index_align = 1;
    int32_t   speed_sum_sp_filt = 0;
    int32_t   speed_sum_pot_filt = 0;
    uint16_t  index_pot_filt = 1;
    int16_t   potent_filtered = 0;
    uint32_t  Tick_cnt = 0;
    uint32_t  counter_ARR_Bemf = 0;
    uint64_t  constant_multiplier_tmp = 0;

               //--------------------------------------------------
               //         Forward refs for routines
               //--------------------------------------------------
int16_t  MC_PI_Controller(SIXSTEP_PI_PARAM_InitTypeDef_t *, int16_t);
uint16_t MC_Potentiometer_filter(uint16_t);
uint64_t MCM_Sqrt(uint64_t );
int32_t  MC_GetElSpeedHz(void);
int32_t  MC_GetMechSpeedRPM(void);
void MC_SixStep_NEXT_step(void);
void MC_Speed_Filter(void);
void MC_SixStep_ARR_step(void);
void MC_SixStep_TABLE(uint8_t);
void MC_SixStep_Speed_Potentiometer(void);
void MC_Set_PI_param(SIXSTEP_PI_PARAM_InitTypeDef_t *);
void MC_Task_Speed(void);
void MC_SixStep_Alignment(void);
void MC_Bemf_Delay(void);
void MC_TIMx_SixStep_timebase(void);
void MC_ADCx_SixStep_Bemf(void);
void MC_SysTick_SixStep_MediumFrequencyTask(void);
void MC_SixStep_Ramp_Motor_calc(void);
void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(void);
void MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(void);
void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(void);
void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D(void);
void MC_SixStep_Start_PWM_driving(void);
void MC_SixStep_Stop_PWM_driving(void);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH1(uint16_t);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH2(uint16_t);
void MC_SixStep_HF_TIMx_SetDutyCycle_CH3(uint16_t);
void MC_SixStep_Current_Reference_Start(void);
void MC_SixStep_Current_Reference_Stop(void);
void MC_SixStep_Current_Reference_Setvalue(uint16_t);
void MC_SixStep_ARR_Bemf(uint8_t);
void MC_UI_INIT(void);
void UART_Set_Value(void);
void UART_Communication_Task(void);
void MC_SixStep_Init_main_data(void);
void CMD_Parser(char* pCommandString);
void MC_SixStep_Speed_Val_target_potentiometer(void);
void MC_SixStep_Nucleo_Init (void);
void MC_SixStep_ADC_Channel (uint32_t adc_ch);
void Bemf_delay_calc (void);
void START_DAC (void);
void STOP_DAC (void);
void SET_DAC_value (uint16_t dac_value);
void  HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc);
void  HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim);
void  HAL_SYSTICK_Callback (void);


/*******************************************************************************
  * @defgroup MC_SixStep_INIT    MC_SixStep_INIT
  *
  * @brief Initialitation function for SixStep library
  * @retval None
*/

void  MC_SixStep_INIT (void)              // WVD - GOES THRU HERE
{
    MC_SixStep_Nucleo_Init();

    SIXSTEP_parameters.HF_TIMx_CCR  = HF_TIMx.Instance->HF_TIMx_CCR1;
    SIXSTEP_parameters.HF_TIMx_ARR  = HF_TIMx.Instance->ARR;
    SIXSTEP_parameters.HF_TIMx_PSC  = HF_TIMx.Instance->PSC;
    SIXSTEP_parameters.LF_TIMx_ARR  = LF_TIMx.Instance->ARR;
    SIXSTEP_parameters.LF_TIMx_PSC  = LF_TIMx.Instance->PSC;

        // MC_SixStep_Current_Reference_Start();  - sets a DAC
    MC_SixStep_Current_Reference_Setvalue (SIXSTEP_parameters.Ireference);

 #ifdef UART_COMM
    SIXSTEP_parameters.Button_ready = FALSE;
    MC_UI_INIT();               /*!<  Start the UART Communication Task*/
 #endif

    MC_SixStep_Init_main_data();

 #ifndef UART_COMM
    SIXSTEP_parameters.Button_ready = TRUE;
 #endif

    MC_SixStep_RESET();
}


/**************************************************************************
    * @defgroup MC_SixStep_RESET    MC_SixStep_RESET
    *
    * @brief Reset all variables used for 6Step control algorithm
    * @retval None     Called as part of INIT logic
  */

void  MC_SixStep_RESET()             // WVD - GOES THRU HERE
{
    SIXSTEP_parameters.CMD                   = TRUE;
    SIXSTEP_parameters.numberofitemArr       = NUMBER_OF_STEPS;
    SIXSTEP_parameters.ADC_BEMF_threshold_UP = BEMF_THRSLD_UP;
    SIXSTEP_parameters.ADC_BEMF_threshold_DOWN = BEMF_THRSLD_DOWN;
    SIXSTEP_parameters.Ireference            = STARTUP_CURRENT_REFERENCE;
    SIXSTEP_parameters.Speed_Loop_Time       = SPEED_LOOP_TIME;
    SIXSTEP_parameters.pulse_value           = SIXSTEP_parameters.HF_TIMx_CCR;
    SIXSTEP_parameters.Speed_target_ramp     = MAX_POT_SPEED;
    SIXSTEP_parameters.ALIGNMENT             = FALSE;
    SIXSTEP_parameters.Speed_Ref_filtered    = 0;
    SIXSTEP_parameters.demagn_value          = INITIAL_DEMAGN_DELAY;

    SIXSTEP_parameters.CurrentRegular_BEMF_ch = 0;
    SIXSTEP_parameters.status_prev           = 0;
    SIXSTEP_parameters.step_position         = 0;

    LF_TIMx.Init.Prescaler = SIXSTEP_parameters.LF_TIMx_PSC;  // setup LF and HF times
    LF_TIMx.Instance->PSC  = SIXSTEP_parameters.LF_TIMx_PSC;
    LF_TIMx.Init.Period    = SIXSTEP_parameters.LF_TIMx_ARR;
    LF_TIMx.Instance->ARR  = SIXSTEP_parameters.LF_TIMx_ARR;
    HF_TIMx.Init.Prescaler = SIXSTEP_parameters.HF_TIMx_PSC;
    HF_TIMx.Instance->PSC  = SIXSTEP_parameters.HF_TIMx_PSC;
    HF_TIMx.Init.Period    = SIXSTEP_parameters.HF_TIMx_ARR;
    HF_TIMx.Instance->ARR  = SIXSTEP_parameters.HF_TIMx_ARR;
    HF_TIMx.Instance->HF_TIMx_CCR1 = SIXSTEP_parameters.HF_TIMx_CCR;

    Rotor_poles_pairs = SIXSTEP_parameters.NUMPOLESPAIRS;
    SIXSTEP_parameters.SYSCLK_frequency = HAL_RCC_GetSysClockFreq();

    MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (0);     // clear all duty cycles to 0
    MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (0);
    MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (0);

    SIXSTEP_parameters.Regular_channel[1] = ADC_Bemf_CH1;   /* BEMF1   */
    SIXSTEP_parameters.Regular_channel[2] = ADC_Bemf_CH2;   /* BEMF2   */
    SIXSTEP_parameters.Regular_channel[3] = ADC_Bemf_CH3;   /* BEMF3   */
    SIXSTEP_parameters.ADC_SEQ_CHANNEL[0] = ADC_CH_1;       /* CURRENT */
    SIXSTEP_parameters.ADC_SEQ_CHANNEL[1] = ADC_CH_2;       /* SPEED   */
    SIXSTEP_parameters.ADC_SEQ_CHANNEL[2] = ADC_CH_3;       /* VBUS    */
    SIXSTEP_parameters.ADC_SEQ_CHANNEL[3] = ADC_CH_4;       /* TEMP    */

    SIXSTEP_parameters.step_position     = 0;
    SIXSTEP_parameters.demagn_counter    = 0;
    SIXSTEP_parameters.ALIGN_OK          = FALSE;
    SIXSTEP_parameters.VALIDATION_OK     = 0;
    SIXSTEP_parameters.ARR_OK            = 0;
    SIXSTEP_parameters.speed_fdbk_filtered = 0;
    SIXSTEP_parameters.Integral_Term_sum = 0;
    SIXSTEP_parameters.Current_Reference = 0;
    SIXSTEP_parameters.Ramp_Start        = 0;
    SIXSTEP_parameters.RUN_Motor         = 0;
    SIXSTEP_parameters.speed_fdbk        = 0;
    SIXSTEP_parameters.BEMF_OK           = FALSE;
    SIXSTEP_parameters.CL_READY          = FALSE;
    SIXSTEP_parameters.SPEED_VALIDATED   = FALSE;
    SIXSTEP_parameters.BEMF_Tdown_count  = 0;   /* Reset of the Counter to detect Stop motor condition when a stall condition occurs*/

    uwTick          = 0;
    index_motor_run = 0;
    test_motor_run  = 1;
    T_single_step   = 0;
    T_single_step_first_value = 0;
    delta           = 0;
    Time_vector_tmp = 0;
    Time_vector_prev_tmp = 0;
    Mech_Speed_RPM  = 0;
    El_Speed_Hz     = 0;
    index_adc_chn   = 0;
    mech_accel_hz   = 0;
    constant_k      = 0;
    ARR_LF          = 0;
    index_array     = 1;
    Enable_start_button = TRUE;
    index_ARR_step  = 1;
    n_zcr_startup   = 0;
    cnt_bemf_event  = 0;
    startup_bemf_failure = 0;
    speed_fdbk_error = 0;

    index_align        = 1;
    speed_sum_sp_filt  = 0;
    speed_sum_pot_filt = 0;
    index_pot_filt     = 1;
    potent_filtered    = 0;
    Tick_cnt           = 0;
    counter_ARR_Bemf   = 0;
    constant_multiplier_tmp = 0;

    HFBufferIndex = 0;
    for (uint16_t i = 0; i < HFBUFFERSIZE;i++)
      {
        HFBuffer[i]=0;
      }

    for (uint16_t i = 0; i < FILTER_DEEP;i++)
      {
        speed_tmp_array[i] = 0;
        speed_tmp_buffer[i]= 0;
      }
    array_completed  = FALSE;
    buffer_completed = FALSE;

    if (PI_parameters.Reference < 0)
        {
          SIXSTEP_parameters.step_position = 1;
        }
     target_speed = TARGET_SPEED;
     MC_Set_PI_param (&PI_parameters);
     MC_SixStep_Current_Reference_Start();      // Startup DAC
     MC_SixStep_Current_Reference_Setvalue (SIXSTEP_parameters.Ireference);
     index_startup_motor = 1;
     MC_SixStep_Ramp_Motor_calc ();
}


/***************************************************************************
  *
  * @brief  Init the STM32 register                   BOARD SPECIFIC - F4_01
  * @retval None
*/
void  MC_SixStep_Nucleo_Init (void)        // WVD - GOES THRU HERE
{
    TIM_ClearInputConfigTypeDef  sClearInputConfig;
    ADC_ChannelConfTypeDef       sConfig;

        /******************** ETR CONFIGURATION *******************P*********/
    sClearInputConfig.ClearInputState     = 1;
    sClearInputConfig.ClearInputSource    = TIM_CLEARINPUTSOURCE_ETR;
    sClearInputConfig.ClearInputPolarity  = TIM_CLEARINPUTPOLARITY_NONINVERTED;
    sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
    sClearInputConfig.ClearInputFilter    = 0;
    HAL_TIM_ConfigOCrefClear (&HF_TIMx, &sClearInputConfig, HF_TIMx_CH1);
    HAL_TIM_ConfigOCrefClear (&HF_TIMx, &sClearInputConfig, HF_TIMx_CH2);
    HAL_TIM_ConfigOCrefClear (&HF_TIMx, &sClearInputConfig, HF_TIMx_CH3);
        /********************************************************************/

    __HAL_FREEZE_TIM1_DBGMCU();                 /* Stop TIM during Breakpoint*/

    __HAL_TIM_ENABLE_IT (&htim1, TIM_IT_BREAK); /* Enable the TIM Break interrupt */

        /**************** REGULAR CHANNELS CONFIGURATION **********************/
    sConfig.Channel      = ADC_CH_1;            /* Current feedabck */
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_CH_1_ST;
    sConfig.Offset       = 0;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);
    sConfig.Channel      = ADC_CH_3;            /* Bus voltage */
    sConfig.SamplingTime = ADC_CH_3_ST;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);
    sConfig.Channel      = ADC_CH_4;            /* Temperature feedback */
    sConfig.SamplingTime = ADC_CH_4_ST;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);
    sConfig.Channel      = ADC_Bemf_CH1;        /* BEMF feedback phase A */
    sConfig.SamplingTime = ADC_Bemf_CH1_ST;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);
    sConfig.Channel      = ADC_Bemf_CH2;        /* BEMF feedback phase B */
    sConfig.SamplingTime = ADC_Bemf_CH2_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel      = ADC_Bemf_CH3;        /* BEMF feedback phase C */
    sConfig.SamplingTime = ADC_Bemf_CH3_ST;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);
    sConfig.Channel      = ADC_CH_2;            /* Potentiometer / Speed Control */
    sConfig.SamplingTime = ADC_CH_2_ST;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);
          /********************************************************************/
}


/***************************************************************************
  *
  * @brief  Select the new ADC Channel                BOARD SPECIFIC - F4_01
  * @param  adc_ch
  * @retval None
*/
void  MC_SixStep_ADC_Channel (uint32_t adc_ch)
{
    __HAL_ADC_DISABLE(&ADCx);
    /* Clear the old SQx bits for the selected rank */
    ADCx.Instance->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, 1);
    /* Set the SQx bits for the selected rank */
    ADCx.Instance->SQR3 |= ADC_SQR3_RK(adc_ch, 1);
    __HAL_ADC_ENABLE(&ADCx);
}


/***************************************************************************
  *
  * @brief  Bemf delay calculation                    BOARD SPECIFIC - F4_01
  * @retval None
*/
void  Bemf_delay_calc (void)
{
if (PI_parameters.Reference>=0)
 {
 if(SIXSTEP_parameters.speed_fdbk_filtered<=12000 && SIXSTEP_parameters.speed_fdbk_filtered>10000)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=10000 && SIXSTEP_parameters.speed_fdbk_filtered>9400)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=9400 && SIXSTEP_parameters.speed_fdbk_filtered>7600)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=7600 && SIXSTEP_parameters.speed_fdbk_filtered>6000)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=6000 && SIXSTEP_parameters.speed_fdbk_filtered>5400)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=5400 && SIXSTEP_parameters.speed_fdbk_filtered>4750)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=4750 && SIXSTEP_parameters.speed_fdbk_filtered>4200)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=4200 && SIXSTEP_parameters.speed_fdbk_filtered>2600)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=2600 && SIXSTEP_parameters.speed_fdbk_filtered>1800)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1800 && SIXSTEP_parameters.speed_fdbk_filtered>1500)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1500 && SIXSTEP_parameters.speed_fdbk_filtered>1300)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1300 && SIXSTEP_parameters.speed_fdbk_filtered>1000)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1000 && SIXSTEP_parameters.speed_fdbk_filtered>500)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
  }
 }
 else
 {
  if(SIXSTEP_parameters.speed_fdbk_filtered>=-12000 && SIXSTEP_parameters.speed_fdbk_filtered<-10000)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_1;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-10000 && SIXSTEP_parameters.speed_fdbk_filtered<-7800)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-7800 && SIXSTEP_parameters.speed_fdbk_filtered<-6400)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-6400 && SIXSTEP_parameters.speed_fdbk_filtered<-5400)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-5400 && SIXSTEP_parameters.speed_fdbk_filtered<-4650)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4650 && SIXSTEP_parameters.speed_fdbk_filtered<-4100)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4100 && SIXSTEP_parameters.speed_fdbk_filtered<-3650)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3650 && SIXSTEP_parameters.speed_fdbk_filtered<-3300)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3300 && SIXSTEP_parameters.speed_fdbk_filtered<-2650)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-2600 && SIXSTEP_parameters.speed_fdbk_filtered<-1800)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1800 && SIXSTEP_parameters.speed_fdbk_filtered<-1500)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1500 && SIXSTEP_parameters.speed_fdbk_filtered<-1300)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1300 && SIXSTEP_parameters.speed_fdbk_filtered<-1000)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1000 && SIXSTEP_parameters.speed_fdbk_filtered<-500)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
  }

 }
}


/***************************************************************************
  *                                                   BOARD SPECIFIC - F4_01
  * @brief  Set the Duty Cycle value for CH1
  * @retval None
*/

void  MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (uint16_t CCR_value)
{
    L6230MotorDriver.HF_TIMx_SetDutyCycle_CH1 (CCR_value);
}


/***************************************************************************
  *                                                   BOARD SPECIFIC - F4_01
  * @brief  Set the Duty Cycle value for CH2
  * @retval None
*/

void  MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (uint16_t CCR_value)
{
    L6230MotorDriver.HF_TIMx_SetDutyCycle_CH2 (CCR_value);
}


/*******************************************************************************
  *                                                   BOARD SPECIFIC - F4_01
  * @brief  Set the Duty Cycle value for CH3
  * @retval None
*/

void  MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (uint16_t CCR_value)  // WVD - GOES THRU HERE
{
    L6230MotorDriver.HF_TIMx_SetDutyCycle_CH3 (CCR_value);
}


/***************************************************************************
  *                                                   BOARD SPECIFIC - F4_01
  * @brief  Enable the Current Reference generation
  * @retval None
*/

void  MC_SixStep_Current_Reference_Start (void)          // WVD - GOES THRU HERE
{
    L6230MotorDriver.Current_Reference_Start();
}


/***************************************************************************
  *                                                   BOARD SPECIFIC - F4_01
  * @brief  Disable the Current Reference generation
  * @retval None
*/

void  MC_SixStep_Current_Reference_Stop (void)
{
  L6230MotorDriver.Current_Reference_Stop();
}


/***************************************************************************
  *                                                   BOARD SPECIFIC - F4_01
  * @brief  Set the value for Current Reference
  * @retval None
*/

void  MC_SixStep_Current_Reference_Setvalue (uint16_t Iref)   // WVD - GOES THRU HERE
{
    L6230MotorDriver.Current_Reference_Setvalue (Iref);
}



/*******************************************************************************
  * @defgroup START_DAC    START_DAC                  BOARD SPECIFIC - F4_01
  *
  *  @brief Start DAC for debug
*/
/**
  * @brief  Start DAC for debug
  * @retval None
*/
void  START_DAC (void)
  {

  }


/***************************************************************************
  *
  * @brief  Stop DAC for debug                        BOARD SPECIFIC - F4_01
  * @retval None
*/
void  STOP_DAC (void)
  {

  }


/***************************************************************************
  * @brief  Set DAC value for debug                   BOARD SPECIFIC - F4_01
  *
  * @param  dac_value: information to plot through DAC
  * @retval None
*/
void  SET_DAC_value (uint16_t dac_value)
  {

  }



/**************************************************************************
  * @defgroup MC_SixStep_Init_main_data    MC_SixStep_Init_main_data
  *
  * @brief Init the main variables for motor driving from MC_SixStep_param.h
  * @retval None
*/

void  MC_SixStep_Init_main_data (void)          // WVD - GOES THRU HERE
{
    SIXSTEP_parameters.Ireference    = STARTUP_CURRENT_REFERENCE;
    SIXSTEP_parameters.NUMPOLESPAIRS = NUM_POLE_PAIRS;
    SIXSTEP_parameters.ACCEL         = ACC;
    SIXSTEP_parameters.KP            = KP_GAIN;
    SIXSTEP_parameters.KI            = KI_GAIN;
    SIXSTEP_parameters.CW_CCW        = DIRECTION;
    SIXSTEP_parameters.Potentiometer = POTENTIOMETER;
}



/**************************************************************************
  * @defgroup MC_TIMx_SixStep_timebase    MC_TIMx_SixStep_timebase
  *
  * @brief Low Frequency Timer Callback - Call the next step and request the filtered speed value
  * @retval None            Invoked by timer rupt started via Start_Motor()
*/

void  MC_TIMx_SixStep_timebase (void)
{
    MC_SixStep_NEXT_step();             /* Change STEP number  */

    if (SIXSTEP_parameters.ARR_OK == 0)
       {
          MC_SixStep_ARR_step();        /* BASE TIMER - ARR modification for STEP frequency changing */
       }

    MC_Speed_Filter();                  /*Calculate SPEED filtered  */
}


/**************************************************************************
  * @defgroup MC_SysTick_SixStep_MediumFrequencyTask    MC_SysTick_SixStep_MediumFrequencyTask
  *
  * @brief Systick Callback - Call the Speed loop
  * @retval None
*/

void  MC_SysTick_SixStep_MediumFrequencyTask (void)
{
    if (SIXSTEP_parameters.ALIGNMENT == TRUE && SIXSTEP_parameters.ALIGN_OK == FALSE)
       {
         MC_SixStep_Alignment();
       }

 #ifdef UART_COMM
    if (UART_FLAG_RECEIVE == TRUE)
       UART_Communication_Task();
 #endif

 #ifdef DEMOMODE
    index_motor_run++;
    if (index_motor_run >= DEMO_START_TIME && test_motor_run == 0)
       {
         MC_StopMotor();
         index_motor_run=0;
         test_motor_run=1;
       }
    if (index_motor_run >= DEMO_STOP_TIME && test_motor_run == 1)
       {
         MC_StartMotor();
         test_motor_run = 0;
         index_motor_run=0;
       }
 #endif

    if (SIXSTEP_parameters.VALIDATION_OK == TRUE && SIXSTEP_parameters.Potentiometer  == TRUE)
       {
         MC_SixStep_Speed_Potentiometer();
       }
          /* Push button delay time to avoid double command */
    if (HAL_GetTick() == BUTTON_DELAY && Enable_start_button != TRUE)
       {
          Enable_start_button = TRUE;
       }

         /* SIXSTEP_parameters.Speed_Loop_Time x 1msec */
     if (Tick_cnt >= SIXSTEP_parameters.Speed_Loop_Time)
        {
          if (SIXSTEP_parameters.STATUS != SPEEDFBKERROR)
             {
                MC_Task_Speed();
             }
          SIXSTEP_parameters.MediumFrequencyTask_flag = TRUE;
          if (SIXSTEP_parameters.VALIDATION_OK == TRUE)
             {
                MC_Set_Speed(0);
             }
          Tick_cnt=0;
        }
       else Tick_cnt++;

    if (startup_bemf_failure == 1)
       {
         SIXSTEP_parameters.ACCEL>>=1;
         if (SIXSTEP_parameters.ACCEL < MINIMUM_ACC)
            {
              SIXSTEP_parameters.ACCEL = MINIMUM_ACC;
            }
         MC_StopMotor();
         cnt_bemf_event = 0;
         SIXSTEP_parameters.STATUS = STARTUP_BEMF_FAILURE;
        }

    if (speed_fdbk_error == 1)
       {
         MC_StopMotor();
         SIXSTEP_parameters.STATUS = SPEEDFBKERROR;
       }
}


/**************************************************************************
  * @defgroup MC_EXT_button_SixStep    MC_EXT_button_SixStep
  *
  * @brief GPIO EXT Callback - Start or Stop the motor through the Blue push button on STM32Nucleo
  * @retval None           Invoked from HAL_GPIO_EXTI_Callback()  ISR CALLBACK
*/

void  MC_EXT_button_SixStep (void)        // WVD - GOES THRU HERE ON BUTTON PUSH
{
    if (Enable_start_button == TRUE)
       {    // Alternating press of button either starts or stops motor
         if (SIXSTEP_parameters.RUN_Motor == 0 && SIXSTEP_parameters.Button_ready == TRUE)
            {
              MC_StartMotor();            // WVD GOES THRU HERE on push
              Enable_start_button = FALSE;
            }
           else
            {
              MC_StopMotor();
              Enable_start_button = FALSE;
            }
      }
}



/**************************************************************************
  *                                ADC
  *
  * @defgroup MC_ADCx_SixStep_Bemf    MC_ADCx_SixStep_Bemf
  *  @{
  * @brief Compute the zero crossing detection
  * @retval None
*/

void  MC_ADCx_SixStep_Bemf (void)
{
  if (__HAL_TIM_DIRECTION_STATUS(&HF_TIMx))
  {
  HAL_GPIO_WritePin(GPIO_PORT_COMM,GPIO_CH_COMM,GPIO_PIN_SET);
     /* UP-counting direction started */

     /* GET the ADC value (PHASE CURRENT)*/
   if (SIXSTEP_parameters.STATUS != START && SIXSTEP_parameters.STATUS != ALIGNMENT)
   {
     switch (SIXSTEP_parameters.step_position)
     {

      case 1:
      if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value)
       {
        SIXSTEP_parameters.ADC_BUFFER[3] = HAL_ADC_GetValue(&ADCx);
       if (PI_parameters.Reference>=0)
        {
        if (SIXSTEP_parameters.ADC_BUFFER[3]< SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
        {
            MC_SixStep_ARR_Bemf(0);
        }
        }
       else
       {
        if (SIXSTEP_parameters.ADC_BUFFER[3]> SIXSTEP_parameters.ADC_BEMF_threshold_UP)
        {
          MC_SixStep_ARR_Bemf(1);
          SIXSTEP_parameters.BEMF_Tdown_count = 0;
        }
       }
       }
       else SIXSTEP_parameters.demagn_counter++;
      break;

      case 2:
       if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value)
       {
        SIXSTEP_parameters.ADC_BUFFER[2] = HAL_ADC_GetValue(&ADCx);
       if (PI_parameters.Reference>=0)
        {
        if (SIXSTEP_parameters.ADC_BUFFER[2]> SIXSTEP_parameters.ADC_BEMF_threshold_UP)
        {
          MC_SixStep_ARR_Bemf(1);
          SIXSTEP_parameters.BEMF_Tdown_count = 0;
        }
       }
       else
       {
        if (SIXSTEP_parameters.ADC_BUFFER[2]< SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
        {
              MC_SixStep_ARR_Bemf(0);
        }
       }
      }
      else SIXSTEP_parameters.demagn_counter++;
      break;

      case 3:
      if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value)
       {
        SIXSTEP_parameters.ADC_BUFFER[1] = HAL_ADC_GetValue(&ADCx);
        if (PI_parameters.Reference>=0)
        {
        if (SIXSTEP_parameters.ADC_BUFFER[1]< SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
        {
            MC_SixStep_ARR_Bemf(0);
        }
        }
        else
        {
         if (SIXSTEP_parameters.ADC_BUFFER[1]> SIXSTEP_parameters.ADC_BEMF_threshold_UP)
            {
              MC_SixStep_ARR_Bemf(1);
              SIXSTEP_parameters.BEMF_Tdown_count = 0;
            }
        }
       }
       else SIXSTEP_parameters.demagn_counter++;
      break;

      case 4:
       if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value)
       {
        SIXSTEP_parameters.ADC_BUFFER[3] = HAL_ADC_GetValue(&ADCx);
       if (PI_parameters.Reference>=0)
        {
        if (SIXSTEP_parameters.ADC_BUFFER[3]> SIXSTEP_parameters.ADC_BEMF_threshold_UP)
        {
          MC_SixStep_ARR_Bemf(1);
          SIXSTEP_parameters.BEMF_Tdown_count = 0;
        }
        }
       else
       {
        if (SIXSTEP_parameters.ADC_BUFFER[3]< SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
        {
            MC_SixStep_ARR_Bemf(0);
        }
       }
       }
       else SIXSTEP_parameters.demagn_counter++;
      break;

      case 5:
       if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value)
       {
        SIXSTEP_parameters.ADC_BUFFER[2] = HAL_ADC_GetValue(&ADCx);
       if (PI_parameters.Reference>=0)
        {
        if (SIXSTEP_parameters.ADC_BUFFER[2]< SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
        {
            MC_SixStep_ARR_Bemf(0);
        }
        }
       else
       {
        if (SIXSTEP_parameters.ADC_BUFFER[2]> SIXSTEP_parameters.ADC_BEMF_threshold_UP)
        {
          MC_SixStep_ARR_Bemf(1);
          SIXSTEP_parameters.BEMF_Tdown_count = 0;
        }
       }
       }
       else SIXSTEP_parameters.demagn_counter++;
      break;

      case 6:
       if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value)
       {
        SIXSTEP_parameters.ADC_BUFFER[1] = HAL_ADC_GetValue(&ADCx);
        if (PI_parameters.Reference>=0)
        {
         if (SIXSTEP_parameters.ADC_BUFFER[1]> SIXSTEP_parameters.ADC_BEMF_threshold_UP)
         {
            MC_SixStep_ARR_Bemf(1);
            SIXSTEP_parameters.BEMF_Tdown_count = 0;
         }
        }
        else
        {
         if (SIXSTEP_parameters.ADC_BUFFER[1]< SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
         {
            MC_SixStep_ARR_Bemf(0);
         }
        }
       }
       else SIXSTEP_parameters.demagn_counter++;
      break;

     }                   //  end  switch()
    }
       /******************* SET ADC CHANNEL FOR SPEED/CURRENT/VBUS *******************/
       /* Set the channel for next ADC Regular reading */
    MC_SixStep_ADC_Channel (SIXSTEP_parameters.ADC_SEQ_CHANNEL[index_adc_chn]);
       /******************************************************************************/
    HAL_GPIO_WritePin(GPIO_PORT_COMM,GPIO_CH_COMM,GPIO_PIN_RESET);
   }

   else
   {
     /* Down-counting direction started */
     /* Set the channel for next ADC Regular reading */

    SIXSTEP_parameters.ADC_Regular_Buffer[index_adc_chn] = HAL_ADC_GetValue(&ADCx);

    if (index_adc_chn == 1)
    {
      HFBuffer[HFBufferIndex++] = HAL_ADC_GetValue(&ADCx);
      if (HFBufferIndex >= HFBUFFERSIZE)
      {
        HFBufferIndex = 0;
      }
    }
    index_adc_chn++;
    if (index_adc_chn>3) index_adc_chn = 0;
     MC_SixStep_ADC_Channel(SIXSTEP_parameters.CurrentRegular_BEMF_ch);
   }

}



/**************************************************************************
  * @defgroup MC_SixStep_ARR_Bemf    MC_SixStep_ARR_Bemf
  *
  * @brief Calculate the new Autoreload value (ARR) for Low Frequency timer
  * @retval None
*/

void  MC_SixStep_ARR_Bemf (uint8_t up_bemf)
{

 if (SIXSTEP_parameters.status_prev != SIXSTEP_parameters.step_position)
    {
      if (SIXSTEP_parameters.SPEED_VALIDATED == TRUE)
         {
           if (GPIO_ZERO_CROSS == 1)
              {
                HAL_GPIO_TogglePin (GPIO_PORT_ZCR, GPIO_CH_ZCR);
              }
           if (cnt_bemf_event> BEMF_CNT_EVENT_MAX)
              {
                startup_bemf_failure = 1;
              }

           if (up_bemf == 1 && SIXSTEP_parameters.BEMF_OK !=TRUE)
              {
                n_zcr_startup++;
                cnt_bemf_event = 0;
              }
             else if (SIXSTEP_parameters.BEMF_OK !=TRUE)
                     {
                       cnt_bemf_event++;
                     }

           if (n_zcr_startup>= NUMBER_ZCR && SIXSTEP_parameters.BEMF_OK !=TRUE )
              {
                SIXSTEP_parameters.BEMF_OK = TRUE;
                n_zcr_startup = 0;
              }
         }

      SIXSTEP_parameters.status_prev = SIXSTEP_parameters.step_position;

      if (SIXSTEP_parameters.VALIDATION_OK == 1)
         {
           counter_ARR_Bemf = __HAL_TIM_GetCounter(&LF_TIMx);
           __HAL_TIM_SetAutoreload(&LF_TIMx,(counter_ARR_Bemf+ARR_LF/2));
         }
    }
}


/**************************************************************************
  * @defgroup MC_Bemf_Delay    MC_Bemf_Delay
  *
  * @brief Take the delay time after each new 6-step commutation
  * @retval None
*/
void  MC_Bemf_Delay (void)
{
   Bemf_delay_calc();
}



/**************************************************************************
  * @defgroup MC_SixStep_Ramp_Motor_calc    MC_SixStep_Ramp_Motor_calc
  *  @{
  * @brief Calculate the acceleration profile step by step for motor during start-up
  * @retval None
*/
void  MC_SixStep_Ramp_Motor_calc (void)                // WVD - GOES THRU HERE
{
  uint32_t constant_multiplier   = 100;
  uint32_t constant_multiplier_2 = 4000000000;

  if (index_startup_motor == 1)
    {
       mech_accel_hz = SIXSTEP_parameters.ACCEL * Rotor_poles_pairs / 60;
       constant_multiplier_tmp = (uint64_t) constant_multiplier * (uint64_t)constant_multiplier_2;
       constant_k    = constant_multiplier_tmp / (3 * mech_accel_hz);
       MC_SixStep_Current_Reference_Setvalue (SIXSTEP_parameters.Ireference);
       Time_vector_prev_tmp = 0;
    }
  if (index_startup_motor < NUMBER_OF_STEPS)
    {
      Time_vector_tmp = ((uint64_t) 1000 * (uint64_t)1000 * (uint64_t) MCM_Sqrt(((uint64_t)index_startup_motor * (uint64_t)constant_k)))/632455;
      delta = Time_vector_tmp - Time_vector_prev_tmp;
      if (index_startup_motor == 1)
         {
           T_single_step_first_value    = (2 * 3141) * delta / 1000;
           SIXSTEP_parameters.ARR_value = (uint32_t) (65535);
         }
        else
         {
           T_single_step = (2 * 3141) * delta/1000;
           SIXSTEP_parameters.ARR_value = (uint32_t)(65535 * T_single_step)/(T_single_step_first_value);
         }
    }
   else index_startup_motor = 1;

  if (index_startup_motor == 1)    // WVD DOES THIS first time
    {
      SIXSTEP_parameters.prescaler_value = (((SIXSTEP_parameters.SYSCLK_frequency/1000000)*T_single_step_first_value)/65535) - 1;
    }
  if (SIXSTEP_parameters.STATUS != ALIGNMENT && SIXSTEP_parameters.STATUS != START)
    {
      index_startup_motor++;       // WVD DOES THIS first tuime
    }
    else Time_vector_tmp = 0;

  Time_vector_prev_tmp =  Time_vector_tmp;
}


/**************************************************************************
  * @defgroup MC_SixStep_ARR_step    MC_SixStep_ARR_step
  *
  * @brief Generate the ARR value for Low Frequency TIM during start-up
  * @retval None
*/

void  MC_SixStep_ARR_step (void)
{

 if (SIXSTEP_parameters.ALIGNMENT == FALSE)
 {
   SIXSTEP_parameters.ALIGNMENT = TRUE;
 }
 if (SIXSTEP_parameters.ALIGN_OK == TRUE)
 {
  if (PI_parameters.Reference >= 0)
   {
     if (SIXSTEP_parameters.VALIDATION_OK != TRUE)
     {
      SIXSTEP_parameters.STATUS = STARTUP;
      MC_SixStep_Ramp_Motor_calc();
      if (index_ARR_step < SIXSTEP_parameters.numberofitemArr)
      {
        LF_TIMx.Init.Period = SIXSTEP_parameters.ARR_value;
        LF_TIMx.Instance->ARR = (uint32_t)LF_TIMx.Init.Period;
        index_ARR_step++;
      }
      else if (SIXSTEP_parameters.ARR_OK == 0)
      {
       index_ARR_step = 1;
       SIXSTEP_parameters.ACCEL>>=1;
       if (SIXSTEP_parameters.ACCEL < MINIMUM_ACC)
       {
         SIXSTEP_parameters.ACCEL = MINIMUM_ACC;
       }
       MC_StopMotor();
       SIXSTEP_parameters.STATUS = STARTUP_FAILURE;
      }
     }
     else
     {
       SIXSTEP_parameters.ARR_OK = 1;
       index_startup_motor = 1;
       index_ARR_step = 1;
     }
   }
  else
    {
     if (SIXSTEP_parameters.VALIDATION_OK != TRUE)
      {
      SIXSTEP_parameters.STATUS = STARTUP;
      MC_SixStep_Ramp_Motor_calc();
      if (index_ARR_step < SIXSTEP_parameters.numberofitemArr)
      {
        LF_TIMx.Init.Period = SIXSTEP_parameters.ARR_value;
        LF_TIMx.Instance->ARR = (uint32_t)LF_TIMx.Init.Period;
        index_ARR_step++;
      }
      else if (SIXSTEP_parameters.ARR_OK==0)
      {
         index_ARR_step = 1;
         SIXSTEP_parameters.ACCEL>>=1;
         if (SIXSTEP_parameters.ACCEL < MINIMUM_ACC)
         {
           SIXSTEP_parameters.ACCEL = MINIMUM_ACC;
         }
         MC_StopMotor();
         SIXSTEP_parameters.STATUS = STARTUP_FAILURE;
      }
      }
      else
       {
         SIXSTEP_parameters.ARR_OK = 1;
         index_startup_motor = 1;
         index_ARR_step = 1;
       }
   }
 }
}


/**************************************************************************
   * @defgroup MC_SixStep_TABLE         MC_SixStep_TABLE
   *
   * @brief  Set the peripherals (TIMx, GPIO etc.) used for each step
   * @param  step_number: step number selected
   * @retval None
***************************************************************************/

void  MC_SixStep_TABLE (uint8_t step_number)
{
 if (GPIO_COMM == 1)
    {
      HAL_GPIO_TogglePin (GPIO_PORT_COMM, GPIO_CH_COMM);
    }

 switch (step_number)
  {
    case 1:
      {
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (0);
          MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D();      // Disable CH3 (used for BEMF)
                // save which channel is being used for BEMF this pass/step
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
      }
     break;
    case 2:
      {
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (0);
          MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E();      // Disable CH2 (used for BEMF)
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
      }
     break;
    case 3:
      {
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (0);
          MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E();      // Disable CH1 (used for BEMF)
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
      }
     break;
    case 4:
      {
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (0);
          MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D();      // Disable CH3 (used for BEMF)
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
      }
     break;
    case 5:
      {
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (0);
          MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E();      // Disable CH2 (used for BEMF)
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
      }
     break;
    case 6:
      {
          MC_SixStep_HF_TIMx_SetDutyCycle_CH3 (SIXSTEP_parameters.pulse_value);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH2 (0);
          MC_SixStep_HF_TIMx_SetDutyCycle_CH1 (0);
          MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E();      // Disable CH1 (used for BEMF)
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
      }
     break;
  }
}


/**************************************************************************
    * @defgroup MC_SixStep_NEXT_step        MC_SixStep_NEXT_step
    *
    * @brief Generate the next step number according with the direction (CW or CCW)
    * @retval uint8_t SIXSTEP_parameters.status
    *      Periodically kicked by LF timer (from Start_Motor)
***************************************************************************/
void  MC_SixStep_NEXT_step()
{
    if ( SIXSTEP_parameters.CMD == TRUE)
       {
         SIXSTEP_parameters.CMD = FALSE;
         MC_SixStep_Start_PWM_driving();
       }

  ARR_LF = __HAL_TIM_GetAutoreload (&LF_TIMx);

  if (SIXSTEP_parameters.ALIGN_OK == TRUE)
    {
      SIXSTEP_parameters.speed_fdbk = MC_GetMechSpeedRPM();
      SIXSTEP_parameters.demagn_counter = 1;
      if (SIXSTEP_parameters.status_prev != SIXSTEP_parameters.step_position)
         {
           n_zcr_startup = 0;
         }
      if (PI_parameters.Reference>=0)
         {
           SIXSTEP_parameters.step_position++;
           if (SIXSTEP_parameters.CL_READY == TRUE)
              {
                SIXSTEP_parameters.VALIDATION_OK = TRUE;
              }
           if (SIXSTEP_parameters.step_position>6)
              {
                SIXSTEP_parameters.step_position = 1;
              }
         }
        else
         {
           SIXSTEP_parameters.step_position--;
           if (SIXSTEP_parameters.CL_READY == TRUE)
              {
                SIXSTEP_parameters.VALIDATION_OK = TRUE;
              }
           if (SIXSTEP_parameters.step_position < 1)
              {
                SIXSTEP_parameters.step_position = 6;
              }
         }
    }

  if (SIXSTEP_parameters.VALIDATION_OK == 1)
    {
        /* Motor Stall condition detection and Speed-Feedback error generation */
     SIXSTEP_parameters.BEMF_Tdown_count++;
     if (SIXSTEP_parameters.BEMF_Tdown_count>BEMF_CONSEC_DOWN_MAX)
        {
          speed_fdbk_error = 1;
        }
       else
        {
          __HAL_TIM_SetAutoreload (&LF_TIMx,0xFFFF);
        }
    }

  MC_SixStep_TABLE (SIXSTEP_parameters.step_position);

      /* This controls if the changing step request appears during DOWNcounting
      *  in this case it changes the ADC channel */

      /* UP-COUNTING direction started  DIR = 0*/
  if (__HAL_TIM_DIRECTION_STATUS(&HF_TIMx))
   {
   switch (SIXSTEP_parameters.step_position)
    {
    case 1:
        SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
        break;

    case 2:
        SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
        break;

    case 3:
        SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
        break;

    case 4:
        SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
        break;

    case 5:
        SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
        break;

    case 6:
        SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
        break;
    }                          /* end switch case*/

     MC_SixStep_ADC_Channel (SIXSTEP_parameters.CurrentRegular_BEMF_ch);
   }

}


/**************************************************************************
  * @defgroup MC_SixStep_Alignment    MC_SixStep_Alignment
  *
  * @brief Generate the motor alignment
  * @retval None
*/

void MC_SixStep_Alignment()
{
   SIXSTEP_parameters.step_position = 6;

   LF_TIMx.Init.Period   = SIXSTEP_parameters.ARR_value;
   LF_TIMx.Instance->ARR = (uint32_t) LF_TIMx.Init.Period;

   SIXSTEP_parameters.STATUS = ALIGNMENT;

   MC_SixStep_Speed_Val_target_potentiometer();
   index_align++;
   if (index_align >= TIME_FOR_ALIGN+1)
     {
       SIXSTEP_parameters.ALIGN_OK = TRUE;
       SIXSTEP_parameters.STATUS   = STARTUP;
       index_startup_motor = 1;
       MC_SixStep_Ramp_Motor_calc();
       LF_TIMx.Init.Prescaler = SIXSTEP_parameters.prescaler_value;
       LF_TIMx.Instance->PSC  = LF_TIMx.Init.Prescaler;
       index_align = 0;
     }
}




/**************************************************************************
  * @defgroup MC_Set_PI_param    MC_Set_PI_param
  *
  * @brief Set all parameters for PI regulator
  @param  PI_PARAM
    * @retval None
*/

void  MC_Set_PI_param (SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM)    // WVD - GOES THRU HERE
{
  if (SIXSTEP_parameters.CW_CCW == 0)
     PI_PARAM->Reference = target_speed;
     else
          PI_PARAM->Reference = -target_speed;

  PI_PARAM->Kp_Gain = SIXSTEP_parameters.KP;
  PI_PARAM->Ki_Gain = SIXSTEP_parameters.KI;

  PI_PARAM->Lower_Limit_Output = LOWER_OUT_LIMIT;
  PI_PARAM->Upper_Limit_Output = UPPER_OUT_LIMIT;
  PI_PARAM->Max_PID_Output     =  FALSE;
  PI_PARAM->Min_PID_Output     =  FALSE;
}


/**************************************************************************
  * @defgroup MC_PI_Controller    MC_PI_Controller
  *
  * @brief Compute the PI output for the Current Reference
  * @param  PI_PARAM PI parameters structure
  * @param  speed_fdb motor_speed_value
  * @retval int16_t Currente reference
*/

int16_t  MC_PI_Controller (SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM, int16_t speed_fdb)
{
  int32_t  wProportional_Term=0, wIntegral_Term=0, wOutput_32=0,wIntegral_sum_temp=0;
  int32_t  Error =0;

  Error = (PI_PARAM->Reference - speed_fdb);

  /* Proportional term computation*/
  wProportional_Term = PI_PARAM->Kp_Gain * Error;

  /* Integral term computation */
  if (PI_PARAM->Ki_Gain == 0)
  {
    SIXSTEP_parameters.Integral_Term_sum = 0;
  }
  else
  {
    wIntegral_Term = PI_PARAM->Ki_Gain * Error;
    wIntegral_sum_temp = SIXSTEP_parameters.Integral_Term_sum + wIntegral_Term;
    SIXSTEP_parameters.Integral_Term_sum = wIntegral_sum_temp;
  }

  if (SIXSTEP_parameters.Integral_Term_sum> KI_DIV * PI_PARAM->Upper_Limit_Output)
     SIXSTEP_parameters.Integral_Term_sum = KI_DIV* PI_PARAM->Upper_Limit_Output;

  if (SIXSTEP_parameters.Integral_Term_sum<-KI_DIV* PI_PARAM->Upper_Limit_Output)
     SIXSTEP_parameters.Integral_Term_sum = -KI_DIV* PI_PARAM->Upper_Limit_Output;

 /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right)
             is used by the compiler to perform the shifts (instead of LSR
             logical shift right)*/

  wOutput_32 = (wProportional_Term/KP_DIV) + (SIXSTEP_parameters.Integral_Term_sum/KI_DIV);

  if (PI_PARAM->Reference>0)
  {
   if (wOutput_32 > PI_PARAM->Upper_Limit_Output)
    {
     wOutput_32 = PI_PARAM->Upper_Limit_Output;
    }
   else if (wOutput_32 < PI_PARAM->Lower_Limit_Output)
    {
     wOutput_32 = PI_PARAM->Lower_Limit_Output;
    }
  }
  else
{
   if (wOutput_32 < (- PI_PARAM->Upper_Limit_Output) )
    {
     wOutput_32 = - (PI_PARAM->Upper_Limit_Output);
    }
   else if (wOutput_32 > (-PI_PARAM->Lower_Limit_Output))
    {
     wOutput_32 = (-PI_PARAM->Lower_Limit_Output);
    }
  }
  return((int16_t)(wOutput_32));
}



/**************************************************************************
  * @defgroup MC_Task_Speed    MC_Task_Speed
  *
  * @brief Main task: Speed Loop with PI regulator
  * @retval None
*/
void  MC_Task_Speed (void)
{

 if (dac_status == TRUE)
   {
     SET_DAC_value (SIXSTEP_parameters.speed_fdbk_filtered);
   }

 if ((SIXSTEP_parameters.speed_fdbk_filtered > (target_speed) || SIXSTEP_parameters.speed_fdbk_filtered < (-target_speed)) && SIXSTEP_parameters.VALIDATION_OK !=TRUE)
   {
     SIXSTEP_parameters.STATUS = VALIDATION;
     SIXSTEP_parameters.SPEED_VALIDATED = TRUE;
   }

 if (SIXSTEP_parameters.SPEED_VALIDATED == TRUE && SIXSTEP_parameters.BEMF_OK == TRUE && SIXSTEP_parameters.CL_READY != TRUE)
   {
     SIXSTEP_parameters.CL_READY = TRUE;
   }

 if (SIXSTEP_parameters.VALIDATION_OK == TRUE)
   {
          /*****************************************************************************/
    SIXSTEP_parameters.STATUS = RUN;
          /*****************************************************************************/

    if (PI_parameters.Reference>=0)
      {
        SIXSTEP_parameters.Current_Reference = (uint16_t) MC_PI_Controller(&PI_parameters,(int16_t)SIXSTEP_parameters.speed_fdbk_filtered);
      }
     else
      {
        SIXSTEP_parameters.Current_Reference = (uint16_t) (-MC_PI_Controller(&PI_parameters,(int16_t)SIXSTEP_parameters.speed_fdbk_filtered));
      }

    MC_SixStep_Current_Reference_Setvalue (SIXSTEP_parameters.Current_Reference);

   }
 MC_Bemf_Delay();
}


/**************************************************************************
  * @defgroup MC_Set_Speed    MC_Set_Speed
  *
  * @brief Set the new motor speed value
  * @param  speed_value:  set new motor speed
  * @retval None
*/
void  MC_Set_Speed (uint16_t speed_value)
{

#if (POTENTIOMETER == 1)
  uint8_t change_target_speed = 0;
  int16_t reference_tmp = 0;

  if (SIXSTEP_parameters.Speed_Ref_filtered > SIXSTEP_parameters.Speed_target_ramp)
  {
    if ((SIXSTEP_parameters.Speed_Ref_filtered - SIXSTEP_parameters.Speed_target_ramp) > ADC_SPEED_TH)
    {
      change_target_speed = 1;
    }
    else
    {
      /* Not change target speed because less than threshold */
    }
  }
  else
  {
    if ((SIXSTEP_parameters.Speed_target_ramp - SIXSTEP_parameters.Speed_Ref_filtered) > ADC_SPEED_TH)
    {
      change_target_speed = 1;
    }
    else
    {
      /* Not change target speed because less than threshold */
    }
  }
  if (change_target_speed == 1)
  {
    SIXSTEP_parameters.Speed_target_ramp = SIXSTEP_parameters.Speed_Ref_filtered;

    if (SIXSTEP_parameters.CW_CCW == 0)
    {
      reference_tmp = SIXSTEP_parameters.Speed_Ref_filtered * MAX_POT_SPEED / 4096;
       if (reference_tmp <= MIN_POT_SPEED)
       {
         PI_parameters.Reference = MIN_POT_SPEED;
       }
       else
       {
         PI_parameters.Reference =  reference_tmp;
       }
    }
    else
    {
      reference_tmp = -(SIXSTEP_parameters.Speed_Ref_filtered * MAX_POT_SPEED / 4096);
       if (reference_tmp >=- MIN_POT_SPEED)
       {
         PI_parameters.Reference = -MIN_POT_SPEED;
       }
       else
       {
         PI_parameters.Reference=  reference_tmp;
       }
    }

  }
#else
   if (speed_value != 0)
    PI_parameters.Reference = speed_value;
#endif

}



/**************************************************************************
  * @defgroup MC_Speed_Filter    MC_Speed_Filter
  *
  * @brief Calculate the speed filtered
  * @retval None
*/

void  MC_Speed_Filter (void)
{
  if (array_completed == FALSE)
  {
     speed_tmp_array[index_array] = SIXSTEP_parameters.speed_fdbk;
     speed_sum_sp_filt = 0;
     for(uint16_t i = 1; i <= index_array;i++)
     {
       speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
     }
     SIXSTEP_parameters.speed_fdbk_filtered = speed_sum_sp_filt/index_array;
     index_array++;

      if (index_array >= FILTER_DEEP)
       {
         index_array = 1;
         array_completed = TRUE;
       }
  }
  else
  {
     index_array++;
     if (index_array >= FILTER_DEEP)
      index_array = 1;

      speed_sum_sp_filt = 0;
      speed_tmp_array[index_array] = SIXSTEP_parameters.speed_fdbk;
      for(uint16_t i = 1; i < FILTER_DEEP;i++)
       {
        speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
       }
      SIXSTEP_parameters.speed_fdbk_filtered = speed_sum_sp_filt/(FILTER_DEEP-1);
  }
}


/**************************************************************************
  * @defgroup MC_SixStep_Speed_Val_target_potentiometer    MC_SixStep_Speed_Val_target_potentiometer
  *
  * @brief Calculate the Motor Speed validation threshold according with the potentiometer value
  * @retval None
*/

void  MC_SixStep_Speed_Val_target_potentiometer (void)
{
  target_speed = SIXSTEP_parameters.ADC_Regular_Buffer[1] * MAX_POT_SPEED/ 4096;

  if (target_speed < MIN_POT_SPEED)
    target_speed = MIN_POT_SPEED;

  if (target_speed > (MAX_POT_SPEED/VAL_POT_SPEED_DIV))
    target_speed = (MAX_POT_SPEED/VAL_POT_SPEED_DIV);
}


/**************************************************************************
  * @defgroup MC_SixStep_Speed_Potentiometer    MC_SixStep_Speed_Potentiometer
  *
  * @brief Calculate the potentiometer value to set the Motor Speed
  * @retval None
*/

void  MC_SixStep_Speed_Potentiometer (void)
{
  uint16_t i=0;
  uint32_t sum = 0;
  uint16_t mean = 0;
  uint16_t max = 0;
  for (i = 0; i < HFBUFFERSIZE; i++)
  {
    uint16_t val = HFBuffer[i];
    sum += val;
    if (val > max)
    {
      max = val;
    }
  }
  sum -= max;
  mean = sum / (HFBUFFERSIZE - 1);

  SIXSTEP_parameters.Speed_Ref_filtered = MC_Potentiometer_filter(mean);

}


/**************************************************************************
  * @defgroup MC_Potentiometer_filter    MC_Potentiometer_filter
  *
  * @brief Calculate the filtered potentiometer value
  * @retval uint16_t Return the filtered potentiometer value
*/

uint16_t  MC_Potentiometer_filter (uint16_t potentiometer_value)
{

#if defined(RAN_OUT_OF_ROOM)
  if (buffer_completed == FALSE)
  {
     speed_tmp_buffer[index_pot_filt] = potentiometer_value;
     speed_sum_pot_filt = 0;
     for(uint16_t i = 1; i <= index_pot_filt;i++)
     {
       speed_sum_pot_filt = speed_sum_pot_filt + speed_tmp_buffer[i];
     }
     potent_filtered = speed_sum_pot_filt/index_pot_filt;
     index_pot_filt++;

      if (index_pot_filt >= FILTER_DEEP)
       {
         index_pot_filt = 1;
         buffer_completed = TRUE;
       }
  }
  else
  {
     index_pot_filt++;
     if (index_pot_filt >= FILTER_DEEP)
     {
      index_pot_filt = 1;
     }

     speed_sum_pot_filt = 0;
     speed_tmp_buffer[index_pot_filt] = potentiometer_value;
     uint16_t speed_max = 0;
     for(uint16_t i = 1; i < FILTER_DEEP;i++)
     {
       uint16_t val = speed_tmp_buffer[i];
       if (val > speed_max)
       {
         speed_max = val;
       }
       speed_sum_pot_filt += val;
     }
     speed_sum_pot_filt -= speed_max;
     potent_filtered = speed_sum_pot_filt/(FILTER_DEEP-2);
  }
  if (potent_filtered==0)
     potent_filtered = 1;
#endif                         // defined(RAN_OUT_OF_ROOM)

 return (potent_filtered);
}



/**************************************************************************
  * @defgroup MC_GetElSpeedHz    MC_GetElSpeedHz
  *
  * @brief Get the _Eletrical_ Motor Speed from ARR value of LF TIM
  * @retval int32_t Return the electrical motor speed
*/
int32_t  MC_GetElSpeedHz (void)
{
   if (__HAL_TIM_GetAutoreload(&LF_TIMx) != 0xFFFF)
      {
        uint16_t prsc = LF_TIMx.Instance->PSC;
        El_Speed_Hz = (int32_t)((SIXSTEP_parameters.SYSCLK_frequency)/(prsc))/(__HAL_TIM_GetAutoreload(&LF_TIMx)*6);
      }
     else El_Speed_Hz = 0;

   if (PI_parameters.Reference < 0)
      return (-El_Speed_Hz);
      else
           return (El_Speed_Hz);
}


/**************************************************************************
  * @defgroup MC_GetMechSpeedRPM    MC_GetMechSpeedRPM
  *
  * @brief Get the Mechanical Motor Speed (RPM)
  * @retval int32_t Return the mechanical motor speed (RPM
*/

int32_t  MC_GetMechSpeedRPM (void)
{
    Mech_Speed_RPM = (int32_t)(MC_GetElSpeedHz() *  60 / Rotor_poles_pairs);
    return (Mech_Speed_RPM);
}



/**************************************************************************
  * @brief  It calculates the square root of a non-negative s64.
  *   It returns 0 for negative s64.
  * @param  Input uint64_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
uint64_t  MCM_Sqrt (uint64_t wInput)
{
  uint8_t biter = 0u;
  uint64_t wtemproot;
  uint64_t wtemprootnew;

    if (wInput <= (uint64_t)((uint64_t)2097152<<shift_n_sqrt))
    {
      wtemproot = (uint64_t)((uint64_t)128<<shift_n_sqrt);
    }
    else
    {
      wtemproot = (uint64_t)((uint64_t)8192<<shift_n_sqrt);
    }

    do
    {
      wtemprootnew = (wtemproot + wInput/wtemproot)>>1;
      if (wtemprootnew == wtemproot)
      {
        biter = (shift_n_sqrt-1);
      }
      else
      {
        biter ++;
        wtemproot = wtemprootnew;
      }
    }
    while (biter < (shift_n_sqrt-1));

  return (wtemprootnew);
}


/**************************************************************************
  * @defgroup MC_StartMotor    MC_StartMotor
  *
  * @brief Start the Motor
  * @retval None               called from XZXX via HAL_GPIO_EXTI_Callback ISR on a USER button push
*/
void  MC_StartMotor (void)              // WVD - GOES THRU HERE ON BUTTON PUSH
{
    uwTick = 0;
    SIXSTEP_parameters.STATUS = START;
                                        // LF_TIMx = htim4 on F1_01
    HAL_TIM_Base_Start_IT (&LF_TIMx);   // start up Timer and assoc interrupts
    HAL_ADC_Start_IT (&ADCx);           // Start up ADC and assoc interrupts

    SIXSTEP_parameters.RUN_Motor = 1;   // denote Motor is now in RUN state

    BSP_X_NUCLEO_FAULT_LED_ON();        // TURN ON FAULT LED UNTIL MOTOR STARTED

    if (dac_status == TRUE)
       {
         START_DAC();
       }
}


/**************************************************************************
  * @defgroup MC_StopMotor    MC_StopMotor
  *
  * @brief Stop the Motor
  * @retval None
*/
void  MC_StopMotor (void)
{
    uwTick   = 0;

    SIXSTEP_parameters.STATUS    = STOP;
    SIXSTEP_parameters.RUN_Motor = 0;

    MC_SixStep_Stop_PWM_driving();

    HF_TIMx.Instance->CR1 &= ~(TIM_CR1_CEN);
    HF_TIMx.Instance->CNT  = 0;

    MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D();

    HAL_TIM_Base_Stop_IT (&LF_TIMx);
    HAL_ADC_Stop_IT (&ADCx);

    MC_SixStep_Current_Reference_Stop();
    BSP_X_NUCLEO_FAULT_LED_OFF();
    MC_SixStep_RESET();
}


//*************************************************************************
//                      HAL   STARTUP   CODE
//*************************************************************************

/*************************************************************************
*  HAL_MspInit
*                           Initializes the Global MSP.
*/
void  HAL_MspInit (void)
{

  HAL_NVIC_SetPriorityGrouping (NVIC_PRIORITYGROUP_4);

      /* System interrupt init*/
      /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (SysTick_IRQn, 2, 0);
}


/*************************************************************************
*  HAL_ADC_MspInit
*                           Initializes the ADCs
*/
void  HAL_ADC_MspInit (ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  if (hadc->Instance==ADC1)
   {
         /* Peripheral clock enable */
    __ADC1_CLK_ENABLE();

       /** ADC1 GPIO Configuration
       *     PC1     ------> ADC1_IN11
       *     PC2     ------> ADC1_IN12
       *     PC3     ------> ADC1_IN13
       *     PA1     ------> ADC1_IN1
       *     PA7     ------> ADC1_IN7
       *     PB0     ------> ADC1_IN8
       *     PB1     ------> ADC1_IN9
       */
    GPIO_InitStruct.Pin  = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_1|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

       /* System interrupt init*/
    HAL_NVIC_SetPriority (ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (ADC_IRQn);
   }

}


/*************************************************************************
*  HAL_TIM_Base_MspInit
*                       Initializes TIM1 timer used for PWMs
*                       Initializes TIM3 timer used for ???
*                       Initializes TIM4 timer used for monitoring BEMF and Commutations
*/
void  HAL_TIM_Base_MspInit (TIM_HandleTypeDef* htim_base)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  if (htim_base->Instance == TIM1)
   {
         //****************************************************
         //           Initialize  TIM1
         //****************************************************
         /* Peripheral clock enable */
    __TIM1_CLK_ENABLE();

        /** TIM1 GPIO Configuration
         *     PA6     ------> TIM1_BKIN
         *     PA8     ------> TIM1_CH1
         *     PA9     ------> TIM1_CH2
         *     PA10    ------> TIM1_CH3
         *     PA12    ------> TIM1_ETR
         */
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
                             | GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

        /* System interrupt init*/
    HAL_NVIC_SetPriority (TIM1_BRK_TIM9_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (TIM1_BRK_TIM9_IRQn);
   }
  else if (htim_base->Instance == TIM3)
   {
         //****************************************************
         //           Initialize  TIM3
         //****************************************************
         /* Peripheral clock enable */
    __TIM3_CLK_ENABLE();

         /** TIM3 GPIO Configuration
         *     PB4     ------> TIM3_CH1
         */
    GPIO_InitStruct.Pin   = GPIO_PIN_4;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);
   }
  else if (htim_base->Instance == TIM4)
   {
         //****************************************************
         //           Initialize  TIM4
         //****************************************************
         /* Peripheral clock enable */
    __TIM4_CLK_ENABLE();
         /* System interrupt init */
    HAL_NVIC_SetPriority (TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ (TIM4_IRQn);
   }
}


/*************************************************************************
*  HAL_TIMEx_HallSensor_MspInit
*                           Initializes TIM2 timer used for HALL Encoder tracking
*/
void  HAL_TIMEx_HallSensor_MspInit (TIM_HandleTypeDef *htimex_hallsensor)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  if (htimex_hallsensor->Instance == TIM2)
   {
         /* Peripheral clock enable */
    __TIM2_CLK_ENABLE();

         /**TIM2 GPIO Configuration
         *     PA0-WKUP  ------> TIM2_CH1
         *     PB10      ------> TIM2_CH3
         *     PB3       ------> TIM2_CH2
         */
    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_10|GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);
  }
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


//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//  Board Specific ISRs                                       STM32F4_01
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

extern  TIM_HandleTypeDef   htim1;
extern  TIM_HandleTypeDef   htim4;
extern  UART_HandleTypeDef  huart2;

void  TIM4_IRQHandler (void);
void  TIM1_BRK_TIM9_IRQHandler (void);
void  ADC_IRQHandler (void);
void  EXTI15_10_IRQHandler (void);
void  USART2_IRQHandler (void);


/*******************************************************************************
*                                  ISR    Handle System tick timer  ISR.
*/
void  SysTick_Handler (void)
{
    HAL_IncTick();

    HAL_SYSTICK_IRQHandler();
}


/*******************************************************************************
*  HAL_SYSTICK_Callback
*                       Systick CALLBACK - _CRITICAL_ this drives BLDC logic
*/

void  HAL_SYSTICK_Callback (void)
{
    MC_SysTick_SixStep_MediumFrequencyTask();
}


/*******************************************************************************
*                     ISR  handle TIM4 global interrupt.
*                                                                    TIM4_IRQn
*/
void  TIM4_IRQHandler (void)
{                                 // never being called under ny code ! 07/29/16
    HAL_TIM_IRQHandler (&htim4);   // Invoked by Timer turned on by Start_Motor
}

/*******************************************************************************
*                     ISR  Handle TIM1 Break interrupt and TIM9 global interrupt
*                                                            TIM9_IRQn
*/
void  TIM1_BRK_TIM9_IRQHandler (void)
{
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET)
       {                              // never being called under my code !  07/26/16
         MC_StopMotor();              // hit an error condition - turn off motor
         SIXSTEP_parameters.STATUS = OVERCURRENT;
       }

    HAL_TIM_IRQHandler (&htim1);
}


/*****************************************************************************
*                     CALLBACK                        BOARD SPECIFIC - F4_01
*
*  HAL_TIM_PeriodElapsedCallback
*                         called by Timer rupt from Start_Motor()
* @param  htim
*/
void  HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
    MC_TIMx_SixStep_timebase();   // Never being called under my code ! 07/26/16
}


/*******************************************************************************
*                        ISR   Handles ADC1 global interrupt.
*                                                          ADC_IRQn 0
*/
void  ADC_IRQHandler (void)
{
    HAL_ADC_IRQHandler (&hadc1);
}


/***************************************************************************
*  HAL_ADC_ConvCpltCallback     CALLBACK
*
*         ADC callback                              BOARD SPECIFIC - F4_01
* @param  hadc
*/
void  HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc)
{
     MC_ADCx_SixStep_Bemf();
}


/*******************************************************************************
*                                                       BOARD SPECIFIC - F4_01
*                        External Line Callback  ISR          FLAG/FAULT pin
*
* @param[in] GPIO_Pin pin number
* @retval None
*******************************************************************************/
extern void Encoder_GPIO_EXTI_Callback (uint16_t GPIO_Pin);

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


/*******************************************************************************
*                   Handle  EXTI Line[15:10] interrupts.   USER BUTTON press
*                                                           EXTI15_10_IRQn
*/
void  EXTI15_10_IRQHandler (void)
{
           // 06/29/16 - is NOT being invoked when USER button pressed
  HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_13);
}



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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
