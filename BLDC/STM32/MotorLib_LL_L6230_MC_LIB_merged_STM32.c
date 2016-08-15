// was MotorLib_LL_L6474.c               SPI is _NOT_ used on L6230 !

int  no_motor_test = 0;    // ??? 1 = run thru code with motor off/un-attached

//*******1*********2*********3*********4*********5*********6*********7**********
//
//                           MotorLib_LL_L6230_STM32.c
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

extern    MOTOR_BLOCK     motor_blk[];    // located in MotorLib_Basic.c
extern    MOTOR_HANDLERS  *_g_mtrdrvr;    //  ditto

void  update_STATUS (SIXSTEP_Base_SystStatus_t  new_STATUS);
void  update_Step_Position (uint8_t step_position);


    int   _g_adc_enabled = 0;
    int   isrFlag        = 0;

    int   mc_init_performed = 0;

    SPI_HandleTypeDef       _g_hspi1_handle;    // SPI 1 support

//  TIM_HandleTypeDef       hTimPwm1;           // Handle for Motor 1's PWM

    ADC_HandleTypeDef       ADC_InitStruct;

#define  TIMER_PRESCALER    1024

// VVVVVVVVVVVVVVVVVVVVVVVVVV  direct lift from L6230 main.c code

void  MC_SixStep_ADC_Channel (uint32_t adc_ch);

//#include "l6474.h"       // TEMP HACK
#define L6230_MAX_PWM_FREQ   (10000)     // Maximum frequency of the PWMs in Hz
#define L6230_MIN_PWM_FREQ   (2)         // Minimum frequency of the PWMs in Hz

// ^^^^^^^^^^^^^^^^^^^^^^^^^^  end  direct lift from L6230 code


int     l6230_GetDeviceState (MOTOR_BLOCK *mtr_blk);
int32_t l6230_get_motor_actual_position (MOTOR_BLOCK *mtr_blk);
int     l6230_motor_do_ctl_action (MOTOR_BLOCK *mtr_blk, int cmd, int flags);
int     l6230_perform_motor_move (MOTOR_BLOCK *mtr_blk, int cmd, int direction, int32_t count);
void    l6230_StartMovement (MOTOR_BLOCK *mtr_blk, int move_cmd);
int     l6230_motor_start (MOTOR_BLOCK *mtr_blk, int stop_type, int flags);
int     l6230_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags);
int32_t l6230_get_mark (MOTOR_BLOCK *mtr_blk);
int32_t l6230_CmdGetStatus (MOTOR_BLOCK *mtr_blk, int status_type);
int     l6230_WaitWhileActive (MOTOR_BLOCK *mtr_blk, int wait_type, int flags);
int     l6230_motor_init_controller_chip (void);

int     l6230_motor_run (MOTOR_BLOCK *mtr_blk, int direction);
int     l6230_set_direction (MOTOR_BLOCK *mtr_blk, int mtr_direction);
int     l6230_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count);
void    l6230_StepClockHandler (uint8_t motor_id);

int     l6230_set_bldc_phase_duty_cycles (MOTOR_BLOCK *mtr_blk, 
                                      uint16_t phaseU_duty_value,
                                      uint16_t phaseV_duty_value,
                                      uint16_t phaseW_duty_value);
int     l6230_set_bldc_phase_pwm_duty_cycle (MOTOR_BLOCK *mtr_blk, int phase_id, uint16_t duty_value);
int     l6230_set_bldc_pwm_enables (MOTOR_BLOCK *mtr_blk, int phase_id_mask);


int     l6230_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr);
int     l6230_motor_adc_next_conversion (void);
int     l6230_motor_init_adc_ctl_pins (void);
int     l6230_motor_init_encoder_gpio(MOTOR_BLOCK *mtr_blk, int encoder_type);
int     l6230_motor_init_gpio_ctl_outputs (void);
int     l6230_motor_init_gpio_ctl_inputs (void);
int     l6230_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long pwm_ticks);

int     l6230_current_reference_start_stop (MOTOR_BLOCK *mtr_blk, int start_stop_flag);
int     l6230_current_reference_set_value (MOTOR_BLOCK *mtr_blk, uint16_t Iref_value);

int     l6230_start_pwm_generation (MOTOR_BLOCK *mtr_blk);
int     l6230_stop_pwm_generation (MOTOR_BLOCK *mtr_blk);

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
void    l6230_start_motor_move(MOTOR_BLOCK *mtr_blk, int move_cmd);
void    l6230_StepClockHandler(uint8_t motor_id);

void     L6230_AttachErrorHandler (void (*callback)(uint16_t));    // L6206 relic
void     L6230_AttachFlagInterrupt (void (*callback)(void));       // L6206 relic
uint32_t L6230_CmdGetParam (MOTOR_BLOCK *mtr_blk, uint32_t param); // L6206 relic
uint8_t  L6230_Convert_Ocd_Thresh_to_RegValue (float Tval);        // L6206 relic
void     L6230_Init (void *pInit);                                 // L6206 relic
uint8_t  L6230_GetFwVersion (void);                                // L6206 relic
void     L6230_HardStop (MOTOR_BLOCK *mtr_blk);                    // L6206 relic
void     L6230_HizStop (MOTOR_BLOCK *mtr_blk);                     // L6206 relic
uint16_t L6230_ReadStatusRegister (MOTOR_BLOCK *mtr_blk);          // L6206 relic
uint16_t L6230_ReadId (void);                                      // L6206 relic
//void   L6230_ResetAllDevices (void);                                // L6206 relic
void     l6230_Release_Chip_Reset (MOTOR_BLOCK *mtr_blk);
void     l6230_Reset_Chip (MOTOR_BLOCK *mtr_blk);
void     L6230_SetDirection (MOTOR_BLOCK *mtr_blk, int dir);
bool     L6230_SoftStop (MOTOR_BLOCK *mtr_blk);

            // NEED EQUIVALENTS OF THESE in sep BOARD file, similar to what did for L6474
            // these take the place of HAL_xxxx.c file for each diff MCU
int L6230_Board_GpioInit (int mtr_id);                            // L6206 relic
int L6230_Board_PwmInit (int mtr_id);                             // L6206 relic
int L6230_Board_Reset (int mtr_id);                               // L6206 relic
int L6230_Board_ReleaseReset (int mtr_id);                        // L6464 relic
int L6230_Board_PwmStop (int mtr_id);                             // L6206 relic
int L6230_Board_SetDirectionGpio (int mtr_id, int dir);  // ??? does this make sense with BLDC ???

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
  MOTOR_HANDLERS * L6230_GetMotorHandle(void);   // return addr of drv6474_table

  MOTOR_HANDLERS  drv6230_table = {
                    TYPE_MOTOR_BLDC,              // Motor Type
                    MOTOR_DRIVER,                 // Chip number / id
                    0L,                           // SPI init
                    0L,                           // SPI I/O
                    l6230_motor_speed_adc_init,   // init routine for speed ctl ADC
                    l6230_motor_adc_next_conversion, // ADC conversion start rtn for speed
                    0L,                           // motor_read_hall_sensors_handler
                    0L,                           // motor_fault_shutdown
                    l6230_motor_init_adc_ctl_pins,    // control ADCs (Current sense)
                    l6230_motor_init_controller_chip, // motor_init_controller_chip  = NOP on L6230
                    0L,                               // motor_init_commute_time
                    l6230_motor_init_encoder_gpio,    // init routine for encoder GPIOs
                    l6230_motor_init_gpio_ctl_inputs, // control input  GPIOs (FAULT)
                    l6230_motor_init_gpio_ctl_outputs,// control output GPIOs (ENABLE)
                    l6230_motor_pwm_init,             // init for motor PWMs

                    0L,                           // motor_check_status
                    0L,                           // motor_do_action_handler - not used by L6230, no SPI
                    0L,                           // motor_do_next_commutate
                    l6230_get_motor_actual_position,// motor_get_actual_position
                    l6230_get_mark,               // motor_get_mark_pos
                    l6230_CmdGetStatus,           // motor_get_motor_status
                    0L,                           // motor_get_period_ticks
                    l6230_perform_motor_move,     // motor_perform_move
                    l6230_motor_run,              // motor_run_handler
                    l6230_set_direction,          // motor_set_direction_handler
                    l6230_set_pwm_duty_count,     // motor_set_pwm_duty_count_handler
                    0L,                           // motor_set_speed_rpm_handler
                    0L,                           // motor_set_step_mode  (STEPPER)
                    0L,                           // motor_set_torque
                    l6230_motor_start,            // motor_start_handler  (BLDC)
                    l6230_motor_stop,             // motor_stop_handler
                    l6230_WaitWhileActive,        // motor_wait_complete
                   l6230_current_reference_start_stop,// motor_current_reference_start_stop
                    l6230_current_reference_set_value,// motor_current_reference_set_value
                    l6230_start_pwm_generation,   // motor_start_pwm_generation (BLDC)
                    l6230_stop_pwm_generation,    // motor_stop_pwm_generation  (BLDC)
                    l6230_set_bldc_phase_duty_cycles, // motor_set_bldc_phase_pwm_duty_cycles (BLDC)
                    l6230_set_bldc_pwm_enables,   // motor_set_bldc_pwm_enables (BLDC)
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
extern  MOTOR_BLOCK  motor_blk [];             // is defined in MotorLib_Basic.c


#define MAX_STEPS       (0x7FFFFFFF)           // Maximum number of steps


    ADC_HandleTypeDef   hadc1;    // direct lift from L6230.c main.c startup code

    TIM_HandleTypeDef   hPwmMtr;       // TIM1  HF_TIM
    TIM_HandleTypeDef   htim2;
    TIM_HandleTypeDef   htim3;         // TIM3
    TIM_HandleTypeDef   hCommuteTmr;   // TIM4  LF_TIM  //  end  direct lift

extern  UART_HandleTypeDef  huart2;


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

#if defined(SETUP_LATER)
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
//  l6230_motor_init_adc_ctl_pins
//
//         Configure ADC to read control inputs for current sensors, BNEMF, ...
//******************************************************************************
int  l6230_motor_init_adc_ctl_pins (void)
{
    ADC_ChannelConfTypeDef   sConfig;
    GPIO_InitTypeDef         GPIO_InitStruct;

       //-----------------------------------------------
       // HAL_ADC_MspInit()
       //
       // Ensure GPIOs used for ADCs are converted over
       //-----------------------------------------------
//  if (hadc->Instance == ADC1)
       {
         ADC_CLK_ENABLE();     // Turn on ADC peripheral's clock

             /** ADC1 GPIO Configuration                     Arduino
             *     PA1     ------> ADC1_IN1   Vbus_Motor         (A1)
             *     PA7     ------> ADC1_IN7   Phase_W_bemf       (D11)
             *     PB0     ------> ADC1_IN8   Phase_V_bemf       (A3)
             *     PB1     ------> ADC1_IN9   Speed Ctl Pot       -
             *     PC1     ------> ADC1_IN11  Phase_V_curr_shunt (A4)
             *     PC2     ------> ADC1_IN12  Temperature         -
             *     PC3     ------> ADC1_IN13  Phase_U_bemf        - 
             *
             * -- Unused during Sensorless operation:
             *     PA0     ------> ADC1_IN0   Phase_U_curr_shunt (A0)
             *     PC0     ------> ADC1_IN10  Phase_W_curr_shunt (A5)
             *     PC9     ------> ADC1_INxx  GP_emf _NO_ ADC on this pin
             */
         GPIO_InitStruct.Pin  = ADC_Vbus_Mtr_GPIN | ADC_Ph_W_Bemf_GPIN;
         GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
         GPIO_InitStruct.Pull = GPIO_NOPULL;
         HAL_GPIO_Init (ADC_Vbus_Mtr_GPIO_PORT, &GPIO_InitStruct);  // PA1 / PA7

         GPIO_InitStruct.Pin  = ADC_Ph_V_Bemf_GPIN | ADC_Speed_Pot_GPIN;
         GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
         GPIO_InitStruct.Pull = GPIO_NOPULL;
         HAL_GPIO_Init (ADC_Ph_V_Bemf_GPIO_PORT, &GPIO_InitStruct); // PB0 / PB1

         GPIO_InitStruct.Pin  = ADC_Ph_V_Curr_GPIN | ADC_Temp_Mtr_GPIN | ADC_Ph_U_Bemf_GPIN;
         GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
         GPIO_InitStruct.Pull = GPIO_NOPULL;
         HAL_GPIO_Init (ADC_Ph_V_Curr_GPIO_PORT, &GPIO_InitStruct); // PC1 / PC2 / PC3

         HAL_NVIC_SetPriority (ADC_INTERRUPT_IRQn, 0, 0);  // setup ADC interrupt
         HAL_NVIC_EnableIRQ (ADC_INTERRUPT_IRQn);
        }

       //-------------------------------------------------------------
       // Configure the base ADC module
       // Configure the global features of the ADC (Clock, Resolution,
       // Data Alignment and number of conversion)
       //-------------------------------------------------------------
    hadc1.Instance                   = ADC_MODULE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.EOCSelection          = EOC_SINGLE_CONV;
    hadc1.Init.Resolution            = ADC_RESOLUTION12b;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    HAL_ADC_Init (&hadc1);

       // Configure for the selected ADC regular channel its corresponding rank
       // in the sequencer and its sample time.
    sConfig.Channel      = ADC_Temp_Mtr_CHANNEL;   // ADC_CHANNEL_12;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);

    return (0);           // denote was successful
}


//******************************************************************************
//  l6230_motor_init_controller_chip
//******************************************************************************
int  l6230_motor_init_controller_chip (void)
{
                // no low level resets or SPI init needed for L6230
    return (0);           // denote was successful
}


//******************************************************************************
//  l6230_motor_init_encoder_gpio                                      ENCODER
//
//         Configure GPIOs for Wheel Encoder pin inputs ...
//              Hall Sensor Left      PA8   Arduino D7    GPIO rupt   Grove D7
//              Hall Sensor Right     PA9   Arduino D8    GPIO rupt   Grove D8
//******************************************************************************
int  l6230_motor_init_encoder_gpio (MOTOR_BLOCK *mtr_blk, int encoder_type)
{

    return (0);           // denote was successful
}


//******************************************************************************
//  l6230_motor_init_gpio_ctl_inputs
//
//         Configure control inputs for Motor GPIOs  - USER BUTTON, FAULT, ...
//******************************************************************************

int  l6230_motor_init_gpio_ctl_inputs (void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    __GPIOH_CLK_ENABLE();    // ensure additional needed clocks turned on

    return (0);              // denote was successful
}


//******************************************************************************
//  l6230_motor_init_gpio_ctl_outputs
//
//         Configure control outputs for Motor GPIOs - ENABLE, and SPI if needed
//******************************************************************************
int  l6230_motor_init_gpio_ctl_outputs (void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

        // Configure GPIO pins : for COMMUTATE and ZCR debug
    GPIO_InitStruct.Pin   = COMMUTATE_DBG_GPIN | ZCR_DBG_GPIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init (COMMUTATE_DBG_GPIO_PORT, &GPIO_InitStruct);

        // Configure PWM ENABLE pins for EN1, EN2, EN3
    GPIO_InitStruct.Pin   = PWM_EN1_U_GPIN | PWM_EN2_V_GPIN | PWM_EN3_W_GPIN;
    HAL_GPIO_Init (PWM_ENx_PORT, &GPIO_InitStruct);

        // Configure GPIO pin for MOTOR_FAULT_LED : e.g. PB2
    GPIO_InitStruct.Pin   = MOTOR_FAULT_LED_GPIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init (MOTOR_FAULT_LED_GPIO_PORT, &GPIO_InitStruct);

        // Note: SPI is _NOT_ used on L6230, so we do not init SPI support

    return (0);           // denote was successful
}


//******************************************************************************
//  l6230_motor_pwm_init
//
//         Configure PWMs to drive BLDC motor U / V / W phases.
//         These use a variable duty cycle to control motor speed.
//         Generates a motor PWM with a frequency of ?? 20,000 Hz (50 usec).
//
//         Also configure a COMMUTATION timer, that is used to cause a periodic
//         "pop" to cause us to step to next entry in "6 Step" commutation table
//******************************************************************************

int  l6230_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long period_ticks)
{
    uint32_t                        channel;
    GPIO_InitTypeDef                GPIO_InitStruct;
    TIM_OC_InitTypeDef              sConfigOC;
    TIM_MasterConfigTypeDef         sMasterConfig;
    TIM_SlaveConfigTypeDef          sSlaveConfig;
    TIM_BreakDeadTimeConfigTypeDef  sBreakDeadTimeConfig;
    TIM_ClearInputConfigTypeDef     sClearInputConfig;
    TIM_ClockConfigTypeDef          sClockSourceConfig;

         //**********************************************************
         //          Initialize Motor PWM TIMER   GPIOs
         //
         //    Initialize PWM GPIOs used for motor U / V / W phases.
         //    Also init ETR and BKIN flags
         //**********************************************************
    PWM_UVW_TIMER_CLK_ENABLE();   // Enable Periph clock for PWM Timer

                  // Configure PWM U / V / W pins for TIMER operation
    GPIO_InitStruct.Pin       = PWM_U_GPIN | PWM_V_GPIN | PWM_W_GPIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = PWM_UVW_ALT_FUNC;
    HAL_GPIO_Init (PWM_UVW_GPIO_PORT, &GPIO_InitStruct);

                  // Configure BKIN / ETR pins for PWM operation
    GPIO_InitStruct.Pin       = PWM_UVW_BKIN_GPIN | PWM_UVW_ETR_GPIN;
    HAL_GPIO_Init (PWM_ETR_BK_GPIO_PORT, &GPIO_InitStruct);

        // initialize system interrupt used for PWM ETR error handler
    HAL_NVIC_SetPriority (PWM_UVW_ETR_INTERRUPT_IRQn, 0, 0);
        // Enable the PWM timer's ETR global Interrupt
    HAL_NVIC_EnableIRQ (PWM_UVW_ETR_INTERRUPT_IRQn);

         //**************************************************************
         //    Initialize  Motor Current PWM and Channel   GPIOs
         //
         // Initialize GPIOs used for PWM to generate Current Reference
         //**************************************************************
    PWM_CURRENT_REF_CLK_ENABLE();    // Enable Periph clock for Current Ref PWM

                  // Configure PWM Current Ref pin for TIMER operation
    GPIO_InitStruct.Pin   = PWM_CURRENT_REF_GPIN; // Configure PWM channel to use
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = PWM_CURRENT_REF_ALT_FUNC;
    HAL_GPIO_Init (PWM_CURRENT_REF_GPIO_PORT, &GPIO_InitStruct);

         //****************************************************************
         //      Initialize  COMMUTATION / BACK-EMF Timer   Interrupt GPIO
         //
         // Initialize TIM4 GPIO Rupt used for BEMF and Commutations
         //****************************************************************
    COMMUTATION_BEMF_CLK_ENABLE();   // Enable Periph clock for PWM Timer

        // initialize system interrupt used for COMMUTATION / BACK-EMF Timer
    HAL_NVIC_SetPriority (COMMUTATION_BEMF_INTERRUPT_IRQn, 1, 0);
        // Enable the COMMUTATION /EMF timer's global Interrupt
    HAL_NVIC_EnableIRQ (COMMUTATION_BEMF_INTERRUPT_IRQn);

         //----------------------------------------------------------
         //    Initialize PWM Timer used for motor U / V / W phases.
         //----------------------------------------------------------
    hPwmMtr.Instance           = PWM_UVW_TIMER;
    hPwmMtr.Init.Period        = PWM_UVW_START_PERIOD;  // PWM Period at startup
    hPwmMtr.Init.Prescaler     = 0;
    hPwmMtr.Init.CounterMode   = TIM_COUNTERMODE_CENTERALIGNED1;
    hPwmMtr.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    hPwmMtr.Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init (&hPwmMtr);

    HAL_TIM_PWM_Init (&hPwmMtr);

    sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_TRIGGER;
    sSlaveConfig.InputTrigger = TIM_TS_ITR3;
    HAL_TIM_SlaveConfigSynchronization (&hPwmMtr, &sSlaveConfig);

        // setup and sync Trigger0 to fire to ADCs for BEMF and Current Sense
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization (&hPwmMtr, &sMasterConfig);

        // setup BREAK to handle any emegency shutoff of PWMs
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime (&hPwmMtr, &sBreakDeadTimeConfig);

        // Configure initial Duty Cycle for the PWM U / V /W Motor Phases
    sConfigOC.Pulse        = PWM_UVW_START_DUTY;
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel (&hPwmMtr, &sConfigOC, PWM_U_CHANNEL);

    HAL_TIM_PWM_ConfigChannel (&hPwmMtr, &sConfigOC, PWM_V_CHANNEL);

    HAL_TIM_PWM_ConfigChannel (&hPwmMtr, &sConfigOC, PWM_W_CHANNEL);

        // ensure ETR flags cleared/reset for each of the  U / V / W channels
    sClearInputConfig.ClearInputSource    = TIM_CLEARINPUTSOURCE_ETR;
    sClearInputConfig.ClearInputPolarity  = TIM_CLEARINPUTPOLARITY_NONINVERTED;
    sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
    sClearInputConfig.ClearInputFilter    = 0;
    HAL_TIM_ConfigOCrefClear (&hPwmMtr, &sClearInputConfig, PWM_U_CHANNEL); // CCR1
    HAL_TIM_ConfigOCrefClear (&hPwmMtr, &sClearInputConfig, PWM_V_CHANNEL); // CCR2
    HAL_TIM_ConfigOCrefClear (&hPwmMtr, &sClearInputConfig, PWM_W_CHANNEL); // CCR3

         //------------------------------------------------------------
         // Initialize PWM Timer used for PWM input to Current Ref
         //------------------------------------------------------------
    htim3.Instance           = PWM_CURRENT_REF_TIMER;
    htim3.Init.Period        = PWM_CURRENT_REF_START_PERIOD;
    htim3.Init.Prescaler     = 0;
    htim3.Init.CounterMode   = TIM_COUNTERMODE_CENTERALIGNED1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init (&htim3);

    HAL_TIM_PWM_Init (&htim3);

    sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_TRIGGER;
    sSlaveConfig.InputTrigger = TIM_TS_ITR3;
    HAL_TIM_SlaveConfigSynchronization (&htim3, &sSlaveConfig);

          // keep us SYNC'ed to motor PWM timer for consistency
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
    HAL_TIMEx_MasterConfigSynchronization (&htim3, &sMasterConfig);

    sConfigOC.Pulse      = PWM_CURRENT_REF_START_DUTY;
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    HAL_TIM_PWM_ConfigChannel (&htim3, &sConfigOC, PWM_CURRENT_REF_CHANNEL); // CCR1

         //----------------------------------------------------------
         // Initialize Timer used for COMMUTATION and BEMF timing.
         //
         // This timer pops with periodic interrupts to cause us
         // to step to next step in Sensored or SensorLess "Six Step"
         // trapezoidal control.
         //----------------------------------------------------------
    hCommuteTmr.Instance           = COMMUTATION_BEMF_TIMER;
    hCommuteTmr.Init.Period        = COMMUTATION_BEMF_START_PERIOD;
    hCommuteTmr.Init.Prescaler     = 0;
    hCommuteTmr.Init.CounterMode   = TIM_COUNTERMODE_UP;
    hCommuteTmr.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init (&hCommuteTmr);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource (&hCommuteTmr, &sClockSourceConfig);

          // keep us SYNC'ed to motor PWM timer for consistency
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
    HAL_TIMEx_MasterConfigSynchronization (&hCommuteTmr, &sMasterConfig);

        //------------------------------------------------------
        // Initialize MC SixStep Library
        //------------------------------------------------------
    if (mc_init_performed == 0)
       {
             // initialize 6-Step lib # - from ST SAMPLE CODE
         MC_SixStep_INIT (mtr_blk);

         mc_init_performed = 1;     // denote we completed the init
       }

    return (0);                     // denote was successful
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
 * @brief  Attaches a user callback to the FAULT flag Interrupt
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


/*******************************************************************************
* l6230_CmdGetStatus
*
*      
*      Gets the OK/ERROR, etc statusof the  L6230 for the specified motor
*
*      NOTE: dumb BLDC chip does not have motion cmds like STEPPER does, so must
*            do manually !
*            L6230 does NOT have any SPI support, so no cmds are ever issued
******************************************************************************/
int32_t  l6230_CmdGetStatus (MOTOR_BLOCK *mtr_blk, int status_type)
{
    return (-1);                  // denote no such support on this chip
}


//******************************************************************************
//  l6230_current_reference_start_stop
//
//               Startup or shutdown the Motor "Current Reference" hardware
//******************************************************************************

int  l6230_current_reference_start_stop (MOTOR_BLOCK *mtr_blk, int start_stop_flag) // WVD - GOES THRU HERE
{
    if (start_stop_flag == START_FLAG)
       {      // start up the Motor's "current Reference" hardware
         PWM_CURRENT_REF_HANDLE.Instance->CCR1 = 0;

         #if (USES_L6230)
            HAL_TIM_PWM_Start (&PWM_CURRENT_REF_HANDLE, PWM_CURRENT_REF_CHANNEL); // was HF_TIMx_CH1
         #else
            REFx.Instance->CCER |= TIM_CCER_CC1E;              // enable/start the DAC
         #endif
       }
      else
       {      // shut down the Motor's "current Reference" hardware
         PWM_CURRENT_REF_HANDLE.Instance->CCR1 = 0;

         #if (USES_L6230)
            HAL_TIM_PWM_Stop (&PWM_CURRENT_REF_HANDLE, PWM_CURRENT_REF_CHANNEL);  // was HF_TIMx_CH1
         #else
            REFx.Instance->CCER &= ~TIM_CCER_CC1E;              // disable   
         #endif
       }

    return (0);                       // denote worked OK
}


//******************************************************************************
//  l6230_current_reference_set_value
//
//               Set the value to use for Motor's "Current Reference".
//               Updates the Iref Current reference value..
//******************************************************************************

int  l6230_current_reference_set_value (MOTOR_BLOCK *mtr_blk, uint16_t Iref_value)  // WVD - GOES THRU HERE
{
    if ( ! VOLTAGE_MODE)   
       PWM_CURRENT_REF_HANDLE.Instance->CCR1 = (uint32_t) ((Iref_value * PWM_CURRENT_REF_HANDLE.Instance->ARR) / 4096);
//PS   else SIXSTEP_parameters.pulse_value = Iref_value;    // UPPER_OUT_LIMIT
}



// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !



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
    return (&drv6230_table);   // L6206 relic
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


//******************************************************************************
//  l6230_get_motor_actual_position
//                            gets the absolute position of the motor, from
//                            the contreoller (stepper) or encoder QEI (BLDC)
//******************************************************************************
int32_t  l6230_get_motor_actual_position (MOTOR_BLOCK *mtr_blk)
{
    int32_t      raw_position;
    int32_t      abs_position;


// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

//  raw_position = L6230_CmdGetParam (mtr_blk, L6230_ABS_POS);   // L6206 relic

    abs_position = L6230_ConvertPosition (raw_position);

    return (abs_position);
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
    L6230_Board_PwmStop (mtr_blk->mtr_id);   // L6206 relic

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

    g_CmdGetStatus = l6230_CmdGetStatus (mtr_blk, 0); // clear any L6230 FAULT status

       /* Set inactive state */
    mtr_blk->mtr_motion_state        = MOTION_NOT_ACTIVE;
//   mtr_blk->mtr_operation_performed = NO_CMD;
    mtr_blk->mtr_steps_to_move       = MAX_STEPS;
}


/******************************************************//**
 *                     Release Chip Reset
 * @brief  Releases the L6230 reset (pin set to High) of all devices
 * @retval None
 **********************************************************/
void  l6230_Release_Chip_Reset (MOTOR_BLOCK *mtr_blk)
{
    L6230_Board_ReleaseReset (mtr_blk->mtr_id);
}


/**********************************************************
* l6230_Reset_Chip
*
*     Resets the L6230 (reset pin set to low) for associated
*     Motor Control Chip
*
* @param[in] motor_id (from 0 to 2)
**********************************************************/
void  l6230_Reset_Chip (MOTOR_BLOCK *mtr_blk)
{
    L6230_Board_Reset (mtr_blk->mtr_id);   // ??? TILT ???
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
//  l6230_motor_start                                        motor_start_handler
//
//                             Start the motor. 
//                             The start_type indicates the type of start.
//******************************************************************************
int  l6230_motor_start (MOTOR_BLOCK *mtr_blk, int stop_type, int flags)
{
                                           // LF_TIMx = hCommuteTmr on F1_01
     HAL_TIM_Base_Start_IT (&hCommuteTmr); // start up Timer and assoc interrupts

           // ensure Motor's "current reference" hardware is enabled
//PS _g_mtrdrvr->motor_current_reference_start_stop (mtr_blk, START_FLAG);

     HAL_ADC_Start_IT (&ADC_HANDLE);       // Start up ADC and assoc interrupts

     return (0);                           // denote worked OK
}


//******************************************************************************
//  l6230_motor_stop                                          motor_stop_handler
//
//                      Stop the motor. The stop_type inidcates
//                      the type of stop (Hard, Soft, Hiz, ...)
//******************************************************************************
int  l6230_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags)
{
    hPwmMtr.Instance->CR1 &= ~(TIM_CR1_CEN); // Shut off Motor PWMs  (NOT IN PS version)
    hPwmMtr.Instance->CNT  = 0;

    HAL_TIM_Base_Stop_IT (&hCommuteTmr); // Turn off PWM timer, including interrupts

    HAL_ADC_Stop_IT (&ADC_HANDLE);       // turn off ADC

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
                             // Disable power stage
                break;
      }
#endif

//  g_CmdGetStatus = l6230_CmdGetStatus (mtr_blk, 0); // clear any L6230 FAULT status (WVD added - for good measure)

       // Clear everything to denote we are in the INACTIVE state
    mtr_blk->mtr_motion_state        = MOTION_NOT_ACTIVE;
//  mtr_blk->mtr_operation_performed = NO_CMD;               ??? FIX RESOLVE
    mtr_blk->mtr_steps_to_move       = MAX_STEPS;

    return (0);                      // denote worked OK
}


//******************************************************************************
//  l6230_set_bldc_phase_duty_cycles
//
//         Set the duty cycle for each Phase (U/V/W) on a BLDC motor
//******************************************************************************
int l6230_set_bldc_phase_duty_cycles (MOTOR_BLOCK *mtr_blk, 
                                      uint16_t phaseU_duty_value,
                                      uint16_t phaseV_duty_value,
                                      uint16_t phaseW_duty_value)
{
              // Set the BLDC Motor Duty Cycle value for Phase U  (CH1)
    PHASE_U_DUTY = phaseU_duty_value;

              // Set the BLDC Motor Duty Cycle value for Phase V  (CH2)
    PHASE_V_DUTY = phaseV_duty_value;

              // Set the BLDC Motor Duty Cycle value for Phase W  (CH3)
    PHASE_W_DUTY = phaseW_duty_value;

    return (0);                      // denote it worked OK
}


//******************************************************************************
//  l6230_set_bldc_phase_pwm_duty_cycle
//
//         Set the duty cycle for a specific Phase (U/V/W) on a BLDC motor
//******************************************************************************
int l6230_set_bldc_phase_pwm_duty_cycle (MOTOR_BLOCK *mtr_blk, int phase_id,
                                         uint16_t duty_ccr_value)
{
    if (phase_id == PHASE_U)
       {      // Set the BLDC Motor Duty Cycle value for Phase U  (CH1)
         PHASE_U_DUTY = duty_ccr_value;
       }
      else if (phase_id == PHASE_V)
              {      // Set the BLDC Motor Duty Cycle value for Phase V  (CH2)
                PHASE_V_DUTY = duty_ccr_value;
              }
             else
              {      // Set the BLDC Motor Duty Cycle value for Phase W  (CH3)
                PHASE_W_DUTY = duty_ccr_value;
              }
    return (0);                      // denote it worked OK
}


//******************************************************************************
//  l6230_set_bldc_pwm_enables
//
//         Sets/clears the PWM ENABLEs for the 3 Phases (U/V/W) on a BLDC motor.
//
//         Only two legs can be active at a time, in a BLDC motor, so this shuts
//         off the PWM for the third "floating" leg in this step  (e.g. 6-Step)
//******************************************************************************
int l6230_set_bldc_pwm_enables (MOTOR_BLOCK *mtr_blk, int phase_id_mask)
{
    if (phase_id_mask == PHASE_DISABLE_ALL)
       {          // wants to disable ALL the phase PWMs  (e.g. motor stop)
         HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN1_U_GPIN, GPIO_RESET); // U EN1 DISABLE
         HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN2_V_GPIN, GPIO_RESET); // V EN2 DISABLE
         HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN3_W_GPIN, GPIO_RESET); // W EN3 DISABLE
       }
     else if (phase_id_mask == PHASE_ENABLE_V_W_DISABLE_U)
             {    // wants to shut off Phase U, and enable the other two
               HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN1_U_GPIN, GPIO_RESET); // U EN1 DISABLE
               HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN2_V_GPIN, GPIO_SET);   // V EN2 ENABLE
               HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN3_W_GPIN, GPIO_SET);   // W EN3 ENABLE
             }
     else if (phase_id_mask == PHASE_ENABLE_U_W_DISABLE_V)
             {    // wants to shut off Phase V, and enable the other two
               HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN1_U_GPIN, GPIO_SET);   // U EN1 ENABLE
               HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN2_V_GPIN, GPIO_RESET); // V EN2 DISABLE
               HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN3_W_GPIN, GPIO_SET);   // W EN3 ENABLE
             }
            else  // == PHASE_ENABLE_U_V_DISABLE_W
             {    // wants to shut off Phase W, and enable the other two
               HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN1_U_GPIN, GPIO_SET);   // U EN1 ENABLE
               HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN2_V_GPIN, GPIO_SET);   // V EN2 ENABLE
               HAL_GPIO_WritePin (PWM_ENx_PORT, PWM_EN3_W_GPIN, GPIO_RESET); // W EN3 DISABLE
             }


}


//******************************************************************************
//  l6230_set_direction
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
int  l6230_set_direction (MOTOR_BLOCK *mtr_blk,  int direction)
{
    int  pwm_duty_count;

    mtr_blk->mtr_direction = direction;     // ensure desired direction is saved


// on BLDC, we do NOT set a DIR pin.  Instead, we wal;k thru COMMUTATE table BACKWARDs

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
//  l6230_set_pwm_duty_count
//
//            Set the duty cycle to be used for a specific motor.
//            Here, duty cycle is explicitly passed as PWM ticks, not as percent.
//            The duty cycle sets the motor's speed. (More juice, more speed)
//
//   Parms:
//      motor_num:   number of the motor to update (1 or 2)
//      pwm_duty:    actual duty cycle PWM count value to be loaded in PWM
//******************************************************************************

int  l6230_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count)
{
     int  adjusted_duty_count;

     mtr_blk->mtr_pwm_duty_count = pwm_duty_count;  // save actual count (not %)

// TBD - Anythings else to do for a stepper >  Does duty even make sense ?
//           or issue REJECT ?

     return (0);                             // do not apply till motor start

}


//******************************************************************************
//  l6230_start_pwm_generation
//
//            Enable generation of PWM signals on PWM channels/CCRs for L6230.
//            called via COMMUTATE Timer rupts started by Start_Motor()
//******************************************************************************

int  l6230_start_pwm_generation (MOTOR_BLOCK *mtr_blk)
{
    HAL_TIM_PWM_Start (&hPwmMtr, PWM_U_CHANNEL);        // TIM1_CH1 start
    HAL_TIM_PWM_Start (&hPwmMtr, PWM_V_CHANNEL);        // TIM1_CH2 start
    HAL_TIM_PWM_Start (&hPwmMtr, PWM_W_CHANNEL);        // TIM1_CH3 start
}


//******************************************************************************
//  l6230_stop_pwm_generation
//
//            Disable the generation of PWM signals on PWM channels for L6230.
//******************************************************************************

int  l6230_stop_pwm_generation (MOTOR_BLOCK *mtr_blk)
{
    HAL_TIM_PWM_Stop (&hPwmMtr, PWM_U_CHANNEL);         // TIM1_CH1 stop
    HAL_TIM_PWM_Stop (&hPwmMtr, PWM_V_CHANNEL);         // TIM1_CH2 stop
    HAL_TIM_PWM_Stop (&hPwmMtr, PWM_W_CHANNEL);         // TIM1_CH3 stop
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
void  L6230_SetDirection (MOTOR_BLOCK *mtr_blk, int dir)
{

// ??? !!!  for BLDC, there is no DIR pin.  Instead, walk thru COMMUTATE table BACKWARDs

     L6230_Board_SetDirectionGpio (mtr_blk->mtr_id, dir);
}


// CAUTION  dumb BLDC vs STEPPER chip


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

         // ensure L6230 powerstage is Enabled

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
//         The following were directly lifted from L6230.c code and X-NUCLEO-IHM07M1.c


//void MC_SixStep_Start_PWM_driving (void);
//void MC_SixStep_Stop_PWM_driving (void);


//void PwmMtr_Enable_Phase_U_V_Disable_W(void);
//void PwmMtr_Enable_Phase_U_W_Disable_V(void);
//void PwmMtr_Enable_Phase_V_W_Disable_U(void);
//void DisableInput_CH1_D_CH2_D_CH3_D(void);
//void Start_PWM_driving(void);
//void Stop_PWM_driving(void);
//void PwmMtr_SetDutyCycle_PhaseU(uint16_t);
//void PwmMtr_SetDutyCycle_PhaseV(uint16_t);
//void PwmMtr_SetDutyCycle_PhaseW(uint16_t);
//void Current_Reference_Start(void);
//void Current_Reference_Stop(void);
//void Current_Reference_Setvalue(uint16_t);

void BSP_X_NUCLEO_FAULT_LED_ON(void);
void BSP_X_NUCLEO_FAULT_LED_OFF(void);



/*****************************************************************************
  *
  *                             PWM  ISR  Callback
  *
  * @param[in] htim PWM handle pointer
  * @retval None
*****************************************************************************/

void  HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
{
    MOTOR_BLOCK   *mtr_blk;

  if ((htim->Instance == TIM3) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
     {     // process motor 1
       mtr_blk = &motor_blk[0];                          // ??? TEMP TEST HACK
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
#include "MC_SixStep_param.h"


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
    uint16_t  Rotor_poles_pairs;                    /*!<  Number of pole pairs of the motor */
    uint32_t  mech_accel_hz        = 0;             /*!<  Hz -- Mechanical acceleration rate */
    uint32_t  constant_k           = 0;             /*!<  1/3*mech_accel_hz */
    uint32_t  Time_vector_tmp      = 0;             /*!<  Startup variable  */
    uint32_t  Time_vector_prev_tmp = 0 ;            /*!<  Startup variable  */
    uint32_t  T_single_step        = 0;             /*!<  Startup variable  */
    uint32_t  T_single_step_first_value = 0;        /*!<  Startup variable  */
    int32_t   delta                = 0;             /*!<  Startup variable  */
    uint16_t  index_array          = 1;             /*!<  Speed filter variable */
    int16_t   speed_tmp_array [FILTER_DEEP];        /*!<  Speed filter variable */
    uint16_t  speed_tmp_buffer [FILTER_DEEP];       /*!<  Potentiometer filter variable */
    uint16_t  HFBuffer [HFBUFFERSIZE];              /*!<  Buffer for Potentiometer Value Filtering at the High-Frequency ADC conversion */
    uint16_t  HFBufferIndex        = 0;             /*!<  High-Frequency Buffer Index */
    uint8_t   array_completed      = FALSE;         /*!<  Speed filter variable */
    uint8_t   buffer_completed     = FALSE;         /*!<  Potentiometer filter variable */
    uint8_t   bemf_array_completed = FALSE;         /*!<  Bemf-delay filter variable */
    uint8_t   UART_FLAG_RECEIVE    = FALSE;         /*!<  UART commmunication flag */
    uint32_t  ARR_LF               = 0;             /*!<  Autoreload LF TIM variable */
    int32_t   Mech_Speed_RPM       = 0;             /*!<  Mechanical motor speed */
    int32_t   El_Speed_Hz          = 0;             /*!<  Electrical motor speed */
    uint16_t  index_adc_chn        = 0;             /*!<  Index of ADC channel selector for measuring */
    uint16_t  index_motor_run      = 0;             /*!<  Tmp variable for DEMO mode */
    uint16_t  test_motor_run       = 1;             /*!<  Tmp variable for DEMO mode */
    uint8_t   Enable_start_button  = TRUE;          /*!<  Start/stop button filter to avoid double command */
    uint32_t  n_zcr_startup        = 0;             /*!<  Counter of zero crossing event before closed loop control */
    uint16_t  index_ARR_step       = 1;             /*!<  Index for number of steps during startup */
    uint16_t  index_startup_motor  = 1;             /*!<  Index for number of steps calculated during startup */
    uint16_t  target_speed         = TARGET_SPEED;  /*!<  Target speed for closed loop control */
    uint16_t  shift_n_sqrt         = 14;            /*!<  Shift of bits for root square function */
    uint16_t  cnt_bemf_event       = 0;             /*!<  Counter for Bemf failure detection during startup */
    uint8_t   startup_bemf_failure = 0;             /*!<  Bemf failure indicator during startup (TRUE if failure occurs)*/
    uint8_t   speed_fdbk_error     = 0;             /*!<  Speedfeedback error indicator during startup (TRUE if failure occurs)*/
__IO uint32_t uwTick               = 0;             /*!<  Tick counter - 1msec updated */
    uint8_t   dac_trace_status     = DAC_ENABLE;    /*!<  Trace out to DAC indicator (TRUE) if enabled */
    uint16_t  index_align          = 1;             /*!<  Index for alignment time calculation */
    int32_t   speed_sum_sp_filt    = 0;             /*!<  Variable for speed filter */
    int32_t   speed_sum_pot_filt   = 0;             /*!<  Variable for potentiometer filter */
    uint16_t  index_pot_filt       = 1;             /*!<  Variable for potentiometer filter */
    int16_t   potent_filtered      = 0;             /*!<  Variable for potentiometer filter */
    uint32_t  Tick_cnt             = 0;             /*!<  Counter for speed loop calling */
    uint32_t  counter_ARR_Bemf     = 0;             /*!<  Store the last value of Autoreload for LF timer */
    uint64_t  constant_multiplier_tmp = 0;          /*!<  Variable for startup calculation */
    uint32_t  demagn_value_tmp     = 0;             /*!<  Demagn delay time dynamically calculated  */
    uint32_t  ARR_divider          = 80;            /*!<  Bemf delay filter variable */
    uint16_t  tmp_val              = 0;             /*!<  Bemf delay filter variable */
    uint32_t  ARR_tmp              = 0;             /*!<  Bemf delay filter variable */
    uint32_t  ARR_tmp_prev         = 0;             /*!<  Bemf delay filter variable */
    uint8_t   flag_change_step     = FALSE;         /*!<  Bemf delay filter variable */

    uint16_t  dmg_tmp              = 0;             /*!<  Bemf delay filter variable */
    int32_t   bemf_sum_filt        = 0;             /*!<  Bemf delay filter variable */
    uint16_t  bemf_tmp_array [FILTER_DEEP];         /*!<  Bemf delay filter variable */
    uint16_t  bemf_index_array     = 1;             /*!<  Bemf delay filter variable */
    uint8_t   synchronous_rect     = SYNCHRONOUS_RECTIFICATION; /*!<  Synchronous rectification enable flag */
    uint8_t   Upcounting_event_flag = FALSE;        /*!<  HF Timer counting flag indicator */

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

//void MC_SixStep_Start_PWM_driving(void);
//void MC_SixStep_Stop_PWM_driving(void);
//void MC_SixStep_PwmMtr_SetDutyCycle_PhaseU(uint16_t);
//void MC_SixStep_PwmMtr_SetDutyCycle_PhaseV(uint16_t);
//void MC_SixStep_PwmMtr_SetDutyCycle_PhaseW(uint16_t);
//void MC_SixStep_Current_Reference_Start(void);
//void MC_SixStep_Current_Reference_Stop(void);
//void MC_SixStep_Current_Reference_Setvalue(uint16_t);
void MC_SixStep_ARR_Bemf(uint8_t);
void MC_UI_INIT(void);
void MC_SixStep_Init_main_data(void);
void CMD_Parser(char* pCommandString);
void MC_SixStep_Speed_Val_target_potentiometer(void);
void MC_SixStep_Nucleo_Init (void);
void MC_SixStep_ADC_Channel (uint32_t adc_ch);
void Bemf_delay_calc (void);

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc);
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim);
void HAL_SYSTICK_Callback (void);
void UART_Set_Value(void);
void UART_Communication_Task(void);



//******************************************************************************
//  MC_SixStep_INIT
//
//              Initialitation function for SixStep library
//******************************************************************************

void  MC_SixStep_INIT (MOTOR_BLOCK *mtr_blk)              // WVD - GOES THRU HERE
{
    MC_SixStep_Nucleo_Init();

    SIXSTEP_parameters.HF_TIMx_CCR  = hPwmMtr.Instance->PHASE_U_CCR1;
    SIXSTEP_parameters.HF_TIMx_ARR  = hPwmMtr.Instance->ARR;
    SIXSTEP_parameters.HF_TIMx_PSC  = hPwmMtr.Instance->PSC;

    SIXSTEP_parameters.LF_TIMx_ARR  = hCommuteTmr.Instance->ARR;
    SIXSTEP_parameters.LF_TIMx_PSC  = hCommuteTmr.Instance->PSC;

#if defined(USES_L6230)
//// _g_mtrdrvr->motor_current_reference_start_stop (mtr_blk, START_FLAG);
     _g_mtrdrvr->motor_current_reference_set_value (mtr_blk, SIXSTEP_parameters.Ireference);
#else
//PS HAL_GPIO_WritePin (GPIO_PORT_BEMF,GPIO_CH_BEMF,GPIO_PIN_SET);
#endif

 #ifdef UART_COMM
    SIXSTEP_parameters.Button_ready = FALSE;
    MC_UI_INIT();               //  Start the UART Communication Task
 #endif

    MC_SixStep_Init_main_data();

 #ifndef UART_COMM
    SIXSTEP_parameters.Button_ready = TRUE;
 #endif

    MC_SixStep_RESET (mtr_blk);
}


//******************************************************************************
//  MC_SixStep_Nucleo_Init
//
//             Init the STM32 registers                BOARD SPECIFIC - F4_01
//******************************************************************************

void  MC_SixStep_Nucleo_Init (void)                  // WVD - GOES THRU HERE
{
    TIM_ClearInputConfigTypeDef  sClearInputConfig;
    ADC_ChannelConfTypeDef       sConfig;

  if ( ! VOLTAGE_MODE)                               // PS
    {
         /******************** ETR CONFIGURATION *******************P*********/
      sClearInputConfig.ClearInputState     = 1;
      sClearInputConfig.ClearInputSource    = TIM_CLEARINPUTSOURCE_ETR;
      sClearInputConfig.ClearInputPolarity  = TIM_CLEARINPUTPOLARITY_NONINVERTED;
      sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
      sClearInputConfig.ClearInputFilter    = 0;
      HAL_TIM_ConfigOCrefClear (&hPwmMtr, &sClearInputConfig, PWM_U_CHANNEL);
      HAL_TIM_ConfigOCrefClear (&hPwmMtr, &sClearInputConfig, PWM_V_CHANNEL);
      HAL_TIM_ConfigOCrefClear (&hPwmMtr, &sClearInputConfig, PWM_W_CHANNEL);
    }

    __HAL_FREEZE_TIM1_DBGMCU();                   // Stop TIM during Breakpoint

    __HAL_TIM_ENABLE_IT (&hPwmMtr, TIM_IT_BREAK); // Enable the TIM Break interrupt

        /**************** REGULAR CHANNELS CONFIGURATION **********************/
        //----------------------------------------------------------------------
        // Complete configuration of the rest of the channels in the ADC
        //----------------------------------------------------------------------
    sConfig.Channel      = ADC_Ph_V_Curr_CHANNEL;  // ADC_CH_1;  // Current feedabck
    sConfig.Rank         = 1;
    sConfig.Offset       = 0;
    sConfig.SamplingTime = ADC_CH_1_ST;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);
    sConfig.Channel      = ADC_Vbus_Mtr_CHANNEL;   // ADC_CH_3;  // Bus voltage
    sConfig.SamplingTime = ADC_CH_3_ST;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);
    sConfig.Channel      = ADC_Temp_Mtr_CHANNEL;   // ADC_CH_4;  // Temperature feedback
    sConfig.SamplingTime = ADC_CH_4_ST;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);

    sConfig.Channel      = ADC_Ph_U_Bemf_CHANNEL;  // ADC_Bemf_CH1;  /* BEMF feedback phase U/A */
    sConfig.SamplingTime = ADC_Bemf_CH1_ST;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);
    sConfig.Channel      = ADC_Ph_V_Bemf_CHANNEL;  // ADC_Bemf_CH2;  /* BEMF feedback phase V/B */
    sConfig.SamplingTime = ADC_Bemf_CH2_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel      = ADC_Ph_W_Bemf_CHANNEL;  // ADC_Bemf_CH3;  /* BEMF feedback phase W/C */
    sConfig.SamplingTime = ADC_Bemf_CH3_ST;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);

    sConfig.Channel      = ADC_Speed_Pot_CHANNEL;  // ADC_CH_2;      /* Potentiometer / Speed Control */
    sConfig.SamplingTime = ADC_CH_2_ST;
    HAL_ADC_ConfigChannel (&hadc1, &sConfig);
}


//******************************************************************************
//  MC_SixStep_RESET
//
//                 Reset all variables used for 6Step control algorithm.
//                 Called as part of INIT logic.
//******************************************************************************

void  MC_SixStep_RESET (MOTOR_BLOCK *mtr_blk)           // WVD - GOES THRU HERE
{
    SIXSTEP_parameters.CMD                   = TRUE;
    SIXSTEP_parameters.numberofitemArr       = NUMBER_OF_STEPS;
    SIXSTEP_parameters.ADC_BEMF_threshold_UP = BEMF_THRSLD_UP;
    SIXSTEP_parameters.ADC_BEMF_threshold_DOWN = BEMF_THRSLD_DOWN;
    SIXSTEP_parameters.Ireference            = STARTUP_CURRENT_REFERENCE;
    SIXSTEP_parameters.Speed_Loop_Time       = SPEED_LOOP_TIME;
    SIXSTEP_parameters.pulse_value           = SIXSTEP_parameters.HF_TIMx_CCR;
//PS if ( ! VOLTAGE_MODE)
//PS    SIXSTEP_parameters.pulse_value = SIXSTEP_parameters.HF_TIMx_CCR;
//PS    else  SIXSTEP_parameters.pulse_value = DUTY_CYCLE_INIT_VALUE;

    SIXSTEP_parameters.Speed_target_ramp     = MAX_POT_SPEED;
    SIXSTEP_parameters.ALIGNMENT             = FALSE;
    SIXSTEP_parameters.Speed_Ref_filtered    = 0;
    SIXSTEP_parameters.demagn_value          = INITIAL_DEMAGN_DELAY;

    SIXSTEP_parameters.CurrentRegular_BEMF_ch = 0;
    SIXSTEP_parameters.status_prev           = 0;
    SIXSTEP_parameters.step_position         = 0;
    update_Step_Position (SIXSTEP_parameters.step_position);     // trace 6-step

    hCommuteTmr.Init.Prescaler = SIXSTEP_parameters.LF_TIMx_PSC; // setup LF and HF times
    hCommuteTmr.Instance->PSC  = SIXSTEP_parameters.LF_TIMx_PSC;
    hCommuteTmr.Init.Period    = SIXSTEP_parameters.LF_TIMx_ARR;
    hCommuteTmr.Instance->ARR  = SIXSTEP_parameters.LF_TIMx_ARR;

    hPwmMtr.Init.Prescaler         = SIXSTEP_parameters.HF_TIMx_PSC;
    hPwmMtr.Instance->PSC          = SIXSTEP_parameters.HF_TIMx_PSC;
    hPwmMtr.Init.Period            = SIXSTEP_parameters.HF_TIMx_ARR;
    hPwmMtr.Instance->ARR          = SIXSTEP_parameters.HF_TIMx_ARR;
    hPwmMtr.Instance->PHASE_U_CCR1 = SIXSTEP_parameters.HF_TIMx_CCR;

    Rotor_poles_pairs = SIXSTEP_parameters.NUMPOLESPAIRS;
    SIXSTEP_parameters.SYSCLK_frequency = HAL_RCC_GetSysClockFreq();

           // initially clear all duty cycles to 0
    _g_mtrdrvr->motor_set_bldc_phase_duty_cycles (mtr_blk, 0, 0, 0);

        // setup ADC channel table for sampling BEMF for phases U / V / W
    SIXSTEP_parameters.Regular_channel[1] = ADC_Ph_U_Bemf_CHANNEL; // ADC_Bemf_CH1; /* BEMF1   */
    SIXSTEP_parameters.Regular_channel[2] = ADC_Ph_V_Bemf_CHANNEL; // ADC_Bemf_CH2; /* BEMF2   */
    SIXSTEP_parameters.Regular_channel[3] = ADC_Ph_W_Bemf_CHANNEL; // ADC_Bemf_CH3; /* BEMF3   */

        // setup ADC channel table for sampling Current / Speed / Vbus / Temperature
    SIXSTEP_parameters.ADC_SEQ_CHANNEL[0] = ADC_Ph_V_Curr_CHANNEL; // ADC_CH_1;     /* CURRENT */
    SIXSTEP_parameters.ADC_SEQ_CHANNEL[1] = ADC_Speed_Pot_CHANNEL; // ADC_CH_2;     /* SPEED   */
    SIXSTEP_parameters.ADC_SEQ_CHANNEL[2] = ADC_Vbus_Mtr_CHANNEL;  // ADC_CH_3;     /* VBUS    */
    SIXSTEP_parameters.ADC_SEQ_CHANNEL[3] = ADC_Temp_Mtr_CHANNEL;  // ADC_CH_4;     /* TEMP    */

//  SIXSTEP_parameters.step_position      = 0;   // DUPLICATE ???
    SIXSTEP_parameters.demagn_counter     = 0;
    SIXSTEP_parameters.ALIGN_OK           = FALSE;
    SIXSTEP_parameters.VALIDATION_OK      = 0;
    SIXSTEP_parameters.ARR_OK             = 0;
    SIXSTEP_parameters.speed_fdbk_filtered = 0;
    SIXSTEP_parameters.Integral_Term_sum  = 0;
    SIXSTEP_parameters.Current_Reference  = 0;
    SIXSTEP_parameters.Ramp_Start         = 0;
    SIXSTEP_parameters.RUN_Motor          = 0;
    SIXSTEP_parameters.speed_fdbk         = 0;
    SIXSTEP_parameters.BEMF_OK            = FALSE;
    SIXSTEP_parameters.CL_READY           = FALSE;
    SIXSTEP_parameters.SPEED_VALIDATED    = FALSE;
    SIXSTEP_parameters.BEMF_Tdown_count   = 0;   /* Reset of the Counter to detect Stop motor condition when a stall condition occurs*/

    synchronous_rect = SYNCHRONOUS_RECTIFICATION;   // PS ONLY
    uwTick           = 0;
    index_motor_run  = 0;
    test_motor_run   = 1;
    T_single_step    = 0;
    T_single_step_first_value = 0;
    delta            = 0;
    Time_vector_tmp  = 0;
    Time_vector_prev_tmp = 0;
    Mech_Speed_RPM   = 0;
    El_Speed_Hz      = 0;
    index_adc_chn    = 0;
    mech_accel_hz    = 0;
    constant_k       = 0;
    ARR_LF           = 0;
    index_array      = 1;
    Enable_start_button = TRUE;
    index_ARR_step   = 1;
    n_zcr_startup    = 0;
    cnt_bemf_event   = 0;
    startup_bemf_failure = 0;
    speed_fdbk_error = 0;
 
    bemf_array_completed = FALSE;       // PS  BEGIN
    bemf_index_array = 1;  
    bemf_sum_filt    = 0;
    flag_change_step = FALSE;
    for (uint16_t i = 0; i < FILTER_DEEP; i++)
      {
        bemf_tmp_array[i] =0;
      }                                 // PS   END

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
          update_Step_Position (SIXSTEP_parameters.step_position); // trace 6-step
        }
     target_speed = TARGET_SPEED;

     MC_Set_PI_param (&PI_parameters);

         // Startup PWM or DAC that is used to generate Motor's current reference
     _g_mtrdrvr->motor_current_reference_start_stop (mtr_blk, START_FLAG);
     _g_mtrdrvr->motor_current_reference_set_value (mtr_blk, SIXSTEP_parameters.Ireference);

//PS if ( ! VOLTAGE_MODE)
//PS    _g_mtrdrvr->motor_current_reference_set_value (mtr_blk, SIXSTEP_parameters.Ireference); 
//PS    else _g_mtrdrvr->motor_current_reference_set_value (mtr_blk,
//                                                          SIXSTEP_parameters.pulse_value); 

     index_startup_motor = 1;

     MC_SixStep_Ramp_Motor_calc ();
}


//******************************************************************************
//                              CALLBACK
//  MC_SixStep_ADC_Channel                        caller:  ADCConvCmplt Callback
//
//             Select the new (BEMF) ADC Channel          BOARD SPECIFIC - F4_01
//  @param  adc_ch        This logic selects 1 ADC channel at a time for BEMF feedback
//                    // 6-STEP rotations thru channels  U / V / W:  13 / 8 / 7
//******************************************************************************

void  MC_SixStep_ADC_Channel (uint32_t adc_ch)
{
    __HAL_ADC_DISABLE (&ADC_HANDLE);

        /* Clear the old SQx bits for the selected rank */
    ADC_HANDLE.Instance->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, 1);

        /* Set the SQx bits for the selected rank */
    ADC_HANDLE.Instance->SQR3 |= ADC_SQR3_RK(adc_ch, 1);

    __HAL_ADC_ENABLE (&ADC_HANDLE);
}


#if defined(USES_L6398)    // PS ONLY
/***************************************************************************
*  MC_Bemf_delay_Filter
*
*                    Calculate the Bemf delay time filtered
*/

uint32_t  MC_Bemf_delay_Filter (uint32_t dmg_tmp_value)
{  
    uint16_t   dmg_value = 0;
  
   if (bemf_array_completed == FALSE)
    {
      bemf_tmp_array[bemf_index_array] = dmg_tmp_value;    
      bemf_sum_filt = 0;
      for (uint16_t i = 1; i <= bemf_index_array;i++)
        {
          bemf_sum_filt = bemf_sum_filt + bemf_tmp_array[i];
        }
      dmg_value = bemf_sum_filt/bemf_index_array;
      bemf_index_array++;
     
      if (bemf_index_array >= FILTER_DEEP) 
         {
           bemf_index_array = 1;
           bemf_array_completed = TRUE;
         }
    }  
   else
    {
      bemf_index_array++;
      if (bemf_index_array >= FILTER_DEEP) 
         bemf_index_array = 1;
     
       bemf_sum_filt = 0;
       bemf_tmp_array[bemf_index_array] = dmg_tmp_value;   
       for (uint16_t i = 1; i < FILTER_DEEP;i++)
         {
           bemf_sum_filt = bemf_sum_filt + bemf_tmp_array[i];
         }      
       dmg_value = bemf_sum_filt / (FILTER_DEEP-1);
    }

   return (dmg_value); 
}
#endif                // defined(L6398)    // PS ONLY


/***************************************************************************
*  Bemf delay calculation                    BOARD SPECIFIC - F4_01
*/
void  Bemf_delay_calc (void)
{
if (PI_parameters.Reference >= 0)
 {
 if(SIXSTEP_parameters.speed_fdbk_filtered <= 12000 && SIXSTEP_parameters.speed_fdbk_filtered > 10000)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 10000 && SIXSTEP_parameters.speed_fdbk_filtered > 9400)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 9400 && SIXSTEP_parameters.speed_fdbk_filtered > 7600)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 7600 && SIXSTEP_parameters.speed_fdbk_filtered > 6000)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 6000 && SIXSTEP_parameters.speed_fdbk_filtered > 5400)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 5400 && SIXSTEP_parameters.speed_fdbk_filtered > 4750)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 4750 && SIXSTEP_parameters.speed_fdbk_filtered > 4200)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 4200 && SIXSTEP_parameters.speed_fdbk_filtered > 2600)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 2600 && SIXSTEP_parameters.speed_fdbk_filtered > 1800)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 1800 && SIXSTEP_parameters.speed_fdbk_filtered > 1500)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 1500 && SIXSTEP_parameters.speed_fdbk_filtered > 1300)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 1300 && SIXSTEP_parameters.speed_fdbk_filtered > 1000)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered <= 1000 && SIXSTEP_parameters.speed_fdbk_filtered > 500)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
  }
 }
 else
 {
  if(SIXSTEP_parameters.speed_fdbk_filtered >= -12000 && SIXSTEP_parameters.speed_fdbk_filtered < -10000)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_1;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -10000 && SIXSTEP_parameters.speed_fdbk_filtered < -7800)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -7800 && SIXSTEP_parameters.speed_fdbk_filtered < -6400)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -6400 && SIXSTEP_parameters.speed_fdbk_filtered < -5400)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -5400 && SIXSTEP_parameters.speed_fdbk_filtered < -4650)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -4650 && SIXSTEP_parameters.speed_fdbk_filtered < -4100)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -4100 && SIXSTEP_parameters.speed_fdbk_filtered < -3650)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -3650 && SIXSTEP_parameters.speed_fdbk_filtered < -3300)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -3300 && SIXSTEP_parameters.speed_fdbk_filtered < -2650)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -2600 && SIXSTEP_parameters.speed_fdbk_filtered < -1800)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -1800 && SIXSTEP_parameters.speed_fdbk_filtered < -1500)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -1500 && SIXSTEP_parameters.speed_fdbk_filtered < -1300)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -1300 && SIXSTEP_parameters.speed_fdbk_filtered < -1000)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered >= -1000 && SIXSTEP_parameters.speed_fdbk_filtered < -500)
  {
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
  }

 }
}


/**************************************************************************
*  MC_SixStep_Init_main_data
*
*       Init the main variables for motor driving from MC_SixStep_param.h
*/

void  MC_SixStep_Init_main_data (void)          // WVD - GOES THRU HERE
{
     SIXSTEP_parameters.Ireference    = STARTUP_CURRENT_REFERENCE;
     SIXSTEP_parameters.NUMPOLESPAIRS = NUM_POLE_PAIRS;
     SIXSTEP_parameters.ACCEL         = ACC;
//PS if ( ! VOLTAGE_MODE)
//PS   {
         SIXSTEP_parameters.KP        = KP_GAIN;
         SIXSTEP_parameters.KI        = KI_GAIN;
//PS   }
//PS  else
//PS   {
//PS     SIXSTEP_parameters.KP = KP_GAIN_VM;   
//PS     SIXSTEP_parameters.KI = KI_GAIN_VM;
//PS   }

     SIXSTEP_parameters.CW_CCW        = DIRECTION;
     SIXSTEP_parameters.Potentiometer = POTENTIOMETER;
}


//******************************************************************************
//                                CALLBACK
// MC_TIMx_SixStep_timebase
//
//     COMMUTATE Timer Callback - Call the next step and request the filtered speed value
//                     Invoked by COMMUTATE timer rupt started via Start_Motor()
//******************************************************************************


void  MC_TIMx_SixStep_timebase (void)
{
    MC_SixStep_NEXT_step();             /* Change STEP number  */

    if (SIXSTEP_parameters.ARR_OK == 0)
       {
          MC_SixStep_ARR_step();        /* BASE TIMER - ARR modification for STEP frequency changing */
       }

    MC_Speed_Filter();                  /*Calculate SPEED filtered  */
}


//******************************************************************************
//                             CALLBACK
//
//  MC_SysTick_SixStep_MediumFrequencyTask
//
//                         Systick Callback - Call the Speed loop
//******************************************************************************

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
         MC_StopMotor (0, 0, 0);              // ??? TEMP TEST HACK
         index_motor_run = 0;
         test_motor_run  = 1;
       }
    if (index_motor_run >= DEMO_STOP_TIME && test_motor_run == 1)
       {
         MC_StartMotor (0, 0, 0);             // ??? TEMP TEST HACK
         test_motor_run  = 0;
         index_motor_run = 0;
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
                MC_Set_Speed (0);
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
         MC_StopMotor (0, 0, 0);       // ??? TEMP TEST HACK
         cnt_bemf_event = 0;
//       SIXSTEP_parameters.STATUS = STARTUP_BEMF_FAILURE;
         update_STATUS (STARTUP_BEMF_FAILURE);
       }

    if (speed_fdbk_error == 1)
       {
         MC_StopMotor (0, 0, 0);       // ??? TEMP TEST HACK
//       SIXSTEP_parameters.STATUS = SPEEDFBKERROR;
         update_STATUS (SPEEDFBKERROR);

       }
}



//******************************************************************************
//                                ADC
//
//  MC_ADCx_SixStep_Bemf
//
//                 Compute the zero crossing detection based on ADC readings
//******************************************************************************

void  MC_ADCx_SixStep_Bemf (void)
{
  if (__HAL_TIM_DIRECTION_STATUS(&hPwmMtr))
    {             // initialize COMMUTATION debug pin
     HAL_GPIO_WritePin (COMMUTATE_DBG_GPIO_PORT, COMMUTATE_DBG_GPIN, GPIO_PIN_SET);  // NOT in PS

          // set flag to denote UP-counting direction started
     Upcounting_event_flag = TRUE;               // PS ONLY

          // GET the ADC value (PHASE CURRENT)
      if (SIXSTEP_parameters.STATUS != START && SIXSTEP_parameters.STATUS != ALIGNMENT)
       {
        switch (SIXSTEP_parameters.step_position)
         {
         case 1:
         if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value)
          {
           SIXSTEP_parameters.ADC_BUFFER[3] = HAL_ADC_GetValue (&ADC_HANDLE);
           if (PI_parameters.Reference >= 0)
              {
                if (SIXSTEP_parameters.ADC_BUFFER[3] < SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
                   {
                     MC_SixStep_ARR_Bemf (0);
                   }
              }
             else
              {
                if (SIXSTEP_parameters.ADC_BUFFER[3] > SIXSTEP_parameters.ADC_BEMF_threshold_UP)
                   {
                     MC_SixStep_ARR_Bemf (1);
                     SIXSTEP_parameters.BEMF_Tdown_count = 0;
                   }
              }
          }
         else SIXSTEP_parameters.demagn_counter++;
      break;

      case 2:
       if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value)
       {
        SIXSTEP_parameters.ADC_BUFFER[2] = HAL_ADC_GetValue (&ADC_HANDLE);
        if (PI_parameters.Reference >= 0)
          {
            if (SIXSTEP_parameters.ADC_BUFFER[2] > SIXSTEP_parameters.ADC_BEMF_threshold_UP)
               {
                 MC_SixStep_ARR_Bemf (1);
                 SIXSTEP_parameters.BEMF_Tdown_count = 0;
               }
          }
         else
          {
            if (SIXSTEP_parameters.ADC_BUFFER[2] < SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
               {
                 MC_SixStep_ARR_Bemf (0);
               }
          }
      }
      else SIXSTEP_parameters.demagn_counter++;
      break;

      case 3:
      if (SIXSTEP_parameters.demagn_counter >= SIXSTEP_parameters.demagn_value)
       {
        SIXSTEP_parameters.ADC_BUFFER[1] = HAL_ADC_GetValue(&ADC_HANDLE);
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
        SIXSTEP_parameters.ADC_BUFFER[3] = HAL_ADC_GetValue(&ADC_HANDLE);
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
        SIXSTEP_parameters.ADC_BUFFER[2] = HAL_ADC_GetValue(&ADC_HANDLE);
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
        SIXSTEP_parameters.ADC_BUFFER[1] = HAL_ADC_GetValue(&ADC_HANDLE);
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
        /******************* SET ADC CHANNEL FOR SPEED/CURRENT/VBUS ***********/
        /* Set the channel for next ADC Regular reading */
     MC_SixStep_ADC_Channel (SIXSTEP_parameters.ADC_SEQ_CHANNEL[index_adc_chn]);
        // above table is 4 entries long, consisting of:
        //                              11           9          1          12
        //                          curr_shunt    Speed Ctl  Vbus_Motor   Temp
        /**********************************************************************/
     HAL_GPIO_WritePin (COMMUTATE_DBG_GPIO_PORT, COMMUTATE_DBG_GPIN, GPIO_PIN_RESET);
    }

   else
    {
        /* Down-counting direction started */
     Upcounting_event_flag = FALSE;               // PS ONLY

        // save the latest BEMF channel reading into appropriate U / V / W slot
     SIXSTEP_parameters.ADC_Regular_Buffer[index_adc_chn] = HAL_ADC_GetValue (&ADC_HANDLE);

     if (index_adc_chn == 1)
        {                         // also trace ADC result into HFBuffer
          HFBuffer [HFBufferIndex++] = HAL_ADC_GetValue (&ADC_HANDLE);
          if (HFBufferIndex >= HFBUFFERSIZE)
             {
               HFBufferIndex = 0;
             }
        }
      index_adc_chn++;            // step to next BEMF slot     (max of 3 slots)
      if (index_adc_chn > 3)
         index_adc_chn = 0;
                 //-------------------------------------------------------------
                 // Set the BEMF channel for next ADC Regular reading
                 // setup ADC to sample the BEMF ADC for this Phase (U, V, or W)
                 // It only samples _one_ ADC channel per pass !
                 //     Rotates:   13,  7,  8
                 //-------------------------------------------------------------
      MC_SixStep_ADC_Channel (SIXSTEP_parameters.CurrentRegular_BEMF_ch);
    }

}



/**************************************************************************
*  MC_SixStep_ARR_Bemf
*
*       Calculate the new Autoreload value (ARR) for Low Frequency timer
*  Called via: HAL_ADC_IRQ_Handler -> HAL_ADC_ConvCplt_Callback -> MC_ADCx_SixStep_Bemf
*/

void  MC_SixStep_ARR_Bemf (uint8_t up_bemf)
{

     if (SIXSTEP_parameters.status_prev != SIXSTEP_parameters.step_position)
//PS if (SIXSTEP_parameters.status_prev != SIXSTEP_parameters.step_position && flag_change_step == TRUE)
     {
          // -------------------------------------------------------------------------
          // Flag indicator to prevent Bemf calculation before the end of new commutation step function 
       flag_change_step = FALSE;     // PS ONLY
          // -------------------------------------------------------------------------
       if (SIXSTEP_parameters.SPEED_VALIDATED == TRUE)
         {
           if (GPIO_ZERO_CROSS == 1)
              {                    // toggle pin to denote ZERO CROSS detected
                HAL_GPIO_TogglePin (ZCR_DBG_GPIO_PORT, ZCR_DBG_GPIN);
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
           counter_ARR_Bemf = __HAL_TIM_GetCounter (&hCommuteTmr);
           __HAL_TIM_SetAutoreload (&hCommuteTmr, (counter_ARR_Bemf + ARR_LF/2));
//PS       ARR_tmp = hCommuteTmr.Instance->ARR;
//PS       if (ARR_tmp>=ARR_divider)
//PS          {
//PS            if ((ARR_tmp >= ARR_tmp_prev && (ARR_tmp-ARR_tmp_prev) <= 1000)
//PS             || (ARR_tmp <  ARR_tmp_prev && (ARR_tmp_prev-ARR_tmp) <= 1000))
//PS               {
//PS                 demagn_value_tmp = (ARR_tmp / ARR_divider);
//PS                 ARR_tmp_prev = ARR_tmp;
//PS               }
//PS          }
         }
    }
}


/**************************************************************************
*  MC_Bemf_Delay
*
*     Take the delay time after each new 6-step commutation
*/
void  MC_Bemf_Delay (void)
{
     Bemf_delay_calc();
//PS if (demagn_value_tmp != 0  &&  demagn_value_tmp != 0xFFF)   
//PS    {
//PS      dmg_tmp = MC_Bemf_delay_Filter (demagn_value_tmp);
//PS      if (dmg_tmp <= 2)
//PS         SIXSTEP_parameters.demagn_value = 2;
//PS         else SIXSTEP_parameters.demagn_value = dmg_tmp; 
//PS    }

}



//******************************************************************************
//  MC_SixStep_Ramp_Motor_calc
//
//    Calculate the acceleration profile step by step for motor during start-up
//******************************************************************************

void  MC_SixStep_Ramp_Motor_calc (void)                // WVD - GOES THRU HERE
{
    uint32_t  constant_multiplier   = 100;
    uint32_t  constant_multiplier_2 = 4000000000;
//  uint64_t  mtr_val_sqrt;
    MOTOR_BLOCK *mtr_blk;

    mtr_blk = &motor_blk [0];                          // ??? TEMP TEST HACK

    if (index_startup_motor == 1)
       {
         mech_accel_hz = SIXSTEP_parameters.ACCEL * Rotor_poles_pairs / 60;
         constant_multiplier_tmp = (uint64_t) constant_multiplier * (uint64_t)constant_multiplier_2;
         constant_k    = constant_multiplier_tmp / (3 * mech_accel_hz);
//PS     if ( ! VOLTAGE_MODE)
           _g_mtrdrvr->motor_current_reference_set_value (mtr_blk, SIXSTEP_parameters.Ireference);
//PS       else _g_mtrdrvr->motor_current_reference_set_value (mtr_blk, 
//                                                             SIXSTEP_parameters.pulse_value);
         Time_vector_prev_tmp = 0;
      }
    if (index_startup_motor < NUMBER_OF_STEPS)
     {
//    mtr_val_sqrt    = (uint64_t) MCM_Sqrt ((uint64_t) index_startup_motor * (uint64_t) constant_k) / 632455; // FAILS
//    Time_vector_tmp = ((uint64_t) 1000 * (uint64_t) 1000 * (uint64_t) mtr_val_sqrt);
      Time_vector_tmp = ((uint64_t) 1000 * (uint64_t)1000 * (uint64_t) MCM_Sqrt(((uint64_t)index_startup_motor * (uint64_t)constant_k)))/632455;
      delta = Time_vector_tmp - Time_vector_prev_tmp;
      if (index_startup_motor == 1)
         {
           T_single_step_first_value    = (2 * 3141) * delta / 1000;
           SIXSTEP_parameters.ARR_value = (uint32_t) (65535);
         }
        else
         {
           T_single_step = (2 * 3141) * delta / 1000;
           SIXSTEP_parameters.ARR_value = (uint32_t) (65535 * T_single_step) / (T_single_step_first_value);
         }
     }
    else index_startup_motor = 1;

    if (index_startup_motor == 1)    // WVD DOES THIS first time
       {
         SIXSTEP_parameters.prescaler_value = (((SIXSTEP_parameters.SYSCLK_frequency/1000000) * T_single_step_first_value)/65535) - 1;
       }
    if (SIXSTEP_parameters.STATUS != ALIGNMENT && SIXSTEP_parameters.STATUS != START)
       {
         index_startup_motor++;       // WVD DOES THIS first tuime
       }
      else Time_vector_tmp = 0;

    Time_vector_prev_tmp =  Time_vector_tmp;
}


/**************************************************************************
  * @defgroup   MC_SixStep_ARR_step
  *
  * @brief Generate the ARR value for COMMUTATION / Low Frequency TIM during start-up
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
//            SIXSTEP_parameters.STATUS = STARTUP;
              update_STATUS (STARTUP);

              MC_SixStep_Ramp_Motor_calc();
              if (index_ARR_step < SIXSTEP_parameters.numberofitemArr)
                 {
                   hCommuteTmr.Init.Period = SIXSTEP_parameters.ARR_value;
                   hCommuteTmr.Instance->ARR = (uint32_t) hCommuteTmr.Init.Period;
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
                           MC_StopMotor (0, 0, 0);                  // ??? TEMP TEST HACK
                           SIXSTEP_parameters.STATUS = STARTUP_FAILURE;
                           update_STATUS (STARTUP_FAILURE);
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
//     SIXSTEP_parameters.STATUS = STARTUP;
       update_STATUS (STARTUP);
       MC_SixStep_Ramp_Motor_calc();
       if (index_ARR_step < SIXSTEP_parameters.numberofitemArr)
          {
            hCommuteTmr.Init.Period   = SIXSTEP_parameters.ARR_value;
            hCommuteTmr.Instance->ARR = (uint32_t) hCommuteTmr.Init.Period;
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
                MC_StopMotor (0, 0, 0);                // ??? TEMP TEST HACK
//              SIXSTEP_parameters.STATUS = STARTUP_FAILURE;
                update_STATUS (STARTUP_FAILURE);
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
*  MC_SixStep_TABLE
*
*           Set the peripherals (TIMx, GPIO etc.) used for each step
*
* @param  step_number: step number selected
***************************************************************************/

void  MC_SixStep_TABLE (uint8_t step_number)      // uint8_t motor_id
{
    MOTOR_BLOCK *mtr_blk;

    mtr_blk = &motor_blk [0];                     // ??? TEMP TEST HACK

  if (GPIO_COMM == 1)
     {                  // toggle COMMUTATION debug pin
       HAL_GPIO_TogglePin (COMMUTATE_DBG_GPIO_PORT, COMMUTATE_DBG_GPIN);
     }

      //-------------------------------------------------------------------------------   
      // Waiting for upcounting event of HF timer - step commutation
      //         syncronized with ETR event
  // while(!Upcounting_event_flag && HF_TIMx.Instance->CNT >= 10);    // PS ONLY
      //------------------------------------------------------------------------------  

  switch (step_number)
   {
    case 1:
// PS     MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D (SIXSTEP_parameters.step_position, synchronous_rect, SIXSTEP_parameters.pulse_value);
// PS     SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
                // Update Duty Cycles for each of the PWM phases as active or off
          _g_mtrdrvr->motor_set_bldc_phase_duty_cycles (mtr_blk, SIXSTEP_parameters.pulse_value,
                                                        0, 0);
                // Disable Phase W leg (used for BEMF CH3)
          _g_mtrdrvr->motor_set_bldc_pwm_enables (mtr_blk, PHASE_ENABLE_U_V_DISABLE_W);
                // save which channel is being used for BEMF this pass/step
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
          break;

    case 2:
// PS     MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(SIXSTEP_parameters.step_position,synchronous_rect,SIXSTEP_parameters.pulse_value);   
// PS     SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
          _g_mtrdrvr->motor_set_bldc_phase_duty_cycles (mtr_blk, SIXSTEP_parameters.pulse_value,
                                                        0, 0);
                // Disable Phase V leg (used for BEMF CH2)
          _g_mtrdrvr->motor_set_bldc_pwm_enables (mtr_blk, PHASE_ENABLE_U_W_DISABLE_V);
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
          break;

    case 3:
// PS     MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(SIXSTEP_parameters.step_position,synchronous_rect,SIXSTEP_parameters.pulse_value);           
// PS     SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
          _g_mtrdrvr->motor_set_bldc_phase_duty_cycles (mtr_blk, 0,
                                                        SIXSTEP_parameters.pulse_value, 0);
                // Disable Phase U leg (used for BEMF CH1)
          _g_mtrdrvr->motor_set_bldc_pwm_enables (mtr_blk, PHASE_ENABLE_V_W_DISABLE_U);
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
          break;

    case 4:
// PS     MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(SIXSTEP_parameters.step_position,synchronous_rect,SIXSTEP_parameters.pulse_value);        
// PS     SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
          _g_mtrdrvr->motor_set_bldc_phase_duty_cycles (mtr_blk, 0,
                                                        SIXSTEP_parameters.pulse_value, 0);
                // Disable Phase W leg (used for BEMF CH3)
          _g_mtrdrvr->motor_set_bldc_pwm_enables (mtr_blk, PHASE_ENABLE_U_V_DISABLE_W);
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
     break;

    case 5:
// PS     MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(SIXSTEP_parameters.step_position,synchronous_rect,SIXSTEP_parameters.pulse_value); 
// PS     SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
          _g_mtrdrvr->motor_set_bldc_phase_duty_cycles (mtr_blk, 0,
                                                        0, SIXSTEP_parameters.pulse_value);
                // Disable Phase V leg (used for BEMF CH2)
          _g_mtrdrvr->motor_set_bldc_pwm_enables (mtr_blk, PHASE_ENABLE_U_W_DISABLE_V);
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
          break;

    case 6:
// PS     MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(SIXSTEP_parameters.step_position,synchronous_rect,SIXSTEP_parameters.pulse_value);              
// PS     SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
          _g_mtrdrvr->motor_set_bldc_phase_duty_cycles (mtr_blk, 0,
                                                        0, SIXSTEP_parameters.pulse_value);
                // Disable Phase U leg (used for BEMF CH1)
          _g_mtrdrvr->motor_set_bldc_pwm_enables (mtr_blk, PHASE_ENABLE_V_W_DISABLE_U);
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
          break;
   }

          /* ---------------------------------------------------------------- */    
          /* Flag indicator to prevent Bemf calculation before the end of new commutation step function */    
// PS  flag_change_step = TRUE; 
          /* ---------------------------------------------------------------- */  
}


//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//  MC_SixStep_NEXT_step                                              KEY  KEY
//
//      Generate the next step numberbased upon the motor direction (CW or
//      CCW).
//
//      If motor direction is CW, we increment forward through the table, 
//      if CCW we decrement backwards through the table.
//
// @retval  SIXSTEP_parameters.status
//
//      Periodically kicked by COMMUTATE timer (from Start_Motor)
//******************************************************************************
//******************************************************************************
//******************************************************************************

void  MC_SixStep_NEXT_step()
{
    MOTOR_BLOCK   *mtr_blk;

    mtr_blk = &motor_blk[0];                          // ??? TEMP TEST HACK

    if ( SIXSTEP_parameters.CMD == TRUE)
       {     // we have been requested to startup the Motor's PWM channels/CCRs
         SIXSTEP_parameters.CMD = FALSE;
         _g_mtrdrvr->motor_start_pwm_generation (mtr_blk);
       }

  ARR_LF = __HAL_TIM_GetAutoreload (&hCommuteTmr);

  if (SIXSTEP_parameters.ALIGN_OK == TRUE)
    {
      SIXSTEP_parameters.speed_fdbk     = MC_GetMechSpeedRPM();
      SIXSTEP_parameters.demagn_counter = 1;
      if (SIXSTEP_parameters.status_prev != SIXSTEP_parameters.step_position)
         {
           n_zcr_startup = 0;
         }
        //----------------------------------------------------------------------
        // Have Phase Angle Alignment with Motor. Update to next 6-Step position
        //----------------------------------------------------------------------
      if (PI_parameters.Reference >= 0)
         {
           SIXSTEP_parameters.step_position++;
           if (SIXSTEP_parameters.CL_READY == TRUE)
              {
                SIXSTEP_parameters.VALIDATION_OK = TRUE;
              }
           if (SIXSTEP_parameters.step_position > 6)
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
      update_Step_Position (SIXSTEP_parameters.step_position);   // Trace 6-step
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
          __HAL_TIM_SetAutoreload (&hCommuteTmr,0xFFFF);
        }
    }

  MC_SixStep_TABLE (SIXSTEP_parameters.step_position);

      /* This controls if the changing step request appears during DOWNcounting
      *  in this case it changes the ADC channel */

      /* UP-COUNTING direction started  DIR = 0*/
  if (__HAL_TIM_DIRECTION_STATUS(&hPwmMtr))
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
*  MC_SixStep_Alignment
*
*                         Generate the motor alignment
*/

void  MC_SixStep_Alignment()
{
    SIXSTEP_parameters.step_position = 6;
    update_Step_Position (SIXSTEP_parameters.step_position);   // Trace STEP

    hCommuteTmr.Init.Period   = SIXSTEP_parameters.ARR_value;
    hCommuteTmr.Instance->ARR = (uint32_t) hCommuteTmr.Init.Period;

//  SIXSTEP_parameters.STATUS = ALIGNMENT;
    update_STATUS (ALIGNMENT);

    MC_SixStep_Speed_Val_target_potentiometer();

    index_align++;
    if (index_align >= TIME_FOR_ALIGN+1)
       {
         SIXSTEP_parameters.ALIGN_OK = TRUE;
//       SIXSTEP_parameters.STATUS   = STARTUP;
         update_STATUS (STARTUP);               // 08/11/16 THIS TRIPS ALIGNMENT
         index_startup_motor = 1;

         MC_SixStep_Ramp_Motor_calc();

         hCommuteTmr.Init.Prescaler = SIXSTEP_parameters.prescaler_value;
         hCommuteTmr.Instance->PSC  = hCommuteTmr.Init.Prescaler;
         index_align = 0;
       }
}



/**************************************************************************
  * @defgroup   MC_Set_PI_param
  *
  * @brief Set all parameters for PI regulator
  * @param  PI_PARAM
*/

void  MC_Set_PI_param (SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM)    // WVD - GOES THRU HERE
{
    if (SIXSTEP_parameters.CW_CCW == 0)
       PI_PARAM->Reference = target_speed;
       else PI_PARAM->Reference = -target_speed;

    PI_PARAM->Kp_Gain = SIXSTEP_parameters.KP;
    PI_PARAM->Ki_Gain = SIXSTEP_parameters.KI;

    PI_PARAM->Lower_Limit_Output = LOWER_OUT_LIMIT;
    PI_PARAM->Upper_Limit_Output = UPPER_OUT_LIMIT;
//PS if ( ! VOLTAGE_MODE)
//PS   {
//PS     PI_PARAM->Lower_Limit_Output = LOWER_OUT_LIMIT;       
//PS     PI_PARAM->Upper_Limit_Output = UPPER_OUT_LIMIT;      
//PS   }
//PS  else
//PS   {
//PS     PI_PARAM->Lower_Limit_Output = LOWER_OUT_LIMIT_VM;       
//PS     PI_PARAM->Upper_Limit_Output = UPPER_OUT_LIMIT_VM;     
//PS   }

    PI_PARAM->Max_PID_Output     =  FALSE;
    PI_PARAM->Min_PID_Output     =  FALSE;
}


//******************************************************************************
//  MC_PI_Controller
//
//      Compute the PI output for the Current Reference
//
// @param  PI_PARAM PI parameters structure
// @param  speed_fdb motor_speed_value
// @retval int16_t Current reference
//******************************************************************************

int16_t  MC_PI_Controller (SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM, int16_t speed_fdb)
{
  int32_t  wProportional_Term = 0,  wIntegral_Term = 0,  wOutput_32 = 0;
  int32_t  wIntegral_sum_temp = 0;
  int32_t  Error = 0;

  Error = (PI_PARAM->Reference - speed_fdb);

     //-----------------------------
     // compute Proportional term
     //-----------------------------
  wProportional_Term = PI_PARAM->Kp_Gain * Error;

     //-----------------------------
     //  compute Integral term 
     //-----------------------------
  if (PI_PARAM->Ki_Gain == 0)
     {
       SIXSTEP_parameters.Integral_Term_sum = 0;
     }
    else
     {
       wIntegral_Term     = PI_PARAM->Ki_Gain * Error;
       wIntegral_sum_temp = SIXSTEP_parameters.Integral_Term_sum + wIntegral_Term;
       SIXSTEP_parameters.Integral_Term_sum = wIntegral_sum_temp;
     }

//PS if ( ! VOLTAGE_MODE)
//PS    {
          if (SIXSTEP_parameters.Integral_Term_sum > KI_DIV * PI_PARAM->Upper_Limit_Output)
             SIXSTEP_parameters.Integral_Term_sum  = KI_DIV * PI_PARAM->Upper_Limit_Output;

          if (SIXSTEP_parameters.Integral_Term_sum < -KI_DIV * PI_PARAM->Upper_Limit_Output)
             SIXSTEP_parameters.Integral_Term_sum  = -KI_DIV * PI_PARAM->Upper_Limit_Output;

            /* WARNING: the below instruction is not MISRA compliant, user should verify
            **       that Cortex-M3 assembly instruction ASR (arithmetic shift right)
            **       is used by the compiler to perform the shifts (instead of LSR
            **       logical shift right)*/
          wOutput_32 = (wProportional_Term/KP_DIV) + (SIXSTEP_parameters.Integral_Term_sum/KI_DIV);
//PS    }
//PS   else
//PS    {
//PS      if (SIXSTEP_parameters.Integral_Term_sum> KI_DIV_VM * PI_PARAM->Upper_Limit_Output)
//PS          SIXSTEP_parameters.Integral_Term_sum = KI_DIV_VM* PI_PARAM->Upper_Limit_Output;
//PS
//PS       if (SIXSTEP_parameters.Integral_Term_sum<-KI_DIV_VM* PI_PARAM->Upper_Limit_Output)
//PS          SIXSTEP_parameters.Integral_Term_sum = -KI_DIV_VM* PI_PARAM->Upper_Limit_Output;
//PS
//PS        /* WARNING: the below instruction is not MISRA compliant, user should verify
//PS        **       that Cortex-M3 assembly instruction ASR (arithmetic shift right)
//PS        **       is used by the compiler to perform the shifts (instead of LSR
//PS        **       logical shift right)*/               
//PS      wOutput_32 = (wProportional_Term/KP_DIV_VM) + (SIXSTEP_parameters.Integral_Term_sum/KI_DIV_VM);
//PS    }

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

  return ((int16_t) (wOutput_32));
}



//******************************************************************************
//   MC_Task_Speed
//
//                   Main task: Speed Loop with PI regulator
//******************************************************************************

void  MC_Task_Speed (void)
{
    MOTOR_BLOCK *mtr_blk;

    mtr_blk = &motor_blk [0];                         // ??? TEMP TEST HACK
 
    if (dac_trace_status == TRUE)
       {         // trace this value out to the DAC Debug port
         MotorLib_Set_DAC_Value (SIXSTEP_parameters.speed_fdbk_filtered);
       }

    if ((SIXSTEP_parameters.speed_fdbk_filtered > (target_speed) || SIXSTEP_parameters.speed_fdbk_filtered < (-target_speed)) && SIXSTEP_parameters.VALIDATION_OK !=TRUE)
       {
//       SIXSTEP_parameters.STATUS = VALIDATION;
         update_STATUS (VALIDATION);
         SIXSTEP_parameters.SPEED_VALIDATED = TRUE;
       }

    if (SIXSTEP_parameters.SPEED_VALIDATED == TRUE && SIXSTEP_parameters.BEMF_OK == TRUE && SIXSTEP_parameters.CL_READY != TRUE)
       {
         SIXSTEP_parameters.CL_READY = TRUE;
       }

  if (SIXSTEP_parameters.VALIDATION_OK == TRUE)
    {
          /********************************************************************/
      SIXSTEP_parameters.STATUS = RUN;
      update_STATUS (RUN);

          /********************************************************************/

      if (PI_parameters.Reference >= 0)
         {
           SIXSTEP_parameters.Current_Reference = (uint16_t) MC_PI_Controller (&PI_parameters,(int16_t)SIXSTEP_parameters.speed_fdbk_filtered);
         }
        else
         {
           SIXSTEP_parameters.Current_Reference = (uint16_t) (-MC_PI_Controller(&PI_parameters,(int16_t)SIXSTEP_parameters.speed_fdbk_filtered));
         }

      _g_mtrdrvr->motor_current_reference_set_value (mtr_blk, SIXSTEP_parameters.Current_Reference);
    }

   MC_Bemf_Delay();
}


//******************************************************************************
//  MC_Set_Speed
//
//                     Set the new motor speed value
//******************************************************************************

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



//******************************************************************************
//  MC_Speed_Filter
//
//                     Calculate the speed filtered
//******************************************************************************

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
      for (uint16_t i = 1; i < FILTER_DEEP;i++)
       {
         speed_sum_sp_filt = speed_sum_sp_filt + speed_tmp_array[i];
       }
      SIXSTEP_parameters.speed_fdbk_filtered = speed_sum_sp_filt/(FILTER_DEEP-1);
  }
}


//******************************************************************************
//  MC_SixStep_Speed_Val_target_potentiometer
//
//        Calculate the Motor Speed validation threshold according to the
//        (scaled) potentiometer value
//******************************************************************************

void  MC_SixStep_Speed_Val_target_potentiometer (void)
{
     target_speed = SIXSTEP_parameters.ADC_Regular_Buffer[1] * MAX_POT_SPEED/ 4096;

//PS if (target_speed <= MIN_POT_SPEED)
     if (target_speed < MIN_POT_SPEED)
        target_speed = MIN_POT_SPEED;

//PS if (target_speed >= (MAX_POT_SPEED/VAL_POT_SPEED_DIV)) 
     if (target_speed > (MAX_POT_SPEED/VAL_POT_SPEED_DIV))
        target_speed = (MAX_POT_SPEED / VAL_POT_SPEED_DIV);
}


//******************************************************************************
//  MC_SixStep_Speed_Potentiometer
//
//               Calculate the potentiometer value to set the Motor Speed
//******************************************************************************

void  MC_SixStep_Speed_Potentiometer (void)
{
  uint16_t  i   = 0;
  uint32_t  sum = 0;
  uint16_t  mean = 0;
  uint16_t  max  = 0;

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

  SIXSTEP_parameters.Speed_Ref_filtered = MC_Potentiometer_filter (mean);

}


/**************************************************************************
  * @defgroup   MC_Potentiometer_filter
  *
  * @brief Calculate the filtered potentiometer value
  *
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
     potent_filtered     = speed_sum_pot_filt / (FILTER_DEEP-2);
  }
  if (potent_filtered == 0)
     potent_filtered = 1;
#endif                         // defined(RAN_OUT_OF_ROOM)

 return (potent_filtered);
}



/**************************************************************************
*  MC_GetElSpeedHz
*
*       Get the _Eletrical_ Motor Speed from ARR value of Commutation / LF TIM
*
* @retval int32_t Return the electrical motor speed
*/
int32_t  MC_GetElSpeedHz (void)
{
   uint16_t  prsc;

   if (__HAL_TIM_GetAutoreload(&hCommuteTmr) != 0xFFFF)
      {
        prsc = hCommuteTmr.Instance->PSC;
        El_Speed_Hz = (int32_t)((SIXSTEP_parameters.SYSCLK_frequency)/(prsc)) / (__HAL_TIM_GetAutoreload(&hCommuteTmr)*6);
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
    Mech_Speed_RPM = (int32_t) (MC_GetElSpeedHz() *  60 / Rotor_poles_pairs);
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


//******************************************************************************
//  MC_StartMotor
//
//              Start the Motor
//              called from XZXX via HAL_GPIO_EXTI_Callback ISR on a USER button push
//******************************************************************************
void  MC_StartMotor (uint8_t motor_id, int start_type, int flags) // WVD - GOES THRU HERE ON BUTTON PUSH
{
    MOTOR_BLOCK *mtr_blk;

    mtr_blk = &motor_blk [motor_id];

    if (SIXSTEP_parameters.RUN_Motor == 0 && SIXSTEP_parameters.Button_ready == TRUE)
       ;                  // we can start the motor and no BUTTON bouncing
// ORIG  else return;       // Motor is not ready - BAIL.  IN FUTURE, return ERROR CODE
         else if (no_motor_test == 0)      // 08/11/16 TEST HACK SEQUENCE
                 return;                   // 08/11/16 
                 else ;  // 08/11/16  plunge ahed - force it to go thru motor start sequence

    uwTick = 0;

// SIXSTEP_parameters.STATUS = START;
    update_STATUS (START);

    _g_mtrdrvr->motor_start_handler (mtr_blk, start_type, flags);  // start BLDC motor

    SIXSTEP_parameters.RUN_Motor = 1;   // denote Motor is now in RUN state

    BSP_X_NUCLEO_FAULT_LED_ON();        // TURN ON FAULT LED UNTIL MOTOR STARTED

    if (dac_trace_status == TRUE)
        {
          MotorLib_Start_DAC();         // turn on debugging, out the DAC port
        }
}


//******************************************************************************
//  MC_StopMotor
//
//                           Stop the Motor
//******************************************************************************
void  MC_StopMotor (uint8_t motor_id, int stop_type, int flags)
{
    MOTOR_BLOCK *mtr_blk;

    mtr_blk = &motor_blk [motor_id];

    uwTick   = 0;

//  SIXSTEP_parameters.STATUS    = STOP;
    update_STATUS (STOP);

    SIXSTEP_parameters.RUN_Motor = 0;

    _g_mtrdrvr->motor_stop_pwm_generation (mtr_blk);    // Disable PWM channels

    _g_mtrdrvr->motor_set_bldc_pwm_enables (mtr_blk, PHASE_DISABLE_ALL); // Shut off MOSFETs

    _g_mtrdrvr->motor_stop_handler (mtr_blk, stop_type, flags); // stop BLDC motor

    _g_mtrdrvr->motor_current_reference_start_stop (mtr_blk, STOP_FLAG);

    BSP_X_NUCLEO_FAULT_LED_OFF();

    MC_SixStep_RESET (mtr_blk);   // reset/re-init everything, to prepare for a new Start
}


//*************************************************************************
//                      HAL   STARTUP   CODE
//*************************************************************************

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


//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//  Board Specific ISRs                                       STM32F4_01
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

extern  TIM_HandleTypeDef   hPwmMtr;
extern  TIM_HandleTypeDef   hCommuteTmr;
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
*     TIM4_IRQHandler
*/
void  TIM4_IRQHandler (void)
{                                 // never being called under ny code ! 07/29/16
    HAL_TIM_IRQHandler (&hCommuteTmr);   // Invoked by Timer turned on by Start_Motor
}


/*******************************************************************************
*                     ISR  Handle TIM1 Break interrupt and TIM9 global interrupt
*                                                            TIM9_IRQn
*     TIM1_BRK_TIM9_IRQHandler
*/
void  TIM1_BRK_TIM9_IRQHandler (void)
{
if (no_motor_test == 0)   // ??? if NO motor is powered up, we keep getting Overcurrent errors
    if (__HAL_TIM_GET_FLAG(&hPwmMtr, TIM_FLAG_BREAK) != RESET)
       {
         MC_StopMotor (0, 0, 0);    // ??? TEMP TEST HACK   // hit an error condition - turn off motor
//       SIXSTEP_parameters.STATUS = OVERCURRENT;
         update_STATUS (OVERCURRENT);
       }

    HAL_TIM_IRQHandler (&hPwmMtr);
}


/*****************************************************************************
*  HAL_TIM_PeriodElapsedCallback         CALLBACK
*                                        CALLBACK       BOARD SPECIFIC - F4_01
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


//----------------------------------------------------------------------------
// Update SIXSTEP_parameters.STATUS, and trace it if requested (for debugging)
//----------------------------------------------------------------------------
SIXSTEP_Base_SystStatus_t  prev_status  = -1;
int      trace_STATUS_buf [100];
uint8_t  trace_Step_buf [100];
int      Sidx  = 0;

void  update_STATUS (SIXSTEP_Base_SystStatus_t  new_STATUS)
{
   SIXSTEP_parameters.STATUS = new_STATUS;

   if (new_STATUS != prev_status)
      {      // record the new STATUS state in trace table
        trace_STATUS_buf [Sidx] = (int) new_STATUS;
        trace_Step_buf [Sidx] = SIXSTEP_parameters.step_position; // capture associated 6-step
        Sidx++;
        if (Sidx >= 100)
           Sidx = 0;                 // wrap back to begin of trace buffer
        prev_status = new_STATUS;    // save latest new updated status
      }
}

//------------------------------------------------------------------------------
// Trace the first 100 6-step updates (for debugging)
//------------------------------------------------------------------------------
uint8_t   Trace_6step_buf [100];
int       Pidx  = 0;

void  update_Step_Position (uint8_t step_position)
{
    if (Pidx < 100)
       { Trace_6step_buf [Pidx] = step_position;
         Pidx++;
       }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
