
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                             MotorLib_LL_L6474.c
//
// Motor Control Library:  Low Level Device support for L6474 Stepper Controller
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

#include "MotorLib_Api.h"                       // pull in common definitions

    int   _g_adc_enabled = 0;
    int   isrFlag        = 0;

    SPI_HandleTypeDef       _g_hspi1_handle;    // SPI 1 support

    TIM_HandleTypeDef       hTimPwm1;           // Handle for Motor 1's PWM

#define  TIMER_PRESCALER    1024


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

int  l6474_motor_run (MOTOR_BLOCK *mtr_blk, int direction);
int  l6474_motor_set_direction (MOTOR_BLOCK *mtr_blk, int mtr_direction);
int  l6474_motor_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count);


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

  MOTOR_HANDLERS * L6474_GetMotorHandle(void);

  MOTOR_HANDLERS  l6474_table = {
                                   TYPE_MOTOR_STEPPER,           // Motor Type
                                   MOTOR_DRIVER,                 // Chip Controller id
                                   0L,                           // SPI init
                                   0L,                           // SPI I/O
                                   l6474_motor_speed_adc_init,   // init routine for speed ctl ADC
                                   l6474_motor_adc_next_conversion, // ADC conversion start rtn for speed
                                   0L,                           // motor_read_hall_sensors_handler
                                   0L,                           // motor_fault_shutdown
                                   l6474_motor_adc_ctl_init,     // control ADCs (Current sense)
                                   0L,                           // motor_init_commute_time
                                   l6474_motor_encoder_gpio_init,// init routine for encoder GPIOs
                                   l6474_motor_init_ctl_gpio_inputs, // control input  GPIOs (FAULT)
                                   l6474_motor_init_ctl_gpio_outputs,// control output GPIOs (ENABLE)
                                   l6474_motor_pwm_init,             // motor PWMs

                                   0L,                           // motor_check_status
                                   l6474_motor_do_ctl_action,    // motor_do_action_handler
                                   0L,                           // motor_do_next_commutate
                                   l6474_get_motor_actual_position, // motor_get_actual_position
                                   l6474_get_mark,               // motor_get_mark_pos
                                   l6474_CmdGetStatus,           // motor_get_motor_status
                                   0L,                           // motor_get_period_ticks
                                   l6474_perform_motor_move,     // motor_perform_move
                                   l6474_motor_run,              // motor_run_handler
                                   l6474_motor_set_direction,    // motor_set_direction_handler
                                   l6474_motor_set_pwm_duty_count, // motor_set_pwm_duty_count_handler
                                   0L,                           // motor_set_speed_rpm_handler
                                   l6474_motor_set_step_mode,    // motor_set_step_mode
                                   0L,                           // motor_set_torque
                                   l6474_motor_stop,             // motor_stop_handler
                                   l6474_WaitWhileActive         // motor_wait_complete
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
    
    
    uint32_t   g_CmdGetStatus   = 0;
    uint32_t   g_CmdGetStatus_2 = 0;
//  uint32_t   inactive_state_rupt = 0;

               /// Function pointer to flag interrupt call back
void (*flagInterruptCallback)(void);
               /// Function pointer to error handler call back
void (*errorHandlerCallback)(uint16_t);

static volatile uint8_t  numberOfDevices;

static uint8_t  spiTxBursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
static uint8_t  spiRxBursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];

static volatile bool    spiPreemtionByIsr   = FALSE;
//static volatile bool    isrFlag             = FALSE;
static uint16_t         l6474DriverInstance = 0;

                      //------------------------------------
                      // L6474 Motor Blocks - 1 per motor
                      //------------------------------------
    MOTOR_BLOCK  motor_blk [MAX_NUMBER_OF_DEVICES];


//******************************************************************************
//  Powerstep01_motor_pwm_init
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

int  Powerstep01_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long period_ticks)
{

    GPIO_InitTypeDef         GPIO_InitStruct;
    TIM_OC_InitTypeDef       sConfigOC;
    TIM_MasterConfigTypeDef  sMasterConfig;
    uint32_t                 channel;


     //----------------------------------------------------------------------
     // for now, only worry about Motor 1 (0) PWM  which uses TIM3  Channel 2
     //----------------------------------------------------------------------
     // Other mostors will be in more generalize in my generic STM32 "board support"

    __TIM3_CLK_ENABLE();                  // enable TIM3 Clock

       // configuration PWM channel's GPIO pin for PWM output
    GPIO_InitStruct.Pin       = GPIO_PIN_7;                    (PC7 / D9)
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

    return (0);                  // denote was successful
}



//******************************************************************************
//  Powerstep01_motor_gpio_init_inputs
//
//         Configure control inputs for Motor GPIOs  - FAULT, ...
//          nFault:  Fault = Active Low  J2-3   PE0
//******************************************************************************

int  Powerstep01_motor_init_ctl_gpio_inputs (void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

        // Configure L6474 - FAULT/FLAG pin                        (PA10 / D2)
    GPIO_InitStruct.Pin   = GPIO_PIN_10;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

        /* Set Priority of External Line Interrupt used for the FLAG interrupt*/
    HAL_NVIC_SetPriority (EXTI15_10_IRQn, 5, 0);

        /* Enable the External Line Interrupt used for the FLAG interrupt
    HAL_NVIC_EnableIRQ (EXTI15_10_IRQn);                        // for PA10 EXTI

    return (0);                  // denote was successful
}


//******************************************************************************
//  Powerstep01_motor_init_ctl_gpio_outputs
//
//         Configure control outputs for Motor GPIOs - ENABLE, as well as SPI
//******************************************************************************
int  Powerstep01_motor_init_ctl_gpio_outputs (void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    int               spimode;

        // Configure L6474 - STBY/RESET pin                        (PA9 / D8)
    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
    L6474_Board_Reset (0);

           // Configure L6474 - DIR 1 pin for first Motor (0)      (PA8 / D7)
    GPIO_InitStruct.Pin   = GPIO_PIN_8;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

        //--------------------------------------------------------
        //  setup SPI for access to L6474 - using SPI1   (F4_01)
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
    spimode = 3;                     // L6474 uses SPI Mode 3
    _g_hspi1_handle.Instance             = SPI1;       // set assoc SPI HW module
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
    _g_hspi1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32
    rc = HAL_SPI_Init (&_g_hspi1_handle);  // go intialize the SPI module

    __HAL_SPI_ENABLE (&_g_hspi1_handle);   // turn on SPI Enable flag SPI_CR1_SPE in CR1

 
    return (0);                  // denote was successful
}


//******************************************************************************
//  Powerstep01_motor_encoder_gpio_init                                 ENCODER
//
//         Configure GPIOs for Wheel Encoder pin inputs ...
//              Hall Sensor Left      PA8   Arduino D7    GPIO rupt   Grove D7
//              Hall Sensor Right     PA9   Arduino D8    GPIO rupt   Grove D8
//******************************************************************************
int  Powerstep01_motor_encoder_gpio_init (MOTOR_BLOCK *mtr_blk, int encoder_type)
{

    return (0);                  // denote was successful
}

//******************************************************************************
//  Powerstep01_motor_adc_ctl_init
//
//         Configure ADC to read contropl inputs for current sensors, ...
//             ADC-6  PA6   ADC12_IN6   "A" side Current-Sense   Arduino D12
//             ADC-4  PA4   ADC12_IN4   "B" side Current-Sense   Arduino A2
//******************************************************************************
int  Powerstep01_motor_adc_ctl_init (void)
{
      // see l6206 code for details

    return (0);                  // denote was successful
}


//******************************************************************************
//  Powerstep01_motor_speed_adc_init                                      SPEED
//
//         Configure ADC to read potentiometer used for speed control setting.
//        -     Speed Ctl Pot ADC    J1-2    PB5  AIN11   Grove J3-7/27 -> J1-2
//******************************************************************************
int  Powerstep01_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr)
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

    return (0);                  // denote was successful
}


//******************************************************************************
//  Powerstep01_motor_adc_next_conversion
//
//         start a new ADC conversion for Speed control  (called every 50 ms)
//         Uses A15  on pin  P6.0
//******************************************************************************
int  Powerstep01_motor_adc_next_conversion (void)
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

    return (0);                  // denote was successful
}




//******************************************************************************
//  Powerstep01_motor_run
//
//               Run the motor, i.e. issue a start (or re-start after a
//                              direction change or after a stop).
//
//               Because a stop will slam off the PWMs, we re-establish both the
//               Direction and the Duty Cycle.
//******************************************************************************
int  Powerstep01_motor_run (MOTOR_BLOCK *mtr_blk, int direction)
{
    mtr_blk->mtr_state = MOTOR_STATE_RUN;     // denote we are now in RUN mode

//  TBD - => free running Stepper (no counts or distance, or REJCT as NOT_SUPPORTED ?

    return (0);           // denote was successful
}


//******************************************************************************
//  Powerstep01_motor_set_direction
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
int  Powerstep01_motor_set_direction (MOTOR_BLOCK *mtr_blk,  int direction)
{
    int  pwm_duty_count;

    mtr_blk->mtr_direction = direction;     // ensure desired direction is saved
        
// TBD -= Anything else needed for a STEPPER?

    return (0);                           // do not apply until motor start
}


//******************************************************************************
//  Powerstep01_get_motor_actual_position
//                            gets the absolute position of the motor, from
//                            the contreoller (stepper) or encoder QEI (BLDC)
//******************************************************************************
int32_t Powerstep01_get_motor_actual_position (MOTOR_BLOCK *mtr_blk)
{
    int32_t      raw_position;
    int32_t      abs_position;

    raw_position = L6474_CmdGetParam (mtr_blk, L6474_ABS_POS);

    abs_position = L6474_ConvertPosition (raw_position);

    return (abs_position);
}

//******************************************************************************
//  Powerstep01_motor_do_ctl_action
//                             Issue a command to perform some kind of control
//                       action on the motor (SetHome, SetMark, EnableMotor, ...
//******************************************************************************
int  Powerstep01_motor_do_ctl_action (MOTOR_BLOCK *mtr_blk, int cmd, int flags)
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
//  Powerstep01_perform_motor_move
//                             Issue a command to start the motor moving
//******************************************************************************
            /* Motor activation */
int  Powerstep01_perform_motor_move (MOTOR_BLOCK *mtr_blk, int cmd, int direction, int32_t count)
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
//  Powerstep01_motor_stop                                          motor_stop_handler
//
//                             Stop the motor. The stop_type inidcates the
//                             type of stop (Hard, Soft, Hiz, ...)
//******************************************************************************
int  Powerstep01_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags)
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


/**************************************************************************//**
  * @file    powerstep01.c 
  * @author  IPC Rennes
  * @version V1.1.0
  * @date    November 12, 2014
  * @brief   Powerstep01 motor driver (Microstepping controller with power MOSFETs)
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
  ******************************************************************************
  */

#include "powerstep01.h"

/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup POWERSTEP01
  * @{
  */   

/* Private constants ---------------------------------------------------------*/

/** @addtogroup Powerstep01_Private_Constants
  * @{
  */
/// Error while initialising the SPI
#define POWERSTEP01_ERROR_0   (0x8000)
/// Error: Bad SPI transaction
#define POWERSTEP01_ERROR_1   (0x8001)

/**
  * @}
  */ 

/* Private variables ---------------------------------------------------------*/

/** @addtogroup Powerstep01_Private_Variables
  * @{
  */
  
/// Function pointer to busy interrupt call back
void (*busyInterruptCallback)(void);
/// Function pointer to flag interrupt call back
void (*flagInterruptCallback)(void);
/// Function pointer to error handler call back
void (*errorHandlerCallback)(uint16_t);
static volatile uint8_t numberOfDevices;
static uint8_t spiTxBursts[POWERSTEP01_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
static uint8_t spiRxBursts[POWERSTEP01_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
static volatile bool spiPreemtionByIsr = FALSE;
static volatile bool isrFlag = FALSE;
static uint16_t powerstep01DriverInstance = 0;

/**
  * @}
  */ 

/* Private function prototypes -----------------------------------------------*/

/** @addtogroup Powerstep01_Private_functions
  * @{
  */  
void Powerstep01_BusyInterruptHandler(void);
int32_t Powerstep01_ConvertPosition(uint32_t abs_position_reg);
void Powerstep01_ErrorHandler(uint16_t error);
void Powerstep01_FlagInterruptHandler(void);
void Powerstep01_SendCommand(MOTOR_BLOCK *mtr_blk, uint8_t param, uint32_t value);
void Powerstep01_SetRegisterToPredefinedValues(MOTOR_BLOCK *mtr_blk);
void Powerstep01_WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte);

/**
  * @}
  */ 

/** @defgroup Powerstep01_Exported_Variables
  * @{
  */

/// powerSTEP01 motor driver functions pointer structure
motorDrv_t   powerstep01Drv = 
{
  Powerstep01_Init,
  Powerstep01_ReadId,
  Powerstep01_AttachErrorHandler,
  Powerstep01_AttachFlagInterrupt,
  Powerstep01_AttachBusyInterrupt,
  Powerstep01_FlagInterruptHandler,
  0,
  0,
  0,
  0,
  Powerstep01_GetFwVersion,
  Powerstep01_GetMark,
  0,
  0,
  Powerstep01_GetPosition,
  Powerstep01_CmdGoHome,
  Powerstep01_CmdGoMark,
  Powerstep01_CmdGoTo,
  Powerstep01_CmdHardStop,
  Powerstep01_CmdMove,
  0,
  0,
  0,
  0,
  Powerstep01_SetHome,
  Powerstep01_SetMark,
  0,
  0,
  0,
  0,
  Powerstep01_WaitWhileActive,
  0,
  0,
  Powerstep01_CmdGetParam,
  Powerstep01_CmdGetStatus,
  Powerstep01_CmdNop,
  Powerstep01_CmdSetParam,
  0,
  Powerstep01_ReleaseReset,
  Powerstep01_Reset,
  Powerstep01_SelectStepMode,
  0,
  Powerstep01_CmdGoToDir,
  Powerstep01_CheckBusyHw,
  Powerstep01_CheckStatusHw,
  Powerstep01_CmdGoUntil,
  Powerstep01_CmdHardHiZ,
  Powerstep01_CmdReleaseSw,
  Powerstep01_CmdResetDevice,
  Powerstep01_CmdResetPos,
  Powerstep01_CmdRun,
  Powerstep01_CmdSoftHiZ,
  Powerstep01_CmdStepClock,
  Powerstep01_FetchAndClearAllStatus,
  Powerstep01_GetFetchedStatus,
  Powerstep01_GetNbDevices,
  Powerstep01_IsDeviceBusy,
  Powerstep01_SendQueuedCommands,
  Powerstep01_QueueCommands,
  Powerstep01_WaitForAllDevicesNotBusy,
  Powerstep01_ErrorHandler,
  Powerstep01_BusyInterruptHandler,
  Powerstep01_CmdSoftStop,
  Powerstep01_StartStepClock,
  Powerstep01_StopStepClock
};

/**
  * @}
  */ 

/** @defgroup Powerstep01_library_functions
  * @{
  */   

/******************************************************//**
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error Hanlder
 * @retval None
 **********************************************************/
void  Powerstep01_AttachErrorHandler(void (*callback)(uint16_t))
{
    errorHandlerCallback = (void (*)(uint16_t))callback;
}

/******************************************************//**
 * @brief  Attaches a user callback to the flag Interrupt
 * The call back will be then called each time the status 
 * flag pin will be pulled down due to the occurrence of 
 * a programmed alarms ( OCD, thermal pre-warning or 
 * shutdown, UVLO, wrong command, non-performable command)
 * @param[in] callback Name of the callback to attach 
 * to the Flag Interrupt
 * @retval None
 **********************************************************/
void  Powerstep01_AttachFlagInterrupt (void (*callback)(void))
{
    flagInterruptCallback = (void (*)())callback;
}

/******************************************************//**
 * @brief  Attaches a user callback to the busy Interrupt
 * The call back will be then called each time the busy 
 * pin is set or reset 
 * @param[in] callback Name of the callback to attach 
 * to the Busy Interrupt
 * @retval None
 **********************************************************/
void  Powerstep01_AttachBusyInterrupt(void (*callback)(void))
{
    busyInterruptCallback = (void (*)())callback;
}

/******************************************************//**
 * @brief Read id
 * @param None
 * @retval Id of the powerSTEP01 Driver Instance
 **********************************************************/
uint16_t  Powerstep01_ReadId(void)
{
    return(powerstep01DriverInstance);
}

/******************************************************//**
 * @brief Starts the Powerstep01 library
 * @param[in] nbDevices Number of Powerstep01 devices to use (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval None
 **********************************************************/
void  Powerstep01_Init (uint8_t nbDevices)
{
  uint8_t           loop;  
  numberOfDevices = nbDevices;
  
  powerstep01DriverInstance++;
  
     /* Initialise the GPIOs */
  BSP_MotorControlBoard_GpioInit (nbDevices);
  
  if (BSP_MotorControlBoard_SpiInit() != 0)
     {
          /* Initialization Error */
       Powerstep01_ErrorHandler (POWERSTEP01_ERROR_0);
     } 
  
     /* configure the step clock */
  BSP_MotorControlBoard_StepClockInit();
  
     /* Reset Powerstep01 */
  BSP_MotorControlBoard_Reset();    
  
     /* Let a delay after reset */
  BSP_MotorControlBoard_Delay(500); 
  
     /* Standby-reset deactivation */
  BSP_MotorControlBoard_ReleaseReset();
  
     /* Let a delay after reset */
  BSP_MotorControlBoard_Delay(500); 
  
     // For all devices
  for (loop = 0; loop < numberOfDevices; loop++)
     {
          // Set all registers to their predefined values from powerstep01_target_config.h 
       Powerstep01_SetRegisterToPredefinedValues (loop);
          // Put the Powerstep01 in HiZ state
       Powerstep01_CmdHardHiZ (loop);
     }   
  
  Powerstep01_FetchAndClearAllStatus();  
}


/******************************************************//**
 * @brief Checks if at least one Powerstep01 is busy by checking 
 * busy pin position. 
 * The busy pin is shared between all devices.
 * @param None
 * @retval One if at least one Powerstep01 is busy, otherwise zero
 **********************************************************/
uint8_t  Powerstep01_CheckBusyHw (void)
{
    if ( ! (BSP_MotorControlBoard_BUSY_PIN_GetState()))
       {
         return 0x01;
       }
      else 
       {
         return 0x00;
       }
}

/******************************************************//**
 * @brief Checks if at least one Powerstep01 has an alarm flag set
 * by reading flag pin position.
 * The flag pin is shared between all devices.
 * @param None
 * @retval One if at least one Powerstep01 has an alarm flag set ,
 * otherwise zero
 **********************************************************/
uint8_t  Powerstep01_CheckStatusHw (void)
{
    if ( ! (BSP_MotorControlBoard_FLAG_PIN_GetState()))
       {
         return 0x01;
       }
      else 
       {
         return 0x00;
       }
}

/******************************************************//**
 * @brief Issues PowerStep01 Get Param command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param PowerStep01 register address
 * @retval Register value - 1 to 3 bytes (depends on register)
 *********************************************************/
uint32_t  Powerstep01_CmdGetParam (MOTOR_BLOCK *mtr_blk, uint32_t param)
{
  uint32_t spiRxData = 0;

  if (numberOfDevices > deviceId)
  {
    uint32_t loop;
    uint8_t maxArgumentNbBytes = 0;
    uint8_t spiIndex = numberOfDevices - deviceId - 1;
   
    for (loop = 0; loop < numberOfDevices; loop++)
    {
      spiTxBursts[0][loop] = POWERSTEP01_NOP;
      spiTxBursts[1][loop] = POWERSTEP01_NOP;
      spiTxBursts[2][loop] = POWERSTEP01_NOP;
      spiTxBursts[3][loop] = POWERSTEP01_NOP;
      spiRxBursts[0][loop] = 0;
      spiRxBursts[1][loop] = 0;
      spiRxBursts[2][loop] = 0;
      spiRxBursts[3][loop] = 0;    
    }
    switch (param)
    {
      case POWERSTEP01_ABS_POS: 
      case POWERSTEP01_MARK:
      case POWERSTEP01_SPEED:
        spiTxBursts[0][spiIndex] = ((uint8_t)POWERSTEP01_GET_PARAM )| (param);
        maxArgumentNbBytes = 3;
        break;
      case POWERSTEP01_EL_POS:
      case POWERSTEP01_ACC:
      case POWERSTEP01_DEC:
      case POWERSTEP01_MAX_SPEED:
      case POWERSTEP01_MIN_SPEED:
      case POWERSTEP01_FS_SPD:
      case POWERSTEP01_INT_SPD:
      case POWERSTEP01_CONFIG:
      case POWERSTEP01_GATECFG1:
      case POWERSTEP01_STATUS:
        spiTxBursts[1][spiIndex] = ((uint8_t)POWERSTEP01_GET_PARAM )| (param);
        maxArgumentNbBytes = 2;
        break;
      default:
        spiTxBursts[2][spiIndex] = ((uint8_t)POWERSTEP01_GET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }
      
    for (loop = POWERSTEP01_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
         loop < POWERSTEP01_CMD_ARG_MAX_NB_BYTES;
         loop++)
    {
       Powerstep01_WriteBytes(&spiTxBursts[loop][0],
                             &spiRxBursts[loop][0]);
    }
    
    spiRxData = ((uint32_t)spiRxBursts[1][spiIndex] << 16)|
                 (spiRxBursts[2][spiIndex] << 8) |
                 (spiRxBursts[3][spiIndex]);
  }      
  return (spiRxData);
}

/******************************************************//**
 * @brief Issues PowerStep01 Get Status command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval Status Register content
 *********************************************************/
uint16_t  Powerstep01_CmdGetStatus (MOTOR_BLOCK *mtr_blk)
{
  uint16_t status = 0;
  if (numberOfDevices > deviceId)
  {
    uint32_t loop;
    uint8_t spiIndex = numberOfDevices - deviceId - 1;
  
      for (loop = 0; loop < numberOfDevices; loop++)
      {
         spiTxBursts[0][loop] = POWERSTEP01_NOP;
         spiTxBursts[1][loop] = POWERSTEP01_NOP;
         spiTxBursts[2][loop] = POWERSTEP01_NOP;
         spiTxBursts[3][loop] = POWERSTEP01_NOP;
         spiRxBursts[0][loop] = 0;
         spiRxBursts[1][loop] = 0;
         spiRxBursts[2][loop] = 0;
         spiRxBursts[3][loop] = 0;       
      }
      spiTxBursts[0][spiIndex] = POWERSTEP01_GET_STATUS;
  
    for (loop = 0; loop < POWERSTEP01_CMD_ARG_NB_BYTES_GET_STATUS + POWERSTEP01_RSP_NB_BYTES_GET_STATUS; loop++)
    {
       Powerstep01_WriteBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
    }
    status = (spiRxBursts[1][spiIndex] << 8) | (spiRxBursts[2][spiIndex]);
  }
  return (status);
}

/******************************************************//**
 * @brief Issues PowerStep01 Go Home command (Shortest path to zero position)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void  Powerstep01_CmdGoHome (MOTOR_BLOCK *mtr_blk)
{
     Powerstep01_SendCommand (mtr_blk, POWERSTEP01_GO_HOME, 0);   
}

/******************************************************//**
 * @brief Issues PowerStep01 Go Mark command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void  Powerstep01_CmdGoMark (MOTOR_BLOCK *mtr_blk)
{
    Powerstep01_SendCommand (mtr_blk, POWERSTEP01_GO_MARK, 0);     
}

/******************************************************//**
 * @brief Issues PowerStep01 Go To command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] abs_pos absolute position where requested to move
 * @retval None
 *********************************************************/
void  Powerstep01_CmdGoTo (MOTOR_BLOCK *mtr_blk, int32_t abs_pos)
{
    Powerstep01_SendCommand (mtr_blk, (uint8_t) POWERSTEP01_GO_TO, abs_pos);  
}

/******************************************************//**
 * @brief Issues PowerStep01 Go To Dir command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] direction movement direction
 * @param[in] abs_pos absolute position where requested to move
 * @retval None
 *********************************************************/
void  Powerstep01_CmdGoToDir (MOTOR_BLOCK *mtr_blk, 
                              DIRECTION_t direction,
                              int32_t abs_pos)
{
  Powerstep01_SendCommand (mtr_blk, 
                           (uint8_t)POWERSTEP01_GO_TO_DIR | 
                           (uint8_t)direction, abs_pos);  
}

/******************************************************//**
 * @brief Issues PowerStep01 Go Until command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] action ACTION_RESET or ACTION_COPY
 * @param[in] direction movement direction
 * @param[in] speed
 * @retval None
 *********************************************************/
void Powerstep01_CmdGoUntil (MOTOR_BLOCK *mtr_blk, 
                             motorAction_t action, 
                             DIRECTION_t direction, 
                             uint32_t speed)
{
  Powerstep01_SendCommand (mtr_blk, (uint8_t) POWERSTEP01_GO_UNTIL | (uint8_t)action | (uint8_t)direction, speed); 
}

/******************************************************//**
 * @brief Issues PowerStep01 Hard HiZ command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void  Powerstep01_CmdHardHiZ (MOTOR_BLOCK *mtr_blk)
{
    Powerstep01_SendCommand (mtr_blk, POWERSTEP01_HARD_HIZ, 0);    
}

/******************************************************//**
 * @brief Issues PowerStep01 Hard Stop command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void   Powerstep01_CmdHardStop (MOTOR_BLOCK *mtr_blk)
{
    Powerstep01_SendCommand (mtr_blk, POWERSTEP01_HARD_STOP, 0);         
}

/******************************************************//**
 * @brief Issues PowerStep01 Move command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] direction Movement direction
 * @param[in] n_step number of steps
 * @retval None
 *********************************************************/
void Powerstep01_CmdMove (MOTOR_BLOCK *mtr_blk, 
                          DIRECTION_t direction, 
                          uint32_t n_step)
{
  Powerstep01_SendCommand (mtr_blk, 
                           (uint8_t) POWERSTEP01_MOVE | 
                           (uint8_t) direction,
                           n_step);  
}

/******************************************************//**
 * @brief Issues PowerStep01 NOP command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void  Powerstep01_CmdNop MOTOR_BLOCK *mtr_blk)
{
       /* Send NOP operation code to PowerStep01 */
    Powerstep01_SendCommand (mtr_blk, POWERSTEP01_NOP, 0);
}

/******************************************************//**
 * @brief Issues PowerStep01 Release SW command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] action
 * @param[in] direction movement direction
 * @retval None
 *********************************************************/
void  Powerstep01_CmdReleaseSw (MOTOR_BLOCK *mtr_blk, 
                                motorAction_t action, 
                                DIRECTION_t    direction)
{
   Powerstep01_SendCommand (mtr_blk, 
                            (uint8_t) POWERSTEP01_RELEASE_SW | 
                            (uint8_t) action | 
                            (uint8_t) direction, 
                            0); 
}

/******************************************************//**
 * @brief Issues PowerStep01 Reset Device command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void   Powerstep01_CmdResetDevice (MOTOR_BLOCK *mtr_blk)
{
    Powerstep01_SendCommand (mtr_blk, POWERSTEP01_RESET_DEVICE, 0);         
}

/******************************************************//**
 * @brief Issues PowerStep01 Reset Pos command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void  Powerstep01_CmdResetPos (MOTOR_BLOCK *mtr_blk)
{
    Powerstep01_SendCommand (mtr_blk, POWERSTEP01_RESET_POS, 0);       
}

/******************************************************//**
 * @brief Issues PowerStep01 Run command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] direction Movement direction (FORWARD, BACKWARD)
 * @param[in] speed in steps/s
 * @retval None
 *********************************************************/
void  Powerstep01_CmdRun (MOTOR_BLOCK *mtr_blk, 
                          DIRECTION_t direction,
                          uint32_t speed)
{
  Powerstep01_SendCommand (mtr_blk, 
                           (uint8_t)POWERSTEP01_RUN | 
                           (uint8_t)direction,
                           speed);
}

/******************************************************//**
 * @brief Issues the SetParam command to the PowerStep01 of the specified motor
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Register adress (POWERSTEP01_ABS_POS, POWERSTEP01_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 *********************************************************/
void  Powerstep01_CmdSetParam (MOTOR_BLOCK *mtr_blk, 
                               uint32_t param, uint32_t value)
{

  if (numberOfDevices > deviceId)
  {
    uint32_t loop;
    uint8_t maxArgumentNbBytes = 0;
    uint8_t spiIndex = numberOfDevices - deviceId - 1;
  
    for (loop = 0;loop < numberOfDevices; loop++)
    {
      spiTxBursts[0][loop] = POWERSTEP01_NOP;
      spiTxBursts[1][loop] = POWERSTEP01_NOP;
      spiTxBursts[2][loop] = POWERSTEP01_NOP;
      spiTxBursts[3][loop] = POWERSTEP01_NOP;
    }
    
    switch (param)
    {
      case POWERSTEP01_ABS_POS: ;
      case POWERSTEP01_MARK:
        spiTxBursts[0][spiIndex] = ((uint8_t)POWERSTEP01_SET_PARAM )| (param);
        spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 3;
        break;
      case POWERSTEP01_EL_POS:
      case POWERSTEP01_ACC:
      case POWERSTEP01_DEC:
      case POWERSTEP01_MAX_SPEED:
      case POWERSTEP01_MIN_SPEED:
      case POWERSTEP01_FS_SPD:
      case POWERSTEP01_INT_SPD:
      case POWERSTEP01_CONFIG:
      case POWERSTEP01_GATECFG1:
        spiTxBursts[1][spiIndex] = ((uint8_t)POWERSTEP01_SET_PARAM )| (param);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 2;
        break;
      default:
        spiTxBursts[2][spiIndex] = ((uint8_t)POWERSTEP01_SET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }
    spiTxBursts[3][spiIndex] = (uint8_t)(value);
  
    /* SPI transfer */
    for (loop = POWERSTEP01_CMD_ARG_MAX_NB_BYTES - 1 - maxArgumentNbBytes;
         loop < POWERSTEP01_CMD_ARG_MAX_NB_BYTES;
         loop++)
    {
      Powerstep01_WriteBytes(&spiTxBursts[loop][0],&spiRxBursts[loop][0]);
    }
  }
}


/******************************************************//**
 * @brief Issues PowerStep01 Soft HiZ command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void  Powerstep01_CmdSoftHiZ (MOTOR_BLOCK *mtr_blk)
{
    Powerstep01_SendCommand (mtr_blk, POWERSTEP01_SOFT_HIZ, 0);           
}

/******************************************************//**
 * @brief Issues PowerStep01 Soft Stop command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void   Powerstep01_CmdSoftStop (MOTOR_BLOCK *mtr_blk)
{
    Powerstep01_SendCommand (mtr_blk, POWERSTEP01_SOFT_STOP, 0);         
}

/******************************************************//**
 * @brief Issues PowerStep01 Step Clock command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] direction Movement direction (FORWARD, BACKWARD)
 * @retval None
 *********************************************************/
void  Powerstep01_CmdStepClock (MOTOR_BLOCK *mtr_blk,
                                DIRECTION_t direction)
{
    Powerstep01_SendCommand (mtr_blk, 
                             (uint8_t) POWERSTEP01_STEP_CLOCK | (uint8_t)direction,
                             0);  
}

/******************************************************//**
 * @brief Fetch and clear status flags of all devices 
 * by issuing a GET_STATUS command simultaneously  
 * to all devices.
 * Then, the fetched status of each device can be retrieved
 * by using the Powerstep01_GetFetchedStatus function
 * provided there is no other calls to functions which 
 * use the SPI in between.
 * @param None
 * @retval None
 *********************************************************/
void  Powerstep01_FetchAndClearAllStatus (void)
{
  uint8_t loop;

  for (loop = 0; loop < numberOfDevices; loop++)
  {
     spiTxBursts[0][loop] = POWERSTEP01_GET_STATUS;
     spiTxBursts[1][loop] = POWERSTEP01_NOP;
     spiTxBursts[2][loop] = POWERSTEP01_NOP;
     spiTxBursts[3][loop] = POWERSTEP01_NOP;
     spiRxBursts[0][loop] = 0;
     spiRxBursts[1][loop] = 0;
     spiRxBursts[2][loop] = 0;
     spiRxBursts[3][loop] = 0;
  }
  for (loop = 0; 
       loop < POWERSTEP01_CMD_ARG_NB_BYTES_GET_STATUS + 
              POWERSTEP01_RSP_NB_BYTES_GET_STATUS; 
       loop++)
  {
     Powerstep01_WriteBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
  }
}

/******************************************************//**
 * @brief Get the value of the STATUS register which was 
 * fetched by using Powerstep01_FetchAndClearAllStatus.
 * The fetched values are available  as long as there
 * no other calls to functions which use the SPI.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval Last fetched value of the STATUS register
 *********************************************************/
uint16_t  Powerstep01_GetFetchedStatus (MOTOR_BLOCK *mtr_blk)
{
    uint16_t status = 0;

    if (numberOfDevices > deviceId)
       {
         uint8_t spiIndex = numberOfDevices - deviceId - 1;
         status = (spiRxBursts[1][spiIndex] << 8) | (spiRxBursts[2][spiIndex]);
       }
    return (status);
}

/******************************************************//**
 * @brief Return motor handle (pointer to the powerSTEP01 motor driver structure)
 * @param None
 * @retval Pointer to the motorDrv_t structure
 **********************************************************/
motorDrv_t  * Powerstep01_GetMotorHandle (void)
{
    return (&powerstep01Drv);
}
       
/******************************************************//**
 * @brief Returns the FW version of the library
 * @param None
 * @retval POWERSTEP01_FW_VERSION
 **********************************************************/
uint8_t  Powerstep01_GetFwVersion (void)
{
    return (POWERSTEP01_FW_VERSION);
}

/******************************************************//**
 * @brief  Returns the mark position  of the specified motor
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval Mark register value converted in a 32b signed integer 
 **********************************************************/
int32_t  Powerstep01_GetMark (MOTOR_BLOCK *mtr_blk)
{
    return Powerstep01_ConvertPosition(Powerstep01_CmdGetParam(mtr_blk, POWERSTEP01_MARK));
}

/******************************************************//**
 * @brief Return the number of devices in the daisy chain 
 * @param None
 * @retval number of devices from 1 to MAX_NUMBER_OF_DEVICES
 *********************************************************/
uint8_t  Powerstep01_GetNbDevices (void)
{
    return (numberOfDevices);
}

/******************************************************//**
 * @brief  Returns the ABS_POSITION of the specified motor
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval ABS_POSITION register value converted in a 32b signed integer
 **********************************************************/
int32_t  Powerstep01_GetPosition (MOTOR_BLOCK *mtr_blk)
{
    return Powerstep01_ConvertPosition(Powerstep01_CmdGetParam(mtr_blk, POWERSTEP01_ABS_POS));
}

/******************************************************//**
 * @brief Checks if the specified motor is busy
 * by reading the Busy flag bit ot its status Register
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval true if device is busy, false zero
 *********************************************************/
bool  Powerstep01_IsDeviceBusy (MOTOR_BLOCK *mtr_blk)
{
  if ( ! (Powerstep01_CmdGetStatus(mtr_blk) & POWERSTEP01_STATUS_BUSY)) 
     {
       return TRUE;
     }
    else 
     {
       return FALSE;
     }
}
  
/******************************************************//**
 * @brief Put commands in queue before synchronous sending
 * done by calling Powerstep01_SendQueuedCommands.
 * Any call to functions that use the SPI between the calls of 
 * Powerstep01_QueueCommands and Powerstep01_SendQueuedCommands 
 * will corrupt the queue.
 * A command for each device of the daisy chain must be 
 * specified before calling Powerstep01_SendQueuedCommands.
 * @param[in] deviceId deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Command to queue (all Powerstep01 commmands 
 * except POWERSTEP01_SET_PARAM, POWERSTEP01_GET_PARAM, 
 * POWERSTEP01_GET_STATUS)
 * @param[in] value argument of the command to queue
 * @retval None
 *********************************************************/
void  Powerstep01_QueueCommands (MOTOR_BLOCK *mtr_blk, uint8_t param, int32_t value)
{
  if (numberOfDevices > deviceId)
  {
    uint8_t spiIndex = numberOfDevices - deviceId - 1;
    
    switch (param & DAISY_CHAIN_COMMAND_MASK)
    {
      case POWERSTEP01_RUN: ;
      case POWERSTEP01_MOVE: ;
      case POWERSTEP01_GO_TO: ;
      case POWERSTEP01_GO_TO_DIR: ;
      case POWERSTEP01_GO_UNTIL: ;
      case POWERSTEP01_GO_UNTIL_ACT_CPY:
             spiTxBursts[0][spiIndex] = param;
             spiTxBursts[1][spiIndex] = (uint8_t) (value >> 16);
             spiTxBursts[2][spiIndex] = (uint8_t) (value >> 8);
             spiTxBursts[3][spiIndex] = (uint8_t) (value);
             break;

      default:
             spiTxBursts[0][spiIndex] = POWERSTEP01_NOP;
             spiTxBursts[1][spiIndex] = POWERSTEP01_NOP;
             spiTxBursts[2][spiIndex] = POWERSTEP01_NOP;
             spiTxBursts[3][spiIndex] = param;
    }
  }
}


/******************************************************//**
 * @brief  Set the stepping mode 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] stepMode from full step to 1/128 microstep as specified in enum STEPPER_MODE_t
 * @retval None
 **********************************************************/
void  Powerstep01_SelectStepMode (MOTOR_BLOCK *mtr_blk, STEPPER_MODE_t stepMode)
{
  uint8_t                stepModeRegister;
  powerstep01_StepSel_t  powerstep01StepMode;

  switch (stepMode)
  {
    case STEP_MODE_FULL:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1;
      break;
    case STEP_MODE_HALF:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_2;
      break;    
    case STEP_MODE_1_4:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_4;
      break;        
    case STEP_MODE_1_8:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_8;
      break;
    case STEP_MODE_1_16:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_16;
      break;        
    case STEP_MODE_1_32:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_32;
      break;
    case STEP_MODE_1_64:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_64;
      break;
    case STEP_MODE_1_128:
    default:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_128;
      break;       
  }
  
     /* Set the powerstep01 in HiZ state */
  Powerstep01_CmdHardHiZ (mtr_blk);  
  
     /* Read Step mode register and clear STEP_SEL field */
  stepModeRegister = (uint8_t) (0xF8 & Powerstep01_CmdGetParam(mtr_blk,POWERSTEP01_STEP_MODE)) ;
  
     /* Apply new step mode */
  Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STEP_MODE, stepModeRegister | (uint8_t)powerstep01StepMode);

     /* Reset abs pos register */
  Powerstep01_CmdResetPos (mtr_blk);
}

/******************************************************//**
 * @brief Sends commands stored previously in the queue by 
 * Powerstep01_QueueCommands
 * @param None
 * @retval None
 *********************************************************/
void  Powerstep01_SendQueuedCommands (void)
{
  uint8_t loop;
  
  for (loop = 0; 
       loop < POWERSTEP01_CMD_ARG_MAX_NB_BYTES; 
       loop++)
     {
       Powerstep01_WriteBytes (&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
     }
}

/******************************************************//**
 * @brief  Sets current position to be the Home position (ABS pos set to 0)
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void Powerstep01_SetHome (MOTOR_BLOCK *mtr_blk)
{
    Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ABS_POS, 0);
}

/******************************************************//**
 * @brief  Sets current position to be the Mark position 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void  Powerstep01_SetMark (MOTOR_BLOCK *mtr_blk)
{
    uint32_t mark = Powerstep01_CmdGetParam (mtr_blk, POWERSTEP01_ABS_POS);
    Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MARK, mark);
}

/******************************************************//**
 * @brief  Locks until the device becomes not busy
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 **********************************************************/
void  Powerstep01_WaitWhileActive (MOTOR_BLOCK *mtr_blk)
{
         /* Wait while motor is running */
     while (Powerstep01_IsDeviceBusy(mtr_blk) != 0)
       ;
}


/******************************************************//**
 * @brief  Locks until all devices become not busy
 * @param None
 * @retval None
 **********************************************************/
void  Powerstep01_WaitForAllDevicesNotBusy (void)
{
    bool busy = TRUE;
    uint8_t loop;
    
    /* Wait while at least one is active */
    while (busy)
  {
    busy = FALSE;
    for (loop = 0; loop < numberOfDevices; loop++)
    {
      busy  |= Powerstep01_IsDeviceBusy(loop);
    }   
  }
}

/******************************************************//**
 * @brief  Handlers of the busy interrupt which calls the user callback (if defined)
 * @param None
 * @retval None
 **********************************************************/
void  Powerstep01_BusyInterruptHandler (void)
{
  if (busyInterruptCallback != 0)
  {
    /* Set isr flag */
    isrFlag = TRUE;
    
    busyInterruptCallback();
    
    /* Reset isr flag */
    isrFlag = FALSE;   
  }
}

/******************************************************//**
 * @brief  Start the step clock by using the given frequency
 * @param[in] newFreq in Hz of the step clock
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void  Powerstep01_StartStepClock (uint16_t newFreq)
{
    BSP_MotorControlBoard_StartStepClock (newFreq);
}

/******************************************************//**
 * @brief  Stops the PWM uses for the step clock
 * @param  None
 * @retval None
 **********************************************************/
void  Powerstep01_StopStepClock (void)
{
    BSP_MotorControlBoard_StopStepClock();
}

/******************************************************//**
 * @brief  Converts the ABS_POSITION register value to a 32b signed integer
 * @param[in] abs_position_reg value of the ABS_POSITION register
 * @retval operation_result 32b signed integer corresponding to the absolute position 
 **********************************************************/
int32_t  Powerstep01_ConvertPosition (uint32_t abs_position_reg)
{
    int32_t operation_result;

     if (abs_position_reg & POWERSTEP01_ABS_POS_SIGN_BIT_MASK) 
       {
            /* Negative register value */
        abs_position_reg  = ~abs_position_reg;
        abs_position_reg += 1;

        operation_result = (int32_t) (abs_position_reg & POWERSTEP01_ABS_POS_VALUE_MASK);
        operation_result = -operation_result;
      } 
     else 
      {
        operation_result = (int32_t) abs_position_reg;
      }
    return operation_result;
}

/******************************************************//**
 * @brief  Releases the Powerstep01 reset (pin set to High) of all devices
 * @param  None
 * @retval None
 **********************************************************/
void  Powerstep01_ReleaseReset (void)
{ 
    BSP_MotorControlBoard_ReleaseReset(); 
}

/******************************************************//**
 * @brief  Resets the Powerstep01 (reset pin set to low) of all devices
 * @param  None
 * @retval None
 **********************************************************/
void  Powerstep01_Reset (void)
{
    BSP_MotorControlBoard_Reset(); 
}

/******************************************************//**
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void  Powerstep01_ErrorHandler (uint16_t error)
{
  if (errorHandlerCallback != 0)
  {
    errorHandlerCallback(error);
  }
  else   
  {
    while(1)
    {
      /* Infinite loop */
    }
  }
}

/******************************************************//**
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @param None
 * @retval None
 **********************************************************/
void  Powerstep01_FlagInterruptHandler (void)
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
                          
/******************************************************//**
 * @brief  Sends a command to a given device Id via the SPI
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Command to send (all Powerstep01 commmands 
 * except POWERSTEP01_SET_PARAM, POWERSTEP01_GET_PARAM, 
 * POWERSTEP01_GET_STATUS)
 * @param[in] value arguments to send on 32 bits
 * @retval None
 **********************************************************/
void  Powerstep01_SendCommand (MOTOR_BLOCK *mtr_blk, uint8_t param, uint32_t value)
{
  if (numberOfDevices > deviceId)
  {
    uint32_t loop;
    uint8_t maxArgumentNbBytes = 0;
    uint8_t spiIndex = numberOfDevices - deviceId - 1;
    
    for (loop = 0; loop < numberOfDevices; loop++)
       {
        spiTxBursts[0][loop] = POWERSTEP01_NOP;
        spiTxBursts[1][loop] = POWERSTEP01_NOP;
        spiTxBursts[2][loop] = POWERSTEP01_NOP;
        spiTxBursts[3][loop] = POWERSTEP01_NOP;   
       }

    switch (param & DAISY_CHAIN_COMMAND_MASK)
    {
      case POWERSTEP01_GO_TO:
      case POWERSTEP01_GO_TO_DIR:
               value = value & POWERSTEP01_ABS_POS_VALUE_MASK;
      case POWERSTEP01_RUN:
      case POWERSTEP01_MOVE:
      case POWERSTEP01_GO_UNTIL:
      case POWERSTEP01_GO_UNTIL_ACT_CPY:
               spiTxBursts[0][spiIndex] = param;
               spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
               spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
               spiTxBursts[3][spiIndex] = (uint8_t)(value);
               maxArgumentNbBytes = 3;
               break;
      default:
               spiTxBursts[0][spiIndex] = POWERSTEP01_NOP;
               spiTxBursts[1][spiIndex] = POWERSTEP01_NOP;
               spiTxBursts[2][spiIndex] = POWERSTEP01_NOP;
               spiTxBursts[3][spiIndex] = param;
     }

    for (loop = POWERSTEP01_CMD_ARG_MAX_NB_BYTES - 1 - maxArgumentNbBytes; 
         loop < POWERSTEP01_CMD_ARG_MAX_NB_BYTES; 
         loop++)
      {
        Powerstep01_WriteBytes (&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
      }    
  }
}


/******************************************************//**
 * @brief  Sets the registers of the Powerstep01 to their predefined values 
 * from powerstep01_target_config.h
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void  Powerstep01_SetRegisterToPredefinedValues (MOTOR_BLOCK *mtr_blk)
{  
  powerstep01_CmVm_t  cmVm;
  
  Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ABS_POS, 0);
  Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_EL_POS, 0);
  Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MARK, 0);
  
  switch (mtr_blk->mtr_id)
  {
    case 0:
      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_0;
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_0));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_0));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_0));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_0|
                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_0));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_0|
                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_0));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_0)); //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_0));       //voltage mode only but not redefined for current mode 
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_0);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_0));      //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_0 |
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_0|
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_0);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_0);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_0 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_0   | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_0|
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_0);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_0 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_0);
      
      // Voltage mode
      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
      {
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_0));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_0));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_0));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_0));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_0));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_0));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_0));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_0 | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_0       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_0       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_0         | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_0       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_0        | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_0       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_0);  
      }
      else
      {
        // Current mode
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_0));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_0));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_0));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_0));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_0 |
                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_0);
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_0));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_0));
        
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_0 | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_0       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_0        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_0         | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_0       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_0        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_0           |
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_0);          
      }
      break;
   case 1:
      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_1;
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_1));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_1));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_1));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_1|
                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_1));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_1|
                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_1));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_1)); //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_1));       //voltage mode only but not redefined for current mode 
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_1);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_1));      //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_1 |
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_1|
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_1);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_1);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_1 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_1   | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_1|
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_1);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_1 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_1);
      
      // Voltage mode
      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
      {
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_1));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_1));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_1));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_1));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_1));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_1));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_1));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_1 | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_1       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_1       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_1         | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_1       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_1        | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_1       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_1);  
      }
      else
      {
        // Current mode
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_1));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_1));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_1));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_1));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_1 |
                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_1);
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_1));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_1));
        
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_1 | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_1       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_1        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_1         | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_1       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_1        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_1           |
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_1);          
      }
      break;      
   case 2:
      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_2;
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_2));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_2));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_2));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_2|
                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_2));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_2|
                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_2));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_2)); //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_2));       //voltage mode only but not redefined for current mode 
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_2);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_2));      //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_2 |
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_2|
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_2);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_2);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_2 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_2   | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_2|
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_2);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_2 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_2);
      
      // Voltage mode
      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
      {
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_2));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_2));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_2));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_2));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_2));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_2));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_2));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_2 | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_2       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_2       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_2         | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_2       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_2        | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_2       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_2);  
      }
      else
      {
        // Current mode
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_2));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_2));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_2));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_2));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_2 |
                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_2);
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_2));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_2));
        
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_2 | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_2       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_2        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_2         | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_2       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_2        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_2           |
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_2);          
      }
      break;      
   case 3:
      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_3;
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_3));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_3));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_3));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_3|
                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_3));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_3|
                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_3));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_3)); //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_3));       //voltage mode only but not redefined for current mode 
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_3);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_3));      //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_3 |
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_3|
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_3);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_3);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_3 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_3   | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_3|
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_3);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_3 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_3);
      
      // Voltage mode
      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
      {
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_3));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_3));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_3));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_3));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_3));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_3));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_3));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_3 | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_3       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_3       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_3         | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_3       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_3        | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_3       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_3);  
      }
      else
      {
        // Current mode
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_3));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_3));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_3));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_3));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_3 |
                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_3);
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_3));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_3));
        
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_3 | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_3       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_3        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_3         | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_3       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_3        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_3           |
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_3);          
      }
      break;  
   case 4:
      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_4;
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_4));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_4));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_4));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_4|
                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_4));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_4|
                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_4));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_4)); //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_4));       //voltage mode only but not redefined for current mode 
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_4);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_4));      //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_4 |
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_4|
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_4);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_4);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_4 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_4   | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_4|
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_4);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_4 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_4);
      
      // Voltage mode
      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
      {
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_4));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_4));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_4));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_4));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_4));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_4));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_4));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_4 | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_4       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_4       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_4         | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_4       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_4        | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_4       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_4);  
      }
      else
      {
        // Current mode
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_4));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_4));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_4));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_4));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_4 |
                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_4);
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_4));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_4));
        
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_4 | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_4       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_4        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_4         | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_4       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_4        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_4           |
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_4);          
      }
      break;      
   case 5:
      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_5;
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_5));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_5));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_5));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_5|
                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_5));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_5|
                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_5));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_5)); //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_5));       //voltage mode only but not redefined for current mode 
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_5);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_5));      //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_5 |
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_5|
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_5);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_5);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_5 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_5   | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_5|
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_5);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_5 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_5);
      
      // Voltage mode
      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
      {
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_5));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_5));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_5));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_5));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_5));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_5));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_5));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_5 | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_5       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_5       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_5         | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_5       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_5        | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_5       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_5);  
      }
      else
      {
        // Current mode
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_5));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_5));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_5));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_5));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_5 |
                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_5);
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_5));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_5));
        
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_5 | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_5       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_5        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_5         | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_5       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_5        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_5           |
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_5);          
      }
      break;      
   case 6:
      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_6;
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_6));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_6));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_6));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_6|
                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_6));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_6|
                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_6));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_6)); //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_6));       //voltage mode only but not redefined for current mode 
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_6);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_6));      //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_6 |
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_6|
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_6);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_6);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_6 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_6   | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_6|
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_6);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_6 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_6);
      
      // Voltage mode
      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
      {
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_6));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_6));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_6));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_6));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_6));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_6));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_6));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_6 | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_6       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_6       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_6         | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_6       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_6        | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_6       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_6);  
      }
      else
      {
        // Current mode
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_6));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_6));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_6));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_6));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_6 |
                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_6);
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_6));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_6));
        
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_6 | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_6       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_6        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_6         | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_6       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_6        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_6           |
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_6);          
      }
      break;      
   case 7:
   default:
      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_7;
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_7));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_7));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_7));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_7|
                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_7));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_7|
                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_7));
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_7)); //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_7));       //voltage mode only but not redefined for current mode 
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_7);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_7));      //voltage mode only but not redefined for current mode
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_7 |
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_7|
                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_7);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_7);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_7 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_7   | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_7|
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_7);
      Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_7 | 
                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_7);
      
      // Voltage mode
      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
      {
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_7));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_7));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_7));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_7));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_7));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_7));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_7));  
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_7 | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_7       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_7       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_7         | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_7       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_7        | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_7       | 
                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_7);  
      }
      else
      {
        // Current mode
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_7));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_7));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_7));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_7));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_7 |
                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_7);
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_7));
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_7));
        
        Powerstep01_CmdSetParam (mtr_blk, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_7 | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_7       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_7        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_7         | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_7       | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_7        | 
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_7           |
                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_7);          
      }
      break;      
  }
}

/******************************************************//**
 * @brief Write and receive a byte via SPI
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte pointer to the received byte
 * @retval None
 *********************************************************/
void  Powerstep01_WriteBytes (uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
{
    if (BSP_MotorControlBoard_SpiWriteBytes(pByteToTransmit, pReceivedByte, numberOfDevices) != 0)
       {
         Powerstep01_ErrorHandler (POWERSTEP01_ERROR_1);
       }
  
    if (isrFlag)
       {
         spiPreemtionByIsr = TRUE;
       }
}                     

#endif                         // defined(powerSTEP_LATER)




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
            Powerstep01_StepClockHandler (0);
          }
     }
  if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
     {     // process motor 2
       mtr_blk = &motor_blk[1];
       if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
          {
            Powerstep01_StepClockHandler (1);
          }
     }

  if ((htim->Instance == TIM4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3))
     {     // process motor 3
       mtr_blk = &motor_blk[2];
       HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_10);
       if ((mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
         && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET))
          {
            Powerstep01_StepClockHandler (2);
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
       Powerstep01_FlagInterruptHandler();
     }
 }

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
