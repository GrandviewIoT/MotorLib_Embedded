
   int  no_motor_test = 0;   // ??? 1 = run thru code with motor off/un-attached

//*******1*********2*********3*********4*********5*********6*********7**********
//
//                           MotorLib_LL_L6389_STM32.c
//
// Motor Control Library:  Low Level Device support for L6389 BLDC Controller
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
#include "MotorLib_SixStep_param.h"   // need for 6Step constants, etc
#include "6Step_Lib.h"
#include "BLDC\STM32\l6398.h"         // L6398 command codes, register defs, etc

void MotorLib_SixStep_ADCx_Bemf (void);
void MotorLib_SixStep_ADC_Channel (uint32_t adc_ch);
void MotorLib_SixStep_INIT (MOTOR_BLOCK *mtr_blk);
void MotorLib_SixStep_SysTick_MediumFrequencyTask(void);
void MotorLib_SixStep_TIMx_timebase(void);
void MotorLib_SixStep_StopMotor (uint8_t motor_id, int stop_type, int flags);


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

// VVVVVVVVVVVVVVVVVVVVVVVVVV  direct lift from l6398 main.c code

#define l6398_MAX_PWM_FREQ   (10000)     // Maximum frequency of the PWMs in Hz
#define l6398_MIN_PWM_FREQ   (2)         // Minimum frequency of the PWMs in Hz

// ^^^^^^^^^^^^^^^^^^^^^^^^^^  end  direct lift from l6398 code


int     l6398_GetDeviceState (MOTOR_BLOCK *mtr_blk);
int32_t l6398_get_motor_actual_position (MOTOR_BLOCK *mtr_blk);
int     l6398_motor_do_ctl_action (MOTOR_BLOCK *mtr_blk, int cmd, int flags);
int     l6398_perform_motor_move (MOTOR_BLOCK *mtr_blk, int cmd, int direction, int32_t count);
void    l6398_StartMovement (MOTOR_BLOCK *mtr_blk, int move_cmd);
int     l6398_motor_start (MOTOR_BLOCK *mtr_blk, int stop_type, int flags);
int     l6398_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags);
int32_t l6398_get_mark (MOTOR_BLOCK *mtr_blk);
int32_t l6398_CmdGetStatus (MOTOR_BLOCK *mtr_blk, int status_type);
int     l6398_WaitWhileActive (MOTOR_BLOCK *mtr_blk, int wait_type, int flags);
int     l6398_motor_init_controller_chip (void);

int     l6398_motor_run (MOTOR_BLOCK *mtr_blk, int direction);
int     l6398_set_direction (MOTOR_BLOCK *mtr_blk, int mtr_direction);
int     l6398_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count);
void    l6398_StepClockHandler (uint8_t motor_id);

int     l6398_set_bldc_phase_duty_cycles (MOTOR_BLOCK *mtr_blk, 
                                      uint16_t phaseU_duty_value,
                                      uint16_t phaseV_duty_value,
                                      uint16_t phaseW_duty_value);
int     l6398_set_bldc_phase_pwm_duty_cycle (MOTOR_BLOCK *mtr_blk, int phase_id, uint16_t duty_value);
int     l6398_set_bldc_pwm_enables (MOTOR_BLOCK *mtr_blk, int phase_id_mask);


int     l6398_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr);
int     l6398_motor_adc_next_conversion (void);
int     l6398_motor_init_adc_ctl_pins (void);
int     l6398_motor_init_encoder_gpio(MOTOR_BLOCK *mtr_blk, int encoder_type);
int     l6398_motor_init_gpio_ctl_outputs (void);
int     l6398_motor_init_gpio_ctl_inputs (void);
int     l6398_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long pwm_ticks);

int     l6398_current_reference_start_stop (MOTOR_BLOCK *mtr_blk, int start_stop_flag);
int     l6398_current_reference_set_value (MOTOR_BLOCK *mtr_blk, uint16_t Iref_value);

int     l6398_start_pwm_generation (MOTOR_BLOCK *mtr_blk);
int     l6398_stop_pwm_generation (MOTOR_BLOCK *mtr_blk);

                         //    =====   TEMP   HACK    =======
void    l6398_ApplySpeed(MOTOR_BLOCK *mtr_blk, uint8_t pwmId, uint16_t newSpeed);
void    l6398_ComputeSpeedProfile(MOTOR_BLOCK *mtr_blk, uint32_t nbSteps);
int32_t l6398_ConvertPosition(uint32_t abs_position_reg);
uint8_t l6398_Convert_Ocd_Thresh_to_RegValue(float Tval);
float   l6398_Convert_Ocd_RegValue_to_Threshold(uint8_t Par);
uint8_t l6398_Convert_Tval_Current_to_RegValue(float Tval);
uint8_t l6398_Convert_Tmin_Time_to_RegValue(float Tmin);
float   l6398_Convert_RegValue_to_Tmin_Time(uint8_t Par);
float   l6398_Convert_Tval_RegValue_to_Current(uint8_t Par);
void    l6398_ErrorHandler(uint16_t error);
void    l6398_FlagInterruptHandler(void);
void    l6398_start_motor_move(MOTOR_BLOCK *mtr_blk, int move_cmd);
void    l6398_StepClockHandler(uint8_t motor_id);

void     l6398_AttachErrorHandler (void (*callback)(uint16_t));    // L6206 relic
void     l6398_AttachFlagInterrupt (void (*callback)(void));       // L6206 relic
uint32_t l6398_CmdGetParam (MOTOR_BLOCK *mtr_blk, uint32_t param); // L6206 relic
uint8_t  l6398_Convert_Ocd_Thresh_to_RegValue (float Tval);        // L6206 relic
void     l6398_Init (void *pInit);                                 // L6206 relic
uint8_t  l6398_GetFwVersion (void);                                // L6206 relic
void     l6398_HardStop (MOTOR_BLOCK *mtr_blk);                    // L6206 relic
void     l6398_HizStop (MOTOR_BLOCK *mtr_blk);                     // L6206 relic
uint16_t l6398_ReadStatusRegister (MOTOR_BLOCK *mtr_blk);          // L6206 relic
uint16_t l6398_ReadId (void);                                      // L6206 relic
//void   l6398_ResetAllDevices (void);                                // L6206 relic
void     l6398_Release_Chip_Reset (MOTOR_BLOCK *mtr_blk);
void     l6398_Reset_Chip (MOTOR_BLOCK *mtr_blk);
void     l6398_SetDirection (MOTOR_BLOCK *mtr_blk, int dir);
bool     l6398_SoftStop (MOTOR_BLOCK *mtr_blk);

            // NEED EQUIVALENTS OF THESE in sep BOARD file, similar to what did for L6474
            // these take the place of HAL_xxxx.c file for each diff MCU
int l6398_Board_GpioInit (int mtr_id);                            // L6206 relic
int l6398_Board_PwmInit (int mtr_id);                             // L6206 relic
int l6398_Board_Reset (int mtr_id);                               // L6206 relic
int l6398_Board_ReleaseReset (int mtr_id);                        // L6464 relic
int l6398_Board_PwmStop (int mtr_id);                             // L6206 relic
int l6398_Board_SetDirectionGpio (int mtr_id, int dir);  // ??? does this make sense with BLDC ???

#define MAX_NUMBER_OF_DEVICES      4
#define l6398_FW_VERSION         101

                         //    =====   TEMP   HACK    =======

           //-----------------------------------------------------------
           //-----------------------------------------------------------
           //             Motor Contoller Chip Function Table
           //
           //  Defines low-level command handlers that are invoked by
           //  BLDC_motor_ctl_lib.c to perform device specific functions
           //-----------------------------------------------------------
           //-----------------------------------------------------------
  MOTOR_HANDLERS * l6398_GetMotorHandle(void);   // return addr of drv6474_table

  MOTOR_HANDLERS  drv6230_table = {
                    TYPE_MOTOR_BLDC,              // Motor Type
                    MOTOR_DRIVER,                 // Chip number / id
                    0L,                           // SPI init
                    0L,                           // SPI I/O
                    l6398_motor_speed_adc_init,   // init routine for speed ctl ADC
                    l6398_motor_adc_next_conversion, // ADC conversion start rtn for speed
                    0L,                           // motor_read_hall_sensors_handler
                    0L,                           // motor_fault_shutdown
                    l6398_motor_init_adc_ctl_pins,    // control ADCs (Current sense)
                    l6398_motor_init_controller_chip, // motor_init_controller_chip  = NOP on l6398
                    0L,                               // motor_init_commute_time
                    l6398_motor_init_encoder_gpio,    // init routine for encoder GPIOs
                    l6398_motor_init_gpio_ctl_inputs, // control input  GPIOs (FAULT)
                    l6398_motor_init_gpio_ctl_outputs,// control output GPIOs (ENABLE)
                    l6398_motor_pwm_init,             // init for motor PWMs

                    0L,                           // motor_check_status
                    0L,                           // motor_do_action_handler - not used by l6398, no SPI
                    0L,                           // motor_do_next_commutate
                    l6398_get_motor_actual_position,// motor_get_actual_position
                    l6398_get_mark,               // motor_get_mark_pos
                    l6398_CmdGetStatus,           // motor_get_motor_status
                    0L,                           // motor_get_period_ticks
                    l6398_perform_motor_move,     // motor_perform_move
                    l6398_motor_run,              // motor_run_handler
                    l6398_set_direction,          // motor_set_direction_handler
                    l6398_set_pwm_duty_count,     // motor_set_pwm_duty_count_handler
                    0L,                           // motor_set_speed_rpm_handler
                    0L,                           // motor_set_step_mode  (STEPPER)
                    0L,                           // motor_set_torque
                    l6398_motor_start,            // motor_start_handler  (BLDC)
                    l6398_motor_stop,             // motor_stop_handler
                    l6398_WaitWhileActive,        // motor_wait_complete
                   l6398_current_reference_start_stop,// motor_current_reference_start_stop
                    l6398_current_reference_set_value,// motor_current_reference_set_value
                    l6398_start_pwm_generation,   // motor_start_pwm_generation (BLDC)
                    l6398_stop_pwm_generation,    // motor_stop_pwm_generation  (BLDC)
                    l6398_set_bldc_phase_duty_cycles, // motor_set_bldc_phase_pwm_duty_cycles (BLDC)
                    l6398_set_bldc_pwm_enables,   // motor_set_bldc_pwm_enables (BLDC)
                  };

    uint32_t   g_CmdGetStatus   = 0;
    uint32_t   g_CmdGetStatus_2 = 0;
//  uint32_t   inactive_state_rupt = 0;

               /// Function pointer to flag interrupt call back
void (*flagInterruptCallback)(void);
               /// Function pointer to error handler call back
void (*errorHandlerCallback)(uint16_t);

static volatile uint8_t  numberOfDevices = 1;   // default is 1 motor

static uint8_t  spiTxBursts[6][3];   // are thses used in l6398 logic or is it relic from L6206 code ?
static uint8_t  spiRxBursts[6][3];

static volatile bool    spiPreemtionByIsr   = FALSE;
//static volatile bool    isrFlag             = FALSE;
static uint16_t         l6474DriverInstance = 0;

                      //------------------------------------
                      // l6398 Motor Blocks - 1 per motor
                      //------------------------------------
extern  MOTOR_BLOCK  motor_blk [];             // is defined in MotorLib_Basic.c


#define MAX_STEPS       (0x7FFFFFFF)           // Maximum number of steps


extern  ADC_HandleTypeDef   hadc1;    // direct lift from l6398.c main.c startup code

extern  TIM_HandleTypeDef   hPwmMtr;     // TIM1  HF_TIM
extern  TIM_HandleTypeDef   htim2;
extern  TIM_HandleTypeDef   htim3;       // TIM3
extern  TIM_HandleTypeDef   hCommuteTmr; // TIM4  LF_TIM  //  end  direct lift

extern  UART_HandleTypeDef  huart2;


//******************************************************************************
//  l6398_start_motor_move
//
//                                                                    KEY  KEY
//                            Start Movement
//
//   Initialises the bridge parameters to start the movement
//   and enable the power bridge
//******************************************************************************

void  l6398_start_motor_move (MOTOR_BLOCK *mtr_blk, int move_cmd)
{

// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

//  mtr_blk->mtr_move_type = MOVE_CMD;

    if (mtr_blk->mtr_accel_end_position != 0)      // Is this redundant w/upper layer ?
       mtr_blk->mtr_motion_state = MOTION_ACCELERATING;
       else mtr_blk->mtr_motion_state = MOTION_DECELERATING;

    mtr_blk->accu        = 0;
    mtr_blk->mtr_new_relative_position = 0;        // clear for new move

                                                              // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
    l6398_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_min_velocity_pc); //   PWMs finally go on at this point - STM32_F4
                                                              // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
}


//******************************************************************************
//  l6398_motor_speed_adc_init                                          SPEED
//
//         Configure ADC to read potentiometer used for speed control setting.
//        -     Speed Ctl Pot ADC    J1-2    PB5  AIN11   Grove J3-7/27 -> J1-2
//******************************************************************************
int  l6398_motor_speed_adc_init (int  channel, CALLBACK_RTN cb_ptr)
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
//  l6398_motor_adc_next_conversion
//
//         start a new ADC conversion for Speed control  (called every 50 ms)
//         Uses A15  on pin  P6.0
//******************************************************************************
int  l6398_motor_adc_next_conversion (void)
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
//  l6398_motor_init_adc_ctl_pins
//
//         Configure ADC to read control inputs for current sensors, BNEMF, ...
//******************************************************************************
int  l6398_motor_init_adc_ctl_pins (void)
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
//  l6398_motor_init_controller_chip
//******************************************************************************
int  l6398_motor_init_controller_chip (void)
{
                // no low level resets or SPI init needed for l6398
    return (0);           // denote was successful
}


//******************************************************************************
//  l6398_motor_init_encoder_gpio                                      ENCODER
//
//         Configure GPIOs for Wheel Encoder pin inputs ...
//              Hall Sensor Left      PA8   Arduino D7    GPIO rupt   Grove D7
//              Hall Sensor Right     PA9   Arduino D8    GPIO rupt   Grove D8
//******************************************************************************
int  l6398_motor_init_encoder_gpio (MOTOR_BLOCK *mtr_blk, int encoder_type)
{

    return (0);           // denote was successful
}


//******************************************************************************
//  l6398_motor_init_gpio_ctl_inputs
//
//         Configure control inputs for Motor GPIOs  - USER BUTTON, FAULT, ...
//******************************************************************************

int  l6398_motor_init_gpio_ctl_inputs (void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    __GPIOH_CLK_ENABLE();    // ensure additional needed clocks turned on

    return (0);              // denote was successful
}


//******************************************************************************
//  l6398_motor_init_gpio_ctl_outputs
//
//         Configure control outputs for Motor GPIOs - ENABLE, and SPI if needed
//******************************************************************************
int  l6398_motor_init_gpio_ctl_outputs (void)
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

        // Note: SPI is _NOT_ used on l6398, so we do not init SPI support

    return (0);           // denote was successful
}


//******************************************************************************
//  l6398_motor_pwm_init
//
//         Configure PWMs to drive BLDC motor U / V / W phases.
//         These use a variable duty cycle to control motor speed.
//         Generates a motor PWM with a frequency of ?? 20,000 Hz (50 usec).
//
//         Also configure a COMMUTATION timer, that is used to cause a periodic
//         "pop" to cause us to step to next entry in "6 Step" commutation table
//******************************************************************************

int  l6398_motor_pwm_init (MOTOR_BLOCK *mtr_blk, long period_ticks)
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
         MotorLib_SixStep_INIT (mtr_blk);

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
void  l6398_AttachErrorHandler (void (*callback)(uint16_t))  // L6206 relic
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
void  l6398_AttachFlagInterrupt (void (*callback)(void))  // L6206 relic
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
void  l6398_ErrorHandler (uint16_t error)
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
void  l6398_FlagInterruptHandler (void)
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
 *        Causes PWMs and rupts to engage by calling l6398_Board_PwmNSetFreq()
 *
 * @param[in] motor_id (from 0 to 2)
 * @param[in] newSpeed in pps
 * @retval None
 ******************************************************************************/
void  l6398_ApplySpeed (MOTOR_BLOCK *mtr_blk, uint8_t pwmId, uint16_t newSpeed)
{

  if (newSpeed < l6398_MIN_PWM_FREQ)
     {
       newSpeed = l6398_MIN_PWM_FREQ;
     }
  if (newSpeed > l6398_MAX_PWM_FREQ)
     {
       newSpeed = l6398_MAX_PWM_FREQ;
     }

  mtr_blk->mtr_current_velocity_pc = newSpeed;

  switch (mtr_blk->mtr_id)
  {
    case  0:
//     l6398_Board_Pwm1SetFreq (newSpeed);  // L6206 relic    ??? NEED l6398 EQUIV
      break;

    case 1:
//    l6398_Board_Pwm2SetFreq (newSpeed);  // L6206 relic
      break;

    case 2:
//    l6398_Board_Pwm3SetFreq (newSpeed);  // L6206 relic
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
int32_t  l6398_ConvertPosition (uint32_t abs_position_reg)
{
    int32_t  operation_result;

// CAUTION: dumb BLDC - need to inquire QEI encoder instead !!!

#if defined(DO_MANUALLY)
    if (abs_position_reg & l6398_ABS_POS_SIGN_BIT_MASK)     // L6206 relic - need generic
       {
              // Handle a Negative register value
         abs_position_reg = ~abs_position_reg;
         abs_position_reg += 1;

         operation_result = (int32_t) (abs_position_reg & l6398_ABS_POS_VALUE_MASK);   // L6206 relic - needs generic
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
* l6398_CmdGetStatus
*
*      
*      Gets the OK/ERROR, etc statusof the  l6398 for the specified motor
*
*      NOTE: dumb BLDC chip does not have motion cmds like STEPPER does, so must
*            do manually !
*            l6398 does NOT have any SPI support, so no cmds are ever issued
******************************************************************************/
int32_t  l6398_CmdGetStatus (MOTOR_BLOCK *mtr_blk, int status_type)
{
    return (-1);                  // denote no such support on this chip
}


//******************************************************************************
//  l6398_current_reference_start_stop
//
//               Startup or shutdown the Motor "Current Reference" hardware
//******************************************************************************

int  l6398_current_reference_start_stop (MOTOR_BLOCK *mtr_blk, int start_stop_flag) // WVD - GOES THRU HERE
{
    if (start_stop_flag == START_FLAG)
       {      // start up the Motor's "current Reference" hardware
         PWM_CURRENT_REF_HANDLE.Instance->CCR1 = 0;

         #if (USES_l6398)
            HAL_TIM_PWM_Start (&PWM_CURRENT_REF_HANDLE, PWM_CURRENT_REF_CHANNEL); // was HF_TIMx_CH1
         #else
            REFx.Instance->CCER |= TIM_CCER_CC1E;              // enable/start the DAC
         #endif
       }
      else
       {      // shut down the Motor's "current Reference" hardware
         PWM_CURRENT_REF_HANDLE.Instance->CCR1 = 0;

         #if (USES_l6398)
            HAL_TIM_PWM_Stop (&PWM_CURRENT_REF_HANDLE, PWM_CURRENT_REF_CHANNEL);  // was HF_TIMx_CH1
         #else
            REFx.Instance->CCER &= ~TIM_CCER_CC1E;              // disable   
         #endif
       }

    return (0);                       // denote worked OK
}


//******************************************************************************
//  l6398_current_reference_set_value
//
//               Set the value to use for Motor's "Current Reference".
//               Updates the Iref Current reference value..
//******************************************************************************

int  l6398_current_reference_set_value (MOTOR_BLOCK *mtr_blk, uint16_t Iref_value)  // WVD - GOES THRU HERE
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
int  l6398_GetDeviceState (MOTOR_BLOCK *mtr_blk)
{
    return (mtr_blk->mtr_motion_state);
}


/******************************************************//**
 * @brief Returns the FW version of the library
 * @retval l6398_FW_VERSION
 **********************************************************/
uint8_t  l6398_GetFwVersion (void)  // L6206 relic
{
    return (l6398_FW_VERSION);  // L6206 relic
}


/******************************************************//**
 * @brief  Return motor handle (pointer to the l6398 motor driver structure)
 * @retval Pointer to the motorDrv_t structure
 **********************************************************/
MOTOR_HANDLERS  * l6398_GetMotorHandle (void)
{
    return (&drv6230_table);   // L6206 relic
}


/******************************************************//**
 * @brief  Returns the mark position  of the specified motor
 * @param[in] motor_id (from 0 to 2)
 * @retval Mark register value converted in a 32b signed integer
 **********************************************************/
int32_t  l6398_get_mark (MOTOR_BLOCK *mtr_blk)

{
// CAUTION:  dumb BLDC vs smart STEPPER

//  return l6398_ConvertPosition(l6398_CmdGetParam(mtr_blk,l6398_MARK));  // L6206 relic
}


//******************************************************************************
//  l6398_get_motor_actual_position
//                            gets the absolute position of the motor, from
//                            the contreoller (stepper) or encoder QEI (BLDC)
//******************************************************************************
int32_t  l6398_get_motor_actual_position (MOTOR_BLOCK *mtr_blk)
{
    int32_t      raw_position;
    int32_t      abs_position;


// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

//  raw_position = l6398_CmdGetParam (mtr_blk, l6398_ABS_POS);   // L6206 relic

    abs_position = l6398_ConvertPosition (raw_position);

    return (abs_position);
}


/******************************************************//**
 *                   Hard Stop
 * @brief  Immediatly stops the motor
 * @param[in] motor_id (from 0 to 2)
 * @retval None
 **********************************************************/
void  l6398_HardStop (MOTOR_BLOCK *mtr_blk)  // L6206 relic
{
       /* Disable corresponding PWM */
    l6398_Board_PwmStop (mtr_blk->mtr_id);   // L6206 relic

    g_CmdGetStatus = l6398_CmdGetStatus (mtr_blk, 0); // clear any l6398 FAULT status

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
void  l6398_HizStop (MOTOR_BLOCK *mtr_blk)  // L6206 relic
{
       /* Disable corresponding PWM */
    l6398_Board_PwmStop (mtr_blk->mtr_id);  // L6206 relic

    g_CmdGetStatus = l6398_CmdGetStatus (mtr_blk, 0); // clear any l6398 FAULT status

       /* Set inactive state */
    mtr_blk->mtr_motion_state        = MOTION_NOT_ACTIVE;
//   mtr_blk->mtr_operation_performed = NO_CMD;
    mtr_blk->mtr_steps_to_move       = MAX_STEPS;
}


/******************************************************//**
 *                     Release Chip Reset
 * @brief  Releases the l6398 reset (pin set to High) of all devices
 * @retval None
 **********************************************************/
void  l6398_Release_Chip_Reset (MOTOR_BLOCK *mtr_blk)
{
    l6398_Board_ReleaseReset (mtr_blk->mtr_id);
}


/**********************************************************
* l6398_Reset_Chip
*
*     Resets the l6398 (reset pin set to low) for associated
*     Motor Control Chip
*
* @param[in] motor_id (from 0 to 2)
**********************************************************/
void  l6398_Reset_Chip (MOTOR_BLOCK *mtr_blk)
{
    l6398_Board_Reset (mtr_blk->mtr_id);   // ??? TILT ???
}


//******************************************************************************
//  l6398_motor_run
//
//               Run the motor, i.e. issue a start (or re-start after a
//                              direction change or after a stop).
//
//               Because a stop will slam off the PWMs, we re-establish both the
//               Direction and the Duty Cycle.
//******************************************************************************
int  l6398_motor_run (MOTOR_BLOCK *mtr_blk, int direction)
{
    mtr_blk->mtr_state = MOTOR_STATE_RUN;     // denote we are now in RUN mode

//  TBD - => free running Stepper (no counts or distance, or REJCT as NOT_SUPPORTED ?

    return (0);           // denote was successful
}


//******************************************************************************
//  l6398_perform_motor_move
//                             Issue a command to start the motor moving
//******************************************************************************
            /* Motor activation */
int  l6398_perform_motor_move (MOTOR_BLOCK *mtr_blk, int cmd, int direction, int32_t count)
{
    int   move_op;

// CAUTION: dumb BLDC chip does not have motion cmds like STEPPER - must do manually !

#if defined(DOMANUALLY_l6398)
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

    l6398_start_motor_move (mtr_blk, move_op);     // start the motor

    return (0);                      // denote worked OK
}


//******************************************************************************
//  l6398_motor_start                                        motor_start_handler
//
//                             Start the motor. 
//                             The start_type indicates the type of start.
//******************************************************************************
int  l6398_motor_start (MOTOR_BLOCK *mtr_blk, int stop_type, int flags)
{
                                           // LF_TIMx = hCommuteTmr on F1_01
     HAL_TIM_Base_Start_IT (&hCommuteTmr); // start up Timer and assoc interrupts

           // ensure Motor's "current reference" hardware is enabled
//PS _g_mtrdrvr->motor_current_reference_start_stop (mtr_blk, START_FLAG);

     HAL_ADC_Start_IT (&ADC_HANDLE);       // Start up ADC and assoc interrupts

     return (0);                           // denote worked OK
}


//******************************************************************************
//  l6398_motor_stop                                          motor_stop_handler
//
//                      Stop the motor. The stop_type inidcates
//                      the type of stop (Hard, Soft, Hiz, ...)
//******************************************************************************
int  l6398_motor_stop (MOTOR_BLOCK *mtr_blk, int stop_type, int flags)
{
    hPwmMtr.Instance->CR1 &= ~(TIM_CR1_CEN); // Shut off Motor PWMs  (NOT IN PS version)
    hPwmMtr.Instance->CNT  = 0;

    HAL_TIM_Base_Stop_IT (&hCommuteTmr); // Turn off PWM timer, including interrupts

    HAL_ADC_Stop_IT (&ADC_HANDLE);       // turn off ADC

#if defined(DO_MANUALLY_l6398)
    switch (stop_type)
      {
        case STOP_HARD_BRAKE:                      // Immediatly stops the motor
                l6398_Board_PwmStop (mtr_blk->mtr_id); // Disable corresponding PWM   // L6206 relic
                break;

        case STOP_SOFT_BRAKE:       // Stops motor by using motor's deceleration (Coast to stop)
                if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
                   mtr_blk->mtr_operation_performed = SOFT_STOP_CMD;
                // the Step Clock Handler ISR will handle this op code and cause Coast shutdown
                break;

        case STOP_HIZ:       // Immediatly stops motor and disables power bridge
                l6398_Board_PwmStop (mtr_blk->mtr_id); // Disable corresponding PWM  // L6206 relic
                             // Disable power stage
                break;
      }
#endif

//  g_CmdGetStatus = l6398_CmdGetStatus (mtr_blk, 0); // clear any l6398 FAULT status (WVD added - for good measure)

       // Clear everything to denote we are in the INACTIVE state
    mtr_blk->mtr_motion_state        = MOTION_NOT_ACTIVE;
//  mtr_blk->mtr_operation_performed = NO_CMD;               ??? FIX RESOLVE
    mtr_blk->mtr_steps_to_move       = MAX_STEPS;

    return (0);                      // denote worked OK
}


//******************************************************************************
//  l6398_set_bldc_phase_duty_cycles
//
//         Set the duty cycle for each Phase (U/V/W) on a BLDC motor
//******************************************************************************
int l6398_set_bldc_phase_duty_cycles (MOTOR_BLOCK *mtr_blk, 
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
//  l6398_set_bldc_phase_pwm_duty_cycle
//
//         Set the duty cycle for a specific Phase (U/V/W) on a BLDC motor
//******************************************************************************
int l6398_set_bldc_phase_pwm_duty_cycle (MOTOR_BLOCK *mtr_blk, int phase_id,
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
//  l6398_set_bldc_pwm_enables
//
//         Sets/clears the PWM ENABLEs for the 3 Phases (U/V/W) on a BLDC motor.
//
//         Only two legs can be active at a time, in a BLDC motor, so this shuts
//         off the PWM for the third "floating" leg in this step  (e.g. 6-Step)
//******************************************************************************
int l6398_set_bldc_pwm_enables (MOTOR_BLOCK *mtr_blk, int phase_id_mask)
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
//  l6398_set_direction
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
int  l6398_set_direction (MOTOR_BLOCK *mtr_blk,  int direction)
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
//  l6398_set_pwm_duty_count
//
//            Set the duty cycle to be used for a specific motor.
//            Here, duty cycle is explicitly passed as PWM ticks, not as percent.
//            The duty cycle sets the motor's speed. (More juice, more speed)
//
//   Parms:
//      motor_num:   number of the motor to update (1 or 2)
//      pwm_duty:    actual duty cycle PWM count value to be loaded in PWM
//******************************************************************************

int  l6398_set_pwm_duty_count (MOTOR_BLOCK *mtr_blk, int pwm_duty_count)
{
     int  adjusted_duty_count;

     mtr_blk->mtr_pwm_duty_count = pwm_duty_count;  // save actual count (not %)

// TBD - Anythings else to do for a stepper >  Does duty even make sense ?
//           or issue REJECT ?

     return (0);                             // do not apply till motor start

}


//******************************************************************************
//  l6398_start_pwm_generation
//
//            Enable generation of PWM signals on PWM channels/CCRs for l6398.
//            called via COMMUTATE Timer rupts started by Start_Motor()
//******************************************************************************

int  l6398_start_pwm_generation (MOTOR_BLOCK *mtr_blk)
{
    HAL_TIM_PWM_Start (&hPwmMtr, PWM_U_CHANNEL);        // TIM1_CH1 start
    HAL_TIM_PWM_Start (&hPwmMtr, PWM_V_CHANNEL);        // TIM1_CH2 start
    HAL_TIM_PWM_Start (&hPwmMtr, PWM_W_CHANNEL);        // TIM1_CH3 start
}


//******************************************************************************
//  l6398_stop_pwm_generation
//
//            Disable the generation of PWM signals on PWM channels for l6398.
//******************************************************************************

int  l6398_stop_pwm_generation (MOTOR_BLOCK *mtr_blk)
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
void  l6398_SetDirection (MOTOR_BLOCK *mtr_blk, int dir)
{

// ??? !!!  for BLDC, there is no DIR pin.  Instead, walk thru COMMUTATE table BACKWARDs

     l6398_Board_SetDirectionGpio (mtr_blk->mtr_id, dir);
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
void  l6398_StartMovement (MOTOR_BLOCK *mtr_blk, int move_cmd)
{
    mtr_blk->mtr_operation_performed = move_cmd;  // save type of move being executed

// CAUTION:  dumb BLDC chip vs STEPPER - DO_MANUALLY

         // ensure l6398 powerstage is Enabled

    if (mtr_blk->mtr_accel_end_position != 0)
       mtr_blk->mtr_motion_state = MOTION_ACCELERATING;
       else mtr_blk->mtr_motion_state = MOTION_DECELERATING;

    mtr_blk->accu        = 0;
    mtr_blk->mtr_new_relative_position = 0;
                                                              // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
    l6398_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_min_velocity_pc); //   PWMs finally go on at this point - STM32_F4
                                                              // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
}


/******************************************************//**
 *                      Soft Stop
 * @brief  Stops the motor by using the device deceleration
 * @param[in] motor_id (from 0 to 2)
 * @retval true if the command is successfully executed, else false
 * @note The command is not performed is the device is in INACTIVE state.
 **********************************************************/
bool  l6398_SoftStop (MOTOR_BLOCK *mtr_blk)
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
int  l6398_WaitWhileActive (MOTOR_BLOCK *mtr_blk, int wait_type, int flags)
{
        /* Wait while the motor is running */
    while (l6398_GetDeviceState(mtr_blk) !=  MOTION_NOT_ACTIVE)
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

void  l6398_StepClockHandler (uint8_t motor_id)
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

#if defined(DEFINED_IN_l6398_BASIC_C)
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
                 l6398_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_current_velocity_pc);
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
            l6398_HardStop (mtr_blk);
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
               l6398_ApplySpeed (mtr_blk, mtr_blk->mtr_id, mtr_blk->mtr_current_velocity_pc);
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




// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
//     The following were lifted from l6398.c code and X-NUCLEO-IHM07M1.c but
//     have been heavily modified.  Will convert to "clean sheet" version next pass

/************************ (C) COPYRIGHT STMicroelectronics ********************/

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
            l6398_StepClockHandler (0);
          }
     }
  if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
     {     // process motor 2
       mtr_blk = &motor_blk[1];
       if (mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
          {
            l6398_StepClockHandler (1);
          }
     }

  if ((htim->Instance == TIM4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3))
     {     // process motor 3
       mtr_blk = &motor_blk[2];
       HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_10);
       if ((mtr_blk->mtr_motion_state != MOTION_NOT_ACTIVE)
         && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET))
          {
            l6398_StepClockHandler (2);
          }
      }
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
    MotorLib_SixStep_SysTick_MediumFrequencyTask();
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
         MotorLib_SixStep_StopMotor (0, 0, 0);    // ??? TEMP TEST HACK   // hit an error condition - turn off motor
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
    MotorLib_SixStep_TIMx_timebase();   // Never being called under my code ! 07/26/16
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
      MotorLib_SixStep_ADCx_Bemf();
}

//******************************************************************************
