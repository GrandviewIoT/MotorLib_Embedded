
extern  int  no_motor_test;  // ??? 1 = run thru code with motor off/un-attached

//*******1*********2*********3*********4*********5*********6*********7**********
//
//                           MotorLib_Bldc_6_Step.c
//
// Motor Control Library:  High Level support for Sensored and Sensorless
//                         BLDC support.
//
// This is a portable code library, designed to work across a number of
// processors, including:   TI Tiva Cortex-M
//                          TI MSP432 with Cortex-M core
//                          TI MSP430 F5529, FR5969, FR6989, FR5994
//                          STM32 F3, F4, L4, L7 Cortex-M series
//
// Low level detailed support is provided by the associated MCU platform's
//     MotorLib_LL_xxxx_yyyy.c module, where xxxx = Chip type and
//     yyyyy = MCU type, e.g. MotorLib_LL_L6230_STM32.c
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
//   05/16/16 - Created as part of Motor Control open source.      Duquaine
//   07/22/16 - Rolled in L6230 specific changes for BLDC support. Duquaine
//   07/30/16 - Got baseline L6230 code running reliably.          Duquaine
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
#include "MotorLib_SixStep_param.h"         // need for 6Step constants, etc
#include "6Step_Lib.h"

extern    MOTOR_BLOCK     motor_blk[];    // located in MotorLib_Basic.c
extern    MOTOR_HANDLERS  *_g_mtrdrvr;    //  ditto

void  update_STATUS (SIXSTEP_Base_SystStatus_t  new_STATUS);
void  update_Step_Position (uint8_t step_position);

    ADC_HandleTypeDef   hadc1;   // direct lift from L6230.c main.c startup code

    TIM_HandleTypeDef   hPwmMtr;       // TIM1  HF_TIM
    TIM_HandleTypeDef   htim2;
    TIM_HandleTypeDef   htim3;         // TIM3
    TIM_HandleTypeDef   hCommuteTmr;   // TIM4  LF_TIM  //  end  direct lift


// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
//              VVVVVVVVVVVVVVVVVVVVVVVVVVV
//  The fowllowing has been lifted from L6230.c code and X-NUCLEO-IHM07M1.c but
//  very heavily modified. Will convert to a "clean sheet" version in next pass

void BSP_X_NUCLEO_FAULT_LED_ON(void);
void BSP_X_NUCLEO_FAULT_LED_OFF(void);


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
#include "MotorLib_SixStep_param.h"


/*****************************************************************************
The main function are the following:

  1) MotorLib_SixStep_TABLE(...) -> Set the peripherals (TIMx, GPIO etc.) for each step
  2) MotorLib_SixStep_ARR_step() -> Generate the ARR value for Low Frequency TIM during start-up
  3) MotorLib_SixStep_INIT()     -> Init the main variables for motor driving from MotorLib_SixStep_param.h
  4) MotorLib_SixStep_RESET()    -> Reset all variables used for 6Step control algorithm
  5) MotorLib_SixStep_Ramp_Motor_calc() -> Calculate the acceleration profile step by step for motor during start-up
  6) MotorLib_SixStep_NEXT_step()-> Generate the next step number according with the direction (CW or CCW)
  7) MotorLib_Task_Speed()       -> Speed Loop with PI regulator
  8) MotorLib_SixStep_Set_Speed(...)     -> Set the new motor speed value
  9) MotorLib_SixStep_StartMotor()       -> Start the Motor
  10)MotorLib_SixStep_StopMotor()       -> Stop the Motor
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
void     Bemf_delay_calc (void);
uint64_t MCM_Sqrt(uint64_t );
void     MotorLib_Bemf_Delay(void);
int32_t  MotorLib_GetElSpeedHz(void);
int32_t  MotorLib_GetMechSpeedRPM(void);
int16_t  MotorLib_PI_Controller(SIXSTEP_PI_PARAM_InitTypeDef_t *, int16_t);
uint16_t MotorLib_Potentiometer_filter(uint16_t);
void     MotorLib_Speed_Filter(void);
void     MotorLib_Set_PI_param(SIXSTEP_PI_PARAM_InitTypeDef_t *);
void     MotorLib_Task_Speed(void);
void     MotorLib_UI_INIT(void);

void     MotorLib_SixStep_ADC_Channel (uint32_t adc_ch);
void     MotorLib_SixStep_ADCx_Bemf(void);
void     MotorLib_SixStep_ARR_step(void);
void     MotorLib_SixStep_ARR_Bemf(uint8_t);
void     MotorLib_SixStep_Alignment(void);
void     MotorLib_SixStep_INIT (MOTOR_BLOCK *mtr_blk);
void     MotorLib_SixStep_Init_main_data(void);
void     MotorLib_SixStep_NEXT_step(void);
void     MotorLib_SixStep_Nucleo_Init (void);
void     MotorLib_SixStep_Ramp_Motor_calc(void);
void     MotorLib_SixStep_RESET (MOTOR_BLOCK *mtr_blk);
void     MotorLib_SixStep_Set_Speed (uint16_t speed_value);
void     MotorLib_SixStep_Speed_Potentiometer(void);
void     MotorLib_SixStep_Speed_Val_target_potentiometer(void);
void     MotorLib_SixStep_StartMotor (uint8_t motor_id, int start_type, int flags);
void     MotorLib_SixStep_StopMotor (uint8_t motor_id, int stop_type, int flags);
void     MotorLib_SixStep_SysTick_MediumFrequencyTask(void);
void     MotorLib_SixStep_TABLE(uint8_t);
void     MotorLib_SixStep_TIMx_timebase(void);

//void   CMD_Parser(char* pCommandString);
//void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc);
//void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim);
//void HAL_SYSTICK_Callback (void);
void UART_Set_Value(void);
void UART_Communication_Task(void);



//******************************************************************************
//  MotorLib_SixStep_INIT
//
//              Initialitation function for SixStep library
//******************************************************************************

void  MotorLib_SixStep_INIT (MOTOR_BLOCK *mtr_blk)       // WVD - GOES THRU HERE
{
    MotorLib_SixStep_Nucleo_Init();

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
    MotorLib_UI_INIT();               //  Start the UART Communication Task
 #endif

    MotorLib_SixStep_Init_main_data();

 #ifndef UART_COMM
    SIXSTEP_parameters.Button_ready = TRUE;
 #endif

    MotorLib_SixStep_RESET (mtr_blk);
}


//******************************************************************************
//  MotorLib_SixStep_Nucleo_Init
//
//             Init the STM32 registers                BOARD SPECIFIC - F4_01
//******************************************************************************

void  MotorLib_SixStep_Nucleo_Init (void)                  // WVD - GOES THRU HERE
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
//  MotorLib_SixStep_RESET
//
//                 Reset all variables used for 6Step control algorithm.
//                 Called as part of INIT logic.
//******************************************************************************

void  MotorLib_SixStep_RESET (MOTOR_BLOCK *mtr_blk)      // WVD - GOES THRU HERE
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

     MotorLib_Set_PI_param (&PI_parameters);

         // Startup PWM or DAC that is used to generate Motor's current reference
     _g_mtrdrvr->motor_current_reference_start_stop (mtr_blk, START_FLAG);
     _g_mtrdrvr->motor_current_reference_set_value (mtr_blk, SIXSTEP_parameters.Ireference);

//PS if ( ! VOLTAGE_MODE)
//PS    _g_mtrdrvr->motor_current_reference_set_value (mtr_blk, SIXSTEP_parameters.Ireference); 
//PS    else _g_mtrdrvr->motor_current_reference_set_value (mtr_blk,
//                                                          SIXSTEP_parameters.pulse_value); 

     index_startup_motor = 1;

     MotorLib_SixStep_Ramp_Motor_calc ();
}


//******************************************************************************
//                              CALLBACK
//  MotorLib_SixStep_ADC_Channel                        caller:  ADCConvCmplt Callback
//
//             Select the new (BEMF) ADC Channel          BOARD SPECIFIC - F4_01
//  @param  adc_ch        This logic selects 1 ADC channel at a time for BEMF feedback
//                    // 6-STEP rotations thru channels  U / V / W:  13 / 8 / 7
//******************************************************************************

void  MotorLib_SixStep_ADC_Channel (uint32_t adc_ch)
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
*  MotorLib_Bemf_delay_Filter
*
*                    Calculate the Bemf delay time filtered
*/

uint32_t  MotorLib_Bemf_delay_Filter (uint32_t dmg_tmp_value)
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
*  MotorLib_SixStep_Init_main_data
*
*       Init the main variables for motor driving from MotorLib_SixStep_param.h
*/

void  MotorLib_SixStep_Init_main_data (void)          // WVD - GOES THRU HERE
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
// MotorLib_SixStep_TIMx_timebase
//
//     COMMUTATE Timer Callback - Call the next step and request the filtered speed value
//                     Invoked by COMMUTATE timer rupt started via Start_Motor()
//******************************************************************************


void  MotorLib_SixStep_TIMx_timebase (void)
{
    MotorLib_SixStep_NEXT_step();             /* Change STEP number  */

    if (SIXSTEP_parameters.ARR_OK == 0)
       {
          MotorLib_SixStep_ARR_step();        /* BASE TIMER - ARR modification for STEP frequency changing */
       }

    MotorLib_Speed_Filter();                  /*Calculate SPEED filtered  */
}


//******************************************************************************
//                             CALLBACK
//
//  MotorLib_SixStep_SysTick_MediumFrequencyTask
//
//                         Systick Callback - Call the Speed loop
//******************************************************************************

void  MotorLib_SixStep_SysTick_MediumFrequencyTask (void)
{
    if (SIXSTEP_parameters.ALIGNMENT == TRUE && SIXSTEP_parameters.ALIGN_OK == FALSE)
       {
         MotorLib_SixStep_Alignment();
       }

 #ifdef UART_COMM
    if (UART_FLAG_RECEIVE == TRUE)
       UART_Communication_Task();
 #endif

 #ifdef DEMOMODE
    index_motor_run++;
    if (index_motor_run >= DEMO_START_TIME && test_motor_run == 0)
       {
         MotorLib_SixStep_StopMotor (0, 0, 0);              // ??? TEMP TEST HACK
         index_motor_run = 0;
         test_motor_run  = 1;
       }
    if (index_motor_run >= DEMO_STOP_TIME && test_motor_run == 1)
       {
         MotorLib_SixStep_StartMotor (0, 0, 0);             // ??? TEMP TEST HACK
         test_motor_run  = 0;
         index_motor_run = 0;
       }
 #endif

    if (SIXSTEP_parameters.VALIDATION_OK == TRUE && SIXSTEP_parameters.Potentiometer  == TRUE)
       {
         MotorLib_SixStep_Speed_Potentiometer();
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
                MotorLib_Task_Speed();
             }
          SIXSTEP_parameters.MediumFrequencyTask_flag = TRUE;
          if (SIXSTEP_parameters.VALIDATION_OK == TRUE)
             {
                MotorLib_SixStep_Set_Speed (0);
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
         MotorLib_SixStep_StopMotor (0, 0, 0);       // ??? TEMP TEST HACK
         cnt_bemf_event = 0;
//       SIXSTEP_parameters.STATUS = STARTUP_BEMF_FAILURE;
         update_STATUS (STARTUP_BEMF_FAILURE);
       }

    if (speed_fdbk_error == 1)
       {
         MotorLib_SixStep_StopMotor (0, 0, 0);       // ??? TEMP TEST HACK
//       SIXSTEP_parameters.STATUS = SPEEDFBKERROR;
         update_STATUS (SPEEDFBKERROR);

       }
}



//******************************************************************************
//                                ADC
//
//  MotorLib_SixStep_ADCx_Bemf
//
//                 Compute the zero crossing detection based on ADC readings
//******************************************************************************

void  MotorLib_SixStep_ADCx_Bemf (void)
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
                     MotorLib_SixStep_ARR_Bemf (0);
                   }
              }
             else
              {
                if (SIXSTEP_parameters.ADC_BUFFER[3] > SIXSTEP_parameters.ADC_BEMF_threshold_UP)
                   {
                     MotorLib_SixStep_ARR_Bemf (1);
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
                 MotorLib_SixStep_ARR_Bemf (1);
                 SIXSTEP_parameters.BEMF_Tdown_count = 0;
               }
          }
         else
          {
            if (SIXSTEP_parameters.ADC_BUFFER[2] < SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
               {
                 MotorLib_SixStep_ARR_Bemf (0);
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
            MotorLib_SixStep_ARR_Bemf(0);
        }
        }
        else
        {
         if (SIXSTEP_parameters.ADC_BUFFER[1]> SIXSTEP_parameters.ADC_BEMF_threshold_UP)
            {
              MotorLib_SixStep_ARR_Bemf(1);
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
          MotorLib_SixStep_ARR_Bemf(1);
          SIXSTEP_parameters.BEMF_Tdown_count = 0;
        }
        }
       else
       {
        if (SIXSTEP_parameters.ADC_BUFFER[3]< SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
        {
            MotorLib_SixStep_ARR_Bemf(0);
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
            MotorLib_SixStep_ARR_Bemf(0);
        }
        }
       else
       {
        if (SIXSTEP_parameters.ADC_BUFFER[2]> SIXSTEP_parameters.ADC_BEMF_threshold_UP)
        {
          MotorLib_SixStep_ARR_Bemf(1);
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
            MotorLib_SixStep_ARR_Bemf(1);
            SIXSTEP_parameters.BEMF_Tdown_count = 0;
         }
        }
        else
        {
         if (SIXSTEP_parameters.ADC_BUFFER[1]< SIXSTEP_parameters.ADC_BEMF_threshold_DOWN)
         {
            MotorLib_SixStep_ARR_Bemf(0);
         }
        }
       }
       else SIXSTEP_parameters.demagn_counter++;
      break;

     }                   //  end  switch()
    }
        /******************* SET ADC CHANNEL FOR SPEED/CURRENT/VBUS ***********/
        /* Set the channel for next ADC Regular reading */
     MotorLib_SixStep_ADC_Channel (SIXSTEP_parameters.ADC_SEQ_CHANNEL[index_adc_chn]);
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
      MotorLib_SixStep_ADC_Channel (SIXSTEP_parameters.CurrentRegular_BEMF_ch);
    }

}



/**************************************************************************
*  MotorLib_SixStep_ARR_Bemf
*
*       Calculate the new Autoreload value (ARR) for Low Frequency timer
*  Called via: HAL_ADC_IRQ_Handler -> HAL_ADC_ConvCplt_Callback -> MotorLib_SixStep_ADCx_Bemf
*/

void  MotorLib_SixStep_ARR_Bemf (uint8_t up_bemf)
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
*  MotorLib_Bemf_Delay
*
*     Take the delay time after each new 6-step commutation
*/
void  MotorLib_Bemf_Delay (void)
{
     Bemf_delay_calc();
//PS if (demagn_value_tmp != 0  &&  demagn_value_tmp != 0xFFF)   
//PS    {
//PS      dmg_tmp = MotorLib_Bemf_delay_Filter (demagn_value_tmp);
//PS      if (dmg_tmp <= 2)
//PS         SIXSTEP_parameters.demagn_value = 2;
//PS         else SIXSTEP_parameters.demagn_value = dmg_tmp; 
//PS    }

}



//******************************************************************************
//  MotorLib_SixStep_Ramp_Motor_calc
//
//    Calculate the acceleration profile step by step for motor during start-up
//******************************************************************************

void  MotorLib_SixStep_Ramp_Motor_calc (void)                // WVD - GOES THRU HERE
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
  * @defgroup   MotorLib_SixStep_ARR_step
  *
  * @brief Generate the ARR value for COMMUTATION / Low Frequency TIM during start-up
  * @retval None
*/

void  MotorLib_SixStep_ARR_step (void)
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

              MotorLib_SixStep_Ramp_Motor_calc();
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
                           MotorLib_SixStep_StopMotor (0, 0, 0);                  // ??? TEMP TEST HACK
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
       MotorLib_SixStep_Ramp_Motor_calc();
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
                MotorLib_SixStep_StopMotor (0, 0, 0);                // ??? TEMP TEST HACK
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
*  MotorLib_SixStep_TABLE
*
*           Set the peripherals (TIMx, GPIO etc.) used for each step
*
* @param  step_number: step number selected
***************************************************************************/

void  MotorLib_SixStep_TABLE (uint8_t step_number)      // uint8_t motor_id
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
// PS     MotorLib_SixStep_EnableInput_CH1_E_CH2_E_CH3_D (SIXSTEP_parameters.step_position, synchronous_rect, SIXSTEP_parameters.pulse_value);
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
// PS     MotorLib_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(SIXSTEP_parameters.step_position,synchronous_rect,SIXSTEP_parameters.pulse_value);   
// PS     SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
          _g_mtrdrvr->motor_set_bldc_phase_duty_cycles (mtr_blk, SIXSTEP_parameters.pulse_value,
                                                        0, 0);
                // Disable Phase V leg (used for BEMF CH2)
          _g_mtrdrvr->motor_set_bldc_pwm_enables (mtr_blk, PHASE_ENABLE_U_W_DISABLE_V);
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
          break;

    case 3:
// PS     MotorLib_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(SIXSTEP_parameters.step_position,synchronous_rect,SIXSTEP_parameters.pulse_value);           
// PS     SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
          _g_mtrdrvr->motor_set_bldc_phase_duty_cycles (mtr_blk, 0,
                                                        SIXSTEP_parameters.pulse_value, 0);
                // Disable Phase U leg (used for BEMF CH1)
          _g_mtrdrvr->motor_set_bldc_pwm_enables (mtr_blk, PHASE_ENABLE_V_W_DISABLE_U);
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[1];
          break;

    case 4:
// PS     MotorLib_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(SIXSTEP_parameters.step_position,synchronous_rect,SIXSTEP_parameters.pulse_value);        
// PS     SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
          _g_mtrdrvr->motor_set_bldc_phase_duty_cycles (mtr_blk, 0,
                                                        SIXSTEP_parameters.pulse_value, 0);
                // Disable Phase W leg (used for BEMF CH3)
          _g_mtrdrvr->motor_set_bldc_pwm_enables (mtr_blk, PHASE_ENABLE_U_V_DISABLE_W);
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[3];
     break;

    case 5:
// PS     MotorLib_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(SIXSTEP_parameters.step_position,synchronous_rect,SIXSTEP_parameters.pulse_value); 
// PS     SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
          _g_mtrdrvr->motor_set_bldc_phase_duty_cycles (mtr_blk, 0,
                                                        0, SIXSTEP_parameters.pulse_value);
                // Disable Phase V leg (used for BEMF CH2)
          _g_mtrdrvr->motor_set_bldc_pwm_enables (mtr_blk, PHASE_ENABLE_U_W_DISABLE_V);
          SIXSTEP_parameters.CurrentRegular_BEMF_ch = SIXSTEP_parameters.Regular_channel[2];
          break;

    case 6:
// PS     MotorLib_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(SIXSTEP_parameters.step_position,synchronous_rect,SIXSTEP_parameters.pulse_value);              
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
//  MotorLib_SixStep_NEXT_step                                              KEY  KEY
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

void  MotorLib_SixStep_NEXT_step()
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
      SIXSTEP_parameters.speed_fdbk     = MotorLib_GetMechSpeedRPM();
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

  MotorLib_SixStep_TABLE (SIXSTEP_parameters.step_position);

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

     MotorLib_SixStep_ADC_Channel (SIXSTEP_parameters.CurrentRegular_BEMF_ch);
   }

}


/**************************************************************************
*  MotorLib_SixStep_Alignment
*
*                         Generate the motor alignment
*/

void  MotorLib_SixStep_Alignment()
{
    SIXSTEP_parameters.step_position = 6;
    update_Step_Position (SIXSTEP_parameters.step_position);   // Trace STEP

    hCommuteTmr.Init.Period   = SIXSTEP_parameters.ARR_value;
    hCommuteTmr.Instance->ARR = (uint32_t) hCommuteTmr.Init.Period;

//  SIXSTEP_parameters.STATUS = ALIGNMENT;
    update_STATUS (ALIGNMENT);

    MotorLib_SixStep_Speed_Val_target_potentiometer();

    index_align++;
    if (index_align >= TIME_FOR_ALIGN+1)
       {
         SIXSTEP_parameters.ALIGN_OK = TRUE;
//       SIXSTEP_parameters.STATUS   = STARTUP;
         update_STATUS (STARTUP);               // 08/11/16 THIS TRIPS ALIGNMENT
         index_startup_motor = 1;

         MotorLib_SixStep_Ramp_Motor_calc();

         hCommuteTmr.Init.Prescaler = SIXSTEP_parameters.prescaler_value;
         hCommuteTmr.Instance->PSC  = hCommuteTmr.Init.Prescaler;
         index_align = 0;
       }
}



/**************************************************************************
  * @defgroup   MotorLib_Set_PI_param
  *
  * @brief Set all parameters for PI regulator
  * @param  PI_PARAM
*/

void  MotorLib_Set_PI_param (SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM)    // WVD - GOES THRU HERE
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
//  MotorLib_PI_Controller
//
//      Compute the PI output for the Current Reference
//
// @param  PI_PARAM PI parameters structure
// @param  speed_fdb motor_speed_value
// @retval int16_t Current reference
//******************************************************************************

int16_t  MotorLib_PI_Controller (SIXSTEP_PI_PARAM_InitTypeDef_t *PI_PARAM, int16_t speed_fdb)
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
//   MotorLib_Task_Speed
//
//                   Main task: Speed Loop with PI regulator
//******************************************************************************

void  MotorLib_Task_Speed (void)
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
           SIXSTEP_parameters.Current_Reference = (uint16_t) MotorLib_PI_Controller (&PI_parameters,(int16_t)SIXSTEP_parameters.speed_fdbk_filtered);
         }
        else
         {
           SIXSTEP_parameters.Current_Reference = (uint16_t) (-MotorLib_PI_Controller(&PI_parameters,(int16_t)SIXSTEP_parameters.speed_fdbk_filtered));
         }

      _g_mtrdrvr->motor_current_reference_set_value (mtr_blk, SIXSTEP_parameters.Current_Reference);
    }

   MotorLib_Bemf_Delay();
}


//******************************************************************************
//  MotorLib_SixStep_Set_Speed
//
//                     Set the new motor speed value
//******************************************************************************

void  MotorLib_SixStep_Set_Speed (uint16_t speed_value)
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
//  MotorLib_Speed_Filter
//
//                     Calculate the speed filtered
//******************************************************************************

void  MotorLib_Speed_Filter (void)
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
//  MotorLib_SixStep_Speed_Val_target_potentiometer
//
//        Calculate the Motor Speed validation threshold according to the
//        (scaled) potentiometer value
//******************************************************************************

void  MotorLib_SixStep_Speed_Val_target_potentiometer (void)
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
//  MotorLib_SixStep_Speed_Potentiometer
//
//               Calculate the potentiometer value to set the Motor Speed
//******************************************************************************

void  MotorLib_SixStep_Speed_Potentiometer (void)
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

  SIXSTEP_parameters.Speed_Ref_filtered = MotorLib_Potentiometer_filter (mean);

}


/**************************************************************************
  * @defgroup   MotorLib_Potentiometer_filter
  *
  * @brief Calculate the filtered potentiometer value
  *
  * @retval uint16_t Return the filtered potentiometer value
*/

uint16_t  MotorLib_Potentiometer_filter (uint16_t potentiometer_value)
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
*  MotorLib_GetElSpeedHz
*
*       Get the _Eletrical_ Motor Speed from ARR value of Commutation / LF TIM
*
* @retval int32_t Return the electrical motor speed
*/
int32_t  MotorLib_GetElSpeedHz (void)
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
  * @defgroup MotorLib_GetMechSpeedRPM    MotorLib_GetMechSpeedRPM
  *
  * @brief Get the Mechanical Motor Speed (RPM)
  * @retval int32_t Return the mechanical motor speed (RPM
*/

int32_t  MotorLib_GetMechSpeedRPM (void)
{
    Mech_Speed_RPM = (int32_t) (MotorLib_GetElSpeedHz() *  60 / Rotor_poles_pairs);
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
//  MotorLib_SixStep_StartMotor
//
//              Start the Motor
//              called from XZXX via HAL_GPIO_EXTI_Callback ISR on a USER button push
//******************************************************************************
void  MotorLib_SixStep_StartMotor (uint8_t motor_id, int start_type, int flags) // WVD - GOES THRU HERE ON BUTTON PUSH
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
//  MotorLib_SixStep_StopMotor
//
//                           Stop the Motor
//******************************************************************************
void  MotorLib_SixStep_StopMotor (uint8_t motor_id, int stop_type, int flags)
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

    MotorLib_SixStep_RESET (mtr_blk);   // reset/re-init everything, to prepare for a new Start
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
