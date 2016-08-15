
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                     MotorLib_LL_Config_PSOC4_BLDC.h
//
// Motor and platform specific settings for BLDC motor control chips.
//
// Contains detailed settings for BLDC Controller chips: CY8CKIT-037
//
// History:
//   07/22/16 - Created for PSoC4. Duquaine
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

#ifndef _MOTOR_BLDC_LL_CONFIG_H_
#define   _MOTOR_BLDC_LL_CONFIG_H_


//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                      STM32  F4_01  NUCLEO   Macros  and  Pin Outs  used
//
// MCU = STM32F401RE
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(STM32_F401_NUCLEO)

#define  TIMER_PRESCALER    1

#define  BSP_MOTOR_CONTROL_BOARD_BR

                    //---------------------------------------------------------
                    //                  ADC  pins   for  BLDC             F4_01
                    //                                              Arduino
                    //   PA1     ------> ADC1_IN1   Vbus_Motor         (A1)
                    //   PA7     ------> ADC1_IN7   Phase_W_bemf       (D11)
                    //   PB0     ------> ADC1_IN8   Phase_V_bemf       (A3)
                    //   PB1     ------> ADC1_IN9   Speed Ctl Pot       -
                    //   PC1     ------> ADC1_IN11  Phase_V_curr_shunt (A4)
                    //   PC2     ------> ADC1_IN12  Temperature         -
                    //   PC3     ------> ADC1_IN13  Phase_U_bemf        - 
                    //
                    // -- Unused during Sensorless operation:
                    //   PA0     ------> ADC1_IN0   Phase_U_curr_shunt (A0)
                    //   PC0     ------> ADC1_IN10  Phase_W_curr_shunt (A5)
                    //   PC9     ------> ADC1_INxx  GP_emf _NO_ ADC on this pin
                    //---------------------------------------------------------
#define  ADC_HANDLE              hadc1
#define  ADC_MODULE              ADC1        // ADC used for BEBF, Current, ...
#define  ADC_CLK_ENABLE()        __ADC1_CLK_ENABLE()
#define  ADC_INTERRUPT_IRQn      ADC_IRQn

#define  ADC_Speed_Pot_GPIO_PORT GPIOB       // GPIO PORT and Pins for Speed Potentiometer
#define  ADC_Speed_Pot_GPIN      GPIO_PIN_1  // PB1  (--)   ADC1_IN9

#define  ADC_Vbus_Mtr_GPIO_PORT  GPIOA       // GPIO PORT and Pins for Motor Vbus
#define  ADC_Vbus_Mtr_GPIN       GPIO_PIN_1  // PA1  (A1)   ADC1_IN1
#define  ADC_Ph_U_Bemf_GPIO_PORT GPIOC       // GPIO PORT and Pins for Phase_U_bemf
#define  ADC_Ph_U_Bemf_GPIN      GPIO_PIN_3  // PC3  (--)   ADC1_IN13
#define  ADC_Ph_V_Bemf_GPIO_PORT GPIOB       // GPIO PORT and Pins for Phase_V_bemf
#define  ADC_Ph_V_Bemf_GPIN      GPIO_PIN_0  // PB0  (A3)   ADC1_IN8
#define  ADC_Ph_W_Bemf_GPIO_PORT GPIOA       // GPIO PORT and Pins for Phase_W_bemf
#define  ADC_Ph_W_Bemf_GPIN      GPIO_PIN_7  // PA7  (D11)  ADC1_IN7
#define  ADC_Ph_V_Curr_GPIO_PORT GPIOC       // GPIO PORT and Pins for Phase_V_Current_Shunt
#define  ADC_Ph_V_Curr_GPIN      GPIO_PIN_1  // PC1  (A4)   ADC1_IN11
#define  ADC_Temp_Mtr_GPIO_PORT  GPIOC       // GPIO PORT and Pins for Motor Temperature
#define  ADC_Temp_Mtr_GPIN       GPIO_PIN_2  // PC2  (--)   ADC1_IN12

#define  ADC_Ph_V_Curr_CHANNEL   ADC_CHANNEL_11  // PC1  (A4)  ADC1_IN11  CURRENT
#define  ADC_Speed_Pot_CHANNEL   ADC_CHANNEL_9   // PB1  (--)  ADC1_IN9   SPEED
#define  ADC_Vbus_Mtr_CHANNEL    ADC_CHANNEL_1   // PA1  (A1)  ADC1_IN1   VBUS
#define  ADC_Temp_Mtr_CHANNEL    ADC_CHANNEL_12  // PC2  (--)  ADC1_IN12  TEMP
#define  ADC_Ph_U_Bemf_CHANNEL   ADC_CHANNEL_13  // PC3  (--)  ADC1_IN13  BEMF1
#define  ADC_Ph_V_Bemf_CHANNEL   ADC_CHANNEL_8   // PB0  (A3)  ADC1_IN8   BEMF2
#define  ADC_Ph_W_Bemf_CHANNEL   ADC_CHANNEL_7   // PA7  (D11) ADC1_IN7   BEMF3

                    //---------------------------------------------------------
                    //                    PWM  pins   for  BLDC
                    //
                    //    Motor  U / V / W  phases   for  BLDC
                    //
                    // Note: L6230 auto-generates complementary signal,
                    //       so we feed in on single U / V / H phase pulses
                    //
                    //   PA8   ---> TIM1_CH1   U phase
                    //   PA9   ---> TIM1_CH2   V phase
                    //   PA10  ---> TIM1_CH3   W phase
                    //   PA6   ---> TIM1_BKIN  Handle errors/overcurrent cutoff
                    //   PA12  ---> TIM1_ETR
                    //---------------------------------------------------------
#define  PWM_UVW_HANDLE      htim1           //   aka HF_TIMx
#define  PWM_UVW_TIMER       TIM1            // Timer used for PWM phases U / V / W
#define  PWM_UVW_START_PERIOD  839           // the initial PWM period to use @ startup  
#define  PWM_UVW_START_DUTY    671           // the initial PWM duty   to use @ startup  
#define  PWM_U_CHANNEL       TIM_CHANNEL_1   // PA8  (D7)   TIM1_CH1 U Phase "IN1"
#define  PWM_V_CHANNEL       TIM_CHANNEL_2   // PA9  (D8)   TIM1_CH2 V Phase "IN2"
#define  PWM_W_CHANNEL       TIM_CHANNEL_3   // PA10 (D2)   TIM1_CH3 W Phase "IN3"

#define  PWM_UVW_GPIO_PORT   GPIOA           // GPIO PORT and Pins for U/V/W
#define  PWM_UVW_ALT_FUNC    GPIO_AF1_TIM1   // Alternate function code to use for config
#define  PWM_U_GPIN          GPIO_PIN_8      // PA8  (D7)   TIM1_CH1
#define  PWM_V_GPIN          GPIO_PIN_9      // PA9  (D8)   TIM1_CH2
#define  PWM_W_GPIN          GPIO_PIN_10     // PA10 (D2)   TIM1_CH3
#define  PWM_ETR_BK_GPIO_PORT GPIOA          // GPIO PORT and Pins for BKIN / ETR
#define  PWM_UVW_BKIN_GPIN   GPIO_PIN_6      // PA6  (D12)  TIM1_BKIN
#define  PWM_UVW_ETR_GPIN    GPIO_PIN_12     // PA12  --    TIM1_ETR   CN10-12

#define  PWM_ENx_PORT        GPIOC           // PWM Enable pins EN1/EN2/EN3 port 
#define  PWM_EN1_U_GPIN      GPIO_PIN_10     // PC10 "EN1" pin for PWM U phase
#define  PWM_EN2_V_GPIN      GPIO_PIN_11     // PC11 "EN2" pin for PWM V phase
#define  PWM_EN3_W_GPIN      GPIO_PIN_12     // PC12 "EN3" pin for PWM W phase

#define  PWM_UVW_TIMER_CLK_ENABLE()  __TIM1_CLK_ENABLE()
#define  PWM_UVW_ETR_INTERRUPT_IRQn  TIM1_BRK_TIM9_IRQn

#define  PHASE_U_CCR1        CCR1            // Channel 1 / HF_TIMx_CCR1
#define  PHASE_V_CCR2        CCR2            // Channel 2 / HF_TIMx_CCR2
#define  PHASE_W_CCR3        CCR3            // Channel 3 / HF_TIMx_CCR3
#define  PHASE_U_DUTY        hPwmMtr.Instance->PHASE_U_CCR1
#define  PHASE_V_DUTY        hPwmMtr.Instance->PHASE_V_CCR2
#define  PHASE_W_DUTY        hPwmMtr.Instance->PHASE_W_CCR3


                    //---------------------------------------------------------
                    //    PWM Current Reference Timer   for  BLDC
                    //---------------------------------------------------------
#define  PWM_CURRENT_REF_HANDLE       htim3      //   timer used for Current Ref DAC
#define  PWM_CURRENT_REF_TIMER        TIM3       // PWM Timer used for Current Ref DAC
#define  PWM_CURRENT_REF_START_PERIOD  839       // the initial period to use @ startup  
#define  PWM_CURRENT_REF_START_DUTY    671       // the initial duty   to use @ startup  
#define  PWM_CURRENT_REF_CHANNEL   TIM_CHANNEL_1 // PB4  (D5)    TIM3_CH1
#define  PWM_CURRENT_REF_GPIO_PORT GPIOB         // GPIO PORT and Pins for TIM3_CH1
#define  PWM_CURRENT_REF_ALT_FUNC  GPIO_AF2_TIM3 // Alternate function code to use for config
#define  PWM_CURRENT_REF_GPIN      GPIO_PIN_4    // PB4   (D5)   TIM3_CH1

#define  PWM_CURRENT_REF_CLK_ENABLE()  __TIM3_CLK_ENABLE()

                    //---------------------------------------------------------
                    //    DAC   ???   for  Tracing
                    //---------------------------------------------------------
#define  DAC_1
#define  DAC_1_CHANNEL     DAC_CHANNEL_1    // Pxx  (Dx)

                    //---------------------------------------------------------
                    //    COMMUTATION and BACK-EMF Timer   for  BLDC
                    //---------------------------------------------------------
#define  COMMUTATION_BEMF_HANDLE        htim4   //   aka LF_TIMx
#define  COMMUTATION_BEMF_TIMER         TIM4    // Timer used for COMMUTATION / Back-EMF
#define  COMMUTATION_BEMF_START_PERIOD  24000   // the initial period to use @ startup  

#define  COMMUTATION_BEMF_CLK_ENABLE()   __TIM4_CLK_ENABLE()
#define  COMMUTATION_BEMF_INTERRUPT_IRQn TIM4_IRQn

                    //--------------------------------------------------------------
                    //  COMMUTATE and ZCR debug pins - are _not_ on any schematics
                    //
                    // These get toggled at Commutate and ZCR events.
                    //--------------------------------------------------------------
#define  COMMUTATE_DBG_GPIO_PORT  GPIOC         //  GPIO port for 6Step commutation debug
#define  COMMUTATE_DBG_GPIN       GPIO_PIN_4    //  PC4 GPIO pin for 6Step commutation debug
#define  ZCR_DBG_GPIO_PORT        GPIOC         //  GPIO port for zero crossing detection debug
#define  ZCR_DBG_GPIN             GPIO_PIN_7    //  PC7 GPIO pin for zero crossing detection

                    //------------------------------------------------------------
                    //     MOTOR  FAULT  LED   GPIO
                    //------------------------------------------------------------
#define  MOTOR_FAULT_LED_GPIO_PORT  GPIOB       //         GPIO port for MOTOR LED
#define  MOTOR_FAULT_LED_GPIN       GPIO_PIN_2  //  PB2  - GPIO pin  for MOTOR LED

                    //------------------------------------------------------------
                    //     USER BUTTON   and   USER LED   GPIOs        on  F4_01
                    //------------------------------------------------------------
#define  USER_LED_GPIO_PORT     GPIOA           //         GPIO port for USER LED
#define  USER_LED_GPIN          GPIO_PIN_5      //  PA5  - GPIO pin  for USER LED
#define  USER_BUTTON_GPIO_PORT  GPIOC           //         GPIO port for USER BUTTON
#define  USER_BUTTON_GPIN       GPIO_PIN_13     //  PC13 - GPIO pin  for USER BUTTON
#define  USER_BUTTON_IRQn       EXTI15_10_IRQn  //  interrupt ISR for button press

                    //----------------------------------
                    //----------------------------------
                    //              BLDC
                    //
                    //  PWM  pins     A / B / C Phase
                    //----------------------------------
                    //----------------------------------
// L6398
//    AH_PWM                      PA8  TIM1_CH1     HIN    AIN1
//    AL_PWM                      PA7  TIM1_CH1N    LIN~   AIN2
//    BH_PWM                      PA9  TIM1_CH2            BIN1
//    BL_PWM                      PB0  TIM1_CH2N           BIN2
//    CH_PWM                      PA10 TIM1_CH3            CIN1
//    CL_PWM                      PB1  TIM1_CH3N           CIN2
//
//    ADC   Phase_A_BEMF_Voltage  PC3  ADC123_IN13
//    ADC   Phase_B_BEMF_Voltage  PC4  ADC12_IN14     or PB11 -
//    ADC   Phase_C_BEMF_Voltage  PC5  ADC12_IN15     or PB13 -
//    ADC   Phase_A_Current       PA0  ADC123_IN0
//    ADC   Phase_B_Current       PC1  ADC123_IN11
//    ADC   Phase_C_Current       PC0  ADC123_IN10
//    ADC   VBUS_Current          PA1  ADC123_IN1
//


// L6230 - higher risk oddball
//       Both of ST's BLDC drivers act more like their BDC dirvers in terms of functioning.
//       Similar concept of AIN1/AIN2 and BIN1/BIN2, extended to CIN1/CIN2
//       Acts like 3 half bridges for BLDC vs 2 half-bridges for BDC
//
//    AIN1   AH_PWM                PA8  TIM1_CH1
//    AIN2   AH_ENABLE             PC10 GPIO
//    BIN1   BH_PWM                PA9  TIM1_CH2
//    BIN2   BH_ENABLE             PC11 GPIO
//    CIN1   CH_PWM                PA10 TIM1_CH3
//    CIN2   CH_ENABLE             PC12 GPIO
//
//    ADC    Phase_A_BEMF_Voltage  PC3  ADC123_IN13
//    ADC    Phase_B_BEMF_Voltage  PB0  ADC12_IN8
//    ADC    Phase_C_BEMF_Voltage  PA7  ADC12_IN7
//    ADC    Phase_A_Current       PA0  ADC123_IN0
//    ADC    Phase_B_Current       PC1  ADC123_IN11
//    ADC    Phase_C_Current       PC0  ADC123_IN10
//    ADC    VBUS_Current          PA1  ADC123_IN1     C7_30   CAUTION: Weirdness with EmbPGA
//

// OFFICIAL DEFS FROM ST CODE  -- BEGIN --

//  #define HF_TIMx               htim1
//  #define LF_TIMx               htim4
//  #define HALL_ENCODER_TIMx     htim2
//  #define ADCx                  hadc1
//  #define DACx                  htim3
//  #define UART                  huart2

//  #define HF_TIMx_CH1           TIM_CHANNEL_1
//  #define HF_TIMx_CH2           TIM_CHANNEL_2
//  #define HF_TIMx_CH3           TIM_CHANNEL_3
//  #define HF_TIMx_CCR1          CCR1                     /* Channel 1 */
//  #define HF_TIMx_CCR2          CCR2                     /* Channel 2 */
//  #define HF_TIMx_CCR3          CCR3                     /* Channel 3 */

  #define DAC_ENABLE            0               /* Enable (1) the DAC peripheral */  

  #define GPIO_SET              GPIO_PIN_SET
  #define GPIO_RESET            GPIO_PIN_RESET

  #define ADC_CH_1_ST           ADC_SAMPLETIME_3CYCLES   // sampling time for CURRENT

  #define ADC_CH_2_ST           ADC_SAMPLETIME_84CYCLES  // sampling time for SPEED POT
  #define ADC_CH_3_ST           ADC_SAMPLETIME_84CYCLES  // sampling time VBUS
  #define ADC_CH_4_ST           ADC_SAMPLETIME_84CYCLES  // sampling time TEMP

  #define ADC_Bemf_CH1_ST       ADC_SAMPLETIME_28CYCLES  // sampling time BEMF1
  #define ADC_Bemf_CH2_ST       ADC_SAMPLETIME_28CYCLES  // sampling time BEMF2
  #define ADC_Bemf_CH3_ST       ADC_SAMPLETIME_28CYCLES  // sampling time BEMF3

//  #define GPIO_PORT_ZCR         GPIOC           /*  GPIO port name for zero crossing detection */
//  #define GPIO_CH_ZCR           GPIO_PIN_7      /*  PC7 GPIO pin name for zero crossing detection */
//  #define GPIO_PORT_COMM        GPIOC           /*  GPIO port name for 6Step commutation */
//  #define GPIO_CH_COMM          GPIO_PIN_4      /*  PC4 GPIO pin name for 6Step commutation  */

//  #define GPIO_PORT_1           GPIOC
//  #define GPIO_CH1              GPIO_PIN_10
//  #define GPIO_PORT_2           GPIOC
//  #define GPIO_CH2              GPIO_PIN_11
//  #define GPIO_PORT_3           GPIOC
//  #define GPIO_CH3              GPIO_PIN_12

//  #define ADC_CH_1              ADC_CHANNEL_11           /* CURRENT */
//  #define ADC_CH_2              ADC_CHANNEL_9            /* SPEED   */
//  #define ADC_CH_3              ADC_CHANNEL_1            /* VBUS    */
//  #define ADC_CH_4              ADC_CHANNEL_12           /* TEMP    */
//  #define ADC_Bemf_CH1          ADC_CHANNEL_13           /* BEMF1   */
//  #define ADC_Bemf_CH2          ADC_CHANNEL_8            /* BEMF2   */
//  #define ADC_Bemf_CH3          ADC_CHANNEL_7            /* BEMF3   */

// OFFICIAL DEFS FROM ST CODE  -- END --


#if (NOT_USED_L6230)
                    //------------------------------------------------------
                    //  PWM  pins    PWMs:  AH,AL  BH,BL  CH,CL
                    //
                    //    L6398 acts like a L6206, but kinda-sorta allows
                    //    a AH/AL approach via its Chan1/Chan1N wiring, It
                    //    uses 6 PWMs, but wired like a L6206 with 3 bridges.
                    //    Treat AH/AL equivalent to AIN1/AIN2 with a twist.
                    //
                    //    L6230 does not use separate AH/AL etc PWMs.
                    //    It uses just 3 PWMs total (AH/BH/CH) plus Enables.
                    //    Somewhat resembles DRV8305 3-PWM mode.
                    //-----------------------------------------------------
#define  TIMER_PRESCALER    1

#define  BSP_MOTOR_CONTROL_BOARD_BRIDGE_TURN_ON_DELAY   20  // MCU wait time in ms after power bridges are enabled

                       // --- THE FOLLOWING ARE FOR L6398.  Need different set for L6230 ---
#define  TIM_CHANNEL_1N    TIM_CHANNEL_1       // Aliases to track N vs non-N channels
#define  TIM_CHANNEL_2N    TIM_CHANNEL_2
#define  TIM_CHANNEL_3N    TIM_CHANNEL_3

#define  AH_PWM_TIMER      TIM1                // Timer used for PWM_AH
#define  AH_PWM_CHANNEL    TIM_CHANNEL_1       // PA8

#define  AL_PWM_TIMER      TIM1                // Timer used for PWM_AL
#define  AL_PWM_CHANNEL    TIM_CHANNEL_1N      // PA7

#define  BH_PWM_TIMER      TIM1                // Timer used for PWM_BH
#define  BH_PWM_CHANNEL    TIM_CHANNEL_2       // PA9

#define  BL_PWM_TIMER      TIM1                // Timer used for PWM_BL
#define  BL_PWM_CHANNEL    TIM_CHANNEL_2N      // PB0

#define  CH_PWM_TIMER      TIM1                // Timer used for PWM_CH
#define  CH_PWM_CHANNEL    TIM_CHANNEL_3       // PA10

#define  CL_PWM_TIMER      TIM1                // Timer used for PWM_CL
#define  CL_PWM_CHANNEL    TIM_CHANNEL_3N      // PB1

//          AH              PA8   Arduino xx        TIM1_CH1
//          AL              PA7   Arduino xx        TIM1_CH1N
#define  AH_PORT            GPIOA            //  PA8   AIN1  (AH)    TIM1_CH1
#define  AH_PIN             GPIO_PIN_8
#define  AH_PWM_MUX_CONFIG  GPIO_AF1_TIM1

#define  AL_PORT            GPIOA            //  PA7   AIN2  (AL)    TIM1_CH1N
#define  AL_PIN             GPIO_PIN_7
#define  AL_PWM_MUX_CONFIG  GPIO_AF1_TIM1

//          BH              PA9   Arduino xx        TIM1_CH2
//          BL              PB0   Arduino xx        TIM1_CH2N
#undef   BH_PORT
#define  BH_PORT            GPIOA            //  PA9   BIN1  (BH)    TIM1_CH2
#define  BH_PIN             GPIO_PIN_9
#define  BH_PWM_MUX_CONFIG  GPIO_AF1_TIM1

#undef   BL_PORT
#define  BL_PORT            GPIOB            //  PB0   BIN2  (BL)    TIM1_CH2N
#define  BL_PIN             GPIO_PIN_0
#define  BL_PWM_MUX_CONFIG  GPIO_AF1_TIM1

//          CH              PA10  Arduino D5        TIM1_CH3
//          CL              PB1   Arduino D4        TIM1_CH3N
#define  CH_PORT            GPIOA            //  PA10  CIN1  (CH)    TIM1_CH3
#define  CH_PIN             GPIO_PIN_10
#define  CH_PWM_MUX_CONFIG  GPIO_AF1_TIM1

#define  CL_PORT            GPIOB            //  PB1   CIN2  (CL)    TIM1_CH3N
#define  CL_PIN             GPIO_PIN_1
#define  CL_PWM_MUX_CONFIG  GPIO_AF1_TIM1


                    //----------------------
                    //  ADC  Inputs
                    //----------------------

// In the future, should put ADCs for voltage om ADC1, and ADCs for Current on ADC2, so get them simultaneously

#define  DMA_CHANNEL_ADC1         DMA_CHANNEL_0      // all our ADCs use ADC1
#define  DMA_INSTANCE             DMA2_Stream0       // ADC1 uses DMA2_Stream0
#define  DMA_STREAM_IRQ           DMA2_Stream0_IRQn

#define  VOLTAGE_BEMF_A_PORT      GPIOC              // PC3  ADC123_IN13 Ardu xx
#define  VOLTAGE_BEMF_A_PIN       GPIO_PIN_3
#define  VOLTAGE_BEMF_A_CHANNEL   ADC_CHANNEL_13
#define  VOLTAGE_BEMF_B_PORT      GPIOC              // PC4  ADC12_IN14  Ardu xx
#define  VOLTAGE_BEMF_B_PIN       GPIO_PIN_4
#define  VOLTAGE_BEMF_B_CHANNEL   ADC_CHANNEL_14
#define  VOLTAGE_BEMF_C_PORT      GPIOC              // PC5  ADC12_IN15  Ardu xx
#define  VOLTAGE_BEMF_C_PIN       GPIO_PIN_5
#define  VOLTAGE_BEMF_C_CHANNEL   ADC_CHANNEL_14
#define  VOLTAGE_VBUS_PORT        GPIOA              // PA1  ADC123_IN1  Ardu xx
#define  VOLTAGE_VBUS_PIN         GPIO_PIN_1
#define  VOLTAGE_VBUS_CHANNEL     ADC_CHANNEL_1

#define  CURRENT_SENSE_A_PORT     GPIOA              // PA0  ADC123_IN0  Ardu xx
#define  CURRENT_SENSE_A_PIN      GPIO_PIN_0
#define  CURRENT_SENSE_A_CHANNEL  ADC_CHANNEL_0
#define  CURRENT_SENSE_B_PORT     GPIOC              // PC1  ADC12_IN11  Ardu xx
#define  CURRENT_SENSE_B_PIN      GPIO_PIN_1
#define  CURRENT_SENSE_B_CHANNEL  ADC_CHANNEL_11
#define  CURRENT_SENSE_C_PORT     GPIOC              // PC0  ADC12_IN10  Ardu xx
#define  CURRENT_SENSE_C_PIN      GPIO_PIN_0
#define  CURRENT_SENSE_C_CHANNEL  ADC_CHANNEL_10

#define  SPEED_CONTROL_ADC_CHANNEL  ADC12_IN4
#define  SPEED_CONTROL_PORT         GPIOA            // PA4  ADC12_IN4   Ardu zz
#define  SPEED_CONTROL_PIN          GPIO_PIN_4
#define  SPEED_CONTROL_CHANNEL      ADC_CHANNEL_4

                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  - -
                //    PWM   CONTROL   SIGNALS  for   COMMUTATION      STM32 F4 46
                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  - -
                // Note: for STM32, only TIM1 and TIM8 support Complementary N outputs
                //       They use the same channel number, but twiddle the
                //       OC_InitTypeDef's  OCNPolarity and OCNIdleState plus
                //       special TIM_CCxNChannelCmd() and TIM_CCxN_ENABLE()
                //       to process the "N" type channels. They use the
                //       same common hTimPwm_xx base PWM timer entity.
#define PWM_AH_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_AH, AH_PWM_CHANNEL, dvalue); }
                                // Turn on PWM output, with normal Active High
#define PWM_AH_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_AH.Instance, AH_PWM_CHANNEL, TIM_CCx_ENABLE); }
                                // Turn off PWM output, with output = 0 / Low
#define PWM_AH_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_AH.Instance, AH_PWM_CHANNEL, TIM_CCx_ENABLE); }
                                // Turn off PWM output, with output = 1 / High
#define PWM_AH_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_AH.Instance, AH_PWM_CHANNEL, TIM_CCx_ENABLE); }

#define PWM_AL_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_AH, AL_PWM_CHANNEL, dvalue); }
#define PWM_AL_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AL_PWM_CHANNEL); \
                              hTimPwm_AH.Instance->CCER &= ~(TIM_CCER_CC1NE << AL_PWM_CHANNEL);   \
                              hTimPwm_AH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << AL_PWM_CHANNEL); }
#define PWM_AL_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AL_PWM_CHANNEL); \
                              hTimPwm_AH.Instance->CCER &= ~(TIM_CCER_CC1NE << AL_PWM_CHANNEL);   \
                              hTimPwm_AH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << AL_PWM_CHANNEL); }
#define PWM_AL_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AL_PWM_CHANNEL); \
                              hTimPwm_AH.Instance->CCER &= ~(TIM_CCER_CC1NE << AL_PWM_CHANNEL);   \
                              hTimPwm_AH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << AL_PWM_CHANNEL); }

#define PWM_BH_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_BH, BH_PWM_CHANNEL, dvalue); }
#define PWM_BH_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_BH.Instance, BH_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define PWM_BH_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_BH.Instance, BH_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define PWM_BH_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_BH.Instance, BH_PWM_CHANNEL, TIM_CCx_ENABLE); }

#define PWM_BL_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_BH, BL_PWM_CHANNEL, dvalue); }
#define PWM_BL_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BL_PWM_CHANNEL); \
                              hTimPwm_BH.Instance->CCER &= ~(TIM_CCER_CC1NE << BL_PWM_CHANNEL);   \
                              hTimPwm_BH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << BL_PWM_CHANNEL); }
#define PWM_BL_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BL_PWM_CHANNEL); \
                              hTimPwm_BH.Instance->CCER &= ~(TIM_CCER_CC1NE << BL_PWM_CHANNEL);   \
                              hTimPwm_BH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << BL_PWM_CHANNEL); }
#define PWM_BL_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BL_PWM_CHANNEL); \
                              hTimPwm_BH.Instance->CCER &= ~(TIM_CCER_CC1NE << BL_PWM_CHANNEL);   \
                              hTimPwm_BH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << BL_PWM_CHANNEL); }

#define PWM_CH_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_CH, CH_PWM_CHANNEL, dvalue); }
#define PWM_CH_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_CH.Instance, CH_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define PWM_CH_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_CH.Instance, CH_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define PWM_CH_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_CH.Instance, CH_PWM_CHANNEL, TIM_CCx_ENABLE); }

#define PWM_CL_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_CH, CL_PWM_CHANNEL, dvalue); }
#define PWM_CL_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CL_PWM_CHANNEL); \
                              hTimPwm_CH.Instance->CCER &= ~(TIM_CCER_CC1NE << CL_PWM_CHANNEL);   \
                              hTimPwm_CH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << CL_PWM_CHANNEL); }
#define PWM_CL_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CL_PWM_CHANNEL); \
                              hTimPwm_CH.Instance->CCER &= ~(TIM_CCER_CC1NE << CL_PWM_CHANNEL);   \
                              hTimPwm_CH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << CL_PWM_CHANNEL); }
#define PWM_CL_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CL_PWM_CHANNEL); \
                              hTimPwm_CH.Instance->CCER &= ~(TIM_CCER_CC1NE << CL_PWM_CHANNEL);   \
                              hTimPwm_CH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << CL_PWM_CHANNEL); }

                                          // PWM externs located in BLDC_motor_ctl_L6398.c
extern TIM_HandleTypeDef    hTimPwm_AH;   // PWM timer for  A-High  AH
//extern TIM_HandleTypeDef    hTimPwm_AL;   // PWM timer for  A-Low   AL
extern TIM_HandleTypeDef    hTimPwm_BH;   // PWM timer for  B-High  BH
//extern TIM_HandleTypeDef    hTimPwm_BL;   // PWM timer for  B-Low   BL
extern TIM_HandleTypeDef    hTimPwm_CH;   // PWM timer for  C-High  CH
//extern TIM_HandleTypeDef    hTimPwm_CL;   // PWM timer for  C-Low   CL
extern TIM_OC_InitTypeDef   TimPwm_OC_Config;    // PWM High/Low/PWM mode defs

extern TIM_OC_InitTypeDef   PWM_ChanConfigOC;    // PWM Channel
#endif                             // defined(NOT_USED_L6230)


                //-----------------------------------------------------
                //                      BLDC
                //
                // Hall / Encoder Configuration   PA15 Hall/Enc A
                //                                PB3  Hall/Enc B      Arduino D3
                //                                PB10 Hall/Enc C / Z
                //-----------------------------------------------------
#define HALL_ENCODER_A_PORT     GPIOA
#define HALL_ENCODER_B_PORT     GPIOB
#define HALL_ENCODER_CZ_PORT    GPIOB
#define HALL_ENCODER_A_PIN      GPIO_PIN_15          // PA15 Left encoder
#define HALL_ENCODER_B_PIN      GPIO_PIN_3           // PB3  Right encoder
#define HALL_ENCODER_CZ_PIN     GPIO_PIN_10          // PB10 Index

#define CONFIG_HALL_ENCODER_A   { GPIO_InitStruct.Mode  = GPIO_MODE_INPUT; \
                                  GPIO_InitStruct.Pull  = GPIO_NOPULL; \
                                  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; \
                                  GPIO_InitStruct.Pin   = HALL_ENCODER_A_PIN; \
                                  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct); }
#define CONFIG_HALL_ENCODER_B   { GPIO_InitStruct.Mode  = GPIO_MODE_INPUT; \
                                  GPIO_InitStruct.Pull  = GPIO_NOPULL; \
                                  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; \
                                  GPIO_InitStruct.Pin   = HALL_ENCODER_B_PIN; \
                                  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct); }
#define CONFIG_HALL_ENCODER_C_Z { GPIO_InitStruct.Mode  = GPIO_MODE_INPUT; \
                                  GPIO_InitStruct.Pull  = GPIO_NOPULL; \
                                  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; \
                                  GPIO_InitStruct.Pin   = HALL_ENCODER_C_PIN; \
                                  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct); }
#define READ_HALL_ENCODER_A     HAL_GPIO_ReadPin(GPIOA,HALL_ENCODER_A_PIN)
#define READ_HALL_ENCODER_B     HAL_GPIO_ReadPin(GPIOB,HALL_ENCODER_B_PIN)
#define READ_HALL_ENCODER_C_Z   HAL_GPIO_ReadPin(GPIOB,HALL_ENCODER_CZ_PIN)

#endif                                 // defined(STM32_F401_NUCLEO)



//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                      STM32  F446     NUCLEO   Macros  and  Pin Outs  used
//
// MCU = STM32F446RE
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(STM32_F446_NUCLEO)

                    //----------------------------------
                    //----------------------------------
                    //              BLDC
                    //
                    //  PWM  pins     A / B / C Phase
                    //----------------------------------
                    //----------------------------------
// L6398
//    AH_PWM                      PA8  TIM1_CH1     HIN    AIN1
//    AL_PWM                      PA7  TIM1_CH1N    LIN~   AIN2
//    BH_PWM                      PA9  TIM1_CH2            BIN1
//    BL_PWM                      PB0  TIM1_CH2N           BIN2
//    CH_PWM                      PA10 TIM1_CH3            CIN1
//    CL_PWM                      PB1  TIM1_CH3N           CIN2
//
//    ADC   Phase_A_BEMF_Voltage  PC3  ADC123_IN13
//    ADC   Phase_B_BEMF_Voltage  PC4  ADC12_IN14     or PB11 -
//    ADC   Phase_C_BEMF_Voltage  PC5  ADC12_IN15     or PB13 -
//    ADC   Phase_A_Current       PA0  ADC123_IN0
//    ADC   Phase_B_Current       PC1  ADC123_IN11
//    ADC   Phase_C_Current       PC0  ADC123_IN10
//    ADC   VBUS_Current          PA1  ADC123_IN1
//


// L6230 - higher risk oddball
//       Both of ST's BLDC drivers act more like their BDC dirvers in terms of functioning.
//       Similar concept of AIN1/AIN2 and BIN1/BIN2, extended to CIN1/CIN2
//       Acts like 3 half bridges for BLDC vs 2 half-bridges for BDC
//
//    AIN1   AH_PWM                PA8  TIM1_CH1
//    AIN2   AH_ENABLE             PC10 GPIO
//    BIN1   BH_PWM                PA9  TIM1_CH2
//    BIN2   BH_ENABLE             PC11 GPIO
//    CIN1   CH_PWM                PA10 TIM1_CH3
//    CIN2   CH_ENABLE             PC12 GPIO
//
//    ADC    Phase_A_BEMF_Voltage  PC3  ADC123_IN13
//    ADC    Phase_B_BEMF_Voltage  PB0  ADC12_IN8
//    ADC    Phase_C_BEMF_Voltage  PA7  ADC12_IN7
//    ADC    Phase_A_Current       PA0  ADC123_IN0
//    ADC    Phase_B_Current       PC1  ADC123_IN11
//    ADC    Phase_C_Current       PC0  ADC123_IN10
//    ADC    VBUS_Current          PA1  ADC123_IN1     C7_30   CAUTION: Weirdness with EmbPGA
//

                    //------------------------------------------------------
                    //  PWM  pins    PWMs:  AH,AL  BH,BL  CH,CL
                    //
                    //    L6398 acts like a L6206, but kinda-sorta allows
                    //    a AH/AL approach via its Chan1/Chan1N wiring, It
                    //    uses 6 PWMs, but wired like a L6206 with 3 bridges.
                    //    Treat AH/AL equivalent to AIN1/AIN2 with a twist.
                    //
                    //    L6230 does not use separate AH/AL etc PWMs.
                    //    It uses just 3 PWMs total (AH/BH/CH) plus Enables.
                    //    Somewhat resembles DRV8305 3-PWM mode.
                    //-----------------------------------------------------
#define  TIMER_PRESCALER    1

#define  BSP_MOTOR_CONTROL_BOARD_BRIDGE_TURN_ON_DELAY   20  // MCU wait time in ms after power bridges are enabled

                       // --- THE FOLLOWING ARE FOR L6398.  Need different set for L6230 ---
#define  TIM_CHANNEL_1N    TIM_CHANNEL_1       // Aliases to track N vs non-N channels
#define  TIM_CHANNEL_2N    TIM_CHANNEL_2
#define  TIM_CHANNEL_3N    TIM_CHANNEL_3

#define  AH_PWM_TIMER      TIM1                // Timer used for PWM_AH
#define  AH_PWM_CHANNEL    TIM_CHANNEL_1       // PA8

#define  AL_PWM_TIMER      TIM1                // Timer used for PWM_AL
#define  AL_PWM_CHANNEL    TIM_CHANNEL_1N      // PA7

#define  BH_PWM_TIMER      TIM1                // Timer used for PWM_BH
#define  BH_PWM_CHANNEL    TIM_CHANNEL_2       // PA9

#define  BL_PWM_TIMER      TIM1                // Timer used for PWM_BL
#define  BL_PWM_CHANNEL    TIM_CHANNEL_2N      // PB0

#define  CH_PWM_TIMER      TIM1                // Timer used for PWM_CH
#define  CH_PWM_CHANNEL    TIM_CHANNEL_3       // PA10

#define  CL_PWM_TIMER      TIM1                // Timer used for PWM_CL
#define  CL_PWM_CHANNEL    TIM_CHANNEL_3N      // PB1

//          AH              PA8   Arduino xx        TIM1_CH1
//          AL              PA7   Arduino xx        TIM1_CH1N
#define  AH_PORT            GPIOA            //  PA8   AIN1  (AH)    TIM1_CH1
#define  AH_PIN             GPIO_PIN_8
#define  AH_PWM_MUX_CONFIG  GPIO_AF1_TIM1

#define  AL_PORT            GPIOA            //  PA7   AIN2  (AL)    TIM1_CH1N
#define  AL_PIN             GPIO_PIN_7
#define  AL_PWM_MUX_CONFIG  GPIO_AF1_TIM1

//          BH              PA9   Arduino xx        TIM1_CH2
//          BL              PB0   Arduino xx        TIM1_CH2N
#undef   BH_PORT
#define  BH_PORT            GPIOA            //  PA9   BIN1  (BH)    TIM1_CH2
#define  BH_PIN             GPIO_PIN_9
#define  BH_PWM_MUX_CONFIG  GPIO_AF1_TIM1

#undef   BL_PORT
#define  BL_PORT            GPIOB            //  PB0   BIN2  (BL)    TIM1_CH2N
#define  BL_PIN             GPIO_PIN_0
#define  BL_PWM_MUX_CONFIG  GPIO_AF1_TIM1

//          CH              PA10  Arduino D5        TIM1_CH3
//          CL              PB1   Arduino D4        TIM1_CH3N
#define  CH_PORT            GPIOA            //  PA10  CIN1  (CH)    TIM1_CH3
#define  CH_PIN             GPIO_PIN_10
#define  CH_PWM_MUX_CONFIG  GPIO_AF1_TIM1

#define  CL_PORT            GPIOB            //  PB1   CIN2  (CL)    TIM1_CH3N
#define  CL_PIN             GPIO_PIN_1
#define  CL_PWM_MUX_CONFIG  GPIO_AF1_TIM1


                    //----------------------
                    //  ADC  Inputs
                    //----------------------

// In the future, should put ADCs for voltage om ADC1, and ADCs for Current on ADC2, so get them simultaneously

#define  DMA_CHANNEL_ADC1         DMA_CHANNEL_0      // all our ADCs use ADC1
#define  DMA_INSTANCE             DMA2_Stream0       // ADC1 uses DMA2_Stream0
#define  DMA_STREAM_IRQ           DMA2_Stream0_IRQn

#define  VOLTAGE_BEMF_A_PORT      GPIOC              // PC3  ADC123_IN13 Ardu xx
#define  VOLTAGE_BEMF_A_PIN       GPIO_PIN_3
#define  VOLTAGE_BEMF_A_CHANNEL   ADC_CHANNEL_13
#define  VOLTAGE_BEMF_B_PORT      GPIOC              // PC4  ADC12_IN14  Ardu xx
#define  VOLTAGE_BEMF_B_PIN       GPIO_PIN_4
#define  VOLTAGE_BEMF_B_CHANNEL   ADC_CHANNEL_14
#define  VOLTAGE_BEMF_C_PORT      GPIOC              // PC5  ADC12_IN15  Ardu xx
#define  VOLTAGE_BEMF_C_PIN       GPIO_PIN_5
#define  VOLTAGE_BEMF_C_CHANNEL   ADC_CHANNEL_14
#define  VOLTAGE_VBUS_PORT        GPIOA              // PA1  ADC123_IN1  Ardu xx
#define  VOLTAGE_VBUS_PIN         GPIO_PIN_1
#define  VOLTAGE_VBUS_CHANNEL     ADC_CHANNEL_1

#define  CURRENT_SENSE_A_PORT     GPIOA              // PA0  ADC123_IN0  Ardu xx
#define  CURRENT_SENSE_A_PIN      GPIO_PIN_0
#define  CURRENT_SENSE_A_CHANNEL  ADC_CHANNEL_0
#define  CURRENT_SENSE_B_PORT     GPIOC              // PC1  ADC12_IN11  Ardu xx
#define  CURRENT_SENSE_B_PIN      GPIO_PIN_1
#define  CURRENT_SENSE_B_CHANNEL  ADC_CHANNEL_11
#define  CURRENT_SENSE_C_PORT     GPIOC              // PC0  ADC12_IN10  Ardu xx
#define  CURRENT_SENSE_C_PIN      GPIO_PIN_0
#define  CURRENT_SENSE_C_CHANNEL  ADC_CHANNEL_10

#define  SPEED_CONTROL_ADC_CHANNEL  ADC12_IN4
#define  SPEED_CONTROL_PORT         GPIOA            // PA4  ADC12_IN4   Ardu zz
#define  SPEED_CONTROL_PIN          GPIO_PIN_4
#define  SPEED_CONTROL_CHANNEL      ADC_CHANNEL_4

                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  - -
                //    PWM   CONTROL   SIGNALS  for   COMMUTATION      STM32 F4 46
                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  - -
                // Note: for STM32, only TIM1 and TIM8 support Complementary N outputs
                //       They use the same channel number, but twiddle the
                //       OC_InitTypeDef's  OCNPolarity and OCNIdleState plus
                //       special TIM_CCxNChannelCmd() and TIM_CCxN_ENABLE()
                //       to process the "N" type channels. They use the
                //       same common hTimPwm_xx base PWM timer entity.
#define PWM_AH_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_AH, AH_PWM_CHANNEL, dvalue); }
                                // Turn on PWM output, with normal Active High
#define PWM_AH_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_AH.Instance, AH_PWM_CHANNEL, TIM_CCx_ENABLE); }
                                // Turn off PWM output, with output = 0 / Low
#define PWM_AH_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_AH.Instance, AH_PWM_CHANNEL, TIM_CCx_ENABLE); }
                                // Turn off PWM output, with output = 1 / High
#define PWM_AH_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_AH.Instance, AH_PWM_CHANNEL, TIM_CCx_ENABLE); }

#define PWM_AL_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_AH, AL_PWM_CHANNEL, dvalue); }
#define PWM_AL_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AL_PWM_CHANNEL); \
                              hTimPwm_AH.Instance->CCER &= ~(TIM_CCER_CC1NE << AL_PWM_CHANNEL);   \
                              hTimPwm_AH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << AL_PWM_CHANNEL); }
#define PWM_AL_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AL_PWM_CHANNEL); \
                              hTimPwm_AH.Instance->CCER &= ~(TIM_CCER_CC1NE << AL_PWM_CHANNEL);   \
                              hTimPwm_AH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << AL_PWM_CHANNEL); }
#define PWM_AL_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_AH, &TimPwm_OC_Config, AL_PWM_CHANNEL); \
                              hTimPwm_AH.Instance->CCER &= ~(TIM_CCER_CC1NE << AL_PWM_CHANNEL);   \
                              hTimPwm_AH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << AL_PWM_CHANNEL); }

#define PWM_BH_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_BH, BH_PWM_CHANNEL, dvalue); }
#define PWM_BH_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_BH.Instance, BH_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define PWM_BH_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_BH.Instance, BH_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define PWM_BH_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_BH.Instance, BH_PWM_CHANNEL, TIM_CCx_ENABLE); }

#define PWM_BL_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_BH, BL_PWM_CHANNEL, dvalue); }
#define PWM_BL_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BL_PWM_CHANNEL); \
                              hTimPwm_BH.Instance->CCER &= ~(TIM_CCER_CC1NE << BL_PWM_CHANNEL);   \
                              hTimPwm_BH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << BL_PWM_CHANNEL); }
#define PWM_BL_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BL_PWM_CHANNEL); \
                              hTimPwm_BH.Instance->CCER &= ~(TIM_CCER_CC1NE << BL_PWM_CHANNEL);   \
                              hTimPwm_BH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << BL_PWM_CHANNEL); }
#define PWM_BL_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_BH, &TimPwm_OC_Config, BL_PWM_CHANNEL); \
                              hTimPwm_BH.Instance->CCER &= ~(TIM_CCER_CC1NE << BL_PWM_CHANNEL);   \
                              hTimPwm_BH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << BL_PWM_CHANNEL); }

#define PWM_CH_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_CH, CH_PWM_CHANNEL, dvalue); }
#define PWM_CH_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_CH.Instance, CH_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define PWM_CH_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_CH.Instance, CH_PWM_CHANNEL, TIM_CCx_ENABLE); }
#define PWM_CH_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CH_PWM_CHANNEL); \
                              TIM_CCxChannelCmd (hTimPwm_CH.Instance, CH_PWM_CHANNEL, TIM_CCx_ENABLE); }

#define PWM_CL_DUTY(dvalue) { __HAL_TIM_SetCompare (&hTimPwm_CH, CL_PWM_CHANNEL, dvalue); }
#define PWM_CL_PWM(dutyval) { TimPwm_OC_Config.OCMode = TIM_OCMODE_PWM1; \
                              TimPwm_OC_Config.Pulse  = dutyval;  \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CL_PWM_CHANNEL); \
                              hTimPwm_CH.Instance->CCER &= ~(TIM_CCER_CC1NE << CL_PWM_CHANNEL);   \
                              hTimPwm_CH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << CL_PWM_CHANNEL); }
#define PWM_CL_LOW          { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_INACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CL_PWM_CHANNEL); \
                              hTimPwm_CH.Instance->CCER &= ~(TIM_CCER_CC1NE << CL_PWM_CHANNEL);   \
                              hTimPwm_CH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << CL_PWM_CHANNEL); }
#define PWM_CL_HIGH         { TimPwm_OC_Config.OCMode = TIM_OCMODE_FORCED_ACTIVE; \
                              _g_PWM_rc = HAL_TIM_PWM_ConfigChannel (&hTimPwm_CH, &TimPwm_OC_Config, CL_PWM_CHANNEL); \
                              hTimPwm_CH.Instance->CCER &= ~(TIM_CCER_CC1NE << CL_PWM_CHANNEL);   \
                              hTimPwm_CH.Instance->CCER |= (uint32_t)(TIM_CCxN_ENABLE << CL_PWM_CHANNEL); }

                                          // PWM externs located in BLDC_motor_ctl_L6398.c
extern TIM_HandleTypeDef    hTimPwm_AH;   // PWM timer for  A-High  AH
//extern TIM_HandleTypeDef    hTimPwm_AL;   // PWM timer for  A-Low   AL
extern TIM_HandleTypeDef    hTimPwm_BH;   // PWM timer for  B-High  BH
//extern TIM_HandleTypeDef    hTimPwm_BL;   // PWM timer for  B-Low   BL
extern TIM_HandleTypeDef    hTimPwm_CH;   // PWM timer for  C-High  CH
//extern TIM_HandleTypeDef    hTimPwm_CL;   // PWM timer for  C-Low   CL
extern TIM_OC_InitTypeDef   TimPwm_OC_Config;    // PWM High/Low/PWM mode defs
extern TIM_OC_InitTypeDef   PWM_ChanConfigOC;    // PWM Channel

                //-----------------------------------------------------
                //                      BLDC
                //
                // Hall / Encoder Configuration   PA15 Hall/Enc A
                //                                PB3  Hall/Enc B      Arduino D3
                //                                PB10 Hall/Enc C / Z
                //-----------------------------------------------------
#define HALL_ENCODER_A_PORT     GPIOA
#define HALL_ENCODER_B_PORT     GPIOB
#define HALL_ENCODER_CZ_PORT    GPIOB
#define HALL_ENCODER_A_PIN      GPIO_PIN_15          // PA15 Left encoder
#define HALL_ENCODER_B_PIN      GPIO_PIN_3           // PB3  Right encoder
#define HALL_ENCODER_CZ_PIN     GPIO_PIN_10          // PB10 Index

#define CONFIG_HALL_ENCODER_A   { GPIO_InitStruct.Mode  = GPIO_MODE_INPUT; \
                                  GPIO_InitStruct.Pull  = GPIO_NOPULL; \
                                  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; \
                                  GPIO_InitStruct.Pin   = HALL_ENCODER_A_PIN; \
                                  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct); }
#define CONFIG_HALL_ENCODER_B   { GPIO_InitStruct.Mode  = GPIO_MODE_INPUT; \
                                  GPIO_InitStruct.Pull  = GPIO_NOPULL; \
                                  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; \
                                  GPIO_InitStruct.Pin   = HALL_ENCODER_B_PIN; \
                                  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct); }
#define CONFIG_HALL_ENCODER_C_Z { GPIO_InitStruct.Mode  = GPIO_MODE_INPUT; \
                                  GPIO_InitStruct.Pull  = GPIO_NOPULL; \
                                  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; \
                                  GPIO_InitStruct.Pin   = HALL_ENCODER_C_PIN; \
                                  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct); }
#define READ_HALL_ENCODER_A     HAL_GPIO_ReadPin(GPIOA,HALL_ENCODER_A_PIN)
#define READ_HALL_ENCODER_B     HAL_GPIO_ReadPin(GPIOB,HALL_ENCODER_B_PIN)
#define READ_HALL_ENCODER_C_Z   HAL_GPIO_ReadPin(GPIOB,HALL_ENCODER_CZ_PIN)

#endif                                 // defined(STM32_F446_NUCLEO)


#endif                                 // _MOTOR_BLDC_LL_CONFIG_H_

//******************************************************************************
