
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                             MotorLib_LL_Config_STM32_BDC.h
//
// Motor and platform specific settings for BDC motor control chips.
//
// Contains detailed settings for BDC Controller chips L6206, L6293, TNS754410.
//
//
// L6206 Xnucleo Usage
// -------------------
//       Vm     Motor 6 V supply         -
//       Gnd    Battery Ground           -
//
//       AIN1  1A   Motor 1   TIM3_CH1   PB4   Arduino D5
//       AIN2  2A   Motor 2   TIM3_CH2   PB5   Arduino D4
//
//       BIN1  1B   Motor 1   TIM2_CH1   PA0   Arduino A0
//       BIN2  2B   Motor 2   TIM2_CH2   PA1   Arduino A1
//
//    A ENABLE / OCD  Motor 1            PA10  Arduino D2
//    B ENABLE / OCD  Motor 2            PC1   Arduino A4
//
//       AOUT1  Motor 1 OUTPUT Left      -
//       AOUT2  Motor 1 OUTPUT Right     -
//       BOUT1  Motor 2 OUTPUT Left      -
//       BOUT2  Motor 2 OUTPUT Right     -
//
//       "A" side Current-Sense          PA6  Arduino D12   ADC12_IN6
//       "B" side Current-Sense          PA4  Arduino A2    ADC12_IN4
//
//        -     Speed Ctl Pot ADC        PB5  Arduino A2    ADC12_IN4   Grove A3
//        -     Fwd/Reverse Slider       PB3  Arduino D3    GPIO rupt   Grove D3
//
//        -     Hall Sensor Left         PA8  Arduino D7    GPIO rupt   Grove D7
//        -     Hall Sensor Right        PA9  Arduino D8    GPIO rupt   Grove D8
//                   White lead = Signal,  Red lead = +3.3   Black lead = Gnd
//
//  Dagu BDC Motor:  Geared, 140 RPM     (gearbox ratio 48:1) Torque: 800 gf-cm
//                   4.5 - 6.0 Volts
//                   No Load:  150ma     Stall Current: 2.75 A at 6V
//                   Measured Motor Resistance (Rload):  5.7 ohms
//
// History:
//   07/22/16 - Carved off into separate file. Duquaine
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

#ifndef _MOTOR_LL_BDC_CONFIG_H_
#define   _MOTOR_LL_BDC_CONFIG_H_

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//                        Internal  Platform  Definitions
//
// Do not alter unless you really know what you are doing
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------


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

                    //-----------------------------------
                    //-----------------------------------
                    //  PWM  pins    AINx/BINx   for BDC
                    //-----------------------------------
                    //-----------------------------------
#define  TIMER_PRESCALER    1

#define  BSP_MOTOR_CONTROL_BOARD_BRIDGE_TURN_ON_DELAY   20  // MCU wait time in ms after power bridges are enabled

#define  AIN1_PWM_TIMER    TIM3                 // Timer used for PWM_1A
#define  AIN1_PWM_CHANNEL  TIM_CHANNEL_1

#define  AIN2_PWM_TIMER    TIM3                 // Timer used for PWM_2A
#define  AIN2_PWM_CHANNEL  TIM_CHANNEL_2

#define  BIN1_PWM_TIMER    TIM2                 // Timer used for PWM_1B
#define  BIN1_PWM_CHANNEL  TIM_CHANNEL_1

#define  BIN2_PWM_TIMER    TIM2                 // Timer used for PWM_2B
#define  BIN2_PWM_CHANNEL  TIM_CHANNEL_2

//---------------------------------------------------------------------------
//       AIN1  1A   Motor 1  TIM3_CH1   PB4      Arduino D5
//       AIN2  2A   Motor 2  TIM3_CH2   PB5      Arduino D4
//---------------------------------------------------------------------------
#define  AIN1_PORT            GPIOB            //  PB4   AIN1  (1A)    TIM3_CH1
#define  AIN1_PIN             GPIO_PIN_4
#define  AIN1_PWM_MUX_CONFIG  GPIO_AF2_TIM3

#define  AIN2_PORT            GPIOB            //  PB5   AIN2  (2A)    TIM3_CH2
#define  AIN2_PIN             GPIO_PIN_5
#define  AIN2_PWM_MUX_CONFIG  GPIO_AF2_TIM3

//---------------------------------------------------------------------------
//       BIN1  1B   Motor 1  TIM2_CH1   PA0      Arduino A0
//       BIN2  2B   Motor 2  TIM2_CH2   PA1      Arduino A1
//---------------------------------------------------------------------------
#undef   BIN1_PORT
#define  BIN1_PORT            GPIOA            //  PA0   BIN1  (1B)    TIM2_CH1
#define  BIN1_PIN             GPIO_PIN_0
#define  BIN1_PWM_MUX_CONFIG  GPIO_AF1_TIM2

#undef   BIN2_PORT
#define  BIN2_PORT            GPIOA            //  PA1   BIN2  (2B)    TIM2_CH2
#define  BIN2_PIN             GPIO_PIN_1
#define  BIN2_PWM_MUX_CONFIG  GPIO_AF1_TIM2

                    //-------------------------------------------
                    // general  GPIO  pins for L6206 BDC
                    //
                    //     EN-A for AIN1/AIN2 enable  Arduino D2
                    //     EN-B for BIN1/BIN2 enable  Arduino A4
                    //-------------------------------------------
#define  AENABLE_OCD_PORT   GPIOA             //  PA10  A ENABLE / OCD  "FLAG"  Ardu D2
#define  AENABLE_OCD_PIN    GPIO_PIN_10       //        Hi = Enabled  Lo = Fault

#define  BENABLE_OCD_PORT   GPIOC             //  PC1   B ENABLE / OCD  "FLAG"  Ardu A4
#define  BENABLE_OCD_PIN    GPIO_PIN_1        //        Hi = Enabled  Lo = Fault

// Assoc Interrupt line used for L6206 Over Current Detection and Over Temperature On Bridge A
#define EXTI_FLAG_A_IRQn           (EXTI15_10_IRQn)
#define A_OCD_EXTI_IRQn            EXTI15_10_IRQn        /* PA10 - Arduino D2 */
#define A_OCD_IRQ_PRIORITY         4

// Assoc Interrupt line used for L6206 Over Current Detection and Over Temperature On Bridge B
#define EXTI_FLAG_B_IRQn           (EXTI1_IRQn)
#define B_OCD_EXTI_IRQn            EXTI1_IRQn            /* PC1 - Arduino A4 */
#define B_OCD_IRQ_PRIORITY         4

                    //------------------------
                    //  ADC  Inputs   for BDC
                    //------------------------
#define  DMA_CHANNEL_ADC1         DMA_CHANNEL_0      // all our ADCs use ADC1
#define  DMA_INSTANCE             DMA2_Stream0       // ADC1 uses DMA2_Stream0
#define  DMA_STREAM_IRQ           DMA2_Stream0_IRQn
#define  CURRENT_SENSE_A_PORT     GPIOA              // PA6  ADC12_IN6  Ardu D12
#define  CURRENT_SENSE_A_PIN      GPIO_PIN_6
#define  CURRENT_SENSE_A_CHANNEL  ADC_CHANNEL_6
#define  CURRENT_SENSE_B_PORT     GPIOA              // PA4  ADC12_IN4  Ardu A2
#define  CURRENT_SENSE_B_PIN      GPIO_PIN_4
#define  CURRENT_SENSE_B_CHANNEL  ADC_CHANNEL_4

#define  SPEED_CONTROL_PORT       GPIOB              // PB0  ADC12_IN8  Ardu A3
#define  SPEED_CONTROL_PIN        GPIO_PIN_0
#define  SPEED_CONTROL_CHANNEL    ADC_CHANNEL_8

                    //----------------------------------------------------------
                    //  HALL Encoder Inputs  Left_Wheel = PA8  Right Wheel = PA9
                    //----------------------------------------------------------
#define  HALL_ENCODER_LEFT_PORT       GPIOA              // PA8  Arduino D7
#define  HALL_ENCODER_LEFT_PIN        GPIO_PIN_8
#define  HALL_ENCODER_LEFT_EXTI_IRQn  EXTI9_5_IRQn

#define  HALL_ENCODER_RIGHT_PORT      GPIOA              // PA9  Arduino D8
#define  HALL_ENCODER_RIGHT_PIN       GPIO_PIN_9
#define  HALL_ENCODER_RIGHT_EXTI_IRQn EXTI9_5_IRQn

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
                    //             BDC
                    //
                    //  PWM  pins        AINx/BINx
                    //----------------------------------
                    //----------------------------------
#define  TIMER_PRESCALER    1

#define  BSP_MOTOR_CONTROL_BOARD_BRIDGE_TURN_ON_DELAY   20  // MCU wait time in ms after power bridges are enabled

#define  AIN1_PWM_TIMER    TIM3                       // Timer used for PWM_1A
#define  AIN1_PWM_CHANNEL  TIM_CHANNEL_1

#define  AIN2_PWM_TIMER    TIM3                       // Timer used for PWM_2A
#define  AIN2_PWM_CHANNEL  TIM_CHANNEL_2

#define  BIN1_PWM_TIMER    TIM2                       // Timer used for PWM_1B
#define  BIN1_PWM_CHANNEL  TIM_CHANNEL_1

#define  BIN2_PWM_TIMER    TIM2                       // Timer used for PWM_2B
#define  BIN2_PWM_CHANNEL  TIM_CHANNEL_2

//       AIN1  1A   Motor 1  TIM3_CH1   PB4   Arduino D5        TIM3_CH1
//       AIN2  2A   Motor 2  TIM3_CH2   PB5   Arduino D4        TIM3_CH2
#define  AIN1_PORT            GPIOB            //  PB4   AIN1  (1A)    TIM3_CH1
#define  AIN1_PIN             GPIO_PIN_4
#define  AIN1_PWM_MUX_CONFIG  GPIO_AF2_TIM3

#define  AIN2_PORT            GPIOB            //  PB5   AIN2  (2A)    TIM3_CH2
#define  AIN2_PIN             GPIO_PIN_5
#define  AIN2_PWM_MUX_CONFIG  GPIO_AF2_TIM3

//       BIN1  1B   Motor 1  TIM2_CH1   PA0   Arduino A0        TIM2_CH1
//       BIN2  2B   Motor 2  TIM2_CH2   PA1   Arduino A1        TIM2_CH2
#undef   BIN1_PORT
#define  BIN1_PORT            GPIOA            //  PA0   BIN1  (1B)    TIM2_CH1
#define  BIN1_PIN             GPIO_PIN_0
#define  BIN1_PWM_MUX_CONFIG  GPIO_AF1_TIM2

#undef   BIN2_PORT
#define  BIN2_PORT            GPIOA            //  PA1   BIN2  (2B)    TIM2_CH2
#define  BIN2_PIN             GPIO_PIN_1
#define  BIN2_PWM_MUX_CONFIG  GPIO_AF1_TIM2

                    //----------------------
                    //  ADC  Inputs
                    //----------------------
#define  DMA_CHANNEL_ADC1         DMA_CHANNEL_0      // all our ADCs use ADC1
#define  DMA_INSTANCE             DMA2_Stream0       // ADC1 uses DMA2_Stream0
#define  DMA_STREAM_IRQ           DMA2_Stream0_IRQn
#define  CURRENT_SENSE_A_PORT     GPIOA              // PA6  ADC12_IN6  Ardu D12
#define  CURRENT_SENSE_A_PIN      GPIO_PIN_6
#define  CURRENT_SENSE_A_CHANNEL  ADC_CHANNEL_6
#define  CURRENT_SENSE_B_PORT     GPIOA              // PA4  ADC12_IN4  Ardu A2
#define  CURRENT_SENSE_B_PIN      GPIO_PIN_4
#define  CURRENT_SENSE_B_CHANNEL  ADC_CHANNEL_4

#define  SPEED_CONTROL_PORT       GPIOB              // PB0  ADC12_IN8  Ardu A3
#define  SPEED_CONTROL_PIN        GPIO_PIN_0
#define  SPEED_CONTROL_CHANNEL    ADC_CHANNEL_8

                    //-------------------------------
                    // general  GPIO  pins for L6206
                    //-------------------------------
#define  AENABLE_OCD_PORT   GPIOA             //  PA10  A ENABLE / OCD  "FLAG"
#define  AENABLE_OCD_PIN    GPIO_PIN_10

#define  BENABLE_OCD_PORT   GPIOC             //  PC1   B ENABLE / OCD  "FLAG"
#define  BENABLE_OCD_PIN    GPIO_PIN_1

// Interrupt line used for L6206 Over Current Detection and over Temperature On Bridge A
#define EXTI_FLAG_A_IRQn           (EXTI15_10_IRQn)
#define A_OCD_EXTI_IRQn            EXTI15_10_IRQn       /* PA10 */
#define A_OCD_IRQ_PRIORITY         4

// Interrupt line used for L6206 Over Current Detection and over Temperature On Bridge B
#define EXTI_FLAG_B_IRQn           (EXTI1_IRQn)
#define B_OCD_EXTI_IRQn            EXTI1_IRQn           /* PC1  */
#define B_OCD_IRQ_PRIORITY         4

                    //----------------------------------------------------------
                    //  HALL Encoder Inputs  Left_Wheel = PA8  Right Wheel = PA9
                    //----------------------------------------------------------
#define  HALL_ENCODER_LEFT_PORT       GPIOA              // PA8  Arduino D7
#define  HALL_ENCODER_LEFT_PIN        GPIO_PIN_8
#define  HALL_ENCODER_LEFT_EXTI_IRQn  EXTI9_5_IRQn

#define  HALL_ENCODER_RIGHT_PORT      GPIOA              // PA9  Arduino D8
#define  HALL_ENCODER_RIGHT_PIN       GPIO_PIN_9
#define  HALL_ENCODER_RIGHT_EXTI_IRQn EXTI9_5_IRQn

#endif                                 // defined(STM32_F446_NUCLEO)


#endif                                 // _MOTOR_LL_BDC_CONFIG_H_

//******************************************************************************
