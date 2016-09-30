
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                         MotorLib_Config.h
//
// If you are doing a custom motor design, use this as your starting point.
//
// Copy it into you project directory, modify it as required, 
// then rename it to MotorLib_Config.h
//
// _Top Level_ Motor and platform specific settings for Motor Control Library.
//
// These must be modified by Developer when configuring his/her system.
// It configures which MCU processor and which Motor Controller chip will
// be used in this project.
//
// The following paramters must be selected below:
//       MOTOR_DRIVER
//       MCU_FAMILY
//       MCU_SPEED_xxMHZ
//       PWM_SPEED_xxKHZ
//       ADC_RESOLUTION_xx_BIT     (some MCUs allow 10 or 12 bits resolution)
//       PULSES_PER_REVOLUTION     (when an encoder or Stepper is used)
//
// History:
//   05/16/16 - Created. Duquaine
//   07/22/16 - Added PSoC4 support. Duquaine
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

#ifndef _MOTOR_LIB_CONFIG_H_
#define   _MOTOR_LIB_CONFIG_H_

#define  MAX_NUMBER_OF_MOTORS    1              // only 1 motor used in this app

         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
         //                     Motor Driver Interface
         //
         // Select the Motor Driver being used by un-commenting the desired
         // motor deriver entry. Leave the others commented out.
         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
                 //--------------
                 //    BDC
                 //--------------
//#define MOTOR_DRIVER_STSPIN250    250   // ST STSpin250 BDC        Xnucleo bd
//#define MOTOR_DRIVER_L6206       6206   // ST L6206 Dual Brush DC  Xnucleo bd
//#define MOTOR_DRIVER_L293D        293   // ST L293D Dual Bridge    IC
//#define MOTOR_DRIVER_SN754410    7544   // TI SN754410 Dual Bridge IC
#define MOTOR_DRIVER_DRV8848     8848   // TI DRV8848 Boosterpack  BDC Board

                 //--------------
                 //   STEPPER
                 //--------------
//#define MOTOR_DRIVER_STSPIN220    220   // ST STSpin220 Stepper   Xnucleo bd
//#define MOTOR_DRIVER_L6470       6470   // ST L470 Dual Stepper   Xnucleo bd
//#define MOTOR_DRIVER_L6474       6474   // ST L474 Stepper        Xnucleo bd
//#define MOTOR_DRIVER_POWERSTEP01 6477   // ST PowerStep01         Xnucleo bd
//#define MOTOR_DRIVER_DRV8711     8711   // TI DRV8711 BoosterPack Stepper bd
//#define PSOC4_CY8CKIT_037_STEPPER 370   // PSoC4 CY8CKIT-037 Motor Control bd

                 //--------------
                 //    BLDC
                 //--------------
//#define MOTOR_DRIVER_STSPIN230    230   // ST STSpin230 BLDC        Xnucleo bd
//#define MOTOR_DRIVER_L6230       6230   // ST L6230 LV BLDC         Xnucleo bd
//#define MOTOR_DRIVER_L6398       6398   // ST L6398/STL220N6F7 LV BLDC Xnucleo (Heavy duty)
//#define MOTOR_DRIVER_DRV8301     8301   // TI DRV8301 BLDC Boosterpack
//#define MOTOR_DRIVER_DRV8305     8305   // TI DRV8305 BLDC Boosterpack
//#define PSOC4_CY8CKIT_037_BLDC    371   // PSoC4 CY8CKIT-037 Motor Control bd

                 //-------------------------------------------------------------
                 //    BLDC ALGORITHM - must use whenever config MOTOR_BLDC_xxx
                 //-------------------------------------------------------------
//#define BLDC_SENSORED_6_STEP        1   // using Hall Sensors trapezoidal 6-step
//#define BLDC_SENSORLESS_6_STEP      2   // using BEMF based trapezoidal 6-step
//#define BLDC_FOC_1_SHUNT            3   // using FOC with 1 current shunt
//#define BLDC_FOC_2_SHUNT            4   // using FOC with 2 current shunts
//#define BLDC_FOC_3_SHUNT            5   // using FOC with 3 current shunts

         //---------------------------------------------------------------------
         //                  MOTOR  CHARACTERISTICs
         //---------------------------------------------------------------------
#define  MOTOR_MAX_RPM    4000   //  Anaheim BLYxxxx-4000 maxes out at 4000/rps
//#if defined(BLY17_SERIES)
              // Anaheim Automation's BLY171-174 BLDC series.
  #ifndef MOTOR_POLE_PAIRS
  #define  MOTOR_POLE_PAIRS   8    // # pole pairs in motor
  #endif
//#define
//#endif

#define VOLTAGE_MODE        0    // 1 = for VOLTAGE mode / 0 = CURRENT mode control - L6938 PS ONLY

#define POTENTIOMETER       1    // 1 = Enable / 0 = Disable = speed potentiometer support


         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
         //                     Processor Family
         //
         // Select the MCU being used by un-commenting the desired processor.
         // Leave the others commented out.
         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
#define MSP430_G2553_LAUNCHPAD 43025    // TI MSP430_G2553 Launchpad
//#define MSP430_FR5969_LAUNCHPAD 43059    // TI MSP430_FR5969 Launchpad
//#define MSP432_LAUNCHPAD          432    // TI MSP432 Launchpad
//#define TIVA_123G_LAUNCHPAD       123    // TI Tiva 123G Launchpad
//#define TIVA_1294_LAUNCHPAD      1294    // TI Tiva 129-4 Launchpad

//#define STM32_F401_NUCLEO           401    // STMicro STM32 F401_RE Nucleo
//#define STM32_F446_NUCLEO         446    // STMicro STM32 F446_RE Nucleo

//#define PSOC4_041_S               410    // PSoC4 CY8CKIT-041 S-Series
//#define PSOC4_042_BASIC           420    // PSoC4 CY8CKIT-042  (Basic)
//#define PSOC4_042_BLE             422    // PSoC4 CY8CKIT-042-BLE
//#define PSOC4_046_L               460    // PSoC4 CY8CKIT-046 L-Series


         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
         //                       MCU_SPEED
         //
         // Select the MCU Speed being used by un-commenting the desired speed.
         // Leave the others commented out.
         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
#define  MCU_SPEED_16MHZ       16000000      // MSP430 Gxxx and Fxxx
//#define  MCU_SPEED_24MHZ       24000000      // MSP430 Fxxx
//#define  MCU_SPEED_48MHZ       48000000      // PSoC4 / MSP432 Fxxx / STM32 Lx
//#define  MCU_SPEED_84MHZ       84000000      // STM32 F4 / F3
//#define  MCU_SPEED_120MHZ     120000000      // Tiva 129, 129E, 129X,


         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
         //                       PWM_SPEED
         //
         // Select the MCU Speed being used. Leave the others commented out.
         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
//#define  PWM_SPEED_10KHZ        10000
#define  PWM_SPEED_20KHZ        20000
//#define  PWM_SPEED_30KHZ        30000
//#define  PWM_SPEED_40KHZ        40000


         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
         //                       ADC_RESOLUTION
         //
         // Select the ADC resolution being used. Leave the others commented out
         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
#define  ADC_RESOLUTION_10_BIT      1
//#define  ADC_RESOLUTION_12_BIT      1
//#define  ADC_RESOLUTION_14_BIT      1

         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
         //                  ENCODER   PULSES_PER_REVOLUTION
         //
         // The pulses per single revolution any Hall or Linear/Rotary Encoder.
         // Note: it must be specified as a floating point number.
         //---------------------------------------------------------------------
         //---------------------------------------------------------------------
#define  PULSES_PER_REVOLUTION        625.0        // Hall generates 625 pulses
                                                   //      per wheel revolution


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
//                      STM32  F4_01  NUCLEO   Macros  and  Includes  used
//
// MCU = STM32F401RE
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(STM32_F401_NUCLEO)

#define  ST_GNU_IAR_COMPILERS       1             // Denote are using GNU or IAR

#define  MCU_SPEED_84MHZ          84000000         // STM32 F401
#define  MCU_CLOCK_SPEED_84_MH z  84000000
#define  ADC_RESOLUTION_12_BIT      1              // with 12 bit ADC resolution
#define  STM32_MCU                  1              // denote is a STM32 MCU

#include "user_api.h"         // pull in high level User API defs
#include "boarddef.h"         // pull in MCU platform defs and board_xx() protos

#include "STM32_F4\stm32f4xx.h"
#include "STM32_F4\system_stm32f4xx.h"
#include "STM32_F4\stm32f4xx_it.h"

                           // Master includes file  (in project ~/Inc). It pulls
                           // in all the HAL GPIO, SPI, ... needed .H files
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"           // pull in basic MCU clock and SYSCFG defs

//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

                // Switch Configuration PB3  (Arduino D3 / Grove D3)
#define SWITCH1_CONFIG  {  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT; \
                           GPIO_InitStruct.Pull  = GPIO_NOPULL; \
                           GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; \
                           GPIO_InitStruct.Pin   = GPIO_PIN_3; \
                           HAL_GPIO_Init (GPIOB, &GPIO_InitStruct); }

#define SWITCH1_READ       HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)

#endif                                 // defined(STM32_F401_NUCLEO)



//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                      STM32  F446     NUCLEO   Macros  and  Includes  used
//
// MCU = STM32F446RE
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(STM32_F446_NUCLEO)

#define  ST_GNU_IAR_COMPILERS       1             // Denote are using GNU or IAR

#define  MCU_SPEED_180MHZ        180000000         // STM32 F446
#define  MCU_CLOCK_SPEED_180_MHz 180000000
#define  ADC_RESOLUTION_12_BIT      1              // with 12 bit ADC resolution
//#define  STM32_MCU                  1              // denote is a STM32 MCU

#include "user_api.h"         // pull in high level User API defs
#include "boarddef.h"         // pull in MCU platform defs and board_xx() protos

#include "STM32_F4\stm32f4xx.h"
#include "STM32_F4\system_stm32f4xx.h"
#include "STM32_F4\stm32f4xx_it.h"

                           // Master includes file  (in project ~/Inc). It pulls
                           // in all the HAL GPIO, SPI, ... needed .H files
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"           // pull in basic MCU clock and SYSCFG defs
#include "stm32f4xx_hal_tim_ex.h"

//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

                // Switch Configuration PB3  (Arduino D3 / Grove D3)
#define SWITCH1_CONFIG  {  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT; \
                           GPIO_InitStruct.Pull  = GPIO_NOPULL; \
                           GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; \
                           GPIO_InitStruct.Pin   = GPIO_PIN_3; \
                           HAL_GPIO_Init (GPIOB, &GPIO_InitStruct); }

#define SWITCH1_READ       HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)

#endif                                 // defined(STM32_F446_NUCLEO)



//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                      PSoC4   Macros  and  Includes  used
//
//
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(PSOC4_042_BASIC) || defined(PSOC4_042_BLE) || defined(PSOC4_041_S) || defined(PSOC4_046_L)

#define CYPRESS_COMPILER     1      // Denote we are using Cypress/Keil compiler

#define MCU_CLOCK_SPEED   48000000
#define TIMER_PRESCALER   (1024)
#define BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER    (1)
#endif                                 // defined(PSOC4_042_BASIC ...)



//******************************************************************************
//******************************************************************************
//     Pull in correct Low Level Driver (LL) based on MCU and Motor Ctl Chip
//******************************************************************************
//******************************************************************************

          //-----------------------------------------------------
          //              PSoC4  Chips and COntrollers
          //-----------------------------------------------------
          // Many of the ST BDC and STEPPER boards work with the PSoC4
#if defined(PSOC4_042_BASIC) || defined(PSOC4_042_BLE) || defined(PSOC4_041_S) || defined(PSOC4_046_L)

// Pull in low level Motor Ctl Pin and Port definitions for PSOC4 chips
#if defined(MOTOR_DRIVER_STSPIN250) || defined(MOTOR_DRIVER_L6206) || defined(MOTOR_DRIVER_L293D) || defined(MOTOR_DRIVER_SN754410)
#define  MOTOR_IS_BDC      1
#include "BDC\PSoC4\MotorLib_LL_Config_Pin_Periph_PSOC4_BDC.h"
#endif

#if defined(PSOC4_CY8CKIT_037) || defined(MOTOR_DRIVER_STSPIN220) || defined(MOTOR_DRIVER_L6470) || defined(MOTOR_DRIVER_L6474) || defined(MOTOR_DRIVER_POWERSTEP01)
#define  MOTOR_IS_STEPPER  1
#include "Stepper\PSoC4\MotorLib_LL_Config_Pin_Periph_PSOC4_STEPPER.h"
#endif

#if defined(PSOC4_CY8CKIT_037) || defined(MOTOR_DRIVER_STSPIN230)
#define  MOTOR_IS_BLDC     1
#include "BLDC\PSoC4\MotorLib_LL_Config_Pin_Periph_PSOC4_BLDC.h"
#endif
#endif            // defined(PSOC4_042_BASIC) || defined(PSOC4_042_BLE) ...


          //-----------------------------------------------------
          //                 STM32  Chips and COntrollers
          //-----------------------------------------------------
#if defined(STM32_F401_NUCLEO) || defined(STM32_F446_NUCLEO)
        // Pull in low level Motor Ctl Pin and Port definitions for STM32 chips
#if defined(MOTOR_DRIVER_STSPIN250) || defined(MOTOR_DRIVER_L6206) || defined(MOTOR_DRIVER_L293D) || defined(MOTOR_DRIVER_SN754410)
#define  MOTOR_IS_BDC      1
#include "BDC\STM32\MotorLib_LL_Config_Pin_Periph_STM32_BDC.h"
#endif

#if defined(MOTOR_DRIVER_STSPIN220) || defined(MOTOR_DRIVER_L6470) || defined(MOTOR_DRIVER_L6474) || defined(MOTOR_DRIVER_POWERSTEP01)
#define  MOTOR_IS_STEPPER  1
#include "Stepper\STM32\MotorLib_LL_Config_Pin_Periph_STM32_STEPPER.h"
#endif

#if defined(MOTOR_DRIVER_STSPIN230) || defined(MOTOR_DRIVER_L6230) || defined(MOTOR_DRIVER_L6398)
#define  MOTOR_IS_BLDC     1
#include "BLDC\STM32\MotorLib_LL_Config_Pin_Periph_STM32_BLDC.h"
#endif
#endif            // defined(STM32_F401_NUCLEO) || defined(STM32_F446_NUCLEO)...


          //-----------------------------------------------------
          //                 TI  Chips and COntrollers
          //-----------------------------------------------------
#if defined(TIVA_123G_LAUNCHPAD) || defined(MSP432_LAUNCHPAD) || defined(MSP430_FR5969_LAUNCHPAD)
        // Pull in low level Motor Ctl Pin and Port definitions for TI chips
#if defined(MOTOR_DRIVER_DRV8848) || defined(MOTOR_DRIVER_SN754410)
#define  MOTOR_IS_BDC      1
#include "BDC\TI\MotorLib_LL_Config_Pin_Periph_TI_BDC.h"
#endif

#if defined(MOTOR_DRIVER_DRV8711)
#define  MOTOR_IS_STEPPER  1
#include "Stepper\TI\MotorLib_LL_Config_Pin_Periph_TI_STEPPER.h"
#endif

#if defined(MOTOR_DRIVER_8301) || defined(MOTOR_DRIVER_8305)
#define  MOTOR_IS_BLDC      1
#include "Stepper\TO\MotorLib_LL_Config_Pin_Periph_TI_BLDC.h"
#endif
#endif              // defined(TIVA_123G_LAUNCHPAD) ...

#ifndef FALSE
  #define  FALSE             0
  #define  TRUE              1
#endif

#endif              // _MOTOR_LIB_CONFIG_H_

//******************************************************************************
