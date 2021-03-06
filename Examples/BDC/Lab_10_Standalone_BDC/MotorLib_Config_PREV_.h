
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                             MotorLib_Config.h
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
#define MOTOR_DRIVER_L6206       6206   // ST L6206 Dual Brush DC  Xnucleo bd
//#define MOTOR_DRIVER_L293D        293   // ST L293D Dual Bridge    IC
//#define MOTOR_DRIVER_SN754410    7544   // TI SN754410 Dual Bridge IC
//#define MOTOR_DRIVER_DRV8848     8848   // TI DRV8848 Boosterpack  BDC Board

                 //--------------
                 //   STEPPER
                 //--------------
//#define MOTOR_DRIVER_STSPIN220    220   // ST STSpin220 Stepper   Xnucleo bd
//#define MOTOR_DRIVER_L6470       6470   // ST L470 Dual Stepper   Xnucleo bd
//#define MOTOR_DRIVER_L6474       6474   // ST L474 Stepper        Xnucleo bd
//#define MOTOR_DRIVER_POWERSTEP01 6477   // ST PowerStep01         Xnucleo bd
//#define MOTOR_DRIVER_DRV8711     8711   // TI DRV8711 BoosterPack Stepper bd
//#define PSOC4_CY8CKIT_037          37   // PSoC4 CY8CKIT-037 Motor COntrol bd

                 //--------------
                 //    BLDC
                 //--------------
//#define MOTOR_DRIVER_STSPIN230    230   // ST STSpin230 BLDC        Xnucleo bd
//#define MOTOR_DRIVER_L6230      6230    // ST L6230 LV BLDC         Xnucleo bd
//#define MOTOR_DRIVER_L6398       6398   // ST L6398/STL220N6F7 LV BLDC Xnucleo (Heavy duty)
//#define MOTOR_DRIVER_DRV8301     8301   // TI DRV8301 BLDC Boosterpack
//#define MOTOR_DRIVER_DRV8305     8305   // TI DRV8305 BLDC Boosterpack
//#define PSOC4_CY8CKIT_037          37   // PSoC4 CY8CKIT-037 Motor COntrol bd

         //---------------------------------------------------------------------
         //                  MOTOR  CHARACTERISTICs
         //---------------------------------------------------------------------
#define  MOTOR_MAX_RPM    4000   //  Anaheim BLYxxxx-4000 maxes out at 4000/rps
//#if defined(BLY17_SERIES)
              // Anaheim Automation's BLY171-174 BLDC series.
#define  MOTOR_POLE_PAIRS   8    // # pole pairs in motor
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
//#define MSP430_FR5969_LAUNCHPAD 43059    // TI MSP430_FR5969 Launchpad
//#define MSP432_LAUNCHPAD          432    // TI MSP432 Launchpad
//#define TIVA_123G_LAUNCHPAD       123    // TI Tiva 123G Launchpad
//#define TIVA_1294_LAUNCHPAD      1294    // TI Tiva 129-4 Launchpad

#define STM32_F401_NUCLEO           401    // STMicro STM32 F401_RE Nucleo
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
//#define  MCU_SPEED_16MHZ        16000000          // MSP430 Fxxx
//#define  MCU_SPEED_24MHZ        24000000          // MSP430 Fxxx
//#define  MCU_SPEED_120MHZ      120000000          // Tiva 129, 129E, 129X,


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
//#define  ADC_RESOLUTION_10_BIT      1
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
//                   MSP430 FR5969     Macros  and  Pin Outs  used
//
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(MSP430_FR5969_LAUNCHPAD)

#define  MCU_SPEED_16MHZ         16000000         // MSP430 FR5969
#define  ADC_RESOLUTION_10_BIT      1             // with 10 bit ADC resolution
#define  MSP430_MCU                 1             // denote is a MSP430 MCU

#include "msp430.h"                               // pull in MCU defs for MSP430

                // Switch Configuration  J1-4  P2.5  GPIO    Grove J1-4 -> J1-3
#define SWITCH1_CONFIG   { P2DIR &= ~(BIT5); }    // DIR: 0 = in,  1 = out
#define SWITCH1_READ     ( P2IN  &= BIT5 )

                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
                //                   Speed  Control  Input   via   ADC
                // Comment this section out if _not_ using a Potentiomerter as a
                // Speed Control. In such case, use XXXX_xxx API to set speed reference
                //
                // The default speed control is assumed to be attached to
                // Launchpad J1-2 connector using pin P6.0 and ADC channel A15
                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
#define SPEED_CONTROL_ADC_CHANNEL   ADC14INCH_15
#define SPEED_CONTROLPIN            BIT0
#define SPEED_CONTROLPIN_MUX_SEL0   P6SEL0
#define SPEED_CONTROLPIN_MUX_SEL1   P6SEL1

#endif                             //  MSP430_FR5969_LAUNCHPAD



//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                      MSP432     Macros  and  Pin Outs  used
//
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(MSP432_LAUNCHPAD)

#define  MCU_SPEED_48MHZ         48000000         // MSP432
#define  ADC_RESOLUTION_10_BIT      1             // with 10 bit ADC resolution
#define  MSP432_MCU                 1             // denote is a MSP432 MCU

#include "msp.h"                                  // pull in MCU defs for MSP432
                // Switch Configuration
#define SWITCH1_CONFIG      { P3DIR &= ~(BIT3); } // P3.3 J1-4  DIR: 0=in, 1=out
#define SWITCH1_READ        ( P3IN  &= BIT3 )

#define CONFIG_HALL_ENCODER_A   { P4DIR &= ~(BIT1); } // P4.1 J1-5  DIR: 0=in, 1=out
#define CONFIG_HALL_ENCODER_B   { P4DIR &= ~(BIT3); } // P4.3 J1-6
#define CONFIG_HALL_ENCODER_C_Z { P4DIR &= ~(BIT6); } // P4.6 J1-8
#define READ_HALL_ENCODER_A     ( P4IN  &= BIT1 )
#define READ_HALL_ENCODER_B     ( P4IN  &= BIT3 )
#define READ_HALL_ENCODER_C_Z   ( P4IN  &= BIT6 )

#define ASSERT_CS()          (P2OUT &= ~BIT5)     // P2.5 is CS
#define DEASSERT_CS()        (P2OUT |= BIT5)


#define FLCTL_BANK0_RDCTL_WAIT__2    (2 << 12)    // TI Clock setup macros
#define FLCTL_BANK1_RDCTL_WAIT__2    (2 << 12)

                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
                //                   Speed  Control  Input   via   ADC
                // Comment this section out if _not_ using a Potentiomerter as a
                // Speed Control. In such case, use XXXX_xxx API to set speed reference
                //
                // The default speed control is assumed to be attached to
                // Launchpad J1-2 connector using pin P6.0 and ADC channel A15
                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
#define SPEED_CONTROL_ADC_CHANNEL   ADC14INCH_15
#define SPEED_CONTROLPIN            BIT0
#define SPEED_CONTROLPIN_MUX_SEL0   P6SEL0
#define SPEED_CONTROLPIN_MUX_SEL1   P6SEL1

                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
                //    PWM   CONTROL   SIGNALS  for   COMMUTATION         MSP432
                //-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
#define PWM_AH_DUTY(dvalue)     { TA0CCR4  = dvalue;  }
#define PWM_AH_PWM(dutyval)     { TA0CCR4  = dutyval; \
                                  TA0CCTL4 = OUTMOD_7; }
#define PWM_AH_PWM_INV(dutyval) { TA0CCR4  = dutyval; \
                                  TA0CCTL4 = OUTMOD_3; }
#define PWM_AH_LOW              { TA0CCTL4 = OUTMOD_0; }
#define PWM_AH_HIGH             { TA0CCTL4 = OUTMOD_0 | OUT; }  // slow decay mode causes motor to spin at full tilt anyway


#define PWM_AL_DUTY(dvalue)     { TA0CCR3  = dvalue;  }
#define PWM_AL_PWM(dutyval)     { TA0CCR3  = dutyval; \
                                  TA0CCTL3 = OUTMOD_7; }
#define PWM_AL_PWM_INV(dutyval) { TA0CCR3  = dutyval; \
                                  TA0CCTL3 = OUTMOD_3; }
#define PWM_AL_LOW              { TA0CCTL3 = OUTMOD_0; }
#define PWM_AL_HIGH             { TA0CCTL3 = OUTMOD_0 | OUT; }


#define PWM_BH_DUTY(dvalue)     { TA0CCR1  = dvalue;  }
#define PWM_BH_PWM(dutyval)     { TA0CCR1  = dutyval; \
                                  TA0CCTL1 = OUTMOD_7; }
#define PWM_BH_PWM_INV(dutyval) { TA0CCR1  = dutyval; \
                                  TA0CCTL1 = OUTMOD_3; }
#define PWM_BH_LOW              { TA0CCTL1 = OUTMOD_0; }
#define PWM_BH_HIGH             { TA0CCTL1 = OUTMOD_0 | OUT; }  // slow decay mode causes motor to spin at full tilt anyway


#define PWM_BL_DUTY(dvalue)     { TA2CCR1  = dvalue;  }
#define PWM_BL_PWM(dutyval)     { TA2CCR1  = dutyval; \
                                  TA2CCTL1 = OUTMOD_7; }
#define PWM_BL_PWM_INV(dutyval) { TA2CCR1  = dutyval; \
                                  TA2CCTL1 = OUTMOD_3; }
#define PWM_BL_LOW              { TA2CCTL1 = OUTMOD_0; }
#define PWM_BL_HIGH             { TA2CCTL1 = OUTMOD_0 | OUT; }


#define PWM_CH_DUTY(dvalue)     { TA2CCR3  = dvalue;  }
#define PWM_CH_PWM(dutyval)     { TA2CCR3  = dutyval; \
                                  TA2CCTL3 = OUTMOD_7; }
#define PWM_CH_PWM_INV(dutyval) { TA2CCR3  = dutyval; \
                                  TA2CCTL3 = OUTMOD_3; }
#define PWM_CH_LOW              { TA2CCTL3 = OUTMOD_0; }
#define PWM_CH_HIGH             { TA2CCTL3 = OUTMOD_0 | OUT; }


#define PWM_CL_DUTY(dvalue)     { TA2CCR4  = dvalue;  }
#define PWM_CL_PWM(dutyval)     { TA2CCR4  = dutyval; \
                                  TA2CCTL4 = OUTMOD_7; }
#define PWM_CL_PWM_INV(dutyval) { TA2CCR4  = dutyval; \
                                  TA2CCTL4 = OUTMOD_3; }
#define PWM_CL_LOW              { TA2CCTL4 = OUTMOD_0; }
#define PWM_CL_HIGH             { TA2CCTL4 = OUTMOD_0 | OUT; }

#endif                              // defined(MSP432_LAUNCHPAD)



//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                      Tiva 123G     Macros  and  Pin Outs  used
//
// MCU = TM4C123GH6PM
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(TIVA_123G_LAUNCHPAD)

#define  MCU_SPEED_80MHZ         80000000          // Tiva 123G
#define  ADC_RESOLUTION_12_BIT      1              // with 12 bit ADC resolution
#define  TIVA_MCU                   1              // denote is a Tiva MCU

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
//#include "inc/hw_ints.h"
#include "inc/hw_pwm.h"
#include "inc/hw_ssi.h"
#include "inc/hw_timer.h"
#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"

#include "inc/tm4c123gh6pm.h"

                // Switch Configuration PB1 (and float PB0)
#define SWITCH1_CONFIG  { MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_1); \
                          MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_0); }
#define SWITCH1_READ      MAP_GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_1)

            //-----------------------------------------------
            // DRV8711 GPIO Port and Pin Mapping Definitions
            //-----------------------------------------------
#define  nSLEEP     GPIO_PIN_2      // BDC nSleep = PB2
//#define  RESET      GPIO_PIN_5    // Stepper RESET = PA5
#define  STEP_AIN1  GPIO_PIN_6
#define  DIR_AIN2   GPIO_PIN_7
#define  BIN1       GPIO_PIN_4
#define  BIN2       GPIO_PIN_3

#define  nSTALL     GPIO_PIN_2
#define  nFAULT     GPIO_PIN_0

#define  INIT_DRV8711_CS()     MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_2)

#define  ASSERT_DRV8711_CS()   (GPIO_PORTA_DATA_R |= GPIO_PIN_2)
#define  DEASSERT_DRV8711_CS() (GPIO_PORTA_DATA_R &= ~GPIO_PIN_2)

#define  NSLEEP_HI_ENABLE      GPIO_PORTB_DATA_R |= nSLEEP
#define  NSLEEP_LO_DISABLE     GPIO_PORTB_DATA_R &= ~nSLEEP

//#define  RESET_HI_RESET        GPIO_PORTA_DATA_R |= RESET
//#define  RESET_LO_RUN          GPIO_PORTA_DATA_R &= ~RESET

#define  STEP_AIN1_HIGH        GPIO_PORTA_DATA_R |= STEP_AIN1
#define  STEP_AIN1_LOW         GPIO_PORTA_DATA_R &= ~STEP_AIN1

#define  DIR_AIN2_FORWARD      GPIO_PORTA_DATA_R |= DIR_AIN2
#define  DIR_AIN2_BACKWARD     GPIO_PORTA_DATA_R &= ~DIR_AIN2

#define  BIN1_HI               GPIO_PORTA_DATA_R |= BIN1
#define  BIN1_LO               GPIO_PORTA_DATA_R &= ~BIN1

#define  BIN2_HI               GPIO_PORTA_DATA_R |= BIN2
#define  BIN2_LO               GPIO_PORTA_DATA_R &= ~BIN2

#endif                                 // defined(TIVA_123G_LAUNCHPAD)


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
//                      STM32  F446     NUCLEO   Macros  and  Pin Outs  used
//
// MCU = STM32F446RE
//******************************************************************************
//******************************************************************************
//******************************************************************************
#if defined(STM32_F446_NUCLEO)

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
#include "BDC\PSoC4\MotorLib_LL_Config_STM32_BDC.h"
#endif

#if defined(PSOC4_CY8CKIT_037) || defined(MOTOR_DRIVER_STSPIN220) || defined(MOTOR_DRIVER_L6470) || defined(MOTOR_DRIVER_L6474) || defined(MOTOR_DRIVER_POWERSTEP01)
#define  MOTOR_IS_STEPPER  1
#include "Stepper\PSoC4\MotorLib_LL_Config_PSOC4_STEPPER.h"
#endif

#if defined(PSOC4_CY8CKIT_037) || defined(MOTOR_DRIVER_STSPIN230)
#define  MOTOR_IS_BLDC     1
#include "BLDC\PSoC4\MotorLib_LL_Config_PSOC4_BLDC.h"
#endif
#endif            // defined(PSOC4_042_BASIC) || defined(PSOC4_042_BLE) ...


          //-----------------------------------------------------
          //                 STM32  Chips and COntrollers
          //-----------------------------------------------------
#if defined(STM32_F401_NUCLEO) || defined(STM32_F446_NUCLEO)
        // Pull in low level Motor Ctl Pin and Port definitions for STM32 chips
#if defined(MOTOR_DRIVER_STSPIN250) || defined(MOTOR_DRIVER_L6206) || defined(MOTOR_DRIVER_L293D) || defined(MOTOR_DRIVER_SN754410)
#define  MOTOR_IS_BDC      1
#include "BDC\STM32\MotorLib_LL_Config_STM32_BDC.h"
#endif

#if defined(MOTOR_DRIVER_STSPIN220) || defined(MOTOR_DRIVER_L6470) || defined(MOTOR_DRIVER_L6474) || defined(MOTOR_DRIVER_POWERSTEP01)
#define  MOTOR_IS_STEPPER  1
#include "Stepper\STM32\MotorLib_LL_Config_STM32_STEPPER.h"
#endif

#if defined(MOTOR_DRIVER_STSPIN230) || defined(MOTOR_DRIVER_L6230) || defined(MOTOR_DRIVER_L6398)
#define  MOTOR_IS_BLDC     1
#include "BLDC\STM32\MotorLib_LL_Config_STM32_BLDC.h"
#endif
#endif            // defined(STM32_F401_NUCLEO) || defined(STM32_F446_NUCLEO)...


          //-----------------------------------------------------
          //                 TI  Chips and COntrollers
          //-----------------------------------------------------
#if defined(TIVA_123G_LAUNCHPAD) || defined(MSP432_LAUNCHPAD) || defined(MSP430_FR5969_LAUNCHPAD)
        // Pull in low level Motor Ctl Pin and Port definitions for TI chips
#if defined(MOTOR_DRIVER_DRV8848) || defined(MOTOR_DRIVER_SN754410)
#define  MOTOR_IS_BDC      1
#include "BDC\TI\MotorLib_LL_Config_TI_BDC.h"
#endif

#if defined(MOTOR_DRIVER_DRV8711)
#define  MOTOR_IS_STEPPER  1
#include "Stepper\TI\MotorLib_LL_Config_TI_STEPPER.h"
#endif

#if defined(MOTOR_DRIVER_8301) || defined(MOTOR_DRIVER_8305)
#define  MOTOR_IS_BLDC      1
#include "Stepper\TO\MotorLib_LL_Config_TI_BLDC.h"
#endif
#endif              // defined(TIVA_123G_LAUNCHPAD) ...

#endif              // _MOTOR_LIB_CONFIG_H_

//******************************************************************************
