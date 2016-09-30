
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                             MotorLib_Config.h
//
// Motor and platform specific settings for BLDC motor control library.
//
// The following paramters must be selected below:
//       MOTOR_DRIVER
//       MCU_FAMILY
//       MCU_SPEED_xxMHZ
//       PWM_SPEED_xxKHZ
//       ADC_RESOLUTION_xx_BIT
//       PULSES_PER_REVOLUTION
//
// History:
//   05/16/16 - Created. Duquaine
//******************************************************************************

#ifndef _MOTOR_BLDC_CONFIG_H_
#define   _MOTOR_BLDC_CONFIG_H_

#define  MAX_NUMBER_OF_MOTORS    1              // only 1 motor used in this app

         //---------------------------------------------------------------------
         //                     Motor Driver Interface
         //
         // Select the Motor Driver being used by un-commenting the desired
         // motor deriver entry. Leave the others commented out.
         //---------------------------------------------------------------------
                 //--------------
                 //    BDC
                 //--------------
//#define MOTOR_DRIVER_DRV8848    8848    // TI DRV8848 Boosterpack
//#define MOTOR_DRIVER_SN754410   7544    // TI SN754410 Dual Bridge IC
//#define MOTOR_DRIVER_L6293      6293    // L6293 Dual Bridge IC
#define MOTOR_DRIVER_L6206      6206    // ST L6206 Dual Brush DC Xnucleo board
                 //--------------
                 //   STEPPER
                 //--------------
                 //--------------
                 //    BLDC
                 //--------------
//#define MOTOR_DRIVER_DRV8305    8305    // TI DRV8305 Boosterpack
//#define MOTOR_DRIVER_DRV8301    8301    // TI DRV8301 Boosterpack
//#define MOTOR_DRIVER_L6230      6230    // ST L6230 LV BLDC Xnucleo
//#define MOTOR_DRIVER_L6398      6398    // ST L6398/STL220N6F7 LV BLDC Xnucleo (Heavy duty)

         //---------------------------------------------------------------------
         //                  MOTOR  CHARACTERISTIC
         //---------------------------------------------------------------------
#define  MOTOR_MAX_RPM    4000    //  Anaheim BLYxxxx-4000 maxes out at 4000/rps
//#if defined(BLY17_SERIES)
              // Anaheim Automation's BLY171-174 BLDC series.
#define  MOTOR_POLE_PAIRS         8       // # pole pairs in motor
//#define
//#endif


         //---------------------------------------------------------------------
         //                     Processor Family
         //
         // Select the MCU being used by un-commenting the desired processor.
         // Leave the others commented out.
         //---------------------------------------------------------------------
//#define MSP430_FR5969_LAUNCHPAD   43059    // TI MSP430_FR5969 Launchpad
//#define MSP432_LAUNCHPAD         432     // TI MSP432 Launchpad
//#define TIVA_123G_LAUNCHPAD      123     // TI Tiva 123G Launchpad
//#define TIVA_1294_LAUNCHPAD     1294     // TI Tiva 129-4 Launchpad

#define STM32_F401_NUCLEO         401      // STMicro STM32 F401_RE Nucleo
//#define STM32_F446_NUCLEO       446      // STMicro STM32 F446_RE Nucleo


         //---------------------------------------------------------------------
         //                       MCU_SPEED
         //
         // Select the MCU Speed being used by un-commenting the desired speed.
         // Leave the others commented out.
         //---------------------------------------------------------------------
//#define  MCU_SPEED_16MHZ        16000000          // MSP430 Fxxx
//#define  MCU_SPEED_24MHZ        24000000          // MSP430 Fxxx
//#define  MCU_SPEED_120MHZ      120000000          // Tiva 129, 129E, 129X,


         //---------------------------------------------------------------------
         //                       PWM_SPEED
         //
         // Select the MCU Speed being used. Leave the others commented out.
         //---------------------------------------------------------------------
//#define  PWM_SPEED_10KHZ        10000
#define  PWM_SPEED_20KHZ        20000
//#define  PWM_SPEED_30KHZ        30000
//#define  PWM_SPEED_40KHZ        40000


         //---------------------------------------------------------------------
         //                       ADC_RESOLUTION
         //
         // Select the ADC resolution being used. Leave the others commented out
         //---------------------------------------------------------------------
//#define  ADC_RESOLUTION_10_BIT      1
//#define  ADC_RESOLUTION_12_BIT      1
//#define  ADC_RESOLUTION_14_BIT      1

         //---------------------------------------------------------------------
         //                  ENCODER   PULSES_PER_REVOLUTION
         //
         // The pulses per single revolution any Hall or Linear/Rotary Encoder.
         // Note: it must be specified as a floating point number.
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
#if defined( MSP430_FR5969_LAUNCHPAD)

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

                    //-------------------------
                    //  PWM  pins    AINx/BINx
                    //-------------------------
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

                    //----------------------------
                    //             BDC
                    //
                    //  PWM  pins        AINx/BINx
                    //----------------------------
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



                    //--------------------------------
                    //              BLDC
                    //
                    //  PWM  pins     A / B / C Phase
                    //--------------------------------
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
//       SHIT SHIT SHIT
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


#endif                                 // _MOTOR_BLDC_CONFIG_H_

//******************************************************************************
