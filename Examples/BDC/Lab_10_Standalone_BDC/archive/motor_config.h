
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                              motor_BDC_config.h
//
// Motor and platform specific settings for BDC motor control library.
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
//   05/09/16 - Split off as separate module, to allow project resuse. Duquaine
//******************************************************************************

#ifndef _MOTOR_CONFIG_H_
#define   _MOTOR_CONFIG_H_

         //---------------------------------------------------------------------
         //                     Motor Driver Interface
         //
         // Select the Motor Driver being used by un-commenting the desired
         // motor deriver entry. Leave the others commented out.
         //---------------------------------------------------------------------
//#define MOTOR_DRIVER_DRV8848    8848    // TI DRV8848 Boosterpack
//#define MOTOR_DRIVER_SN754410   7544    // TI SN754410 Dual Bridge IC
//#define MOTOR_DRIVER_L6293      6293    // L6293 Dual Bridge IC
#define MOTOR_DRIVER_L6206      6206    // ST L6206 Dual Brush DC Xnucleo board


         //---------------------------------------------------------------------
         //                     Processor Family
         //
         // Select the MCU being used by un-commenting the desired processor.
         // Leave the others commented out.
         //---------------------------------------------------------------------
//#define MSP432_LAUNCHPAD       432     // TI MSP432 Launchpad
//#define TIVA_123G_LAUNCHPAD    123     // TI Tiva 123G Launchpad
//#define TIVA_1294_LAUNCHPAD   1294     // TI Tiva 129-4 Launchpad

#define STM32_F446_NUCLEO     446      // STMicro STM32 F446_RE Nucleo


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

#if defined(PWM_SPEED_10KHZ)
#define  PWM_SPEED              10000
#elif defined(PWM_SPEED_20KHZ)
#define  PWM_SPEED              20000
#elif defined(PWM_SPEED_30KHZ)
#define  PWM_SPEED              30000
#elif defined(PWM_SPEED_40KHZ)
#define  PWM_SPEED              40000
#else
#error "You must choose a valid PWM Speed"
#endif

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


//******************************************************************************
//******************************************************************************
//******************************************************************************
//
//                        Internal  Platform  Definitions
//
// Do not alter unless you really know what you are doing
//******************************************************************************
//******************************************************************************
//******************************************************************************

//******************************************************************************
//                      MSP432  Macros  and  Pin Outs  used
//******************************************************************************
#if defined(MSP432_LAUNCHPAD)

#define  MCU_SPEED_48MHZ         48000000         // MSP432
#define  ADC_RESOLUTION_10_BIT      1             // with 10 bit ADC resolution

#include "msp.h"                                  // pull in MCU defs for MSP432
                // Switch Configuration
#define SWITCH1_CONFIG   { P3DIR &= ~(BIT3); }    // DIR: 0 = in,  1 = out
#define SWITCH1_READ     ( P3IN  &= BIT3 )

#define FLCTL_BANK0_RDCTL_WAIT__2    (2 << 12)    // TI Clock setup macros
#define FLCTL_BANK1_RDCTL_WAIT__2    (2 << 12)

#endif                              // defined(MSP432_LAUNCHPAD)



//******************************************************************************
//                      Tiva 123G  Macros  and  Pin Outs  used
// MCU = TM4C123GH6PM
//******************************************************************************
#if defined(TIVA_123G_LAUNCHPAD)

#define  MCU_SPEED_80MHZ         80000000          // Tiva 123G
#define  ADC_RESOLUTION_12_BIT      1              // with 12 bit ADC resolution

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
#define SWITCH1_READ    MAP_GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_1)

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
//                      STM32  F446  NUCLEO   Macros  and  Pin Outs  used
// MCU = STM32F446RE
//******************************************************************************
#if defined(STM32_F446_NUCLEO)

#define  MCU_SPEED_180MHZ        180000000         // STM32 F446
#define  MCU_CLOCK_SPEED_180_MHz 180000000
#define  ADC_RESOLUTION_12_BIT      1              // with 12 bit ADC resolution

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

#endif                                 // defined(STM32_F446_NUCLEO)




#if defined(MCU_SPEED_16MHZ)
#define  MCU_SPEED              16000000
#elif defined(MCU_SPEED_24MHZ)
#define  MCU_SPEED              24000000
#elif defined(MCU_SPEED_48MHZ)
#define  MCU_SPEED              48000000
#elif defined(MCU_SPEED_80MHZ)
#define  MCU_SPEED              80000000
#elif defined(MCU_SPEED_120MHZ)
#define  MCU_SPEED             120000000
#elif defined(MCU_SPEED_180MHZ)
#define  MCU_SPEED             180000000
#else
#error "You must choose a valid MCU Speed"
#endif

#if defined(ADC_RESOLUTION_10_BIT)
#define ADC_SCALING_VALUE            1023.0
#elif defined(ADC_RESOLUTION_12_BIT)
#define ADC_SCALING_VALUE            4095.0
#elif defined(ADC_RESOLUTION_14_BIT)
#define ADC_SCALING_VALUE           16383.0
#else
#error "You must choose a valid ADC Resolution"
#endif


#endif                        // _MOTOR_CONFIG_H_

//******************************************************************************
