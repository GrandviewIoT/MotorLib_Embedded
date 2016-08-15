// (1) ENCODERS: not seeing any signal on encoder signal line, even on Oscscope.
//           ==> pullups on MSP432 are not strong enough.
//           ==> need to run 5V to pullups and power, divide down for MSP432 GPIO
//           ==> need to do standalone diagnostics of Encoders.
// (2) DRV8848   Current Control VRef ADC is not properly being read.   04/16/16
//           ==> problem in sequencing/logic of ADC14 for multi-channel.

   long  spi_rupts = 0;

#if ! defined(MODBUS_SUPPORT)

void  board_spi_IRQ_Handler (int motor_id);

//---------------------------------------------------------
//                        SPI  ISR  Handler
//---------------------------------------------------------
void  board_spi_IRQ_Handler (int motor_id)
{
    spi_rupts++;   // should not be called. We are polling, not using interrupts
}


//*******1*********2*********3*********4*********5*********6*********7**********
//
//                              board_support_STM32F4.c
//
// Board specific support for Tiva123G, for clocks, Systick, GPIOs, etc
//
// Handles a STM32 F4_46    STM32F446RE
//
// History:
//   05/09/16 - Carved off to provide cross_MCU support for BDC ctl lib. Duquaine
//   08/13/16 - At gpio_init(), float all key control pins, because some BDC
//              motors can create implicit pull-ups on outputs, and falsely
//              cause the motor to start before normal initialization sequence
//              completed (e.g. L6206 and cheap 12V consumer motors).
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

#include "Board_Support_Api"                // pull in high level User API defs
#include "MotorLib_Api.h"                   // pull in common definitions

    uint32_t  _g_systick_millisecs = 0;     // SysTick tick counter in 1 ms incs

    uint32_t  _g_SysClk_Ticks;      // Global to hold clock frequency (ticks/sec)
    float     _g_fusec_to_ticks;    // Float version of how many ticks in 1 usec

    uint32_t  _g_SysClk_Ticks = MCU_CLOCK_SPEED; // Global to hold clock frequency (ticks/sec)

extern     GPIO_InitTypeDef      GPIO_InitStruct;

void  board_init (long mcu_clock_rate, int option_flags);
void  board_system_clock_config (long  mcu_clock_hz, int options_flags);
void  board_stop_WDT (void);


//******************************************************************************
//                           SYSTICK    ISR
//
// Directly called from Systick ISR !
//
//                               Increment Systick 1 ms count, on every 1ms pop
//
//THIS OVERRIDEs the DEFAULT STM32 HAL Systick function weak in stm32f0xxx_hal.c
//*****************************************************************************
void  HAL_IncTick (void)
{
    _g_systick_millisecs++;                 // increment our 1 ms tick count

    motor_systick_1ms_update();             // tell motor code we got a 1ms pop
}


//*****************************************************************************
//                                HAL_GetTick
//
//            Get the current value of the "SYSTICK" style Interval Timer.
//
//THIS OVERRIDEs the DEFAULT STM32 HAL Systick function weak in stm32f0xxx_hal.c
//*****************************************************************************

uint32_t  HAL_GetTick (void)
{
    return (_g_systick_millisecs);
}


//******************************************************************************
//  board_init
//
//            Initializes board clocks and basic GPIOs.
//******************************************************************************

void  board_init (long mcu_clock_rate, int option_flags)
{
    uint32_t  SysFreq;
    uint32_t  SysCoreClk;

       //-----------------------------------------------------------------------
       // Reset all peripherals, Initialize Flash interface and Systick.
       // This calls the generic HAL_Init() in stm32f3xx_hal.c
       // which in turn does a callback to our specific peripherals GPIO init
       // for SPI, DMA, ... by invoking HAL_MspInit() in our stm32f3xx_hal_msp.c
       //-----------------------------------------------------------------------

#if defined(STM32F746NGHx) || defined(STM32F746xx)
           //------------------------------------------------------------------
           // Enable the F7 CPU caches, both Instruction (Flash) and Data (RAM)
           //------------------------------------------------------------------
    SCB_EnableICache();             // Enable F7 I-Cache
    SCB_EnableDCache();             // Enable F7 D-Cache
#endif

    board_stop_WDT();               // ensure any watchdog timer is off/disabled

#if defined(STM32F334x8)   // || defined(STM32F303xE) || defined(STM32F303xC)
        // For STM32 F3_34, we need to initialize the System Clocks first,
        // before we invoke HAL_Init(), otherwise it tries to set the Systick
        // period to 0, because default SystemCoreClock = 0 at startup on F3_34.

       //--------------------------------------
       // Invoke MCU dependent CPU clock setup
       //--------------------------------------
    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate,option_flags); // user has specified MCU speed
       else board_system_clock_config (MCU_CLOCK_SPEED,option_flags);  // use default MCU speed
    SysFreq    = HAL_RCC_GetSysClockFreq();        // ensure we now have valid clocks
    SysCoreClk = HAL_RCC_GetHCLKFreq();

    HAL_Init();                     // Invoke ST's HAL startup logic

#else

    HAL_Init();                     // Invoke ST's HAL startup logic
                                    // Note that it always enables SYSTICK timer.

       //--------------------------------------
       // Invoke MCU dependent CPU clock setup
       //--------------------------------------
    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate,
                                  option_flags); // user has specified MCU speed
       else board_system_clock_config (MCU_CLOCK_SPEED,
                                  option_flags);  // use default MCU speed

    __enable_irq();                 // Ensure interrupts enabled for SysTick
#endif

       //------------------------------------------
       // Invoke MCU dependent GPIO clock startup
       //------------------------------------------
    board_gpio_init();
}


//******************************************************************************
//  board_delay
//                    wait N number of milliseconds
//******************************************************************************
void  board_delay_ms (long wait_time)
{
    HAL_Delay ((uint32_t) wait_time);
}


//*****************************************************************************
//  board_disable_global_interrupts
//
//         Turn off Global interrupts  (for CC3000, Timer config, ...)
//
//      ASM ("cpsid i");      // CAUTION: do NOT do any SVC calls or will crash
//*****************************************************************************

void  board_disable_global_interrupts (void)
{
        //------------------------------
        // Disable processor interrupts.
        //------------------------------
     __disable_irq();              // GNU and IAR synonym
//   ___disableinterrupt();        // native IAR
}


//*****************************************************************************
//  board_enable_global_interrupts
//
//         Turn on Global interrupts  (for CC3000, Interval Timer,
//                                     PWM Timer, ADC, config ...)
//         ASM ("cpsie i");
//*****************************************************************************

void  board_enable_global_interrupts (void)
{
        //------------------------------
        // Enable processor interrupts.
        //------------------------------
     __enable_irq();              // GNU and IAR synonym
//   ___enableinterrupt();        // native IAR
}


//*****************************************************************************
//  board_error_handler
//
//      Catastrophic error occurred. Stop the train.
//*****************************************************************************

void  board_error_handler (void)
{
           /* Turn LED on */
    // BSP_LED_On (LED2);
    while (1)
      {                 // loop in here so Debugger can find it
      }
}


//*****************************************************************************
//  board_system_clock_get_frequency
//
//       Return the board's MCU clock frequency in ticks. (usually = MHz value)
//*****************************************************************************

long  board_system_clock_get_frequency (void)
{
    return (_g_SysClk_Ticks);     // return the MCU clock frequency in ticks
}


//******************************************************************************
//  board_clock_init
//
//         Configure Clock to run at 48 MHz using internal oscillator or
//         external crystal.
//******************************************************************************
int  board_clock_init (long  mcu_clock_rate,  int  use_crystal)
{
    uint32_t  SysFreq;
    uint32_t  SysCoreClk;

       //-----------------------------------------------------------------------
       // Reset all peripherals, Initialize Flash interface and Systick.
       // This calls the generic HAL_Init() in stm32f3xx_hal.c
       // which in turn does a callback to our specific peripherals GPIO init
       // for SPI, DMA, ... by invoking HAL_MspInit() in our stm32f3xx_hal_msp.c
       //-----------------------------------------------------------------------

#if defined(STM32F746NGHx) || defined(STM32F746xx)
           //------------------------------------------------------------------
           // Enable the F7 CPU caches, both Instruction (Flash) and Data (RAM)
           //------------------------------------------------------------------
    SCB_EnableICache();             // Enable F7 I-Cache
    SCB_EnableDCache();             // Enable F7 D-Cache
#endif

    board_stop_WDT();               // ensure any watchdog timer is off/disabled

#if defined(STM32F334x8)   // || defined(STM32F303xE) || defined(STM32F303xC)
        // For STM32 F3_34, we need to initialize the System Clocks first,
        // before we invoke HAL_Init(), otherwise it tries to set the Systick
        // period to 0, because default SystemCoreClock = 0 at startup on F3_34.

       //--------------------------------------
       // Invoke MCU dependent CPU clock setup
       //--------------------------------------
    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate,option_flags); // user has specified MCU speed
       else board_system_clock_config (MCU_CLOCK_SPEED,option_flags);  // use default MCU speed
    SysFreq    = HAL_RCC_GetSysClockFreq();        // ensure we now have valid clocks
    SysCoreClk = HAL_RCC_GetHCLKFreq();

    HAL_Init();                     // Invoke ST's HAL startup logic

#else

    HAL_Init();                     // Invoke ST's HAL startup logic
                                    // Note that it always enables SYSTICK timer.

       //--------------------------------------
       // Invoke MCU dependent CPU clock setup
       //--------------------------------------
    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate,
                                  0); // user has specified MCU speed
       else board_system_clock_config (MCU_CLOCK_SPEED,
                                  0);  // use default MCU speed

    __enable_irq();                 // Ensure interrupts enabled for SysTick
#endif

       // For busy_wait, 1 usec Delay = (50 MHz / 1 million) / 3 = 16.6667
       //                               (80 MHz / 1 million) / 3 = 26.6667
    _g_fusec_to_ticks = ((float) _g_SysClk_Ticks / 3000000.0);

    board_gpio_init();    // fire up needed peripheral clocks

    return (0);           // everything worked OK
}


//******************************************************************************
//  board_systick_init
//
//          Configure SYSTICK to create 1 ms periodic timer ticks, used to
//          measure time intervals.  Used by speed calculations, etc
//******************************************************************************
void  board_systick_init (void)
{
     // Nothing to do here, because HAL_Init() automatically starts
     // up the SysteickTimer with a 1 milli-second period
}


//******************************************************************************
//  board_gpio_init
//
//******************************************************************************

void  board_gpio_init (void)
{
    volatile int      rc;
    GPIO_InitTypeDef  GPIO_InitStruct;

        // Enable clocks for most common GPIO GPIO peripherals: Ports A, B, E
    __GPIOA_CLK_ENABLE();     // turn on GPIO clocks for Ports A, B, C
    __GPIOB_CLK_ENABLE();     // since they are used in many spots
    __GPIOC_CLK_ENABLE();
    __GPIOD_CLK_ENABLE();
    __GPIOE_CLK_ENABLE();

        // Ensure key ctl pins are defaulted to floating/HiZ inputs
    GPIO_InitStruct.Pin   = GPIO_PIN_0  | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                           | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
                           | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10| GPIO_PIN_11;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);
    HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);
}

//******************************************************************************


//*****************************************************************************
//  board_stop_WDT
//*****************************************************************************

void board_stop_WDT (void)
{
      // chip default is it comes up with WWDG and IWDG in reset state
      // This is done in SystemInit(), which is part of the CMSIS startup logic.
}


#if defined(STM32F401xE) || defined(STM32F401xC)

/*******************************************************************************
*
*                             F4_01   CPU  Clock  Init
*
* System Clock Configuration
*
*         The system Clock is configured as follow :
*            System Clock source            = PLL (HSI)
*            SYSCLK(Hz)                     = 84000000
*            HCLK(Hz)                       = 84000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 2
*            APB2 Prescaler                 = 1
*            HSI Frequency(Hz)              = 16000000
*            PLL_M                          = 16
*            PLL_N                          = 336
*            PLL_P                          = 4
*            PLL_Q                          = 7
*            VDD(V)                         = 3.3
*            Main regulator output voltage  = Scale2 mode
*            Flash Latency(WS)              = 2
*******************************************************************************/

void  board_system_clock_config (long mcu_clock_hz, int option_flags)
{
    RCC_ClkInitTypeDef   RCC_ClkInitStruct;
    RCC_OscInitTypeDef   RCC_OscInitStruct;

    memset (&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));  // clear struct
    memset (&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));  // clear struct

    __PWR_CLK_ENABLE();                  // Enable Power Control clock

        // The voltage scaling allows optimizing the power consumption when
        // the device is clocked below the maximum system frequency.
        // To update the voltage scaling value, refer to product datasheet
    __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE2);

       //------------------------------------
       //     setup  _g_SysClk_Ticks
       //------------------------------------
    if (mcu_clock_hz == 0)
       _g_SysClk_Ticks = MCU_CLOCK_SPEED_84_MHz; // use default clock rate = 84 MHz
       else _g_SysClk_Ticks = mcu_clock_hz;

    if (option_flags != 0)
       {     // Use external HSE Oscillator
             // Enable HSE Oscillator and activate PLL with HSE as source
         RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
         RCC_OscInitStruct.HSIState       = RCC_HSE_ON;   // EXTERNAL Crystal clock
       }
      else
       {
             // Enable internal  HSI Oscillator and activate PLL with HSI as source
         RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
         RCC_OscInitStruct.HSIState       = RCC_HSI_ON;   // INTERNAL RC clock
       }
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLM       = 16;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
       {
         board_error_handler();
       }

        // Select the PLL as the system clock source and configure the HCLK,
        // PCLK1, and PCLK2  clocks dividers to facilitate 84 MHz
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
       {
         board_error_handler();
       }

/// _g_SysClk_Ticks = MCLK_MHz * 84000000;  // save MCU clock frequency in ticks

}
#endif                                  // defined(STM32F401xE)


#if defined(STM32F429xx) || defined(STM32F446xx)

/*******************************************************************************
*
*                           F4_29  and  F4_46   CPU  Clock  Init
*
* System Clock Configuration
*         The F4_46 system Clock is configured as follows:
*            System Clock source            = PLL (HSI)
*            SYSCLK(Hz)                     = 180000000
*            HCLK(Hz)                       = 180000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 4
*            APB2 Prescaler                 = 2
*            HSI Frequency(Hz)              = 16000000
*            PLL_M                          = 16
*            PLL_N                          = 360
*            PLL_P                          = 2
*            PLL_Q                          = 7
*            PLL_R                          = 6
*            VDD(V)                         = 3.3
*            Main regulator output voltage  = Scale1 mode
*            Flash Latency(WS)              = 5
*******************************************************************************/

void  board_system_clock_config (long  mcu_clock_hz, int options_flags)
{
    RCC_ClkInitTypeDef   RCC_ClkInitStruct;
    RCC_OscInitTypeDef   RCC_OscInitStruct;
    HAL_StatusTypeDef    ret;

    memset (&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));  // clear struct
    memset (&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));  // clear struct

    ret = HAL_OK;

    __HAL_RCC_PWR_CLK_ENABLE();            // Enable Power Control clock

         // Voltage scaling allows optimizing the power consumption when
         // the device is clocked below the maximum system frequency.
         // To update the voltage scaling value, refer to product datasheet.
    __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);

       //------------------------------------
       //     setup  _g_SysClk_Ticks
       //------------------------------------
    if (mcu_clock_hz == 0)
       _g_SysClk_Ticks = MCU_CLOCK_SPEED_180_MHz; // use default clock rate = 180 MHz
       else _g_SysClk_Ticks = mcu_clock_hz;

        // Enable HSI Oscillator and activate PLL with HSI as source
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;          // INTERNAL RC clock
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM       = 16;
    RCC_OscInitStruct.PLL.PLLN       = 360;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    RCC_OscInitStruct.PLL.PLLR       = 6;
    ret = HAL_RCC_OscConfig (&RCC_OscInitStruct);
    if (ret != HAL_OK)
       {
         board_error_handler();
       }

        // Activate the OverDrive to reach the 180 MHz Frequency
    ret = HAL_PWREx_EnableOverDrive();
    if (ret != HAL_OK)
       {
         board_error_handler();
       }
        // Select PLL as system clock source and configure the HCLK,
        // PCLK1 and PCLK2 clocks dividers to facilitate 180 MHz
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    ret = HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_5);
    if (ret != HAL_OK)
       {
         board_error_handler();
       }

/// _g_SysClk_Ticks = MCLK_MHz * 1800000000; // save MCU clock frequency in ticks

}
#endif                                  // defined(STM32F446xx)

#endif                                  //  ! defined(MODBUS_SUPPORT)
