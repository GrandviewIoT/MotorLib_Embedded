
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                           board_support_PSoC4.c
//
// Contains all the board specific (PSoC) routines.
//
//
// History:
//   06/13/16 - Created as part of Open Source Motor Library project. Duquaine
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

#include <project.h>
#include "MotorLib_Api.h"              // pull in common MotorLib definitions
#include "Board_Support_Api.h"         // pull in defs for Board Support


   uint32_t   _g_systick_millisecs = 0;     // SysTick tick counter in 1 ms incs

   uint32_t   _g_SysClk_Ticks = MCU_CLOCK_SPEED; // Global to hold clock frequency (ticks/sec)

void  board_error_handler (void);           // internal APIs fwd refs
void  board_systick_init (void);
void  board_gpio_init (void);
void  board_stop_WDT (void);
void  board_system_clock_config (long  mcu_clock_hz, int options_flags);


//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//
//    These are all board_xxxx() functions          ala MODBUS terminology
//
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

//******************************************************************************
//  board_init
//
//            Initializes board clocks and basic GPIOs.
//******************************************************************************

void  board_init (long mcu_clock_rate, int option_flags)
{
    board_stop_WDT();               // ensure any watchdog timer is off/disabled

       //--------------------------------------
       // Invoke MCU dependent CPU clock setup
       //--------------------------------------
    board_system_clock_config (mcu_clock_rate, option_flags);

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
    CyDelayCycles (wait_time);
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
    CyGlobalIntDisable;       // Disable global interrupts
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
    CyGlobalIntEnable;        // Enable global interrupts.
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
}


//*****************************************************************************
//  board_stop_WDT
//*****************************************************************************

void board_stop_WDT (void)
{
      // chip default is it comes up with WWDG and IWDG in reset state
      // This is done in SystemInit(), which is part of the CMSIS startup logic.
}


//*****************************************************************************
//  board_system_clock_config
//                             System Clock Configuration
//
//*****************************************************************************

void  board_system_clock_config (long mcu_clock_hz, int option_flags)
{
       //------------------------------------
       //     setup  _g_SysClk_Ticks
       //------------------------------------
    if (mcu_clock_hz == 0)
       _g_SysClk_Ticks = MCU_CLOCK_SPEED;     // use default clock rate = 48 MHz
       else _g_SysClk_Ticks = mcu_clock_hz;

}

//******************************************************************************
