/********1*********2*********3*********4*********5*********6*********7**********
*
*                              board_tiva_123g.c
*
*
* History:
*   12/10/14 - Significantly revised for IoT/PLC project. Duquaine
*   04/08/15 - Added a simple string edit for board_uart_read_string() support.
*
* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
*
* The MIT License (MIT)
*
* Copyright (c) 2014-2015, 2016 Wayne Duquaine / Grandview Systems
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*******************************************************************************/

#include "Board_Support_Api"                // pull in high level User API defs
#include "boarddef.h"

//---------------------------------------------------------
//                    Global Variables
//---------------------------------------------------------

    uint32_t  _g_SysClk_Ticks;      // Global to hold clock frequency (ticks/sec)
    float     _g_fusec_to_ticks;    // Float version of how many ticks in 1 usec

    uint32_t  _g_systick_millisecs = 0; // Global to how many 1 ms ticks have
                                        // accumulated startup (poor mans TOD)

    char      _g_pwm_mclock_configured = 0;      // PWM master clock was configured
    uint32_t  _g_pwm_mdivider = 1;  // PWM Master clock : CPU divider ratio

    uint32_t  _g_PWM_IO_clock = 0;  // Global to hold PWM Max Clock Frequency
    uint32_t  _g_PWMperiod    = 0;  // Global to hold PWM period  (MaxClock/Freq)
    uint32_t  _g_PWMduty      = 0;  // Global to hold latest PWM duty cycle
    uint32_t  _g_PWM_RuptFlags = 0;


//**************************************************************************
//**************************************************************************
//
//                     BOARD   -   COMMON   Routines
//
//**************************************************************************
//**************************************************************************

//*****************************************************************************
//  board_init
//
//           Initialize system clocks, and basic GPIOs
//*****************************************************************************

void  board_init (long mcu_clock_rate)
{
    board_stop_WDT();

    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate); // user specified clock speed
       else board_system_clock_config (80000000);  // use default: run at 80 MHz

    board_gpio_init();              // turn on key GPIO clocks, ...

    board_systick_timer_config();   // ensure "Systick" timer is turned on
}


//*****************************************************************************
//  board_delay_ms
//
//           Continuously loop until a fixed time delay has completed.
//           This function provides a means of generating a constant
//           length. The function delay (in cycles) = 3 * parameter, so
//           divide the SystemClock to yield the approperiate usec value.
//             1 usec Delay = (50 MHz / 1 million) / 3 = 16.6667
//             1 usec Delay = (80 MHz / 1 million) / 3 = 26.6667
//*****************************************************************************

void  board_delay_ms (long msec_delay)
{
   long  total_ticks;

   total_ticks = (long) (_g_fusec_to_ticks * (float) msec_delay);

   MAP_SysCtlDelay (total_ticks * 1000);
}


//*****************************************************************************
//  board_disable_global_interrupts
//
//         Turn off Global interrupts  (for Interval Timer, ...)
//*****************************************************************************
void  board_disable_global_interrupts (void)
{
    IntMasterDisable();
}


//*****************************************************************************
//  board_enable_global_interrupts
//
//         Turn on Global interrupts  (for Interval Timer,
//                                     PWM Timer, ADC, ...)
//*****************************************************************************
void  board_enable_global_interrupts (void)
{
    IntMasterEnable();
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
//  board_frequency_to_period_ticks
//
//      Takes a frequency (e.g. for PWM or Timer), and converts it to the 
//      equivalent clock ticks period value to be loaded into PWM registers.
//      If the result value would be too large, PWM/Timer code would have
//      to further sub-divide it using pre-scalars
//*****************************************************************************

long  board_frequency_to_period_ticks (long frequency)
{
    long   pticks;

        // take the current clock frequency, and compute the associated ticks
        // needed to generate that frequency.
     pticks = _g_SysClk_Ticks / frequency;

     return (pticks);
}




//*****************************************************************************
//*****************************************************************************
//                               GPIO   Routines
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_gpio_init
//
//          Configure Port Directions and Peripherals as needed
//*****************************************************************************
void  board_gpio_init (void)
{
        // Enable clocks for most common GPIO GPIO peripherals: Ports A, B, E
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
}



//*****************************************************************************
//*****************************************************************************
//                          System CPU / Systick   Routines
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_stop_WDT
//
//           Shutoff WDT
//*****************************************************************************

void  board_stop_WDT (void)
{
    // WDT is disabled on startup on Tiva
}


//*****************************************************************************
//  board_system_clock_config
//
//          Setup CPU clocks.
//          Run Tiva 123G CPU at 50 MHz.
//*****************************************************************************

void  board_system_clock_config (long  mcu_clock_hz)
{
       // The FPU should be enabled because some compilers use floating-
       // point registers, even for non-floating-point code.  
       // If the FPU is not enabled this will cause a fault.
       // This also ensures that floating-point operations could be
       // added to this application and would work correctly and use the
       // hardware floating-point unit. Finally, lazy stacking is 
       // enabled for interrupt handlers.  This allows floating-point
       // instructions to be used within interrupt handlers, but at the
       // expense of extra stack usage.
    FPUEnable();
    FPULazyStackingEnable();

       // Initialize the device using the on-board 16 MHz Crystal as clock.
       // The PLL multiplies the 16 MHz crystal up to 200 MHz.
       // For 20 MHz, configure M4 Clock with PLL to (200 MHz /10) = SYSCTL_SYSDIV_10
       // For 50 MHz, configure M4 Clock with PLL to (200 MHz / 4) = SYSCTL_SYSDIV_4
       // For 80 MHz, configure M4 Clock with PLL to (200 MHz/2.5) = SYSCTL_SYSDIV_2_5
    MAP_SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                        | SYSCTL_XTAL_16MHZ);

    _g_SysClk_Ticks = MAP_SysCtlClockGet();   // save total # clock ticks/sec

       // For busy_wait, 1 usec Delay = (50 MHz / 1 million) / 3 = 16.6667
       //                               (80 MHz / 1 million) / 3 = 26.6667
    _g_fusec_to_ticks = ((float) _g_SysClk_Ticks / 3000000.0);
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


//*****************************************************************************
//  board_sys_IO_clock_get_frequency
//
//       Return the board's Peripheral IO clock frequency in ticks.
//       On Tiva, we set I/O clock to same rate as MCU clock.
//*****************************************************************************

long  board_sys_IO_clock_get_frequency (void)
{
    return (_g_SysClk_Ticks);     // return the I/O clock frequency in ticks
}


//*****************************************************************************
// board_systick_timer_config
//
//          Provide a "SYSTICK" style Interval Timer.
//
//*****************************************************************************
void  board_systick_timer_config (void)
{
        // Initialize the SysTick Timer and its interrupt
    MAP_SysTickPeriodSet (SysCtlClockGet() / 1000);   // 1 millisec interrupts
    MAP_SysTickEnable();
    MAP_SysTickIntEnable();
    IntMasterEnable();
}


//*****************************************************************************
// board_systick_timer_get_value
//
//            Get the current value of the "SYSTICK" style Interval Timer.
//            It returns a unsigned long of how many millseconds have
//            elapsed since the board started up.
//*****************************************************************************
unsigned long  board_systick_timer_get_value (void)
{
    return (_g_systick_millisecs);
}


//*****************************************************************************
//                               SYSTICK   ISR
//
// Handles all Systick interrupts
//
// Note that the max SYSTICK value = 0xFFFFFFFF = 1193 hours (49 days)
//      so logic to handle that kind of wrap-around is required in code that
//      uses SYSTICK  e.g. in board_delay_ms(), board_vtimer_check(), etc
//*****************************************************************************

void  SysTick_ISR_Handler (void)
{
    _g_systick_millisecs++;    // update # elapsed Systicks (poor man's TOD)
}

//*****************************************************************************
