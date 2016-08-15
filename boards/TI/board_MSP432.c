
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                                  board_MSP432.c
//
// Common logic for all MSP432 MCUs, mainly GPIOs and common System/Systick calls.
//
//  Uses a combination of MSP432 DriverLib (mostly for configuration) and
//  direct register calls (main execution).
//
//  Specific chip dependences are mainly constrained to tables (GPIOs used
//  specific ADC modules supported, etc). They are pulled in as #includes
//  based MCU type.
//
//  In general, the use of #ifdefs within the code has been minimized, in
//  order to keep things read-able and maintainable.
//
// History:
//   03/29/15 - Board arrived - Created initial version.
//   04/04/15 - Got working with TCP Client/Server test apps using CC3100.
//   04/24/15 - Upgraded to add ADC "Multiple Sequenced Conversions" support. Duq
//   04/26/15 - Fixed config issue that caused timing problems with "Sequenced
//              Conversions". D Dob
//   05/16/15 - Removed dependency on DriverLib for ADC channel config. WORKS. Duqu
//   06/29/15 - Merged latest PWM/Timer changes in for consistency across
//              platforms, and to provide added features (OC_Mode). Duqu
//   06/30/15 - Rolled in DAC support, using external 8-channel SPI-based
//              DAC128S085 EVM/Boosterpack. Duqu
//
// -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
//
// The MIT License (MIT)
//
// Copyright (c) 2014-2015,2016 Wayne Duquaine / Grandview Systems
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

#ifndef __BOARD_COMMON_C__
#define __BOARD_COMMON_C__

#include "Board_Support_Api"  // pull in high level User API defs
#include "boarddef.h"         // pull in MCU platform defs and board_xx() protos

#include <math.h>

//#include <gpio.h>              // ensure have DriverLib GPIO defs


/*******************************************************************************
*     Pull in the appropriate board_xxx.c file, based on MCU device type
*******************************************************************************/

//******************************************************************************
//
//                      COMMON  SYSTEM  and  GPIO  Code
//
//******************************************************************************


             //********************************************************
             //                 Common System Variables
             //********************************************************
    uint32_t  _g_SysClk_Ticks;          // Global to hold clock frequency (ticks/sec)

    uint32_t  _g_systick_millisecs = 0; // current SYSTICK time in ticks that have
                                        // accumulated since startup (poor mans TOD)

    char      _g_pwm_mclock_configured = 0; // PWM master clock was configured
    uint32_t  _g_pwm_mdivider = 1;          // PWM Master clock : CPU divider ratio

    uint32_t  _g_MCLK     = 0;          // CPU MCLK setting
    uint32_t  _g_SMCLK    = 0;          // peripheral SMCLK setting
    uint32_t  _g_ACLK     = 0;          // low speed  ACLK setting


//**************************************************************************
//**************************************************************************
//
//                        COMMON   SYSTEM   Routines
//
//**************************************************************************
//**************************************************************************

//******************************************************************************
//  board_init
//
//           Initialize system clocks, and basic GPIOs
//******************************************************************************

void  board_init (long mcu_clock_rate)
{
    board_stop_WDT();

    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate); // user specified clock speed
       else board_system_clock_config (MCU_DEFAULT_SPEED); // use default

       //----------------------------------------------------------
       //   Configure basic peripherals needed for W5200 / CC3000
       //----------------------------------------------------------
    board_gpio_init();              // turn on key GPIO clocks, ...

    board_systick_timer_config();   // ensure "Systick" timer is turned on
}


//*****************************************************************************
// board_delay_ms
//
//            Produce a delay in milli-seconds (ms)
//*****************************************************************************
void  board_delay_ms (long  ms_delay)
{
    uint32_t  tick_begin,   tick_endtime,    i;

        // Systick not used - do delay by spinning cycles
   for (i = 0; i < ms_delay; i++)
      __delay_cycles (8000);     // requires a constant !

}


//*****************************************************************************
//  board_disable_global_interrupts
//
//         Turn off Global interrupts  (for Interval Timer, ...)
//*****************************************************************************
void  board_disable_global_interrupts (void)
{
    __disable_interrupt();     // Globally disable interrupts
}


//*****************************************************************************
//  board_enable_global_interrupts
//
//         Turn on Global interrupts  (for Interval Timer,
//                                     PWM Timer, ADC, ...)
//*****************************************************************************
void  board_enable_global_interrupts (void)
{
    __enable_interrupt();      // Globally enable interrupts
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

        // take the current I/O clock frequency, and compute the associated
        // ticks needed to generate that frequency.(e.g. for Timers, PWMs, ...)
     pticks = _g_SysClk_Ticks / frequency;

     return (pticks);
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
//       Return the board's SMCLK Peripheral clock frequency in ticks.
//
//       If MSP432 MCU is running at 48 MHz, then SMCLK is running at only
//       one-half that, because max recommended limit for SMCLK is 24 MHz.
//*****************************************************************************

long  board_sys_IO_clock_get_frequency (void)
{
    return (_g_SMCLK);     // return the I/O clock frequency in ticks
}


//*****************************************************************************
//  board_stop_WDT
//
//           Shutoff WDT
//*****************************************************************************

void  board_stop_WDT (void)
{
    WDTCTL = WDTPW | WDTHOLD;
}


//*****************************************************************************
// board_systick_timer_config
//
//          Provide a "SYSTICK" style Interval Timer.
//
//*****************************************************************************
void  board_systick_timer_config (void)
{
    uint32_t  systick_config;

        // Configure and enable SysTick for use as interval timer
    systick_config = (_g_SysClk_Ticks / 1000);   // set for 1 ms period/pop
    SysTick_setPeriod (systick_config);
    SysTick_enableModule();
    SysTick_enableInterrupt();
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


/*******************************************************************
 *                             SYSTICK    ISR
 *
 * SysTick interrupt handler.
 *
 *                   Increment Systick 1 ms count, on every 1ms pop
 *******************************************************************/
void  SysTick_ISR_Handler (void)
{
    _g_systick_millisecs++;  // inc Systick counter (add 1 for each 1 ms rupt)
}


//*****************************************************************************
//  board_system_clock_config
//
//          Setup CPU clocks and turn off Watch_Dog_Timer
//*****************************************************************************

void  board_system_clock_config (long  mcu_clock_hz)
{

    _g_MCLK  = CS_getMCLK();           // The default startup MCLK is 3000000 (3 MHz)

// the following code was pulled from Richard S's FreeRTOS demo for MSP432.
	    //-----------------------------------------------------------------
            // Configure the clocks for maximum frequency.
	    // From the datasheet:  For AM_LDO_VCORE1 and AM_DCDC_VCORE1 modes,
	    // the maximum CPU operating frequency is 48 MHz
	    // and maximum input clock frequency for peripherals is 24 MHz.
	    //-----------------------------------------------------------------
    PCM_setCoreVoltageLevel (PCM_VCORE1);

// Add support for 3 MHz (default startup), 12 MHz (0 wait states), 24 MHz (sweetspot 1 wait state), 
//             and 48 MHz (2 wait states)
//   3 MHz  DCO = 0 ?,   12 MHz  DCO = 3,   24 MHz  DCO = 4,   48 MHz  DCO = 5


    CS_setDCOCenteredFrequency (CS_DCO_FREQUENCY_48);

    CS_initClockSignal (CS_HSMCLK, CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1);
    CS_initClockSignal (CS_MCLK,   CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1);
//  if (cpu_clock > 24000000)
       CS_initClockSignal (CS_SMCLK,  CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_2);  // SMCLK cannot exceed 24 MHz
//     else CS_initClockSignal (CS_SMCLK,  CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1);
    CS_initClockSignal (CS_ACLK,   CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);


        // save the configured clock ticks (MHz) settings
    _g_SysClk_Ticks = CS_getMCLK();       // save the MCU clock ticks setting
    _g_MCLK  = CS_getMCLK();             // save main CPU clock ticks
    _g_SMCLK = CS_getSMCLK();            // save the SMCLK peripheral clock ticks
    _g_ACLK  = CS_getACLK();             // save the ACLK  slow clock ticks

//      // compute Timer divisor factor for 1 ms timer. Yields needed CCR0 value
//  g_TA1_1ms_ticks = _g_SMCLK / 1000;   // save # of SMCLK ticks in 1 milli-sec
}



//******************************************************************************
//  board_gpio_init
//
//******************************************************************************

void  board_gpio_init (void)
{
}
#endif                       //  __BOARD_COMMON_C__

/******************************************************************************/
