
//*******1*********2*********3*********4*********5*********6*********7**********
//                           board_MSP430_FR5969.c
//
// Board dependent code, moved to a single module for MSP430 FR5969 Launchpad
//
// MSP430 Notes:
//   SMCLK_MHz (used to drive Timer PWM) is set to  2 MHz
//   MCLK_MHz  (used to drive main CPU)  is set to 16 MHz
//   WDT       (used as interval timer)  is WDT_MDLY_32        = 32 ms delay/pop
//   Single Step pulse width             is delay_cycles(1000) = 0.0000625 sec
//                                                             = 62.5  usec
//                                         (delay_cycles = value * MCLK cycles)
//
//   PWM frequency range:                SMCLK / TARGET_SPEED_in_PULSES_per_SEC
//       Target PPS = 512-1024           Period = 3906 - 1953
//
// History:
//   11/29/14 - Created.
//   12/11/14 - Works properly after fixing CS issues and Pull-Ups issues.
//   04/27/15 - Added ADC12 sequence of channels support to optimize ADC. Duqu
// 
// -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
//
// The MIT License (MIT)
//
// Copyright (c) 2014-2015, 2016 Wayne Duquaine / Grandview Systems
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
#include "boarddef.h"

//---------------------------------------------------------
//                    Global Variables
//---------------------------------------------------------

    uint32_t  _g_SysClk_Ticks;      // Global to hold clock frequency (ticks/sec)
    uint32_t  _g_MCLK          = 0; // CPU MCLK setting
    uint32_t  _g_SMCLK         = 0; // peripheral SMCLK setting
    uint32_t  _g_TA1_1ms_ticks = 0; // number of SMCLK ticks in 1 millisecond

    uint32_t  _g_systick_millisecs = 0;   // used by "Systick" emulator logic.
                                          // provides a 1 ms periodic timer pop

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
//           A non-zero flags value = NO_SYSTICK_ENABLE
//*****************************************************************************

void  board_init (long mcu_clock_rate)
{
    board_stop_WDT();

    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate); // user specified clock speed
       else board_system_clock_config (8000000);   // use default: run at 8 MHz

    board_gpio_init();                 // turn on key GPIO clocks, ...

    if (flags == 0)
       board_systick_timer_config();   // turn on "Systick" timer
}


//*****************************************************************************
// board_delay_ms
//
//            Produce a delay in milli-seconds (ms)
//*****************************************************************************
void  board_delay_ms (long  ms_delay)
{
    uint32_t  tick_begin,   tick_endtime,   i;

        // Systick not usde - do delay by spinning cycles
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
//      If the result value would be too large, PWM/Timer code has
//      to further sub-divide it using pre-scalars
//
// Note we use SMCLK as base for determining Ticks, because that is used to 
//      drive all our MSP430 peripherals (PWM, Timer, ADC, ...)
//*****************************************************************************

long  board_frequency_to_period_ticks (long frequency)
{
    long   pticks;

        // take the current clock frequency, and compute the associated ticks
        // needed to generate that frequency.
     pticks = _g_SMCLK / frequency;

     return (pticks);
}





//*****************************************************************************
//  board_gpio_init
//
//        Configure Port Directions and Peripherals as needed.
//
//        CAUTION: at chip startup, FR5969 drives all GPIOs
//                 (and clocks) to High Impendance. Must hit
//                 it with a PM5CTL0 &= ~LOCKLPM5 to kick it
//                 out of high-impandance mode, and Enable the
//                 GPIOs. Otherwise all the pins will be
//                 floating, even though they have been configured.
//*****************************************************************************
void  board_gpio_init (void)
{
    PM5CTL0 &= ~LOCKLPM5;  // Disable GPIO power-on default High-Impedance mode
                           // to activate configured port settings.
                           // This is ONLY for FRxxx FRAM devices !
                           // Normal Fxxx (F5529, ...) MCUs will crash on this instr.
}

//*****************************************************************************
//*****************************************************************************
//                      System CPU Clocks  /  Systick   Routines
//*****************************************************************************
//*****************************************************************************

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
//  board_system_clock_config
//
//          Setup CPU clocks.                          Max is 16 MHz on FR5969
//*****************************************************************************

void  board_system_clock_config (long  mcu_clock_hz)
{
        // this is the original clock logic from WORKING version, that was running at 8 MHz
    WDTCTL   = WDTPW | WDTHOLD;  // turn off watchdog timer

    if (mcu_clock_hz >= 16000000)
       {            // Setup for 16 MHz DCO and MCLK
                    //---------------------------------
                    //   Set DCO to 16 MHz   CPU clock
                    //---------------------------------
         FRCTL0 = FRCTLPW | NWAITS_1;   // > 8 MHz requires adding a wait state
                                        // to FRAM access

         CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
         CSCTL1   = DCORSEL | DCOFSEL_4;           // Set DCO to 16 MHz
                    // Set ACLK = VLOCLK, Set SMCLK and MCLK = DCO,
         CSCTL2   = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
         CSCTL3   = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers to 1
         CSCTL0_H = 0;                             // Lock CS registers

#if (HAL_API)
          CS_setDCOFreq (CS_DCORSEL_0, CS_DCOFSEL_4);  // Set DCO freq to 16 MHz
             // Set MCLK = DCO with frequency divider of 1
          CS_initClockSignal (CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
             // Set SMCLK = DCO with frequency divider of 1
          CS_initClockSignal (CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
#endif
       }
      else if (mcu_clock_hz >= 8000000)
              {     // Setup for 8 MHz DCO and MCLK
                    //---------------------------------
                    //   Set DCO to 8MHz   CPU clock  =  ORIGINAL WORKING LOGIC
                    //---------------------------------
                CSCTL0_H = 0xA5;                          // Unlock CS registers
                CSCTL0_L = 0;
                CSCTL1   = DCORSEL + DCOFSEL0 + DCOFSEL1; // Set DCO to 8 MHz
                CSCTL2   = SELA_1  + SELS_3 + SELM_3;     // set ACLK - VLO, the 
                                                          // rest: MCLK/SMCLK = DCO
                CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;        // set all dividers to 0

// And see msp430fr59xx_cs_04.c as to how to set this up using XT1 clock as ACLK ref
// ALT          CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
// ALT          CSCTL1 = DCOFSEL_6;                       // Set DCO to 8MHz
// ALT          CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;  // Set SMCLK and MCLK = DCO
                                                                      // set ACLK = VLOCLK
// ALT          CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers to 1
// ALT          CSCTL0_H = 0;                             // Lock CS registers
              }
      else    {     // Setup for 1 MHz (startup default) DCO and MCLK
                    //---------------------------------
                    //   Set DCO to 1 MHz   CPU clock
                    //---------------------------------
                CS_setDCOFreq (CS_DCORSEL_0, CS_DCOFSEL_1);  // Set DCO freq to 1 MHz  <== IS GUESS _ TWEAK THIS !!!
                    // Set MCLK = DCO with frequency divider of 1
                CS_initClockSignal (CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
                    // Set SMCLK = DCO with frequency divider of 1
                CS_initClockSignal (CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
              }


#if NEW_API_CLOCK_LOGIC
       // Configure one FRAM wait state as required by the device datasheet for
       // MCLK operation beyond 8 MHz _before_ configuring the clock system.
/// FRCTL0 = FRCTLPW | NWAITS_1;

       // Set DCO frequency to 16 MHz
/// CS_setDCOFreq (CS_DCORSEL_0, CS_DCOFSEL_4);  // Set DCO frequency to 16 MHz
    CS_setDCOFreq (CS_DCORSEL_0, CS_DCOFSEL_6);  // Set DCO frequency to 8 MHz

       // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal (CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

       // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal (CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

       // Set ACLK=LFXT
    CS_initClockSignal (CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);

       // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource (32768, 0);

        // Start XT1 with no time out
//  CS_turnOnLFXT (CS_LFXT_DRIVE_0);    HANGING ==> Needs Crystal Mounted - WVD
#endif                    // NEW_CLOCK_LOGIC


        // save the configured clock ticks (MHz) settings
    _g_SysClk_Ticks = CS_getMCLK();       // save the MCU clock ticks setting
    _g_MCLK  = CS_getMCLK();             // save main CPU clock ticks
    _g_SMCLK = CS_getSMCLK();            // save the SMCLK peripheral clock ticks

        // compute Timer divisor factor for 1 ms timer. Yields needed CCR0 value
    _g_TA1_1ms_ticks = _g_SMCLK / 1000;   // save # of SMCLK ticks in 1 milli-sec
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
//*****************************************************************************

long  board_sys_IO_clock_get_frequency (void)
{
    return (_g_SMCLK);     // return the I/O clock frequency in ticks
}


//*****************************************************************************
// board_systick_timer_config
//
//          Emulate a "SYSTICK" style Interval Timer.
//
//          This is required by MQTTCC3100.c support, as well as other
//          packages that need a milli-second iterval timer.
//
//          The FR5969 has two 16-bit Timer_A3 style timers (TA0/TA1),
//                         two 16-bit Timer_A2 style timers (TA2/TA3),
//                         one 16-bit Timer_B7 stype timer  (TB0).
//
//          We use Timer TA3 since it is free and not pinned out to any
//           external pins, and leaves TA0 available for use by an RTOS.
//*****************************************************************************
void  board_systick_timer_config (void)
{
    TA3CCTL0 = CCIE;                     // enable CCR0 interrupt
    TA3CCR0  = _g_TA1_1ms_ticks;         // number of SMCLKS that = 1 milli-sec
    TA3CTL   = TASSEL_2 + MC_1 + TACLR;  // use SMCLK, up mode, clear TAR
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
//                            TA3   CCR0   ISR
//
// SysTick interrupt handler.
//
//                            Increment Systick 1 ms count, on every 1ms pop
//*****************************************************************************
#pragma vector=TIMER3_A0_VECTOR
__interrupt void  TIMER_A3_A0CCR0_ISR (void)
{
    _g_systick_millisecs++;  // inc Systick counter (add 1 for each 1 ms rupt)

#if defined(USES_MQTT)
    SysTickIntHandler();     // invoke MQTTCC3100.c to update its Timer
#endif

#if defined(USES_VTIMER)
    if (_g_vtimers_active > 0)
       board_vtimer_check_expiration();
#endif
}



//---------------------------------------------------------------
//                 ISR HANDLER  -  Interval Timer mode WDT
//
// This pops every 32 milli-seconds, and is used to process
// the acceleration/deceleration ramp.
// It sets the flag for the mainline to compute the next speed
// step, ir a ramp up or ramp down is occurring. If just
// a constant speed is being performed, the speed stays as is.
//---------------------------------------------------------------
#pragma vector=WDT_VECTOR
__interrupt void  WatchDog_Timer (void)
{
       // Signal Main Thread to Calculate Next Speed Value
    G_ACCEL_FLAG = true;

       // Wake Up the Main Thread
//  __bic_SR_register_on_exit(LPM0_bits);
}

#pragma vector=PORT1_VECTOR
__interrupt void  Trap_ISR2 (void)
{
      while (1 == 1)
        ;               // Hang if we get here - should not occur
}

#endif                            // #if defined(USES_DRV8711)


//**************************************************************************
//**************************************************************************
//
//                     BOARD   -   COMMON   ISRs
//
//**************************************************************************
//**************************************************************************

//---------------------------------------------------------------
//                     Un-assigned ISRs
//---------------------------------------------------------------
//      TIMER0_A0_VECTOR, TIMER0_A1_VECTOR, \
//
#pragma vector=PORT2_VECTOR, ADC10_VECTOR, USCIAB0TX_VECTOR, \
        COMPARATORA_VECTOR, NMI_VECTOR
__interrupt void  Trap_ISR (void)
{
      while (1 == 1)
        ;               // Hang if we get here - should not occur
}

