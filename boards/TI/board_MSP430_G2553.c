
// 05/03/15 - PWM works, but is STILL "jittery" and runs at 19.6 Khz _even when_
//            16 MHz clock yields medium CCR values (800/200) leading to
//            loss of accuracy.  Suggest using a /2 or /4 pre-scalar. Ditto F5529

//******************************************************************************
//                           board_MSP430_G2553.c
//
// Board dependent code, moved to a single module, in prep for Tiva, C2000
//
// MSP430 Notes:
//   MCLK_MHz  (used to drive main CPU)  is set to 16 MHz
//   SMCLK_MHz (used to drive Timer PWM) is set to  2 MHz
//   TA0       (used as interval timer)  is set to 1 ms pops
//   Single Step pulse width             is delay_cycles(1000) = 0.0000625 sec
//                                                             = 62.5  usec
//                                         (delay_cycles = value * MCLK cycles)
//
//   PWM frequency range:                SMCLK / TARGET_SPEED_in_PULSES_per_SEC
//       Target PPS = 512-1024           Period = 3906 - 1953
//
// History:
//   11/29/14 - Created
//   05/02/15 - Changed "Systick timer" to use TA0, rather than TA1, so that
//              PWM support could use the two channels of TA1. Duqu
//   05/02/15 - Added multi-channel PWM support. Duqu
//******************************************************************************

#include "MotorLib_Api.h"                         // pull in common definitions

#include <msp430.h>

int   board_clock_init (long mcu_clock_rate, int flags);
void  board_stop_WDT (void);
void  board_system_clock_config (long  cpu_freq);

//---------------------------------------------------------
//                    Global Variables
//---------------------------------------------------------

    uint32_t  _g_SysClk_Ticks;     // Global to hold clock frequency (ticks/sec)
    uint32_t  _g_MCLK          = 0; // CPU MCLK setting
    uint32_t  _g_SMCLK         = 0; // peripheral SMCLK setting
    uint32_t  _g_TA1_1ms_ticks = 0; // number of SMCLK ticks in 1 millisecond

    uint32_t  _g_systick_millisecs = 0;   // used by "Systick" emulator logic.


//*****************************************************************************
//  board_init
//
//           Initialize system clocks, and basic GPIOs
//           A non-zero flags value = NO_SYSTICK_ENABLE
//*****************************************************************************

int  board_clock_init (long mcu_clock_rate, int flags)
{
    board_stop_WDT();

    if (mcu_clock_rate != 0)
       board_system_clock_config (mcu_clock_rate); // user specified clock speed
       else board_system_clock_config (16000000);  // use default: run at 16 MHz

    board_gpio_init();              // turn on any key GPIO clocks, ...

    if (flags == 0)
       board_systick_init();        // set up 1 ms "Systick" Timer period

    return (0);                     // denote completed OK
}


//*****************************************************************************
// board_busy_wait_usec
//
//           Continuously loop until a fixed time delay has completed.
//           This function provides a means of generating a constant
//           length. The function delay (in cycles) = 3 * parameter.
//*****************************************************************************

void  board_busy_wait_usec (long usec_delay)
{
   int   i;

   for (i = 0; i < 1000; i++)
      __delay_cycles (8000);                     // requires a constant !
//    __delay_cycles (SINGLE_STEP_PULSE_WIDTH);  // DRV8711 support
}


//*****************************************************************************
// board_delay_ms
//
//            Produce a delay in milli-seconds (ms)
//*****************************************************************************
void  board_delay_ms (long  ms_delay)
{
   int   i;

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
//        CAUTION: at chip startup, FR4133 drives all GPIOs
//                 (and clocks) to High Impendance. Must hit
//                 it with a PM5CTL0 &= ~LOCKLPM5 to kick it
//                 out of high-impandance mode, and Enable the
//                 GPIOs. Otherwise all the pins will be
//                 floating, even though they have been configured.
//*****************************************************************************
void  board_gpio_init (void)
{
}

//*****************************************************************************
//*****************************************************************************
//                       System CPU Clocks  /  Systick   Routines
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//  board_stop_WDT
//
//          Turn off Watch_Dog_Timer
//*****************************************************************************

void  board_stop_WDT (void)
{
    WDTCTL   = WDTPW | WDTHOLD;  // turn off watchdog timer
}


//*****************************************************************************
//  board_system_clock_config
//
//          Setup CPU clocks:  DCO, MCLK, SMCLK
//*****************************************************************************

void  board_system_clock_config (long  cpu_freq)
{
       // Set DCO to 16 MHz   CPU clock
    DCOCTL  = CALDCO_16MHZ;
    BCSCTL1 = CALBC1_16MHZ;

       // Set SMCLK to  16 MHz  I/O clock
    BCSCTL2 = DIVS_0;       // 16 MHz SMCLK required for PWM resolution at 20kHz yields 20 KHz
//  BCSCTL2 = DIVS_1;       //  8 MHz SMCLK required for PWM resolution at 20kHz yields 10 KHz
//  BCSCTL2 = DIVS_2;       //  4 MHz SMCLK required for PWM resolution at 20kHz yields  5 kHz
//  BCSCTL2 = DIVS_4;       //  2 MHz SMCLK required for PWM resolution at 20kHz

    BCSCTL3 = LFXT1S_2;     // ACLK = VLO

#if (LATER)
        // save the configured clock ticks (MHz) settings
    _g_SysClk_Ticks = CS_getMCLK();       // save the MCU clock ticks setting
    _g_MCLK  = CS_getMCLK();              // save main CPU clock ticks
    _g_SMCLK = CS_getSMCLK();             // save the SMCLK peripheral clock ticks
    _g_TA1_1ms_ticks = _g_SMCLK / 1000;   // save # SMCLK ticks in 1 milli-sec
#endif

    _g_SysClk_Ticks = cpu_freq;
    _g_SMCLK        = cpu_freq;
        // compute Timer divisor factor for 1 ms timer. This yields needed CCR0 value
    _g_TA1_1ms_ticks = _g_SMCLK / 1000;   // save # SMCLK ticks in 1 milli-sec
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
//          The F5529  has one 16-bit Timer_A5 style timer  (TA0)
//                         two 16-bit Timer_A3 style timers (TA1/TA2),
//                         one 16-bit Timer_B7 style timers (TB0).
//          The FR5969 has two 16-bit Timer_A3 style timers (TA0/TA1),
//                         two 16-bit Timer_A2 style timers (TA2/TA3),
//                         one 16-bit Timer_B7 stype timer  (TB0).
//          The FR4133 haas two 16-bit Timer_A3 style timers (TA0/TA1). CCR0/CCR1/CCR2 only
//                                                           TA0CCR0 CCIFG0 + TA0CCR1 CCIFG1, TA0CCR2 CCIFG2, TA0IFG (TA0IV)
//                                                           TA1CCR0 CCIFG0 + TA1CCR1 CCIFG1, TA1CCR2 CCIFG2, TA1IFG (TA1IV)
//
               // TI is mandating TA1.x for standard Launchpad
               // "PWM Out" on connector J2-2. 
               // So for MSP430-F5529 support, we have to use TB0 for "SysTick" 

//          We will use Timer TA1 since it is common across all the  -- OUCH -- THIS CONFLICTS WITH PWMOUT
//                                                                            CHOICES ARE USE TB0 or screw PWMOUT
//          target MCUs, and leaves TA0 available for use by an RTOS.
//*****************************************************************************
void  board_systick_init (void)
{
    TA0CCTL0 = CCIE;                     // enable CCR0 interrupt
    TA0CCR0  = _g_TA1_1ms_ticks;         // # SMCLKS that = 1 milli-sec
    TA0CTL   = TASSEL_2 + MC_1 + TACLR;  // use SMCLK, up mode, clear TAR
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
//                            TA0   CCR0   ISR
//
// SysTick interrupt handler.
//
//                               Increment Systick 1 ms count, on every 1ms pop
//*****************************************************************************
#pragma vector=TIMER0_A0_VECTOR
__interrupt void  TIMER_TA0_CCR0_ISR (void)
{
    _g_systick_millisecs++;  // inc Systick counter (add 1 for each 1 ms rupt)
}


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
//             USCIAB0TX_VECTOR, \
//
#pragma vector=PORT1_VECTOR, PORT2_VECTOR, \
               TIMER0_A1_VECTOR, \
               COMPARATORA_VECTOR, NMI_VECTOR
__interrupt void  Trap_ISR (void)
{
      while (1 == 1)
        ;               // Hang if we get here - should not occur
}
