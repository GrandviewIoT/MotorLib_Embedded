
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                       MotorLib_LL_DRV830x_MSP430_FR5969.c
//
//
// Support for DRV8301 and DRV8305 BLDC controllers. 
//
// The command sets are similar, so we use #ifdefs to handle the variances.
//
//
// History:
//   04/15/15 - Created to provide clocks and timer checkout. WORKS.  Duquaine
// -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
//
// The MIT License (MIT)
//
// Copyright (c) 2015-2016 Wayne Duquaine / Grandview Systems
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

#include "user_api.h"            // pull in high level User API defs
#include "boarddef.h"            // pull in standard MCU defs
                                 //         and any key DriverLib/HAL_Lib defs

#include <string.h>              // Include defs for any strxxx() / memxxx()

    int        ret_code    = 0;
    char       uart_cmd_buf [40];    // holds any user input via UART
    char       uart_test1  = 1;
    long       pwm_period  = 0;
    long       mg_PWMclock = 0;
    long       mg_PWMperiod = 0;
    long       mg_Sysclock = 0;
    long       nmap_SysClkTicks = 0;

    uint32_t   pwm_duty_value = 0;
    uint16_t   adc_cur_value  = 0;
    uint16_t   adc_value      = 0;

#define TARGET_STEPPER_PPS         512     // typically 512-1024 pulse steps/sec
#define BASE_PWM_FREQUENCY   TARGET_STEPPER_PPS

#define PWM_20K_FREQUENCY   20000

/*******************************************************************************
*                              MAIN               Application's entry point
*******************************************************************************/

int  main (int argc, char **argv)
{
    board_init();                      // Turn off WDT, init MCU clocks, ...

    board_systick_timer_config();      // configure 1ms timer

    LED1_INIT();                       // Setup LED1 for output
    LED2_INIT();                       // Setup LED2 for output

#if (UART_CHECKOUT)
while (uart_test1)  // TEMP DEBUG of Baud Rate
   CONSOLE_WRITE ("\n\rInitialization complete. Type in a config parm:\n\r");
while ( ! CONSOLE_CHECK_FOR_INPUT() )
      ;        // loop till get some data
CONSOLE_READ_STRING ((char*) uart_cmd_buf, 12);

CONSOLE_WRITE ("Type in a single char: ");
ret_code = 0;
while (ret_code == 0)              // wait till we get a char in
   ret_code = CONSOLE_GET_CHAR();
CONSOLE_WRITE_CHAR (ret_code);     // echo it
#endif

#if defined(__MSP432P401R__)
       //-------------------------------------------------------------
       // Ensure DRV8848 is OFF during config.
       // setup nSLEEP PB_2 pin (J2-2) to LOW to turn off the DRV8848
       //-------------------------------------------------------------
    pin_Config (Pin19, GPIO_OUTPUT);
    pin_Low (Pin19);
#endif

#if defined(TIVA_MCU)
       //-------------------------------------------------------------
       // Configure the ADC potentiometer on DRV8848 board.
       //
       // setup setu VREF pin PE_4  aka  Adc9 on J1-5
       //-------------------------------------------------------------
MAP_GPIOPinTypeADC (GPIO_PORTE_BASE, GPIO_PIN_4);      // TEMP HARD_CODED HACK for channel 9
    adc_Config (Adc9, ADC_TRIG_BY_USER_APP, ADC_LAST);
    adc_Enable();
adc_Trigger_Start();   // Tiva does not yet match MSP430/432 that auto-triggers first go

       //-------------------------------------------------------------
       // Ensure DRV8848 is OFF during config.
       // setup nSLEEP PB_2 pin (J2-2) to LOW to turn off the DRV8848
       //-------------------------------------------------------------
    pin_Config (Pin19, GPIO_OUTPUT);
    pin_Low (Pin19);
#endif

      //-----------------------------------------------------------------
      // Setup PWMs to drive the DRV8711 Stepper
      //-----------------------------------------------------------------
      // first, get the period for a 20 K Hz PWM frequency
    pwm_period = board_frequency_to_period_ticks (PWM_20K_FREQUENCY);

      // then initialize the needed PWMs, passing in startup period
        //-------------------------------------------------------------
        // do basic setup for PWM and Interval Timer  (turn on clocks)
        // Skipping the /32 gives better accuracy (20.004 KHz vs 20.81 K Hz using /32)
        //-------------------------------------------------------------
//  MAP_SysCtlPWMClockSet (SYSCTL_PWMDIV_32);         // do not do / 32 of Master clock and see results
    MAP_SysCtlPWMClockSet (SYSCTL_PWMDIV_1);          // Setup Master PWM I/O clock
    MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_PWM1);  // turn on PWM module 1 clock
//  mg_PWMclock  = MAP_SysCtlClockGet() / 32;   // save Max PWM Clock value
    mg_PWMclock  = MAP_SysCtlClockGet();        // save Max PWM Clock value
    mg_Sysclock  = MAP_SysCtlClockGet();        // This says 80 MHz
    nmap_SysClkTicks = MAP_SysCtlClockGet();    // This says 80 MHz after switching over to MAP_ version.
                                                // Before that, non MAP_ version was saying 66 MHz, which is whacked out

       //------------------------------------------------------------------------------
       // Setup PWM pins that are used by DRV8848
       //
       //    PWM A / AIN1 = PA_4 TA1.2   J2-8        PWM B / BIN1 = PA_6 TA1.1   J1-9
       //    DIR A / AIN2 = PA_3 TA1CCR2 J2-9        DIR B / BIN2 = PA_7 TA1CCR1 J1-10
       //    nSLEEP = PB_2               J2-2        nFAULT = PE_0               J2-3
       //    VREF   = PE_4  Adc9         J1-5
       //------------------------------------------------------------------------------

       // setup DRV8701 STEP_AIN1 PA_6 pin (J1-9) for PWM mode
       // setup DRV8848 BIN1      PA_6 pin (J1-9) for PWM mode
    MAP_GPIOPinTypePWM (GPIO_PORTA_BASE, GPIO_PIN6);   // setup BIN1 PWM (J1-9)
    MAP_GPIOPinConfigure (GPIO_PA6_M1PWM2);

       //------------------------------------------------
       // Setup PWM and associated Count mode.
       // Tiva only supports DOWN counts, not UP counts
       //------------------------------------------------
    MAP_PWMGenConfigure (PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);

       // setup default PWM Period and duty cycle
    mg_PWMperiod = (mg_PWMclock / PWM_20K_FREQUENCY) - 1;   // default period /32  = 124   (125 - 1)
                                                            // default period /1   = 3999  (4000 - 1)
    MAP_PWMGenPeriodSet (PWM1_BASE, PWM_GEN_1, mg_PWMperiod);

    MAP_PWMPulseWidthSet (PWM1_BASE, PWM_OUT_2, mg_PWMperiod / 4);  // duty of 25 %
    pwm_duty_value = mg_PWMperiod / 4;                   // save it for ADC checks

      //-----------------------
      //   Start up the PWM
      //-----------------------
    MAP_PWMGenEnable (PWM1_BASE, PWM_GEN_1);             // start up the PWM  ??? need ???

    MAP_PWMOutputState (PWM1_BASE, PWM_OUT_2_BIT, true); // Enable PWM pin output

       //-----------------------------------------
       // setup DIR BIN2 PA_7 pin (J1-10) to LOW
       //-----------------------------------------
    pin_Config (Pin10, GPIO_OUTPUT);   // setup BIN2 DIR (J1-10)
    pin_Low (Pin10);

       //-----------------------------------------
       // Turn off AIN1 PWM and AIN2 DIR pins PA_4 / PA_3 pins (J2-8/J2-9) to LOW
       //-----------------------------------------
    pin_Config (Pin13, GPIO_OUTPUT);   // setup AIN1 PWM (J2-8)
    pin_Low (Pin13);
    pin_Config (Pin12, GPIO_OUTPUT);   // setup AIN2 DIR (J2-9)
    pin_Low (Pin12);

       //-------------------------------------------------------------
       // set nSLEEP PB_2 pin (J2-2) to HIGH to turn on the DRV8848
       //-------------------------------------------------------------
    pin_High (Pin19);

    while (1)
      {
 //      LED1_TOGGLE();            // Setup LED1 for output
         if  (adc_Check_All_Complete())
             {     // we have a completed ADC reading. Pull in the latest reading
               adc_Read (&adc_value);
               if ( (adc_value > (adc_cur_value +10)) ||  (adc_value < (adc_cur_value - 1)) )
                  {    // we have a major change in valu. Update the PWM
                    adc_cur_value = adc_value;
                       // scale the PWM Duty_Cycle (o - 4000) value to match the ADC value (0-4096)
                    if (adc_cur_value > 3999)
                       pwm_duty_value = 3999;
                       else pwm_duty_value = adc_value;
                       // set the new PWM duty ccyle
                    MAP_PWMPulseWidthSet (PWM1_BASE, PWM_OUT_2, pwm_duty_value);
                       // start a new ADC sampling cycle
                    adc_Trigger_Start();
                  }
             }
         board_delay_ms (250);     // wait 250 ms (1/4 sec) between samples
      }
}
