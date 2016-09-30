
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

#if defined(USES_DRV8711)
#include "devices/DRV8711_Spin_Routines.h"
#endif

#if defined(__MSP430G2553__)

//---------------------------------------------------------
//                    Global Variables
//---------------------------------------------------------

    uint32_t  _g_SysClk_Ticks;     // Global to hold clock frequency (ticks/sec)
    uint32_t  _g_MCLK          = 0; // CPU MCLK setting
    uint32_t  _g_SMCLK         = 0; // peripheral SMCLK setting
    uint32_t  _g_TA1_1ms_ticks = 0; // number of SMCLK ticks in 1 millisecond

    uint32_t  _g_systick_millisecs = 0;   // used by "Systick" emulator logic.


//******************************************************************************
//  Simulate GPIO calls that DriverLib on MSP430's provides
//******************************************************************************
void  GPIO_setOutputHighOnPin (uint16_t gpio_port_id,  uint16_t pin)
{
    if (gpio_port_id == P1OUT)
       P1OUT |= pin;
       else P2OUT |= pin;
}

void  GPIO_setOutputLowOnPin (uint16_t gpio_port_id,  uint16_t pin)
{
    if (gpio_port_id == P1OUT)
       P1OUT &= ~(pin);
       else P2OUT &= ~(pin);
}

void  GPIO_toggleOutputOnPin (uint16_t gpio_port_id,  uint16_t pin)
{
    if (gpio_port_id == P1OUT)
       P1OUT ^= pin;
       else P2OUT ^= pin;
}

int  GPIO_getInputPinValue (uint16_t gpio_port_id,  uint16_t pin)
{
    if (gpio_port_id == P1OUT)
       return (P1OUT & pin);
       else return (P2OUT & pin);
    return (0);
}



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
       else board_system_clock_config (16000000);  // use default: run at 16 MHz

    board_gpio_init();              // turn on key GPIO clocks, ...

#if defined(USES_SYSTICK) || defined(USES_MQTT) || defined(USES_ADC) || defined(USES_VTIMER)
    board_systick_timer_config();   // turn on "Systick" timer
#endif

        // Enable the UART if user wants to invoke CONSOLE or DEBUG_LOG calls
#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE_READ) || defined(USES_DEBUG_LOG)
    board_uart_init();              // go ahead and enable the default UART
#endif

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
   uint32_t  tick_begin,   tick_endtime;

#if defined(USES_MQTT) || defined(USES_ADC)
        // use Systick timer to do delay timing
    tick_begin   = _g_systick_millisecs;    //  get the current time

    tick_endtime = tick_begin + ms_delay;

// WVD: THIS LOGIC IS BROKEN  04/25/15
 #if (TO_FIX)
       // handle the oddball case of if the end time wraps past max long value
    if (tick_endtime < tick_begin)
       {          // handle the once in a blue moon wrap-around condition
          while (g_systick_millisecs != tick_endtime)
            ;                // loop till we hit the wrap-around value
       }
      else {     // loop until the current time hits the requested delay time 
             while (g_systick_millisecs < tick_endtime)     // hangs in here a LOOOOONG TIME
               ;
           }
 #else
   for (i = 0; i < ms_delay; i++)
      __delay_cycles (8000);     // requires a constant !

 #endif

#else

        // Systick not in use - do delay by spinning cycles
   for (i = 0; i < ms_delay; i++)
      __delay_cycles (8000);     // requires a constant !
#endif

}


//*****************************************************************************
// Delay
//
//          Produce a delay in milli-seconds (ms)      Used by CC3xxx Simplelink
//*****************************************************************************
void  Delay (unsigned long interval)
{
     board_delay_ms ((long) interval);
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



#if defined(USES_ADC)

//*****************************************************************************
//*****************************************************************************
//                               ADC   Routines
//*****************************************************************************
//*****************************************************************************

    int                _g_ADC_complete   = 0;      // 1 = all ADC conversions/DMAs are complete
    char               _g_DMA_overrun    = 0;      // 1 = DMA completed a new set before the previous one was processed
    char               _g_adc_configured = 0;
    char               _g_trigger_source = 0;
    char               _g_num_configured_channels = 0;
    uint16_t           _g_highest_chan_num = 0;

    int                _g_result_idx   = 0;        // ISR result retrieve index

    unsigned char      _g_adc_step_map [32];       // indexed by channel number
    unsigned short     _g_adc_conv_results [32];   // Internal buffer to hold DMA results

    unsigned long      dma_callback_seen = 0;      // DEBUG COUNTERs
    unsigned long      dma_rupt_seen     = 0;


typedef struct adc_channel_def        /* ADC Channel definitions */
    {
        uint8_t   chan_gpio_port_id;  /* Associated GPIO port    */
        uint8_t   chan_gpio_pin_bit;  /* Associated GPIO pin     */
        uint16_t  chan_adc_num;       /* Associated ADCINCH_x channel num */
    } ADC_CHANNEL_BLK;


//SYSCFG2 |= ADCPCTL0 | ADCPCTL1 | ADCPCTL2 | ADCPCTL3; // P 1.0, P 1.1, P n.n, P 1.3
//chblk->chan_syscfg2_bit
//chblk->chan_input_num
//ADCMCTL0 |= ADCINCH_2;                 // A0~2(EoS)


const ADC_CHANNEL_BLK  _g_adc_channels [] =  //                    User   Energia     LP   Grove
        {  { GP_PORT_1, BIT0, INCH_0 }, // P 1.0 Adc0   A4     4    J1-4
           { GP_PORT_1, BIT1, INCH_1 }, // P 1.1 Adc1   A3     3    J1-3
           { GP_PORT_1, BIT2, INCH_2 }, // P 1.2  -- not connected on LP --
           { GP_PORT_1, BIT3, INCH_3 }, // P 1.3 Adc3   A13   13    J2-8
           { GP_PORT_1, BIT4, INCH_4 }, // P 1.4 Adc4   A12   12    J2-9
           { GP_PORT_1, BIT5, INCH_5 }, // P 1.5 Adc5   A11   11    J2-10
           { GP_PORT_1, BIT6, INCH_6 }, // P 1.6 Adc6   A18   18    J2-3
           { GP_PORT_1, BIT7, INCH_7 }, // P 1.7 Adc7   A19   19    J2-2
           { GP_PORT_2, BIT0, INCH_8 }, // P 8.0 Adc8   A6     6    J1-6
           { GP_PORT_2, BIT1, INCH_9 }  // P 8.1 Adc9   A2     2    J1-2
// ???     { GPIO_PORTB_BASE, GPIO_PIN_5, INCH_10 },  // internal Temperature Sensor
// ???     { GPIO_PORTB_BASE, GPIO_PIN_5, ADC12_B_INPUT_BATSENSOR },   // internal Battery Monitor Sensor
        };


//*****************************************************************************
//  board_adc_init
//
//         Configure the overall sampling clock used for the ADCs on a module.
//
//         The default is set to XXXX.
//         This call allows an experienced programmer to override that default.
//
//         trigger_type specifies the type of trigger that will be used
//           - User App
//           - PWM Module 1  Channel   1 / 2 / 3 / 4
//           - PWM Module 2  Channel   1 / 2 / 3 / 4
//           - Timer n
//
// FR5969 has 1 ADC moodule, with a single sequencer allowing up to 16 channels.
// The sequencer steps are individually identified:  ADC10MCTL0 -> ADC10MCTL15.
// ADC results are stored in corresponding registers: ADC10MEM0 -> ADC10MEM15.
// Results are stored in the 16-bit ADC10MEMx registers.
//
// The Launchpad MSP430 FR4133 pin device physically supports up to 16 channels,
// of which 9 are wired out to the Launchpad pins, and 2 internal channels
// (Temperature Sensor, Battery Monitor) are available.
// Due to this layout, we support 11 total channels.
//
// The FR4133 supports only a 10-bit ADC.
//*****************************************************************************

int  board_adc_init (int adc_module_id, uint32_t clock_rate, 
                     int trigger_type, int flags)
{

// ??? !!!  WVD  Add support for trigger_type ??? !!!

    _g_trigger_source = (char) trigger_type;

    if (_g_adc_configured == 0)
       {
            //Initialize the ADC10 Module
            /*
            * Use internal ADC10 bit as sample/hold signal to start conversion
            * USE MODOSC 5MHZ Digital Oscillator as clock source
            * Use default clock divider/pre-divider of 1
            * Not use internal channel
            */
             //--------------------------------------
             // do basic configuration of ADC module
             //--------------------------------------
             // CONSEQ_1 = Sequence-of-Channels operation
         ADC10CTL0  = ADC10SHT_2 + MSC + SREF_0;  // Set sample time 16 A/D clks
                                                  // and REF = AVCC + / AVSS -
// ??? !!! need to tweak SHS in future: SHS_1 thru SHS_3 = Timer TA0/TA1/TA2 CCR 0/1/2 output
         ADC10CTL1  = SHS_0 + CONSEQ_1;      // Use sampling timer, Sequence-of-Channels
                                             // SHS_0 = User App Triggered
                                             // default = 10-bit conversions

         _g_adc_configured = 1;              // denote basic configuration was done

#if (DMA_FUTURE)
         case DMA_CH7_ADC12C:
            DMA->rCH7_SRCCFG.r = (mapping >> 24) & 0x1F;

             //-----------------------------------------------------------------
             // Initialize the DMA associated with the ADC module.
             //   ?? Or should we hold off on this until we hit adc_enable() ?
             //-----------------------------------------------------------------
         DmaHandle.Instance         = DMA2_Stream0;
         DmaHandle.Init.Channel     = DMA_CHANNEL_0;
         DmaHandle.Init.Direction   = DMA_PERIPH_TO_MEMORY;    // ADC -> RAM

         DmaHandle.Init.PeriphInc   = DMA_PINC_DISABLE;
         DmaHandle.Init.MemInc      = DMA_MINC_ENABLE;         // step buf ptr
         DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  // for 16 bit results
         DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;  // ditto
//       DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//       DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
         DmaHandle.Init.Mode        = DMA_CIRCULAR;     // Treat as circular buf
         DmaHandle.Init.Priority    = DMA_PRIORITY_HIGH;
         DmaHandle.Init.FIFOMode    = DMA_FIFOMODE_DISABLE;
         DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
         DmaHandle.Init.MemBurst    = DMA_MBURST_SINGLE;
         DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;

            // Associate the initialized DMA handle to the the ADC handle
         __HAL_LINKDMA (&AdcHandle, DMA_Handle, DmaHandle);

            // Configure the NVIC for DMA interrupts.
            // NVIC configuration for DMA transfer complete interrupt.
         HAL_NVIC_SetPriority (DMA2_Stream0_IRQn, 0, 0);
         HAL_NVIC_EnableIRQ (DMA2_Stream0_IRQn);
#endif
       }

//  ADCClockConfigSet (uint32_t ui32Base, uint32_t ui32Config,
//                     uint32_t ui32ClockDiv);

       // Configure the ADC to use PLL at 480 MHz divided by 24 to get an ADC
       // clock of 20 MHz.
       //
//  ADCClockConfigSet (ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 24);

       // Configure the ADC to use PIOSC divided by one (16 MHz) and sample at
       // half the rate.
//  ADCClockConfigSet (ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_HALF, 1);

    return (0);           // denote success
}


//*****************************************************************************
//  board_adc_config_channel
//
//         Configure a single ADC channel.
//
//         Note that the sampling rate is determined by the trigger source.
//
//         adc_module parm is really just an index number (0-6)
//         that denote which ADC module to use.
//         For the FR4133, there is only 1 ADC module: ADC12
//         so we effectively ignore the adc_module_id
//
//         The flags value is passed to denote:
//           - priority 0-3                            (NOW)
//           - if interrupts and/or DMA should be used (FUTURE UPGRADE)
//           - Comparator and/or Reference             (FUTURE_UPGRAGDE)
//
//         ADC12 has just one module.
//         ADC12 has just one sequencer.
//         It suppports up to 16 channels, of which 9 are hooked up on the LP.
//*****************************************************************************

int  board_adc_config_channel (int adc_module_id, int channel_num,
                               int sequencer,     int step_num,
                               int last,          int flags)
{
    ADC_CHANNEL_BLK  *chblk;
    uint16_t         *dir_address;
    uint16_t         *sel_address;
    uint16_t         *sel2_address;
    int              rc;

          //*******************************************************************
          //      Add a new ADC channel to be sampled
          //
          // Setting up the ADC for the FR4133 is much different than other
          // MSP430s, like F5529, FR5969, etc.
          //   - GPIO configuration is not used, instead the SYSCFG2
          //     is used, and it also forms a poor man's sequencer.
          //   - The highest channel to be converted is stored in ADCMEMCTL0.
          //     There is no EOS bit.
          //   - There is only 1 ADCMEM0 register which feeds back each
          //     ADC converted channel one at a time, until the highest
          //     channel is reached.
          //   - An interrupt is raised as each ADC channel is converted.
          //******************************************************************

          //------------------------------------------------------------------
          // set the associated GPIO pin to act as an ADC input.
          //
          // Convert channel number to an index into table of entries that
          // contains SYSCFG2 bit setting, and associated ADCINCH_n number
          //------------------------------------------------------------------
       // Enable A/D channels A1 for GROVE CODE
// ADC10AE0 |= 0x02;                       // setup P1.1 (A1) for ADC operation

     chblk = (ADC_CHANNEL_BLK*) &_g_adc_channels [channel_num];
          // Turn on the ADC10AE0 bit associated with this channel, to enable it
          // enable it for ADC operation
     ADC10AE0 |= chblk->chan_gpio_pin_bit;

        // see if this is the highest ADC channel - if so, save it
     if (chblk->chan_adc_num > _g_highest_chan_num)
        _g_highest_chan_num = chblk->chan_adc_num;

    _g_num_configured_channels++;  // inc # channels we have configured for ADC

    return (0);                    // denote success
}


//*****************************************************************************
//  board_adc_check_conversions_done
//
//          Checks if the ALL the conversions for a sinple ADC are done.
//
//          For groups, the starting channel of the group, as specified as
//          the first channel listed in the board_adc_group_init() channels[]
//          array, is passed.
//*****************************************************************************
int  board_adc_check_conversions_done (int adc_module_id, int sequencer)
{
    int     rc;

// this is always returning busy after first Seqence-of_Conversions complete, even when SW triggerd !
///   rc = ADC12_A_isBusy (ADC12_A_BASE);

    if ( ! _g_ADC_complete)
       return (0);                     // ADC and DMA are still busy

    return (1);                        // ADC/DMA conversion sequenc is complete
}


//*****************************************************************************
//  board_adc_enable
//
//          Turn on one or all the sequencers that have been configured.
//          To ensure cross-platform compatibility, we also kick off
//          an initial ADC conversion, if we are configured for USER SW Trigger
//*****************************************************************************
int  board_adc_enable (int adc_module_id, int sequencer)
{
    _g_ADC_complete = 0;                 // clear I/O flags
    _g_DMA_overrun  = 0;

         // Set highest channel number to convert, and associated Clock Divders
//  ADC10CTL1 = INCH_2 + ADC10DIV_3;              // A2 + ADC10CLK/4
    ADC10CTL1 |= _g_highest_chan_num;  // Set Highest channel

    ADC10CTL0 &= ~(ADC10IFG);          // Clear out any old interrupts

    ADC10CTL0 |= ADC10IE;              // Enable ADC conv complete interrupts

    ADC10CTL0 |= ADC10ON  + REFON;     // Turn on the ADC module for sampling

#if (TO_DO)
    if (_g_trigger_source == TRIGGER_USER_APP)
#endif
       board_adc_user_trigger_start (0, 0); // kick off initial Conversion

    return (0);                          // denote success
}


//*****************************************************************************
//  board_adc_disable
//
//          Turn off one or all the sequencers that have been configured.
//
//          Reset everything in preparattioon for a re-configuratioon of
//          channels.
//*****************************************************************************
int  board_adc_disable (int adc_module_id, int sequencer)
{
//   if (ADC12_A_isBusy(ADC12_A_BASE)  &&  _g_ADC_complete == 0)
     if (_g_ADC_complete == 0)
        return (-1);                 // must wait till all conversions are done
                                     // ??? do it in here - loop ???

    ADC10CTL0 &= ~(ADC10ON);         // turn off the ADC module for sampling

    return (0);                      // denote success
}


//*****************************************************************************
//  board_adc_get_results
//
//          Returns an array of all the ADC values that were converted,
//          related to a/all the sequenced group(s)
//*****************************************************************************
int  board_adc_get_results (int adc_module_id, int sequencer, int *channel_results)
{
    int             i;
    unsigned short  *adc_mem_data;

#if (TO_DO)
//  channel_results[0] = HAL_ADC_GetValue (ADC1);   DMA will have already put it in channel_results

    if (_g_DMA_complete == 0)
       return (0);             // denote 0 results because DMA rupt has not happened yet
    if (_g_DMA_overrun)
       {    // we had an overrun. Discard the results and tell user try again
         _g_DMA_overrun  = 0;  // clear error flag
         _g_DMA_complete = 0;  // reset for new pass
         return (-1);          // denote we had an overrun condition
       }
#endif

    for (i = 0;  i < _g_num_configured_channels;  i++)
       {       // copy the internally ADC/DMA staged results into user's buffer
         channel_results[i] = _g_adc_conv_results[i];
       }

    _g_ADC_complete = 0;                 // reset for new pass

    return (_g_num_configured_channels); // pass back number of completed conversions
}


//*****************************************************************************
//  board_adc_user_trigger_start
//
//          Trigger sampling for a sequenced group.
//
//          For MSP430, the various SHP/SHSx settings require different
//          combinations of manually toggling SC and/or ENC to start a new
//          conversion. For some combinations of SHP/SHSx, just triggering
//          ADCxxSC is insufficient. So rather than try to track the 6
//          different SHP/SHSx combinations, just brute force it and toggle
//          ENC each time. That handles all the weird combinations.
//*****************************************************************************
int  board_adc_user_trigger_start (int adc_module_id, int sequencer)
{
    ADC10CTL0 &= ~(ENC);           // clear ENC to force state machine reset  late change 05/19/15 - UNTESTED
    ADC10CTL0 |= ADC10SC | ENC;    // Start new conversion via software trig

    return (0);                  // denote success
}


/****************************************************************************
  *                         ADC Conversion ISR
  *
  *         If running straight interrupts (xxx_IT), this gets called by
  *         the HAL ADC library support when the ADC interrupt completes.
  *
  *         If running with DMA (xxx_DMA), this gets called
  *         when the DMA interrupt completes, i.e. this gets
  *         invoked as a callback indirectly via the HAL_DMA_IRQHandler()
  *
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion,
  *         and you can add your own implementation.
  * @retval None
****************************************************************************/

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void  ADC_ISR (void)
{
    int             i;
    unsigned short  *adc_mem_data;

//  if (ADC10CTL0 & ADC10IFG)   // ADC10IFG is NOT set. If we enter here it is assumed.  05/19/15
       {     // this is the ending IFG0 - IFG14 that signals end of sequence
             // completed.  Move converted results to our internal staging buf.
             // FR2533 pops out 1 results per interrupt, so must track things
             // with a result index.  Here is where DMA could be warranted !
          _g_adc_conv_results[_g_result_idx] = ADC10MEM;  // Move result, IFG is cleared
          _g_result_idx++;
          if (_g_result_idx >= _g_num_configured_channels)
                  {     // last channel of a "sequnce of channels" has been processed.
                    _g_ADC_complete = 1;  // set status that ADCs and DMA completed.
                                          // Alternative = invoke user callback rtn.
                                          // Used by adc_Check_All_Complete() logic
                    _g_result_idx = 0;    // Wrap back to first entry for a new pass
                  }
       }

    dma_callback_seen++;                        // DEBUG COUNTER
}


/************************************************************************
*                              DMA   ISR
*
* @brief  This function handles DMA interrupt request.
*         It routes it to the ST hAL library, which does some
*         pre-pcoessing on it, then invokes the associated
*         callback we defined (HAL_ADC_ConvCpltCallback)
*
* @param  None
* @retval None
************************************************************************/

#if (TO_DO)

void  DMA2_Stream0_IRQHandler (void)
{
    dma_rupt_seen++;                        // DEBUG COUNTER
    HAL_DMA_IRQHandler (AdcHandle.DMA_Handle);

//  HAL_ADC_Stop_DMA(hadc);   // ??? need - bit is blow up when re-enable ADC_IT
}
#endif

#endif                        // USES_ADC



//*****************************************************************************
//*****************************************************************************
//                               GPIO   Routines
//*****************************************************************************
//*****************************************************************************


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

#if defined(DRV8711)
       // Configure GPIOs
    P1SEL  &= ~(POT | nSLEEP);
    P1SEL2 &= ~(POT | nSLEEP);

    P1DIR  |= (POT | nSLEEP);
    P1OUT  |= (POT | nSLEEP);

    P2SEL  &= ~(RESET | STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1 | nFAULT | nSTALL);
    P2SEL2 &= ~(RESET | STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1 | nFAULT | nSTALL);

    P2DIR  |=  (RESET | STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1);
    P2OUT  &= ~(RESET | STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1);

    P2DIR  &= ~(nFAULT | nSTALL);  // FAULT/STALL: ensure direction = IN
    P2REN  |=  (nFAULT | nSTALL);  // Turn on Pull-Ups for FAULT and STALL

    P3DIR  |=  (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
    P3OUT  &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
#endif

}


/*******************************************************************************
*  Board GPIO Pin Config
*
*        Configure an individual GPIO pin.   Use default strength = 2ma/pin
*******************************************************************************/
void  board_gpio_pin_config (uint32_t gpio_port, unsigned long pin,
                             int dir, int pull)
{
#if (LATER)
    if (dir == GPIO_OUTPUT)
       GPIO_setAsOutputPin (gpio_port, pin);              // set pin to OUTPUT
       else {      // set as input, based on Pullup Resistor settings
              if (pull == GPIO_NOPULL)
                 GPIO_setAsInputPin (gpio_port, pin);     // set pin to INPUT
                 else if (pull == GPIO_PULLUP)
                         GPIO_setAsInputPinWithPullDownResistor (gpio_port,pin);
                 else if (pull == GPIO_PULLDOWN)
                         GPIO_setAsInputPinWithPullUpResistor (gpio_port, pin);
            }
#endif
}


/*******************************************************************************
*  board_irq_pin_config
*
*        Configure an individual GPIO pin to support incoming Interrupt requests
*******************************************************************************/
void  board_irq_pin_config (uint32_t gpio_port,  unsigned long pin,
                            int rise_fall, int pullup, uint32_t irq_vector_num,
                            unsigned long priority)
{
#if (LATER)
    unsigned char  rupt_mode;

    board_gpio_pin_config (gpio_port, pin, GPIO_INPUT,
                           pullup);                     // config base pin props

    if (rise_fall == GPIO_RUPT_MODE_RISING)
       rupt_mode = GPIO_LOW_TO_HIGH_TRANSITION;         // rising edge
       else rupt_mode = GPIO_HIGH_TO_LOW_TRANSITION;    // falling edge

    GPIO_clearInterruptFlag (gpio_port, pin);     // clear out out any old rupts

    GPIO_interruptEdgeSelect (gpio_port,pin,rupt_mode); // set type of interrupt

        // Model TI's support, and avoid any "early interrupts" that 
        // ISR may not be expecting (e.g. CC3100 Simplelink)
    GPIO_disableInterrupt (gpio_port, pin);

// caller must call  board_irq_pin_enable();  when ready/init complete
#endif
}



#if defined(USES_PWM) || defined(USES_DRV8711) || defined(USES_DRV8848) || defined(USES_DRV8301)

//*****************************************************************************
//*****************************************************************************
//                               PWM   Routines
//*****************************************************************************
//*****************************************************************************

               //--------------------------------------------------------------
               //          Supported Modules
               //
               //   Module 1  TA1      CCR 1,2,0
               //
               // On this platform, we use TA0 for the "Systick emulation"
               // logic, so we do not support TA0.1 (P 2.6) on J2-2.
               //
               // None of them support built-in Complementary operation.
               // None of them support built-in Deadtime.
               //
               // As a rule, I do not externalize CCR0, since that is normally used
               // for setting the base period for the other CCRs. But a couple
               // of Boosterpacks (BDC / Stepper) use pairs of TA1.0/TA1.1
               // for their operation, so we need to externalize it.
               //--------------------------------------------------------------
#define  PWM_NUM_MODULES     1   /* are 1 PWM capable modules on MSP430 LP      */
#define  PWM_MAX_CHANNELS    3   /* are 3 main channels and 3 alternate mappings*/

      uint32_t           uwPrescalerValue = 0;

      char               _g_pwm_module_status [PWM_NUM_MODULES+1]  = { 0,0 };
      unsigned char      _g_pwm_channel_config [PWM_NUM_MODULES+1] = { 0,0 };

const unsigned char      _g_pwm_chan_mask [9]
                                   = { 0,       // unused
                                       0x01, 0x02, 0x04, 0x08,  // Channels  1 - 4
                                       0x00, 0x00, 0x00, 0x00   // Channels  5 - n
                                     };

typedef struct pwm_channel_def         /* PWM Channel definitions */
    {
        uint8_t  pwm_gpio_port_id;    /* Associated GPIO port         */
        uint8_t  pwm_gpio_pin_bit;    /* Associated GPIO pin          */
        uint8_t   pwm_even;           /* port is even = must shift bits by 8 */
    } PWM_CHANNEL_BLK;

                           // PWM_MODULE_1  -  TA1
const PWM_CHANNEL_BLK  _g_pwm_mod_1_channels [] =
        {              //                    PWM   Energia   LP   Grove
          { GP_PORT_2, BIT0, 0 },  // P 2.0  TA1.0    8     J1-8
          { GP_PORT_2, BIT1, 0 },  // P 2.1  TA1.1    9     J1-9
          { GP_PORT_2, BIT4, 0 },  // P 2.4  TA1.2   12     J2-9
                           // Alternate mappings
          { GP_PORT_2, BIT3, 0 },  // P 2.3  TA1.0   11     J2-11
          { GP_PORT_2, BIT2, 0 },  // P 2.2  TA1.1   10     J1-10
          { GP_PORT_2, BIT5, 0 }   // P 2.5  TA1.2   13     J2-8
        };

//*****************************************************************************
//  board_pwm_init
//                  was  timer_pwm_init
//
//         Initialize PWM module
//
//         flags parm is for future use.
//*****************************************************************************

int  board_pwm_init (int modgen_id, int count_mode, long period_val, int flags)
{
    uint32_t          prescalar;

    if (modgen_id !=1)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    if (count_mode == TIMER_PERIODIC_COUNT_UP || count_mode == TIMER_PERIODIC_COUNT_UPDOWN)
       ;                                           // mode is valid
      else return (ERR_PWM_COUNT_MODE_INVALID);    // count_mode is bad - bail

// ??? revisit the following
        /* Compute the prescaler value to have TIMx counter clock equal to 16 MHz */
//  prescalar = ((SystemCoreClock /2) / 16000000) - 1;

      //--------------------------------------------------------
      // Configure the associated TAx / TBx  module
      //--------------------------------------------------------
    TA1CCR0 = period_val;                           // set period to use in TA1
    _g_pwm_module_status [modgen_id] = count_mode;  // save count mode for Enable

    return (0);                   // denote success
}


//*****************************************************************************
//  board_pwm_config_channel
//
//         Configure a specific channel (0-2) on a specific PWM module (TA1)
//*****************************************************************************

int  board_pwm_config_channel (int modgen_id,  int chan_num,
                               long initial_duty,  int flags)
{
    PWM_CHANNEL_BLK     *pwmcblk;
    uint32_t            scaleduty;
    unsigned char       chan_mask;
    uint16_t            pwm_mode;
    uint8_t             bit_mask;

    if (modgen_id != 1)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (chan_num < 1  ||  chan_num > PWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

       //-------------------------------------------------------------
       // Setup associated GPIO(s) into Timer/PWM mode
       //
       //-------------------------------------------------------------
    pwmcblk = (PWM_CHANNEL_BLK*) &_g_pwm_mod_1_channels [chan_num];
          // set associated DIR SEL bits to denote this is being used
          // as a PWM channel.  On G2553, all our PWM ports are on GPIO PORT 2
    bit_mask = pwmcblk->pwm_gpio_pin_bit;
    P2DIR  |= bit_mask;     // set for Output
    P2SEL  |= bit_mask;     // Set for PRIMARY
    P2SEL2 &= ~(bit_mask);  //    Periph Function

// ??? probably have to divide duty cycle based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;    // get Master PWM Clock value
//  scaleduty = (_g_PWM_IO_clock / duty_cycle) - 1;   // set new scaled duty cycle

    chan_mask = _g_pwm_chan_mask [chan_num];    // get associated channel mask

      //-------------------------------------------------------------------
      // Configure the desired CCR channel for PWM in the TAx / TBx module
      //-------------------------------------------------------------------
    pwm_mode = OUTMOD_7;                        // default = standard PWM
    if (flags == PWM_TOGGLE_OUTPUT)
       pwm_mode = OUTMOD_4;                     // use CCR to just toggle outputs
    switch (chan_num)
               {
                 case 0:                        // PWM_CHANNEL_0
                 case 3:                        // PWM_CHANNEL_0A
                       TA1CCTL0 = pwm_mode;     // config CCR0 PWM as reset/set
                       TA1CCR1  = initial_duty;
                       break;
                 case 1:                        // PWM_CHANNEL_1
                 case 4:                        // PWM_CHANNEL_1A
                       TA1CCTL1 = pwm_mode;     // config CCR1 PWM as reset/set
                       TA1CCR1  = initial_duty;
                       break;
                 case 2:                        // PWM_CHANNEL_2
                 case 5:                        // PWM_CHANNEL_2A
                       TA1CCTL2 = pwm_mode;     // config CCR2 PWM as reset/set
                       TA1CCR2  = initial_duty;
                       break;
                 default:
                       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
               }

// ??? does OUTMOD_4 auto-reload the CCR after expiration, or does it require
//     interrupts, and the CCR must be manually reloaded after each pass,
//     in the ISR via:    CCRx += duty_cycle;


    _g_pwm_channel_config [modgen_id] |= chan_mask; // denote channel was configured

    return (0);               // denote successful configuration
}



//*****************************************************************************
// board_pwm_disable
//
//            Disable the Speed or PWM timer.
//            (in preparation for Stopping or Reconfiguring the motor)
//*****************************************************************************
int  board_pwm_disable (int modgen_id, int flags)
{
    if (modgen_id != 1)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

      //--------------------------------------------------------
      //Disable the associated TAx / TBx  module
      //--------------------------------------------------------
    TA1CTL &= ~(MC_3);      // Disable PWM counting

    return (0);             // denote completed successfully
}


//*****************************************************************************
// board_pwm_enable
//
//            Enable and Start the timer in Speed or PWM mode.
//*****************************************************************************
int  board_pwm_enable (int modgen_id, int flags)
{
    int  count_mode;

    if (modgen_id != 1)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

    if (_g_pwm_module_status [modgen_id] == TIMER_PERIODIC_COUNT_UP)
       count_mode = MC_1;                             // up mode
       else count_mode = MC_3;                        // up/down mode

      //--------------------------------------------------------
      // Enable the associated TAx / TBx  module
      //--------------------------------------------------------
    TA1CTL = TASSEL_2 | count_mode | TACLR;  // Enable PWM: SMCLK, clear TAR

    return (0);                // denote completed successfully
}


//*****************************************************************************
//  board_pwm_set_period
//
//         Set aa new PWM Period.
//*****************************************************************************

int  board_pwm_set_period (int modgen_id, long period_val)
{
    uint32_t            scaleperiod;

    if (modgen_id != 1)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

// ??? probably have to re-divide period based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;  // get Master PWM Clock value
//  scaleperiod = (_g_PWM_IO_clock / period_val) - 1;    // set new scaled period

      //--------------------------------------------------------
      // set new PWM period into PWM module TAx / TBx
      //--------------------------------------------------------
    TA1CCR0 = period_val;                       // TA1  module

    return (0);                    // denote completed successfully
}


//*****************************************************************************
//  board_pwm_set_duty_cycle
//
//         Set the PWM's duty cycle.
//         Scale it if necessary, based on any pre-scalars we had
//         to use on the PWM's base Clock.
//*****************************************************************************

int  board_pwm_set_duty_cycle (int modgen_id, int chan_num, long duty_cycle)
{
    uint32_t            scaleduty;

    if (modgen_id != 1)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (chan_num < 1  ||  chan_num > PWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);

// ??? probably have to divide duty cycle based on what did for initial period ???
//  _g_PWM_IO_clock = MAP_SysCtlClockGet() / _g_pwm_mdivider;    // get Master PWM Clock value
//  scaleduty = (_g_PWM_IO_clock / duty_cycle) - 1;   // set new scaled duty cycle

      //--------------------------------------------------------
      // Update the desired CCR channel in the TAx / TBx module
      //--------------------------------------------------------
    switch (chan_num)
               {
                 case 0:                        // PWM_CHANNEL_0
                 case 3:                        // PWM_CHANNEL_0A
                       TA1CCR0  = duty_cycle;
                       break;
                 case 1:                        // PWM_CHANNEL_1
                 case 4:                        // PWM_CHANNEL_1A
                       TA1CCR1  = duty_cycle;
                       break;
                 case 2:                        // PWM_CHANNEL_2
                 case 5:                        // PWM_CHANNEL_2A
                       TA1CCR2  = duty_cycle;
                       break;
                 default:
                       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
               }
    return (0);                  // denote completed successfully
}


//*****************************************************************************
//  board_pwm_set_channel_output
//
//         Set polarity of channel output (start high or low)
//*****************************************************************************
int  board_pwm_set_channel_output (int modgen_id, int channel_num,
                                   int output_mode, int flags)
{
    if (modgen_id != 1)
       return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);
    if (channel_num < 1  ||  channel_num > PWM_MAX_CHANNELS)
       return (ERR_PWM_CHANNEL_NUM_OUT_OF_RANGE);
    if (_g_pwm_module_status [modgen_id] == 0)
       return (ERR_PWM_MODULE_NOT_INITIALIZED);


      // ??? is this supported on MSP430 - perhaps tweak TAxCCTLx = OUTMOD_
      //     to do set/reset instead of reset/set ?


//  return (ERR_PWM_MODULE_ID_OUT_OF_RANGE);

    return (0);                // denote completed successfully
}

#endif                         // defined(USES_PWM)



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
void  board_systick_timer_config (void)
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

#if defined(USES_MQTT)
    SysTickIntHandler();     // invoke MQTTCC3100.c to update its Timer
#endif

#if defined(USES_VTIMER)
    if (_g_vtimers_active > 0)
       board_vtimer_check_expiration();
#endif
}



#if (LATER)
//**********************************************************************
//                          TIMER  SUPPORT
//
// The DRV8711 can be used to drive either 1 bi-polar stepper motor,
// or up to two Brushed DC (BDC) motors.
//
// When supporting a Stepper motor, the timer is used to time motor
// movements, especially for accelerating or decelerating, as part of
// the "speed profile" ramp up, run, then ramp down sequencing.
// So in that mode, the timer is configured as a pure timer, and the
// CCRs (timer compare registers) are set to the proper durations
// of a programmed speed timing sequence.
//
// When supporting BDC motors, the timer is used to generate PWM signals
// to drive the motors. The DRV8711 is re-configured for BDC mode, and
// is used to act as a gate driver for the PWM signals to the FETs
// that drive the BDC motors. In that mode, the timer is configured as
// a PWM generator, and the CCRs are set to the "duty cycle" for the
// PWMs.
//
// During testing, the user can dynamically change the motor
// configuration, from Stepper to BDC or vice versa. When that occurs,
// the Timer needs to be stopped and re-configured for the new Mode.
//
// MSP430 Timer1 Notes:
//  - TACCR0 is the master period register
//  - TAR    is the incrementing/decrementing timer counter
//  - TACCR1 is Compare value 1  (with separate interrupt)
//  - TACCR2 is compare value 2
//  - Counter Modes:  1 = count up to TACCR0      2 = count up to 0xFFFF
//                    3 = count up/down to TACCR0
//  - Output Modes:   0 = force timer out to immediate value  (0 or 1)
//                    3 = (PWM) set/reset mode: low while < TACCR1,
//                       high while > TACCR1,  reset low when hit TACCR0
//**********************************************************************

//--------------------------------------------------------------------
// interval_timer_init
//
//            Initialize the periodic Interval Timer.
//---------------------------------------------------------------------
void  interval_timer_init (void)
{
    // WDT requires no additional configuration.
    // It is used in "Interval Timer" mode.
    // This routine is mainly for Tiva and C2000 implementations
}


//--------------------------------------------------------------------
// interval_timer_disable
//
//            Shut off the Interval Timer
//            WatchDog Timer is used as an Interval Timer.
//---------------------------------------------------------------------
void  interval_timer_disable (void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop Watchdog Timer
    IE1   &= ~WDTIE;            // Disable Watchdog Interrupts
}


//--------------------------------------------------------------------
// interval_timer_enable
//
//            Turn on the Interval Timer, used for Accel/Decel Updates
//            WatchDog Timer is used as an Interval Timer.
//---------------------------------------------------------------------
void  interval_timer_enable (void)
{
    WDTCTL = WDT_MDLY_32;   // setup Watch dog time duration
    IE1   |= WDTIE;         // Enable Watch dog interrupt
}


//------------------------------------------------------------------
//  timer_pwm_init
//
//         Initialize PWM mode
//
//         flags parm is for future use.
//
//  Tiva pin PA6 uses M1PWM2, which is Module 1 PWM 2.  (PWM2EN)
//  It is controlled by                Module 1 PWM, Generator 1.
//------------------------------------------------------------------

void  timer_pwm_init (long flags)
{
    // No special PWM startup processing required by MSP430.
    // This routine is mainly for Tiva and C2000 implementations
}


//------------------------------------------------------------------
//  timer_config_pwm_mode
//
//         Turn on PWM mode, output goes to associated STEP1_AIN pin.
//         Used when want to drive BDC motors with PWM.       VERIFY
//
//         flags parm is for future use.
//------------------------------------------------------------------

void  timer_config_pwm_mode (long flags)
{
       // Setup PWM Output mode and enable Interrupts
    TA1CCTL0 = CCIE;
    TA1CCTL1 = OUTMOD_3 | CCIE;
}


//------------------------------------------------------------------
//  timer_config_period_duty_cycle
//
//         Set the PWM Period, and starting duty cycle.
//------------------------------------------------------------------

void  timer_config_period_duty_cycle (long period, long duty_cycle)
{
    TA1CCR0 = period;        // set PWM period into timer
    TA1CCR1 = duty_cycle;    // set starting duty cycle into timer
}


//-------------------------------------------------------------------
// timer_disable_pwm
//
//            Disable the Speed or PWM timer.
//            (in preparation for Stopping or Reconfiguring the motor)
//--------------------------------------------------------------------
void  timer_disable_pwm (void)
{
    TA1CTL = TASSEL_2 | MC_0 | TACLR;     // turn off the timer
}


//-------------------------------------------------------------------
// timer_enable_pwm
//
//            Enable and Start the timer in Speed or PWM mode.
//--------------------------------------------------------------------
void  timer_enable_pwm (void)
{
    TA1CTL = TASSEL_2 | MC_1 | TACLR;     // start up the timer
}


//-------------------------------------------------------------------
// timer_force_pwm_pin_low
//
//            Forcibly turn off PWM mode, and force output pin low
//
//            pin_num parm is for future use.
//--------------------------------------------------------------------
void  timer_force_pwm_pin_low (short pin_num)
{
            TA1CCTL1 = OUTMOD_0;       // Turn off Timer PWM mode
            P2OUT   &= ~STEP_AIN1;     // force timer output pin LOW
}


//------------------------------------------------------------------
//  timer_set_pin_for_pwm
//
//         Setup GPIO pins and associated "Alternate Function" bits
//         to run the pin(s) as PWM outputs from the Timer
//------------------------------------------------------------------

void  timer_set_pin_for_pwm (short pin_num)
{
       // Setup pin(s) for Timer PWM Output
    P2DIR  |= STEP_AIN1;
    P2SEL  |= STEP_AIN1;    // Route Timer1 CCR1 output (PWM) to pin
    P2SEL2 &= ~STEP_AIN1;
}


//------------------------------------------------------------------
//  timer_set_pin_for_gpio
//
//         Setup GPIO pins and associated "Alternate Function" bits
//         to run the pin(s) as standard GPIO pins  (turn off PWM)
//------------------------------------------------------------------

void  timer_set_pin_for_gpio (short pin_num)
{
       // Re-configure pin(s) to operate as normal GPIOs
    P2DIR  |= STEP_AIN1;
    P2SEL  &= ~STEP_AIN1;   // Disable Timer1 output. Set to std GPIO
    P2SEL2 &= ~STEP_AIN1;
    P2OUT  &= ~STEP_AIN1;
}


/****************************Interrupt Service Routines*****************************/

//---------------------------------------------------------------
//                       ISR   Timer1 - A0
//
// This Timer (PWM) interrupt pops for the following event:
//      - PWM end of period: This is used to roll in the new
//        Period and Duty cycle to be used for the next cycle.
//        In general, the period remains the same, and
//        only the duty cycle changes, in reponse to ramp up
//        or ramp down situations.
//        At constant speeds, the duty cycle is 50 %.
//---------------------------------------------------------------

#pragma  vector=TIMER1_A0_VECTOR
__interrupt void  Timer1_A0 (void)
{
       // Update Timer PWM period and duty cycle at End of a PWM Period
    if (G_LOAD_CCR_VALS == true)
       {
         G_CUR_SPEED = G_CUR_SPEED_TEMP;
         TA1CCR0     = G_TA1CCR0_TEMP;   // set (new) Period
         TA1CCR1     = G_TA1CCR1_TEMP;   // set new Duty Cycle
         G_LOAD_CCR_VALS = false;
       }
}


//---------------------------------------------------------------
//                      ISR   Timer1 - A1
//
// This Timer (PWM) interrupt pops for the following event:
//      - CCR match/completion: This is used to increment the
//        Step Counter, to count the number of steps (PWM pulses)
//        that have been issued to the Stepper motor.
//---------------------------------------------------------------
#pragma vector=TIMER1_A1_VECTOR
__interrupt void  Timer1_A1 (void)
{
    switch (TA1IV)
    {
        case TA1IV_NONE: break;         // Vector 0: No Interrupt

               //-----------------------------------------
               // CCR1 (Duty Cycle) completion interrupt
               //-----------------------------------------
        case TA1IV_TACCR1:              // Vector 2: CCR1 CCIFG
          {
                // Increment Step Counter
            G_CUR_NUM_STEPS++;
            if (G_CUR_NUM_STEPS == G_TOTAL_NUM_STEPS)
               {
//               __bic_SR_register_on_exit (LPM0_bits);
               }
            TA1CCTL1 &= ~CCIFG;         // Clear interrupt flag
            break;
          }

        case TA1IV_TACCR2:              // Vector 4: CCR2 CCIFG
          {
            TA1CCTL2 &= ~CCIFG;
            break;
          }

               //-----------------------------------------
               // CCR0 (Period) completion interrupt   ???
               //-----------------------------------------
        case TA1IV_6: break;            // Vector 6: Reserved CCIFG
        case TA1IV_8: break;            // Vector 8: Reserved CCIFG
        case TA1IV_TAIFG:               // Vector 10: Overflow
          {
            TACTL &= ~TAIFG;            // Clear interrupt flag
            break;
          }

        default: break;
    }
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
#endif


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

#endif                  // defined(__MSP430G2553__)
