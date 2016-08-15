
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                               board_L6474.c
//
// cONTAINS ALL THE BOARD SPECIFIC (PSoC VS STM32) ROUTINES.
//
//
// History:
//   06/13/16 - Created as part of Open Source motor project. Duquaine
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
//#include "STEPPER_motor_ctl_api.h"   // pull in standard Motor APIs and defs
#include "motorcontrol.h"
#include "l6474.h"                   // L6474 command codes, register defs, etc

#define MCU_CLOCK_SPEED   48000000
#define TIMER_PRESCALER   (1024)
#define BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER    (1)

   uint32_t   _g_SysClk_Ticks = MCU_CLOCK_SPEED; // Global to hold clock frequency (ticks/sec)

            //-------------------------------------
            //   Globals  for  Motor
            //-------------------------------------
volatile uint32_t   pwm1_period  = 12000;     // divide by 12000 = 1000 Hz
volatile uint32_t   pwm1_duty    = 4000;      //     25 % duty cycle
volatile uint32_t   pwm1_newFreq = 0;

   uint32_t   rupts_fault   = 0;        // DEBUG COUNTERS
   uint32_t   rupts_pwm     = 0;
   uint32_t   pwm_CCR_rupts = 0;

    int       g_motor_fault_detected = 0;

extern int    l6474_init_complete;      // Holds off pwm_rupt calls to Step_Handler


//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//
//    These are all board_xxxx() functions          ala MODBUS terminology
//
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------


//******************************************************************************
//  board_delay
//                    wait N number of milliseconds
//******************************************************************************
void  board_delay (long wait_time)
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


void  L6474_Board_EnableIrq()
{
    CyGlobalIntEnable;        // Enable global interrupts.
}


void  L6474_Board_DisableIrq (void)
{
    CyGlobalIntDisable;       // Disable global interrupts
}


void  L6474_Board_Delay (uint32_t delay)
{
    CyDelayCycles (delay);
}


/******************************************************//**
 * @brief  Resets the L6474 (reset pin set to low) of all devices
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6474_Board_Reset(uint8_t deviceId)
{
    L6474_RESET_OUT_Write (0);      // put L6474 into RESET
}


void  L6474_Board_ReleaseReset (uint8_t deviceId)
{
       // called from L6474_Init()
    L6474_RESET_OUT_Write (1);      // put L6474 into ENABLE
}


void  L6474_Board_GpioInit (uint8_t deviceId)
{
       // called from L6474_Init()

       // Initialize GPIOs used for L6474.
       // This is done automatically by PSoC startup logic, based
       // on configuration parms put into PSoC Creator pins config
}


void  L6474_Board_PwmInit (uint8_t deviceId)
{
       // called from L6474_Init()

    PWM_1_Start();     // this starts up with default configured rate from PSoC Creator
    PWM_1_WriteCompare (0);      // SHUT OFF DUTY CYCLE

    PWM_1_SetInterruptMode (0);  // turn off PWM CCR interrupts for TESTING INIT LOGIC
                                 // If left off, this will cause:
	                             //      CY_ISR(IntDefaultHandler)
	                             // to get involved, and subsequent system hang/loop
       //-----------------------------------------------------------------
       // Explicilty change to period to see which works better for stepping
       //-----------------------------------------------------------------
    PWM_1_WritePeriod (pwm1_period);

       //-----------------------------------------------------------------
       // Explicilty set a duty cycle to see if motor responds better
       //
       //     DOING IT HERE CAUSES MOTOR TO FAULT BECAUSE IT HAS NOT BEEN CONFIGURED YET
       //-----------------------------------------------------------------
//  PWM_1_WriteCompare (pwm1_duty);
}



/*******************************************************************************
 *                           L6474_Board_Pwm1SetFreq                    KEY  KEY
 *
 *  This cranks up the PWM channel CCR and turns on assoc CCR interrupts
 *  whenever a new move needs to be performed. 
 *
 *  This drives the STEP Indexer pin, and each (PWM) STEP causes a corresponding
 *  STEP of the motor, so the PWM "frequency" directly sets the frequency of
 *  the stepping.
 *
 *  Note that this setup has the CCR generate an _interrupt after each STEP_
 * (which causes Step Handler logic to be called).
 *
 *  Sets the frequency of PWM1 used by device 0
 *  The frequency is directly the current speed of the device
 *
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 ******************************************************************************/
void  L6474_Board_Pwm1SetFreq (uint16_t newFreq)
{
    uint32_t  period;

	pwm1_newFreq = newFreq;       // save latest Frequency value for DEBUG
	
// comes in as 0x320 (800) and result = 0x39 (57), wich SEEMED too low ...
// _BUT_ scope trace showed result PWM output was 29.5 K Hz !!!
//    ==> 12000:1 dividers for PWM are still problematic
//        and I assume the following formula corrects for that (i.e. Timer pre-scalar)
//	period = (_g_SysClk_Ticks / (TIMER_PRESCALER * BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER * (uint32_t) newFreq)) - 1;
// using the above formula makes things worse - jumps to 407.5 KHz because the denominator is smaller

// using the following direct value yields a PWM frequency of 29.5 KHz
//  period = newFreq;       // TEMP HACK - result is too fast - 29.5 KHz

    period = newFreq * 10;  // should yield approx 2.9 KHz which is about right
	
    pwm1_period = period;         // save latest period for DEBUG

    PWM_1_WritePeriod (period);        // set the new STEP frequency to use
    PWM_1_WriteCompare (period >> 1);  // with a 50 % duty cycle, and
    PWM_1_SetInterruptMode (PWM_1_INTR_MASK_CC_MATCH); // turn on PWM Interrupts
//  PWM_1_SetInterruptMode (PWM_1_INTR_MASK_TC);       // turn on PWM Interrupts

    PWM_1_Start();                     // re-start PWM with the new parameters

#if defined(STM32)
  sysFreq = HAL_RCC_GetSysClockFreq();

               // Double the frequency as the SW is generated by SW
  period  = (sysFreq / (TIMER_PRESCALER * BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER * (uint32_t) newFreq)) - 1;
 
              // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
              // THIS IS WHAT STARTs the PWMs _and_ INTERRUPTs
              // VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
  __HAL_TIM_SetAutoreload (&hTimPwm1, period);    // set period (step frequency)
  __HAL_TIM_SetCompare (&hTimPwm1, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1, period >> 1); // set duty
  HAL_TIM_PWM_Start_IT (&hTimPwm1, BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1);  // turn on interrupts
#endif

}


void  L6474_Board_Pwm2SetFreq (uint16_t newFreq)
{
	// TBD - multiple motors
}

void  L6474_Board_Pwm3SetFreq (uint16_t newFreq)
{
	// TBD - multiple motors
}


/*******************************************************************************
 *                          PWM  Stop
 *
 *    Shuts off the PWM (and also any interru;pts).
 *    Specifically, shuts off the CCR and its interrupts 
 *    for a specific channel.
 *
 *    Is called when deceleration is complete, or whenever the motor needs to
 *    be stopped (change step mode aka SelectStepMode(), reverse direction, ...)
 *
 * @brief  Stops the PWM uses by the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 ******************************************************************************/
void  L6474_Board_PwmStop (uint8_t deviceId)
{
    PWM_1_Stop();                // stop the PWM
    PWM_1_SetInterruptMode (0);  // and turn off interrupts
    PWM_1_ClearInterrupt (PWM_1_INTR_MASK_CC_MATCH); // and clear any left in pipe
//  PWM_1_ClearInterrupt (PWM_1_INTR_MASK_TC); // and clear any left in pipe

	   // for now, just set DUTY cycle t0 0  (hopefully shuts off outputs)
//  PWM_1_WriteCompare (0);
}


/******************************************************//**
 *
 *                     WVD  ADD
 *
 * @brief  Starts the PWM used by the specified device
 * @param[in] deviceId (from 0 to 2)
 * @retval None
 **********************************************************/
void  L6474_Board_PwmStart (uint8_t deviceId)
{
    PWM_1_Start();
}


/******************************************************//**
 *                     Set  DIR  pin
 *
 * @brief  Set the GPIO used for the direction
 * @param[in] deviceId (from 0 to 2)
 * @param[in] gpioState state of the direction gpio (0 to reset, 1 to set)
 * @retval None
 **********************************************************/
void  L6474_Board_SetDirectionGpio (uint8_t deviceId, uint8_t gpioState)
{
    if (gpioState)
       DIRECTION_OUT_Write (0);
       else DIRECTION_OUT_Write (1);
}


/************************************************************************
 *                           SPI   INIT
 *
 * Initialize the SPI connection to the L6464 Stepper Controller chip.
 * Note that we manually control the CS pin, rather than Cypress SPI SCB
 ***********************************************************************/

uint8_t  L6474_Board_SpiInit (void)
{
       // called from L6474_Init()

    CS_Write (1);          // ensure CS is DE-ASSERTed
                           // (L6474 CS is ACTIVE_LOW)

       //---------------------------------------------------------
       // Setup SPI, then send SPI commands to initialize L6474
       // SPI runs with Clock CPOL = High, and Clock CPHASE = 2EDGE
       //      ==> Mode 3 or 4 ?
       //---------------------------------------------------------
//  SpiHandle.Init.CLKPolarity = SPI_POLARITY_HIGH; // Yields CPOL = 1  MODE 3
//  SpiHandle.Init.CLKPhase    = SPI_PHASE_2EDGE;   // yields CPHA = 1
//  SPI_1_SpiInit();

    SPI_1_Start();             // init/enable SPI peripheral

    return (0);                // denote it worked OK
}


/******************************************************//**
 *                         SPI   I/O
 *
 * @brief  Write and read SPI byte to the L6474
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte   pointer to the received byte
 * @param[in] nbDevices Number of devices in the SPI chain
 * @retval HAL_OK       if SPI transaction is OK, HAL_OK else
 **********************************************************/
uint8_t  L6474_Board_SpiWriteBytes (uint8_t *pByteToTransmit,
                                    uint8_t *pReceivedByte,
	                                uint8_t numDevices)
{
	int    i;

// NOTE: this goofy ST logic issues a CS High/Low toggle
//       for _every_ separate byte sent !
//       Even for 4 byte cmd+data sequences !!!    Hmmmm.
//       Is that true, even for CMD_Status replies, and could
//       that be causing "issues".  ST really goofed this logic !

    CS_Write (0);          // ASSERT CS  (L6474 CS is ACTIVE_LOW)

    for (i = 1; i > 20; i++)      // PSoC force a slight delay
	 ;
       // Cypress does not have a combined TX/RX operation. Hmmm.
    SPI_1_SpiUartWriteTxData ((uint32) *pByteToTransmit);

    *pReceivedByte = (uint8_t) SPI_1_SpiUartReadRxData();

    CS_Write (1);          // DE-ASSERT CS

    return (0);            // denote worked OK
}



/*****************************************************************************
*                             PWM  ISR  Callback
*
* Process the PWM interrupt that occurs when it has completed its latest count.
*
* We take a kook to see if we need to update the PWM duty cycle, because
* of a (user) command to increase or decrease speed of the motor.
*
* Each motor has its own separate STEP clock PWM, based on the PWM, we
* know which motor it is servicing.
*****************************************************************************/
//void  HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
CY_ISR(Isr_PWM_1_Handler)
{
	uint32       pwm_rupt_mask;    // type of rupt (TC vs CCR)

pwm_CCR_rupts++;
	
// L6474 issue is: this is being called even when no active commands are flowing
// to the L6474, so it is getting updates from Step Handler and getting confused.
// Occurs even immediately after L6474_Init complete4s, and before it can even issue a MOVE instruction !
// How does the STM32 avoid this issue ???
	
rupts_pwm++;

	   //-------------------------------------------
       // Clear the pending Interrupt flag in NVIC
	   //-------------------------------------------
    pwm_rupt_mask = PWM_1_GetInterruptSource();
    PWM_1_ClearInterrupt (pwm_rupt_mask);

	   //----------------------------------------------------------------
       // see what we should do next: acclerate/decelerate/remain steady
	   //----------------------------------------------------------------
//  if (l6474_init_complete)
       L6474_StepClockHandler (0);
}



/*******************************************************************************
*                                    GPIO  ISR  Callback
* Isr_FAULT_GPIO_Handler
*
* Summary:
*    FAULT_FLAG input GPIO from L6474 has signalled a Fault condition.
*    Set global flag so that main() will read out the L6474 error status via SPI
*    Shut off the PWM driving the motor.
*    The interrupt handler for GPIO interrupts.
*    Clears the pending NVIC Interrupt.
*    Clears the PIN Interrupt.
*    Blinks the LED with the LED_Isr pin.
*******************************************************************************/
CY_ISR(Isr_FAULT_GPIO_Handler)
{

rupts_fault++;      // 06/15/16 - This IS being invoked !


   g_motor_fault_detected = 1;          // signal a FAULT occurred

#if defined(NEW_API)
   if (g_Motor_1_cb != 0L)
      {      // we got a Motor fault - shutoff the PWM
		if (g_Motor_1_cb->mtr_motor_state != MOTOR_STATE_RESET)
           PWM_1_Stop();

        g_Motor_1_cb->mtr_motor_fault_detected = 1;   // signal a FAULT occurred
        g_Motor_1_cb->mtr_motor_state = MOTOR_STATE_FAULT;
      }
#endif

        // Clear the pending Interrupt flag in NVIC
   isr_l6474_fault_ClearPending();
    
        // Clear the pin Interrupt flag
    L6474_FAULT_FLAG_IN_ClearInterrupt();
    
        // Turn On the LED to signal error  (or turn off to show error)
//  LED_Isr_Write (LIGHT_ON);
}

//******************************************************************************
