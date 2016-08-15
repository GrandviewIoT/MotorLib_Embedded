
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                          main_indexer_plc.c
//
//          This example shows how to use 1 IHM01A1 expansion board
//
//
//
// History:
//    06/20/16 - Created for Stepper support on STM32 with L6474 chip. Duquane
//
// L6474:
//    - Intelligent, SPI based controller, capable of creating on-the fly
//      Motion Profiles in conjunction with MCU based SPI commands.
//
// Stepper Motors tested  (operating in Bi-Polar Series mode)
//   Leadshine  Model 42HS03  Bi-Polar/Uni-Polar  Nema 17   1.8 degree step size
//       Holding Torque 66.55 Oz-in / 0.47 Nm     Current:  0.7 to 1.0 Amps
//       Operating Range:  24 VDC to 36 VDC    (all tests used 24 VDC)
//       Series Wiring:   A+   Black    
//                        A-   Orange   Green and Yellow tied together in series
//                        B+   Red
//                        B-   Brown    Blue and White tied together in series
//
// History:
//   05/22/16 - Created as part of Motor Control open source.     Duquaine
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

#include <stdint.h>                        // pull in standard int16_t, etc defs
#include <stdbool.h>
#include "MotorLib_Api.h"                 // pull in Motor Control API defs

//#include "main.h"
//#include "boarddef.h"                      // pull in boards.c API
#include "stm32f4xx_nucleo.h"              // pull in defs for BUTTON and LED
//#include "motorcontrol.h"
#include "l6474.h"                    // L6474 command codes, register defs, etc

#define  BUTTON_KEY    0                      // turd above is not properly picking up

   GPIO_InitTypeDef    Local_GPIO_InitStruct;


   int                 do_all_moves = 0;      // 1 = do entire entire sequence
                                              // of moves for standalone testing
   volatile uint16_t   gLastError;
   int32_t             pos;
   uint16_t            mySpeed;

   uint32_t num_bf_passes    = 0;

   uint32_t g_statusRegister = 0;

void  MyFlagInterruptHandler (void);
void  Error_Handler (uint16_t error);
void  L6474_Board_PwmStart (uint8_t motor_id);
void  standalone_stepper_moves (void);


//******************************************************************************
//  MyFlagInterruptHandler
//                      This function is the User handler for the flag interrupt
//******************************************************************************

void  MyFlagInterruptHandler (void)
{
  uint16_t  statusRegister;


     // Get the value of the status register via the L6474 command GET_STATUS
//statusRegister = BSP_MotorControl_CmdGetStatus( 0);
  statusRegister = MotorLib_Get_Motor_Status (0, 0);

     // Check HIZ flag: if set, power brigdes are disabled
  if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
     {
        // HIZ state
        // Action to be customized
     }

     /* Check direction bit */
  if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
    {
       // Forward direction is set
       // Action to be customized
    }
   else
    {
       // Backward direction is set
       // Action to be customized
    }

     /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
     /* This often occures when a command is sent to the L6474 */
     /* while it is in HIZ state */
  if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
    {
      // Command received by SPI can't be performed
      // Action to be customized
    }

     /* Check WRONG_CMD flag: if set, the command does not exist */
  if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
    {
      //command received by SPI does not exist
      // Action to be customized
    }

     /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & L6474_STATUS_UVLO) == 0)
    {
      //undervoltage lock-out
      // Action to be customized
    }

     /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
    {
      //thermal warning threshold is reached
      // Action to be customized
    }

     /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_SD) == 0)
    {
      //thermal shut down threshold is reached
      // Action to be customized
   }

     /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & L6474_STATUS_OCD) == 0)
     {
        //overcurrent detection
        // Action to be customized
     }

}


//******************************************************************************
//  Error_Handler
//                        This function is executed in case of error occurrence.
//******************************************************************************
void  Error_Handler (uint16_t error)
{
  gLastError = error;         // Save/backup error number

  while (1)                   // Infinite loop for debugger hook
    {
    }
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void  assert_failed (uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ** ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

      /* Infinite loop */
  while (1)
   {
   }
}
#endif


//******************************************************************************
// standalone_stepper_moves
//                           Will perform a pre-programmed series of moves using
//                           the on-board PWM TIM_3 to drive the STEP/INDEXER.
//                           Is invoked by pressing the Blue user button.
//******************************************************************************
void  standalone_stepper_moves (void)
{
   int  i;

      //---------------------------------------------------------------
      // Restore our local control over the STEP and DIR signals.
      // Change from GPIO Inputs back to PWM output and DIR output
      //---------------------------------------------------------------
      // re-Configure L6474 - STEP pin for LOCAL PWM output
#if defined(PLC_USAGE)
    Local_GPIO_InitStruct.Pin       = BSP_MOTOR_CONTROL_BOARD_PWM_1_PIN;
    Local_GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    Local_GPIO_InitStruct.Pull      = GPIO_NOPULL;
    Local_GPIO_InitStruct.Speed     = GPIO_SPEED_MEDIUM;
    Local_GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM1;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_1_PORT, &Local_GPIO_InitStruct);

      // re-Configure L6474 - DIR pin for LOCAL output
   Local_GPIO_InitStruct.Pin   = BSP_MOTOR_CONTROL_BOARD_DIR_1_PIN;
   Local_GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
   Local_GPIO_InitStruct.Pull  = GPIO_NOPULL;
   Local_GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
   HAL_GPIO_Init (BSP_MOTOR_CONTROL_BOARD_DIR_1_PORT, &Local_GPIO_InitStruct);
#endif
              // 08/13/16 When / where is CPU ENABLE_IRQ turned on, and Release_Reset issued ?
   L6474_Board_ReleaseReset (0);   // ensure L6474 is out of reset (D8 = HIGH)
   L6474_Board_PwmStart (0);       // re-startup the PWM
                                   // 08/13/16 - but PWM1 D9 TIM3_CH2 out = 0 because PSC = 0x3FF ARR = 0 and CCR2 = 0
   if (do_all_moves)               //              RESET  D8  = HIGH   FAULT D2 = HIGH
      {                            //              CS     D10 = HIGH
           //-------------------------------------------------
           //      Move of 16000 steps in the FW direction
           //-------------------------------------------------
           // Move device 0 to 16000 steps in the FORWARD direction
        MotorLib_Move_Steps (0, DIRECTION_FORWARD, 16000);  // drops into ERROR rtn, probably because of timeout PWM not running !

           // Wait for the motor of device 0 to end moving
//      BSP_MotorControl_WaitWhileActive (0);
        MotorLib_Wait_Complete (0, 0, 0);

           // Wait for 2 seconds
        board_delay_ms (2000);

           //-----------------------------------------------
           //      Move of 16000 steps in the BW direction
           //-----------------------------------------------
           // Move device 0 of 16000 steps in the BACKWARD direction
        MotorLib_Move_Steps (0, DIRECTION_BACKWARD, 16000);

           // Wait for the motor of device 0 ends moving
//      BSP_MotorControl_WaitWhileActive (0);
        MotorLib_Wait_Complete (0, 0, 0);

           // Set the current position of device 0 to be the Home position
        MotorLib_Set_Home (0);

           // Wait for 2 seconds
        board_delay_ms (2000);

           //---------------------------------------------
           //       Go to position -6400
           //---------------------------------------------
           // Request device 0 to go to position -6400
        MotorLib_Goto_Position (0, -6400);

           // Wait for the motor ends moving
 //     BSP_MotorControl_WaitWhileActive (0);
        MotorLib_Wait_Complete (0, 0, 0);

           // Get current position of device
        pos = MotorLib_Get_Position (0);

        if (pos != -6400)
           Error_Handler (11);

           // Set the current position of device 0 to be the Mark position
        MotorLib_Set_Mark (0);

           //  Wait for 2 seconds
        board_delay_ms (2000);

           //---------------------------------------------
           //       Go Home
           //---------------------------------------------
           // Request device 0 to go to Home
        MotorLib_Goto_Home (0);
//      BSP_MotorControl_WaitWhileActive (0);
        MotorLib_Wait_Complete (0, 0, 0);

           // Get current position of device 0
        pos = MotorLib_Get_Position (0);

           // Wait for 2 seconds
        board_delay_ms (2000);

           //---------------------------------------------
           //      Go to position 6400
           //---------------------------------------------
           // Request device 0 to go to position 6400
        MotorLib_Goto_Position (0, 6400);

           // Wait for the motor of device 0 ends moving
//      BSP_MotorControl_WaitWhileActive (0);
        MotorLib_Wait_Complete (0, 0, 0);

           // Get current position of device 0
        pos = MotorLib_Get_Position (0);

           // Wait for 2 seconds
        board_delay_ms (2000);

           //------------------------------------------------------
           //   Go Mark which was set previously after go to -6400
           //------------------------------------------------------
           // Request device 0 to go to Mark position
        MotorLib_Goto_Mark (0);

           // Wait for the motor of device 0 to end moving
//      BSP_MotorControl_WaitWhileActive (0);
        MotorLib_Wait_Complete (0, 0, 0);

           // Get current position of device 0
        pos = MotorLib_Get_Position (0);

           // Wait for 2 seconds
        board_delay_ms (2000);

           //--------------------------------
           //     Run the motor BACKWARD
           //--------------------------------
           // Request device 0 to run BACKWARD
        MotorLib_Run (0, DIRECTION_BACKWARD);
        board_delay_ms (5000);

           // Get current speed of device 0
        mySpeed = MotorLib_Get_Current_Speed (0, SPEED_PPS);

           //---------------------------------------------
           //     Increase the speed while running
           //---------------------------------------------
           // Increase speed of device 0 to 2400 step/sec
        MotorLib_Set_Maximum_Speed (0, 2400);
        board_delay_ms (5000);

           // Get current speed of device 0
        mySpeed = MotorLib_Get_Current_Speed (0, SPEED_PPS);

           //---------------------------------------------
           //      Decrease the speed while running
           //---------------------------------------------
           // Decrease speed of device 0 to 1200 step/sec
        MotorLib_Set_Maximum_Speed (0, 1200);
        board_delay_ms (5000);

           // Get current speed
        mySpeed = MotorLib_Get_Current_Speed (0, SPEED_PPS);

           //---------------------------------------------
           //    Increase acceleration while running
           //---------------------------------------------
           // Increase acceleration of device 0 to 480 step/s^2
        MotorLib_Set_Acceleration (0, 480);
        board_delay_ms (5000);

           // Increase speed of device 0 to 2400 step/sec
        MotorLib_Set_Maximum_Speed (0, 2400);
        board_delay_ms (5000);

           // Get current speed of device 0
        mySpeed = MotorLib_Get_Current_Speed (0, SPEED_PPS);

        if (mySpeed != 2400)
           Error_Handler (10);

           //--------------------------------------------
           //   Increase deceleration while running
           //------------------------------------------
           // Increase deceleration of device 0 to 480 step/s^2
        MotorLib_Set_Deceleration (0, 480);
        board_delay_ms (5000);

           // Decrease speed of device 0 to 1200 step/sec
        MotorLib_Set_Maximum_Speed (0, 1200);
        board_delay_ms (5000);

           // Get current speed
        mySpeed = MotorLib_Get_Current_Speed (0, SPEED_PPS);

           //---------------------------------------------
           //     Soft stopped required while running
           //---------------------------------------------
           // Request soft stop of device 0
        MotorLib_Stop (0, STOP_SOFT_BRAKE);

           // Wait for the motor of device 0 to end moving
//      BSP_MotorControl_WaitWhileActive (0);
        MotorLib_Wait_Complete (0, 0, 0);

           // Wait for 2 seconds
        board_delay_ms (2000);

           //-------------------------------
           //     Run stopped by hardstop
           //-------------------------------
           // Request device 0 to run in FORWARD direction
        MotorLib_Run (0, DIRECTION_FORWARD);
        board_delay_ms (5000);

           // Request device 0 to immediatly stop
        MotorLib_Stop (0, STOP_HARD_BRAKE);
//      BSP_MotorControl_WaitWhileActive (0);
        MotorLib_Wait_Complete (0, 0, 0);

           // Request device 0 to disable bridge
        MotorLib_Motor_Engage (0, 0, 0);

           // Wait for 2 seconds
        board_delay_ms (2000);

           //---------------------------------
           //     GOTO stopped by softstop
           //---------------------------------
           // Request device 0 to go to position 20000
        MotorLib_Goto_Position (0, 20000);
        board_delay_ms (5000);

           // Request device 0 to perform a soft stop
        MotorLib_Stop (0, STOP_SOFT_BRAKE);
//      BSP_MotorControl_WaitWhileActive (0);
        MotorLib_Wait_Complete (0, 0, 0);

           //  Wait for 2 seconds
        board_delay_ms (2000);

           //-----------------------------------------------------------
           // Read non-existent register to test MyFlagInterruptHandler
           //-----------------------------------------------------------
           // Try to read an inexistent register
           // the flag interrupt should be raised
           // and the MyFlagInterruptHandler function called
////    BSP_MotorControl_CmdGetParam (0, 0x1F);
        board_delay_ms (500);

           //-------------------------------------------
           //    Change step mode to full step mode
           //-------------------------------------------
           // Select full step mode for device 0
        MotorLib_Set_Step_Mode (0, STEP_MODE_FULL);

           // Set speed and acceleration to be consistent with full step mode
        MotorLib_Set_Maximum_Speed (0, 100);
        MotorLib_Set_Minimum_Speed (0, 50);
        MotorLib_Set_Acceleration (0, 10);
        MotorLib_Set_Deceleration (0, 10);

           // Request device 0 to go position 200
        MotorLib_Goto_Position (0, 200);

           // Wait for the motor of device 0 ends moving
//      BSP_MotorControl_WaitWhileActive (0);
        MotorLib_Wait_Complete (0, 0, 0);

           // Get current position
        pos =  MotorLib_Get_Position (0);

           // Wait for 2 seconds
        board_delay_ms (2000);
      }                           //  end  if  (do_all_moves)

      //----------------------------------------
      //    Restore 1/16 microstepping mode
      //----------------------------------------
      // Reset device 0 to 1/16 microstepping mode */
   MotorLib_Set_Step_Mode (0, STEP_MODE_1_16);

      // Update speed, acceleration, deceleration for 1/16 microstepping mode*/
// motor_set_maximum_speed (0,1600);  // set speed of device to 1600 step/s
   MotorLib_Set_Maximum_Speed (0,800);
   MotorLib_Set_Acceleration (0,160);
   MotorLib_Set_Deceleration (0,160);
           // Increase speed of device 0 to 2400 step/sec
   MotorLib_Set_Maximum_Speed (0, 2400);

           //---------------------------------------------------
           // perform a small set of back and forth Motor moves
           //---------------------------------------------------
   for (i = 0;  i < 2;  i++)
    {
          // Request device 0 to go position -6400 */
      MotorLib_Goto_Position (0, -6400);     // 08/13/16 starting come to to life, but does a loud BANG and then tick-tick-tci
                                             // PWM1 D10 is running at only 10 Hz ! on new code.  ORIG code = 
          // Wait for the motor of device 0 to end moving */
//    BSP_MotorControl_WaitWhileActive (0);
      MotorLib_Wait_Complete (0, 0, 0);

      board_delay_ms (330);           // pause 1/3 second between passes

          // Request device 0 to go position 6400 */
      MotorLib_Goto_Position (0, 6400);

          // Wait for the motor of device 0 to end moving */
//    BSP_MotorControl_WaitWhileActive (0);
      MotorLib_Wait_Complete (0, 0, 0);

      board_delay_ms (330);           // pause 1/3 second between passes
    }

      //---------------------------------------------------------------
      // To allow PLC to control the Stepper Index and DIR signals,
      // shutoff on-board PWM, and set PWM and DIR pins to GPIO Inputs
      //---------------------------------------------------------------
   L6474_Board_PwmStop (0);    // turn off the PWM

#if defined(PLC_USAGE)
      // re-Configure L6474 - STEP pin for INPUT from PLC
    Local_GPIO_InitStruct.Pin       = BSP_MOTOR_CONTROL_BOARD_PWM_1_PIN;
    Local_GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
    Local_GPIO_InitStruct.Pull      = GPIO_NOPULL;
    Local_GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    Local_GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_1_PORT, &Local_GPIO_InitStruct);

      // re-Configure L6474 - DIR pin for INPUT from PLC
   Local_GPIO_InitStruct.Pin   = BSP_MOTOR_CONTROL_BOARD_DIR_1_PIN;
   Local_GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
   Local_GPIO_InitStruct.Pull  = GPIO_PULLUP;
   Local_GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
   HAL_GPIO_Init (BSP_MOTOR_CONTROL_BOARD_DIR_1_PORT, &Local_GPIO_InitStruct);
#endif

}


//******************************************************************************
//                               Main program
//******************************************************************************

int  main (void)
{

// HAL_Init();                  // STM32xx HAL library initialization
// SystemClock_Config();        //  Configure the system clock
   board_init (0, 0);           //  Configure system clock using default parms

      //--------------------------------------
      //    Setup on-board LED and Button
      //--------------------------------------
// BSP_LED_Init (LED2);         // PA5 on F4   -- THIS CONFLICTS WITH SPI CLK ON PA5 !!!
// BSP_LED_On (LED2);
// BSP_LED_Off (LED2;
// BSP_LED_Toggle (LED2);

// BSP_PB_Init (BUTTON_KEY, BUTTON_MODE_EXTI);  // interrupt  PC13/EXTI15_10_IRQn on F4 - not hooked up
   BSP_PB_Init (BUTTON_KEY, BUTTON_MODE_GPIO);  // polled     PC13 on F4
   BSP_PB_GetState (BUTTON_KEY);

      //------------------------------------------------------------------------
      //       Set the L6474 library to use 1 device
      //------------------------------------------------------------------------
// BSP_MotorControl_SetNbDevices (BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);

      //------------------------------------------------------------------------
      // When BSP_MotorControl_Init is called with NULL pointer,
      // the L6474 registers and parameters are set with the predefined values
      // from file l6474_target_config.h, otherwise the registers are set using
      // the L6474_Init_t pointer structure
      // The first call to BSP_MotorControl_Init initializes the first device
      // whose Id is 0.
      // The nth call to BSP_MotorControl_Init initializes the nth device
      // whose Id is n-1.
      //
      // Uncomment the call to BSP_MotorControl_Init below to initialize the
      // device with the structure gL6474InitParams declared in the the main.c
      // file and comment the subsequent call having the NULL pointer
      //
      // Uses SPI1 on F4
      //------------------------------------------------------------------------
///BSP_MotorControl_Init (BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);
// motor_lib_init (MCU_SPEED);
   MotorLib_Init_Motor (0, PWM_SPEED, 2);
// BSP_MotorControl_Init (BSP_MOTOR_CONTROL_BOARD_ID_L6474, NULL);

      // Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt
// TBD   BSP_MotorControl_AttachFlagInterrupt (MyFlagInterruptHandler);

      // Attach the function Error_Handler (defined below) to the error Handler
// TBD   BSP_MotorControl_AttachErrorHandler (Error_Handler);

#if defined(PLC_USAGE)
      //---------------------------------------------------------------
      // To allow PLC to control the Stepper Index and DIR signals,
      // shutoff on-board PWM, and set PWM and DIR pins to GPIO Inputs.
      // Then wait for PLC to start toggling the Stepper via is Index pulses
      //---------------------------------------------------------------
   L6474_Board_PwmStop (0);      // turn off the PWM

      // re-Configure L6474 - STEP pin for INPUT from PLC
    Local_GPIO_InitStruct.Pin       = BSP_MOTOR_CONTROL_BOARD_PWM_1_PIN;
    Local_GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
    Local_GPIO_InitStruct.Pull      = GPIO_NOPULL;
    Local_GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    Local_GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_1_PORT, &Local_GPIO_InitStruct);

      // re-Configure L6474 - DIR pin for INPUT from PLC
   Local_GPIO_InitStruct.Pin   = BSP_MOTOR_CONTROL_BOARD_DIR_1_PIN;
   Local_GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
   Local_GPIO_InitStruct.Pull  = GPIO_PULLUP;
   Local_GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
   HAL_GPIO_Init (BSP_MOTOR_CONTROL_BOARD_DIR_1_PORT, &Local_GPIO_InitStruct);

// BSP_LED_On (LED2);            // PA5 SPI interference - turn ON LED to show Drive is enabled for PLC
#endif

      //-------------------------------------------------
      //                main()  while  loop
      //
      // By default, we wait for PLC to run the Indexer.
      //-------------------------------------------------
   while (1)
     {
#if defined(PLC_USAGE)
       if (BSP_PB_GetState(BUTTON_KEY) == 0)            // button is Active LOW
          {     // button was pressed. Perform one sequence of standalone mode.
///         BSP_LED_Off (LED2);    // PA5 SPI interference turn OFF LED to show Drive is OFFLINE
            standalone_stepper_moves();
///         BSP_LED_On (LED2);     // turn ON LED to show Drive is enabled for PLC
          }
#endif

       standalone_stepper_moves();   // 08/13/16 - skips all the above
     }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
