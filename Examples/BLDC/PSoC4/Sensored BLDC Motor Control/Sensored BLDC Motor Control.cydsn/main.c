/*******************************************************************************
*
* Filename:             main.c
* Owner :               Rod Wang (rowa@cypress.com)
*
* Version:              V1.0 
* Description:          BLDC Motor Control. 
*                       Support 3 phase BLDC motor control with Hall sensor Input. 
*                       Speed is adjustable.
* -------------------------------------------------------------------------------
* ChangeList: 
*   V1.0                Initial version
* -------------------------------------------------------------------------------
* Known issues:         
*   V1.0                N/A
* -------------------------------------------------------------------------------
* Hardare Dependency:   
*   1. CY8CKIT-042(MCU board) + CY8CKIT-037(Driver Board)  
* -------------------------------------------------------------------------------
* Related documents:
*   N/A
* -------------------------------------------------------------------------------
* Code Tested with:     PSoC Creator  3.2 (3.2.0.724)
*                       ARM GCC 4.8.4;
*
********************************************************************************
* Copyright (2014), Cypress Semiconductor Corporation.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is 
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable 
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress Source Code and derivative works for the sole purpose of creating 
* custom software in support of licensee product to be used only in conjunction 
* with a Cypress integrated circuit as specified in the applicable agreement. 
* Any reproduction, modification, translation, compilation, or representation of 
* this software except as specified above is prohibited without the express 
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH 
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the 
* materials described herein. Cypress does not assume any liability arising out 
* of the application or use of any product or circuit described herein. Cypress 
* does not authorize its products for use as critical components in life-support 
* systems where a malfunction or failure may reasonably be expected to result in 
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of 
* such use and in doing so indemnifies Cypress against all charges. Use may be 
* limited by and subject to the applicable Cypress software license agreement. 
*******************************************************************************/

#include <project.h>                 // Cypress top level project definitions
#include "motor.h"                   // Cypress defines for Motor, UI, and ISRs
#include "MotorLib_Api.h"            // pull in MotorLib common definitions
#include "Board_Support_Api.h"       // pull in defs for Board Support
#include "speed.h"
#include "userinterface.h"
#include "getvalue.h"


void  main ()
{  
    uint8   HallReader  = 0;
    uint16  pwmCntLocal = 0;
    uint32  tmp1;
    uint32  tmp2;

    board_disable_global_interrupts(); // disable global interrupts

    board_init (MCU_SPEED, 0);

       //--------------------------------------------------------------------
       // initialize system hardware components (PWMs, ADCs).
       //
       // ultimately this will go into LL_BLDC code via MotorLib_Init_Motor()
       //--------------------------------------------------------------------
    Init_HW();                         // located in motor.c 

       //--------------------------------
       // initialize parameters in UI FW
       //--------------------------------
    Init_UI_FW();                      // located in motor.c

    board_enable_global_interrupts();  // enable global interrupts

       //-------------------------------------------------------------
       // initialize UI hardware components: USER BUTTON, Motor Status.
       //
       // need to come up with common apporach and names for:
       //    USER BUTTON, SPEED ADC, DIRECTION SWITCH.
       // Even within Cypress's example code they are all different
       // despite the fact it is the same pins/controls in each case.
       // This will make porting to other boards (PSoC_4_BLE, L-Series)
       // a major mess.
       //-------------------------------------------------------------
    Init_UI_HW();                      // located in motor.c

       /* set up "Motor Stopped" as default at startup */
    UI_Cmd.run =  FALSE;

       //--------------------------------------------------------
       // Set initial Motor Direction
       //   CLOCK_WISE = 0x01, and COUNTER_CLOCK_WISE = 0x00
       //
       // ultimately this will go into LL_BLDC code via MotorLib_Set_Direction()
       //--------------------------------------------------------
    CtrlReg_Dir_Write (UI_Data.Dir);

    while (1)
      {
            // Scan sensors and handle events
        ButtonProcess();

            // Measure and check Vbus voltage
        VoltageCheck();                // located on getvalue.c

            //--------------------------------------
            // Stop motor by disabling PWM output
            //--------------------------------------
        if (UI_Cmd.run == FALSE)
           {
                // Turn off LED when motor is stopping
             STATUS_LED_Write (1);
        
             CtrlReg_PWMOut_Write (0x00);    // shutoff PWMs
             PWM_Drive_WriteCompare (PWM_Drive_ReadPeriod() >> 16);
            
             piOut = INIT_PIOUT;
             HallReader = TRUE;
           } 

            //----------------------------------------------------------
            // Error Protection: Disable PWM and then toggle LED at 1Hz
            //----------------------------------------------------------
        if (errorState != 0)
           {
             CtrlReg_PWMOut_Write (0x00);
            
             STATUS_LED_Write (~(STATUS_LED_Read()));
             board_delay_ms (500);
             STATUS_LED_Write (~(STATUS_LED_Read()));
             board_delay_ms (500);
           }        

            //--------------------------------------
            //           Motor Running
            //--------------------------------------
        if (UI_Cmd.run == TRUE && errorState == 0)
           {
                /* Turn on LED when motor is running */
             STATUS_LED_Write (0);

                /* only send out UART data when motor is running */
             BCPPoll();

             CtrlReg_PWMOut_Write (0x03);  // ensure PWM output is enabled

             pwmCntLocal = pwmCnt;

                /* Update every 12.5 ms */
             if ((pwmCntLocal & 0xff) == 0xff) 
               {
                 pwmCntLocal++;

                     // Calculate motor speed reference, unit is RPM
                 UI_Data.speedRpmRef = ReadRpmRef();   

                 speedRef = (60 * FREQ_CAPTURE_CLK) / (uint32)(MOTOR_POLE_PAIRS * UI_Data.speedRpmRef);

                 if (UI_Cmd.run == TRUE)
                    {
                      SpeedPID();   // Do closed loop Speed control
                    }     

                    // Check for any detected Hall Sensor errors
                 HallReader  = Hall_Error_Read();
                 if (HallReader == TRUE)
                    {               
                      errorState = hallError;
                      UpdateStatusError();
                    }            
               }
                /* Calculate the real time motor speed every 2000 PWM period, unit: RPM*/
            if (pwmCntLocal >= 2000) 
               {
                 pwmCntLocal = 0;
                 tmp1 = 0;
                 tmp2 = 0;
                 tmp1 = (60 * FREQ_CAPTURE_CLK);
                 tmp2 = (MOTOR_POLE_PAIRS * speedCur);
                 UI_Data.speedRpm = (tmp1 / tmp2);
               }

            pwmCnt = pwmCntLocal;
        }
    }
}

/* [] END OF FILE */
