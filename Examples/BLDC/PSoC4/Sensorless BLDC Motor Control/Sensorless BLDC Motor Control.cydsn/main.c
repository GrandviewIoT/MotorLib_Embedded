/*******************************************************************************
*
* Filename:             main.c
* Owner :               Bob Hu (bobh@cypress.com)
*
* Version:              V1.0 
* Description:          Sensorless BLDC Motor Control
*                       
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
#include "BLDCController.h"          // Cypress defines for Motor, UI, and ISRs
#include "Protection.h"
#include "userinterface.h"
#include "Debug.h"
#include "MotorLib_Api.h"            // pull in MotorLib common definitions
#include "Board_Support_Api.h"       // pull in defs for Board Support

/*******************************************************************************
* Function Name: Main
********************************************************************************
* Summary:
* Main function
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void  main ()
{   
    board_disable_global_interrupts(); // disable global interrupts

    board_init (MCU_SPEED, 0);
     
    BLDC_ParameterInit();              // initialize BLDC parameters

        //---------------------------------------------------------------
        // initialize BLDC Motor Controller, default state is motor stop
        //---------------------------------------------------------------
    BLDC_ControllerInit();

    ButtonInit();                      // initialize button processing

    ADC_Start();                       // Start ADC
    
        // initialize status LED
#if (DEBUG_ZC_DETECTION_OUTPUT_ENABLE)   
        // enable Status_LED pin output zero-crossing detection signal
    CtrlReg_Debug_Control &= ~(DEBUG_STATUS_LED_SEL_MASK);
#else    
        // set Status LED off
    CtrlReg_Debug_Control |= (DEBUG_STATUS_LED_MASK);
    CtrlReg_Debug_Control |= (DEBUG_STATUS_LED_SEL_MASK);
#endif  // DEBUG_ZC_DETECTION_OUTPUT_ENABLE

#if (DEBUG_GENERAL_TP_OUTPUT_ENABLE)
    CtrlReg_Debug_Control &= ~(DEBUG_GENERAL_TP_MASK);
#endif
    
        /* UART initialization for BCP communication */ 
    UART_BCP_Start();   
 
    board_enable_global_interrupts();  // enable global interrupts
    
    /***************************************************************************
    * Main Loop
    ****************************************************************************/    
    for (;;)
      {   
            /* firmware trigger to start ADC convertion */
        ADC_StartConvert();

            /* check if conversion is completed */
        ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);

            /* read speed reference from potentiometer */
        ReadSpeedRef();

            /* process any button press event */
        ButtonProcess();

            // check for any over voltage and under voltage errors
        VoltageCheck();

            //-------------------------------------------------
            // run BLDC Motor based on FSM/control flag status
            //-------------------------------------------------
        BLDC_Run();

            /* UART Comm to display speed on Cypress Bridge Control Panel */   
        if (BLDC_Control.runFlag == TRUE)
           BCPPoll();
      }
}

/* [] END OF FILE */
