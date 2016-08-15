/*******************************************************************************
*
* Filename:             motor.c
* Owner :               Rod Wang (rowa@cypress.com)
*
* Version:              V1.0 
* Description:          This file contains pwm isr and functions that initialize
*                       motor and measure motor speed.
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

#include <project.h>
#include "motor.h"
#include "userinterface.h"
#include "getvalue.h"

                 /* UI_MOTOR_STATUS Status blocks */
    UI_CMD   UI_Cmd;
    UI_DATA  UI_Data;

    uint16  speedRef = 0;          // Motor speed Reference
    uint16  speedCur = 3000;       // Current motor speed. */

    uint16  preSpeedCur  = 0;
    uint16  preCntCaptur = 30000;

    uint8   dutyCycle  = 0;        // actually used as BYTE since use 8-bit pwm
    uint16  pwmCnt     = 0;
    uint8   firstRun   = 1;
    uint8   ocBlankCnt = 0;
    Error_T errorState = no_error;
    uint8   stateSys   = STATUS_STOP;


/*******************************************************************************
*                                     ISR
* Function Name: pwm_isr
********************************************************************************
*
* Summary:
* This function is PWM ISR when TC happens, it increases PWM ticker, clear flag and 
* Enable over current protection ISR after startup.
*
* Parameters: None
*
* Return:     None
*******************************************************************************/
CY_ISR(pwm_isr)
{    
    pwmCnt++;  
    
        /* Avoid the current pulse interference in motor start-up */
    if (UI_Cmd.run)
      {
        if (firstRun == 1)
           {
             ocBlankCnt++;
           }
        if (ocBlankCnt > 2)    /*Ignore the first 2 PWM period = 50uS*3=150uS, then enable over current ISR */
           {
             firstRun   = 0;
             ocBlankCnt = 0;
             isr_oc_Enable();
             isr_oc_ClearPending();
           }
      }
    
        /* Calculate the real time motor speed every 2000 PWM period*/
    if (pwmCnt >= 2001)
       {
         pwmCnt = 0;
       }

    PWM_Drive_ClearInterrupt (PWM_Drive_INTR_MASK_TC);
}


/*******************************************************************************
*                                     ISR
* Function Name: speed_measure_isr
********************************************************************************
*
* Summary:
* This function is ISR for motor speed measurement. 
* 
* Parameters: None
*
* Return:     None
*******************************************************************************/
CY_ISR(speed_measure_isr)
{
    uint16 cntCaptur = 0;
    
    cntCaptur = Counter_Spd_ReadCapture();
    
    speedCur  = preCntCaptur - cntCaptur;
    
        /* If speed is too low, Regard motor is stopped*/
     if (speedCur > 5000)        /* < 300Rpm*/
        speedCur = 5000; 
    
        /* filter for speed measured */
    speedCur = (preSpeedCur >> 2) + (preSpeedCur >> 1) + (speedCur >> 2);    
    
    preCntCaptur = cntCaptur;
    preSpeedCur  = speedCur; 
    
    Counter_Spd_ClearInterrupt (Counter_Spd_INTR_MASK_CC_MATCH);
}


/*******************************************************************************
*                                     ISR
* Function Name: over_current_isr
********************************************************************************
*
* Summary:
* This function is ISR for over current protection, updates the error state flag. 
* 
* Parameters: None
*
* Return:     None
*******************************************************************************/
CY_ISR(over_current_isr)
{    
    UpdateStatusError();        /* If over current happens, stop motor*/
    errorState = overCur;
    LPComp_OC_ClearInterrupt (LPComp_OC_INTR_RISING);
}


/*******************************************************************************
* Function Name: Init_HW
********************************************************************************
* Summary: This function initializes system hardware peripharals
*  
* Parameters: None  
*
* Return:     None
*******************************************************************************/

void  Init_HW (void)
{
        /* PWM Initialization */
    PWM_Drive_Start();

         /* Enable PWM, but disable PWM IO output */
    CtrlReg_PWMOut_Write (0x01);       
   
         /* Speed Counter Initialization */
    Counter_Spd_Start();
    
         /* For voltage sample */
    ADC_SAR_Seq_1_Start();
    
         /* For over current protection */
    IDAC_Iref_Start();
    IDAC_Iref_SetValue (0x7d);
    
    LPComp_OC_Start();
    
         /* UART Init */
    UART_BCP_Start();

    isr_pwm_Start();
    isr_pwm_StartEx (pwm_isr);
    isr_pwm_Enable();  
    
    isr_spd_Start(); 
    isr_spd_StartEx (speed_measure_isr);    
    isr_spd_Enable();     
    
    isr_oc_Start(); 
    isr_oc_StartEx (over_current_isr);    
    isr_oc_Disable();  
}


/*******************************************************************************
* Function Name: Init_UI_FW
********************************************************************************
* Summary: This function initializes parameters used in motor running.
*  
* Parameters: None  
*
* Return:     None
*******************************************************************************/

void  Init_UI_FW (void)
{
        /* Setting UI Initial parameter */
    UI_Data.Dir         = CLOCK_WISE;                
    UI_Data.maxSpeedRpm = 4000;
    UI_Data.minSpeedRpm = 500;
    UI_Data.speedRpmRef = 1000;
    UI_Data.polePairs   = 4;
    UI_Data.maxCurr     = MAX_CURR_MEDIUM; 
    UI_Data.kp          = 500;
    UI_Data.ki          = 50;
}


/*******************************************************************************
* Function Name: Init_UI_HW
********************************************************************************
* Summary: This function initializes hardware peripharals for user interface
*  
* Parameters: None  
*
* Return: None
*******************************************************************************/
void  Init_UI_HW ()
{
    UpdateStatusInit();
}

/* [] END OF FILE */
