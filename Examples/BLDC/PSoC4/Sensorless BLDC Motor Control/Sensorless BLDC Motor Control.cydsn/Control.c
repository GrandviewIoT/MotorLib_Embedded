/*******************************************************************************
*
* Filename:             Control.c
* Owner :               Bob Hu (bobh@cypress.com)
*
* Version:              V1.0 
* Description:          Sensorless BLDC Motor Control
*                       This file contains the PID control for speed
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
#include <project.h>
#include "Control.h"
#include "BLDCController.h"

/******************************************************************************
 * Global variables definition
 * ----------------------------------------------------------------------------
 * These varialbes should be populated to other modules. Header file contain 
 * the extern statement for these variables.
 ******************************************************************************/ 

/******************************************************************************
 * Local Macro definition
 * ----------------------------------------------------------------------------
 * These Macro are only used in this module. These Macro should not be populated
 * to other modules.
 ******************************************************************************/

/******************************************************************************
 * Local variables definition
 * ----------------------------------------------------------------------------
 * These varialbes are only used in this module. These varialbes should not be 
 * populated to other modules.
 ******************************************************************************/

/*******************************************************************************
* Function Name: SpeedPID
********************************************************************************
* Summary:
* The SpeedPID function implements PID regulator for clos loop control.
* voltage. 
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void SpeedPID(void)
{
    static int32 preSpeedErr = 0;
    
    int32   curSpeedErr;
    int32   speedErrDelta;
    int32   pidOutput;
    uint16  switchSpeed;    
    uint32  kp;
    uint32  ki;


    curSpeedErr = speedCurCount - speedRefCount;  // compute error amount
    speedErrDelta = 0;
    pidOutput   = 0;    // reset PID value for this new pass

       /* change Ki and Kp values based upon  high or low speed */
    switchSpeed = 0; 
    switchSpeed = SECOND_PER_MINUTE / BLDC_Config.polePairNumber;
    switchSpeed = switchSpeed * SPEED_MEASURE_CLOCK_FREQ / SWITCH_SPEED_RPM;    

       /* adjust Kp and Ki based on speed */
    kp = 0;
    ki = 0;
    if (speedRefCount > switchSpeed) 
      {
        kp = BLDC_Config.kp;
        ki = BLDC_Config.ki;
      }
    else /* High speed, High PI gain */
      {
        kp = BLDC_Config.kp << PID_KP_SCALER;
        ki = BLDC_Config.ki << PID_KI_SCALER;
      }
    
    BLDC_Control.pidSpeedErr = curSpeedErr;

        /* Calculate output of intergration	*/
    pidOutput += (int32) curSpeedErr * (int32) ki;
    
        /* calculate delta of speed error */		
    speedErrDelta = (curSpeedErr - preSpeedErr);		
    preSpeedErr   = curSpeedErr;

	/* Calculate output of proportional */
    pidOutput += (int32) speedErrDelta * (int32) kp;
       
    if (pidOutput > (5<<PI_RANGE_SCALE))
        pidOutput = (5<<PI_RANGE_SCALE);
    if (pidOutput < -(5<<PI_RANGE_SCALE))
        pidOutput = -(5<<PI_RANGE_SCALE);
    
       /* update PWM duty cycle with PID output */	
    BLDC_Control.pidOutput += pidOutput;
    
       /* limit the PID output */
    if (BLDC_Control.pidOutput > LIMIT_MAX)
        BLDC_Control.pidOutput = LIMIT_MAX;
    if (BLDC_Control.pidOutput < LIMIT_MIN)
        BLDC_Control.pidOutput = LIMIT_MIN;
}


/*******************************************************************************
* Function Name: CalcPWMDuty
********************************************************************************
* Summary:
* The CalcPWMDuty function calculates the new duty cycle based on the
* above Speed Control's PID output
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void  CalcPWMDuty (void)
{
    int32   pidOutput;
    uint16  dutyLocal = 0;

    pidOutput = BLDC_Control.pidOutput;

       /* shift negtive PID output to an all positive range */
    pidOutput += LIMIT_HALF_RANGE;
 
    dutyLocal  = pidOutput >> PI_RANGE_SCALE;   
    
    dutyCycle  = dutyLocal;       
}

/* [] END OF FILE */
