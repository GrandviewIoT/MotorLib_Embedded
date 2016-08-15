/*******************************************************************************
*
* Filename:             getvalue.c
* Owner :               Rod Wang (rowa@cypress.com)
*
* Version:              V1.0 
* Description:          This file samples bus voltage and potentimeter voltage  
*                       from SARADC. 
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
#include "speed.h"
#include "motor.h"
#include "getvalue.h"
#include "userinterface.h"


/*******************************************************************************
* Function Name:  VoltageCheck                           (ReadVolt)
********************************************************************************
* Summary:
* The VoltageCheck function samples Vbus to protect board from low-voltage 
* and high voltage. 
*
* Parameters: None 
*
* Return:     None
*******************************************************************************/

void  VoltageCheck (void)
{
    static  uint16  highVoltCounter = 0;
    static  uint16  lowVoltCounter  = 0;
            uint16  adcResult       = 0;    

// in future, convert the following to a MotorLib_Get_Vbus_Voltage() call

       /* Read ADC, and discard unwanted MSB bits. 2011,12,29 */
    adcResult = (ADC_SAR_Seq_1_GetResult16(0) & 0x00ff);

       /* check if Over voltage or under voltage */
    if (adcResult < LVTHRESHOLD)
       {
         lowVoltCounter++;                 /* Record low voltage count */
       }
      else if (adcResult > HVTHRESHOLD)
       {
         highVoltCounter++;                /* Record high voltage count */
       }       
      else
       {    // Eliminate pulse high or low voltage count
         if ((lowVoltCounter > 0) || (highVoltCounter > 0))
            {
              lowVoltCounter--;
              highVoltCounter--;
            }
       }
    
       /* If we are more than voltage error time threshold, then stop motor */
    if (lowVoltCounter > LVCNTTHRES)
       {       
         errorState = lowVolt;
         UpdateStatusError();        
       }
    if (highVoltCounter > HVCNTTHRES)
       {
         errorState = highVolt;
         UpdateStatusError();
       }
}


/*******************************************************************************
* Function Name:  ReadRpmRef
********************************************************************************
* Summary:
* The ReadRpmRef function read speed reference value from potentimeter. 
*
* Parameters: None  
*
* Return:     uint16  
*******************************************************************************/
uint16  ReadRpmRef (void)
{
    uint16  speedSampleCur; 
    uint16  speedRange;
    uint16  speedGiven = 0;

// in future, convert the following to a MotorLib_Get_Speed_Reference_Value() call

       /* Read ADC, and discard unwanted MSB bits */
    speedSampleCur = (ADC_SAR_Seq_1_GetResult16(SPEED_REF_CHAN) & 0x0FFF); 

    if (speedSampleCur > MAX_SPEED_REF_ADC_VALUE)
        speedSampleCur = MAX_SPEED_REF_ADC_VALUE;
    if (speedSampleCur < MIN_SPEED_REF_ADC_VALUE)
        speedSampleCur = MIN_SPEED_REF_ADC_VALUE;
    
    speedSampleCur -= MIN_SPEED_REF_ADC_VALUE;

    speedRange = UI_Data.maxSpeedRpm - UI_Data.minSpeedRpm;
    speedGiven = UI_Data.minSpeedRpm + ((uint32) (speedSampleCur * speedRange) / 
                                               (MAX_SPEED_REF_ADC_VALUE - MIN_SPEED_REF_ADC_VALUE));
    
    return (speedGiven);
}

/* [] END OF FILE */
