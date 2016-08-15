/*******************************************************************************
*
* Filename:             Cymc_HAL_ADC.c
* Owner :               Luffy Li (lufl@cypress.com)
*
* Version:              V1.0 
* Description:          Sensorless Foc Motor Control
*                       Support 2-shunt sensorless FOC 
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
#include "Cymc.h"

	

#define __SADC_START(name)       				name##_Start
#define _SADC_START(name)        				__SADC_START(name)
#define SADC_Start()            				_SADC_START(ADC_NAME)()
	
#define __ADC_GetResult16(name)       			name##_GetResult16
#define _ADC_GetResult16(name)        			__ADC_GetResult16(name)
#define SADC_GetResult16(value)            		_ADC_GetResult16(ADC_NAME)(value)
	

	
/*ADC Sample Values*/
uint16 adcRawData[CHANNEL_NUM];


/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_ADCStart											  *
*																			  *
*   Description: Init SAR ADC Module					  					  *
*																			  *				
*   Paramters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_ADCStart()
{
    /* start SARADC	*/
    SADC_Start();    
}
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_ADCReadSample										  *
*																			  *
*   Description: Read ADC Sample Values and Put into wADCRawData			  *
*																			  *				
*   Paramters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_ADCReadSample()
{
    uint8 i = 0;
    for(i = 0; i < CHANNEL_NUM; i++)
    {
        adcRawData[i] = SADC_GetResult16(i) & (0x0FFF);
    }
}

/* [] END OF FILE */
