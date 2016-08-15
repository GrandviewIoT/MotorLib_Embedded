/*******************************************************************************
*
* Filename:             getvalue.h
* Owner :               Rod Wang (rowa@cypress.com)
*
* Version:              V1.0 
* Description:          This file contains the function prototypes and constants used
*                       in the external inputs reading.  
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
#ifndef _GETVALUE_H_
#define _GETVALUE_H_
    
#include <cytypes.h>

#define  MAX_CURR_LOW         50        /* 1.0A */
#define  MAX_CURR_MEDIUM      75        /* 1.5A */
#define  MAX_CURR_HIGH        100       /* 2.08A */

#define LVTHRESHOLD  40                 /* 16V */
#define HVTHRESHOLD  90                /* 36V */  
	
#define LVCNTTHRES  2560                 
#define HVCNTTHRES  2560                

/* ADC channel number for speed reference potentiometer */     
#define SPEED_REF_CHAN                      ((uint8)(1))     
/* ADC result range */    
#define ADC_RANGE                           ((uint16)(4096))      
/* define the maximum resistor value for potentiometer */    
#define SPEED_POT_VALUE                     (uint16)(10000)    
/* define the accepted maximum/minimum value for the sample result of speed reference potentiometer,
   minus 100 to reduce the offset at the boundary*/    
#define MAX_SPEED_REF_ADC_VALUE             (uint16)((((SPEED_POT_VALUE + 330) * ADC_RANGE)/(SPEED_POT_VALUE + 330 + 330)) - 30)    
#define MIN_SPEED_REF_ADC_VALUE             (uint16)((((330) * ADC_RANGE)/(SPEED_POT_VALUE + 330 + 330)) + 30)        

void VoltageCheck(void);
uint16 ReadRpmRef(void);

#endif  /* _GETVALUE_H_ */

