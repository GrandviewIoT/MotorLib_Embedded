/*******************************************************************************
*
* Filename:             Cymc_BCL_SVPWM.h
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
#ifndef __CYMC_BCL_SVPWM_H__
#define __CYMC_BCL_SVPWM_H__
#include <cytypes.h>
#include "CymcLib\Cymc_GFL_Math.h"
	
/* struct definition for SVPWM Calculation*/	
typedef struct 	
{ 
	q15_t  Uu;              /* phase-u phase voltage  */
	q15_t  Uv;				/* phase-v phase voltage  */
	q15_t  Uw;				/* phase-w phase voltage  */
	q15_t  Tu;				/* phase-u on time		 */	
	q15_t  Tv;				/* phase-v on time  	 */
	q15_t  Tw;				/* phase-w on time		 */
	uint16  Period;			/* PWM Period 			 */
	uint16  CmpU;			/* Output: phase-u compare value */
	uint16  CmpV;			/* Output: phase-v compare value */
	uint16  CmpW;			/* Output: phase-w compare value */
} SVPWM;	

extern void Cymc_BCL_SVPWM(SVPWM *svm);

#endif	/* __CYMC_BCL_SVPWM_H__ */
/* [] END OF FILE */
