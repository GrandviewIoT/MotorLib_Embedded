/*******************************************************************************
*
* Filename:             Cymc_GFL_PID.h
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
#ifndef __CYMC_GFL_PID_H__
#define __CYMC_GFL_PID_H__

#include "CymcLib\Cymc_GFL_Math.h"

typedef struct 
{  
	q15_t  ref;   			/* reference set-point	*/
	q15_t  fbk;   			/* feedback				*/
	q15_t  out;   			/* Output: controller output 	*/
	q15_t  kr;				/* reference set-point weighting	*/ 
	q15_t  kp;				/* proportional loop gain		*/
	q15_t  ki;			    /* integral gain					*/
	q15_t  outMax;			/* upper output limit		*/
	q15_t  outMin;			/* lower output limit		*/
	q15_t  pOut;				/* proportional term					*/
	q15_t  iOut;				/* integral term						*/
	q15_t  outPre;			/* pre-saturated controller output	*/
} PIController;


extern void Cymc_GFL_PICal(PIController *v);
#endif  /* __CYMC_GFL_PID_H__ */
/* [] END OF FILE */
