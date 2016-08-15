/*******************************************************************************
*
* Filename:             Cymc_BCL_Transform.h
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
#ifndef __CYMC_BCL_TRANSFORM_H__
#define __CYMC_BCL_TRANSFORM_H__
#include "CymcLib\Cymc_GFL_Math.h"
	
/* struct definition for clark transformation*/
typedef struct 
{  
	q15_t  inU;  		/* phase-U variable    	*/
	q15_t  inV;			/* phase-V variable    	*/
	q15_t  outAlpha;	/* Output: α-axis variable    	*/
	q15_t  outBeta;		/* Output: β-axis variable    	*/
} ClarkTrans;
/* struct definition for inverse clark transformation*/	
typedef struct 
{  
	q15_t  outU;  		/* Output: phase-U variable	  	*/
	q15_t  outV;		/* Output: phase-V variable	  	*/
	q15_t  outW;		/* Output: phase-W variable	  	*/
	q15_t  InAlpha;	    /* α-axis  variable   	*/
	q15_t  InBeta;		/* β-axis  variable	  	*/
} InvClarkTrans;
/* struct definition for park transformation*/		
typedef struct 
{  
	q15_t  inAlpha;  	/* α-axis  variable 	*/
	q15_t  inBeta;	 	/* β-axis  variable 	*/
	q15_t  outD;		/* Output: d-axis  variable 	*/
	q15_t  outQ;		/* Output: q-axis  variable		*/
	q15_t  inSin;		/* angle sin  value 	*/
	q15_t  inCos; 		/* angle cos  value 	*/	 
} ParkTrans;
/* struct definition for inverse park transformation*/
typedef struct 
{  
	q15_t  outAlpha;  	/* α-axis  variable     */
	q15_t  outBeta;	 	/* β-axis  variable 	*/
	q15_t  inD;		    /* Output: d-axis  variable 	*/
	q15_t  inQ;		    /* Output: q-axis  variable		*/
	q15_t  inSin;		/* angle sin  value 	*/
	q15_t  inCos; 	 	/* angle cos  value 	*/	
} InvParkTrans;

extern void Cymc_BCL_Clark(ClarkTrans *clark);
extern void Cymc_BCL_Park(ParkTrans *park);
extern void Cymc_BCL_InvPark(InvParkTrans *invPark);
extern void Cymc_BCL_InvClark(InvClarkTrans *invClark);

#endif  /* __CYMC_BCL_TRANSFORM_H__ */
/* [] END OF FILE */
