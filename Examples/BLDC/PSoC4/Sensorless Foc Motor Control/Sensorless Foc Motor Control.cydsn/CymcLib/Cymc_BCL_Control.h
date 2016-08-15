/*******************************************************************************
*
* Filename:             Cymc_BCL_Control.h
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
#ifndef __CYMC_BCL_CONTROL_H__
#define __CYMC_BCL_CONTROL_H__
#include <cytypes.h>	
#include "CymcLib\Cymc_GFL_Math.h"
	
/* struct definition for speed estimator*/
typedef struct 
{
   q15_t curTheta;  	        /* Current Electrical angle			*/ 
   q15_t oldTheta;   		    /* Old Electrical angle				*/
   q15_t estimatedSpeed;        /* Estimated speed in per-unit		*/
   q15_t baseRpm;     		    /* Base speed in rpm				*/
   q15_t intervalConst;       	/* Constant for differentiator  	*/
   q15_t filterConst;     		/* Constant for low-pass filter		*/
   q15_t estimatedSpeedRpm;     /* Estimated speed in rpm  		    */
} SpeedEstimator;  	
/* struct definition for postion angle generator*/	
typedef struct 
{ 
	q15_t  speed; 				/* given speed  	      			*/
	q15_t  stepAngle;			/* step angle in rated speed        */	
 	q15_t  angle;				/* Step angle 				  		*/	  			 
} AngleGenerator;
/* struct definition for speed ramp up*/	
typedef struct 
{ 
	q15_t    RefSpeed; 		 	/* reference speed 					*/
	q15_t 	 delayPrescaler; 	/* delay  Prescaler     			*/
	q15_t    stepSpeed;			/* step speed                       */
	q15_t    output;		 	/* output speed						*/		 
} RampUp;

/* ADC offset for U, V phase current*/	
extern q15_t adcOffsetU, adcOffsetV;

extern void Cymc_BCL_SpeedCal(SpeedEstimator *speedEst);
extern void Cymc_BCL_AngleGenerator(AngleGenerator *angleGen);
extern void Cymc_BCL_RampUp(RampUp *rampUp);
extern void Cymc_BCL_GetADCOffset(uint16 adcU, uint16 adcV);

#endif	/* __CYMC_BCL_CONTROL_H__ */
/* [] END OF FILE */
