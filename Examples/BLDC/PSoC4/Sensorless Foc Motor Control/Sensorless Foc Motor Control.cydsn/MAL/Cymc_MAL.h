/*******************************************************************************
*
* Filename:             Cymc_MAL.h
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
#ifndef __CYMC_MAL_H__
#define __CYMC_MAL_H__
#include <cytypes.h>	
#include "CymcLib\CymcLib.h"
    
/*-----------------------------------------------------------------------*/  
#define PI              3.14159265358979
#define SQRT_3          1.732
    
/*-----------------------------------------------------------------------*/    
#define M_RS            0.8
#define M_LS            0.0012
#define M_POLES         8	
#define M_TS            0.0001
#define M_RATED_SPEED   4000u 
#define M_BASEFREQUENCY 266.67
    
/*-----------------------------------------------------------------------*/      
#define DC_BUS                  (uint16)(24)                /* V        */
#define VDD                             (3300)              /* mV       */
#define SENSING_RESISTOR        (uint16)(30)                /* mOhm     */
#define OPAMP_GAIN              (uint16)(10000/2400)        /* 10K/2.4K */
#define DC_BUS_SCALER           (uint16)((1000+19100)/1000)    /* resistor network, (R9+R10)/R10 */
#define MAX_PHASE_CURRENT       (uint16)(5)                 /* 5A       */
#define ADC_RESOLUTION                  (4096)     
    
/*-----------------------------------------------------------------------*/      
#define M_STOP	  		0
#define M_OPENLOOP   	1
#define M_CURRENTLOOP   2
#define M_SPEEDLOOP     3	

/*-----------------------------------------------------------------------*/      
#define SPEED_MIN_STEP  _FQ(0.0000610)	
 
/*-----------------------------------------------------------------------*/  
#define ERR_OVER_CURRENT     (1<<0)
#define ERR_OVER_VOLTAGE     (1<<1)
#define ERR_UNDER_VOLTAGE    (1<<2)	
#define ERR_START_FAILED     (1<<3)
#define ERR_LOSE_STEP        (1<<4)	
	
/*-----------------------------------------------------------------------*/  
#define PID_ID_KP       _FQ(1.0)
#define PID_ID_KR       _FQ(1.0)
#define PID_ID_KI       _FQ(0.2)
#define PID_ID_OMAX     _FQ(0.7)
#define PID_ID_OMIN     _FQ(-0.7)	

/*-----------------------------------------------------------------------*/      
#define PID_IQ_KP       _FQ(1.0)
#define PID_IQ_KR       _FQ(1.0)
#define PID_IQ_KI       _FQ(0.2)
#define PID_IQ_OMAX     _FQ(0.95)
#define PID_IQ_OMIN     _FQ(-0.95)

/*-----------------------------------------------------------------------*/      
#define PID_SPEED_KP    _FQ(0.8333)
#define PID_SPEED_KR    _FQ(1.0)
#define PID_SPEED_KI    _FQ(0.0033264)
#define PID_SPEED_OMAX  _FQ(0.55)
#define PID_SPEED_OMIN  _FQ(-0.55)	

/*-----------------------------------------------------------------------*/      
#define DEFAULT_SPEEDLOOPSCALER	10
#define DEFAULT_RAMPUP_DELAY    50
#define SLOW_RAMPUP_DELAY       50	
#define MIDDLE_RAMPUP_DELAY     50
#define FAST_RAMPUP_DELAY       50	

/*-----------------------------------------------------------------------*/      
#define SPEED_RPM_MAX   M_RATED_SPEED                       /* rated speed */
#define SPEED_RPM_MIN   500u                                /* 500 RPM */  
#define SPEEDREF_MAX    _FQ((float)SPEED_RPM_MAX/M_RATED_SPEED)
#define SPEEDREF_MIN    _FQ((float)SPEED_RPM_MIN/M_RATED_SPEED)
  
/* struct definition for motor controller*/
typedef struct 
{
   float Rs;  	         /* motor Rs 			*/ 
   float Ls;   		     /* motor Ls			*/
   uint8 Poles;          /* motor Poles			*/
   float baseFrequency;  /* base frequency		*/
   float Ts;  	         /* PWM Period			*/
   int16 ratedSpeedRPM; /* rated Speed in RPM Format */
   int16 speedRPM;      /* Speed in RPM Format */
   uint8 runState;	     /* Motor run state  	*/
   q15_t speedRef; 		 /* speed reference		*/
   q15_t vdRef; 		 /* vd reference		*/
   q15_t vqRef; 		 /* vq reference		*/
} MotorController; 
	
extern MotorController motor;	
extern RampUp         rampUp;
extern AngleGenerator angleGen;
extern ClarkTrans     clarkTrans;
extern ParkTrans      parkTrans;
extern InvParkTrans   invParkTrans;
extern PIController   pidSpeed;
extern PIController   pidId;
extern PIController   pidIq;
extern SpeedEstimator speedEst;
extern InvClarkTrans  invClarkTrans;
extern SVPWM          svpwmCal;
extern SMO            smoEst;
extern uint16 speedLoopPrescaler;
extern uint16  errState;
	
extern void Cymc_MAL_MotorInit(void);	
extern void Cymc_MAL_MotorStart(void);
extern void Cymc_MAL_MotorStop(void);


#endif	
/* [] END OF FILE */
