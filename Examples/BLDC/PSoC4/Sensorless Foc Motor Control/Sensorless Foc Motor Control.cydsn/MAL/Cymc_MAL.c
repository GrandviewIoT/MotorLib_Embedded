/*******************************************************************************
*
* Filename:             Cymc_MAL.c
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
#include "MAL\Cymc_MAL.h"
#include "HAL\Cymc_HAL_PWM.h"
#include "Cymclib\Cymc_GFL_Math.h"


uint16 speedLoopPrescaler = DEFAULT_SPEEDLOOPSCALER;      /* Speed loop prescaler */
uint16  errState = 0;

MotorController motor;   /* motor control struct */


RampUp         rampUp;       /* motor control struct */
AngleGenerator angleGen;     /* angle generator struct */
ClarkTrans     clarkTrans;   /* clark transformation struct */
ParkTrans      parkTrans;    /* park transformation struct */
InvParkTrans   invParkTrans; /* inverse park transformation struct */
PIController   pidSpeed;     /* speed PID controller */
PIController   pidId;        /* Id PID controoler */
PIController   pidIq;        /* Iq PID controller */
SpeedEstimator speedEst;     /* speed estimator struct */
InvClarkTrans  invClarkTrans;/* inverse clark transformation struct */
SVPWM          svpwmCal;     /* svpwm struct */
SMO            smoEst;       /* slider mode observer struct */

/****************************************************************************************
*																			  			*
*   Function:    Cymc_MAL_MotorInit										  			    *
*																			  			*
*   Description: Init motor lib struct paramters and motor paramters		            *
*																			  			*				
*   Paramters:   None 																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
void Cymc_MAL_MotorInit()
{   
	motor.Rs = M_RS;	           /* Ω */
	motor.Ls = M_LS;              /* H */
	motor.Poles = M_POLES;
	motor.Ts = M_TS;              /* S */
	motor.baseFrequency = M_BASEFREQUENCY; /* Hz */
	motor.ratedSpeedRPM = M_RATED_SPEED;   /* RPM */
	
	/* Initialize the angle generator parameters */
	angleGen.stepAngle = _FQ(motor.baseFrequency*motor.Ts);
	
	/* Initialize the SMO parameters */
    Cymc_ACL_SMOParamtersInit(motor.Rs, motor.Ls, motor.Ts, motor.baseFrequency, &smoEst);
    smoEst.slideError = _FQ(0.5);
	
    /* Initialize the Id PID parameters */ 
	pidId.kp   	= PID_ID_KP;
    pidId.kr   	= PID_ID_KR;
	pidId.ki   	= PID_ID_KI;
    pidId.outMax = PID_ID_OMAX;
    pidId.outMin = PID_ID_OMIN;
 
	/* Initialize the Iq PID parameters */ 
	pidIq.kp   	= PID_IQ_KP;
    pidIq.kr   	= PID_IQ_KR;
	pidIq.ki   	= PID_IQ_KI;
    pidIq.outMax = PID_IQ_OMAX;
    pidIq.outMin = PID_IQ_OMIN;
    
	/* Initialize the Speed PID parameters */
    pidSpeed.kp   = PID_SPEED_KP;
    pidSpeed.kr   = PID_SPEED_KR;
	pidSpeed.ki   = PID_SPEED_KI;
    pidSpeed.outMax = PID_SPEED_OMAX;
    pidSpeed.outMin = PID_SPEED_OMIN;
	
	/* Initialize the Speed Estimator parameters */    
    speedEst.intervalConst = _FQ(1/(motor.baseFrequency*motor.Ts));
    speedEst.filterConst = _FQ(1/(1 + motor.Ts*2*PI*5));  
    speedEst.baseRpm = 120*(motor.baseFrequency/motor.Poles);
	
	/* Initialize the SVPWM module parameters */
	svpwmCal.Period = pwmPeriod;
}
/****************************************************************************************
*																			  			*
*   Function:    Cymc_MAL_MotorStart										  			*
*																			  			*
*   Description: Init motor startup paramters and enable PWM output  		            *
*																			  			*				
*   Paramters:   None 																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
void Cymc_MAL_MotorStart()
{
    
	motor.speedRef  = _FQ(0.16);
    motor.vqRef = _FQ(0.13);			 
    motor.vdRef = _FQ(0);
	rampUp.output = 0;
	rampUp.stepSpeed = 5;
	rampUp.delayPrescaler = DEFAULT_RAMPUP_DELAY;
	motor.vqRef = (rampUp.output>>1) + _FQ(0.06);
    Cymc_MAL_MotorInit();
	motor.runState = M_OPENLOOP;
	Cymc_HAL_PWMOutputEnable();
}
/****************************************************************************************
*																			  			*
*   Function:    Cymc_MAL_MotorStop										  				*
*																			  			*
*   Description: Sisable PWM output and stop motor 		            					*
*																			  			*				
*   Paramters:   None 																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
void Cymc_MAL_MotorStop()
{
	motor.runState = M_STOP;
	Cymc_HAL_PWMOutputDisable();
}

/* [] END OF FILE */
