/*******************************************************************************
*
* Filename:             Cymc_HAL_PWM.c
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

#define __PWMU_START(name)       				name##_Start
#define _PWMU_START(name)        				__PWMU_START(name)
#define PWMU_Start()            				_PWMU_START(PWMU_NAME)()
#define PWMV_Start()            				_PWMU_START(PWMV_NAME)()	
#define PWMW_Start()            				_PWMU_START(PWMW_NAME)()	
	
#define __PWMU_STOP(name)       				name##_Stop
#define _PWMU_STOP(name)        				__PWMU_STOP(name)
#define PWMU_Stop()            					_PWMU_STOP(PWMU_NAME)()
#define PWMV_Stop()            					_PWMU_STOP(PWMV_NAME)()
#define PWMW_Stop()            					_PWMU_STOP(PWMW_NAME)()	

#define __PWMU_SetPWMMode(name)       			name##_SetPWMMode
#define _PWMU_SetPWMMode(name)        			__PWMU_SetPWMMode(name)
#define PWMU_SetPWMMode(value)            		_PWMU_SetPWMMode(PWMU_NAME)(value)
#define PWMV_SetPWMMode(value)            		_PWMU_SetPWMMode(PWMV_NAME)(value)
#define PWMW_SetPWMMode(value)            		_PWMU_SetPWMMode(PWMW_NAME)(value)	

#define __PWMU_UNDERFLOW_SET(name)       		name##_UNDERFLOW_SET
#define _PWMU_UNDERFLOW_SET(name)        		__PWMU_UNDERFLOW_SET(name)
#define PWMU_UNDERFLOW_SET		         		_PWMU_UNDERFLOW_SET(PWMU_NAME)
#define PWMV_UNDERFLOW_SET		         		_PWMU_UNDERFLOW_SET(PWMV_NAME)
#define PWMW_UNDERFLOW_SET		         		_PWMU_UNDERFLOW_SET(PWMW_NAME)	
	
#define __PWMU_UNDERFLOW_CLEAR(name)       		name##_UNDERFLOW_CLEAR
#define _PWMU_UNDERFLOW_CLEAR(name)        		__PWMU_UNDERFLOW_CLEAR(name)
#define PWMU_UNDERFLOW_CLEAR		         	_PWMU_UNDERFLOW_CLEAR(PWMU_NAME)
#define PWMV_UNDERFLOW_CLEAR		         	_PWMU_UNDERFLOW_CLEAR(PWMV_NAME)
#define PWMW_UNDERFLOW_CLEAR		         	_PWMU_UNDERFLOW_CLEAR(PWMW_NAME)	

#define __PWMU_TRIG_CONTROL2_REG(name)       	name##_TRIG_CONTROL2_REG
#define _PWMU_TRIG_CONTROL2_REG(name)        	__PWMU_TRIG_CONTROL2_REG(name)
#define PWMU_TRIG_CONTROL2_REG		         	_PWMU_TRIG_CONTROL2_REG(PWMU_NAME)
#define PWMV_TRIG_CONTROL2_REG		         	_PWMU_TRIG_CONTROL2_REG(PWMV_NAME)
#define PWMW_TRIG_CONTROL2_REG		         	_PWMU_TRIG_CONTROL2_REG(PWMW_NAME)	

#define __PWMU_OVERLOW_NO_CHANGE(name)       	name##_OVERLOW_NO_CHANGE
#define _PWMU_OVERLOW_NO_CHANGE(name)       	__PWMU_OVERLOW_NO_CHANGE(name)
#define PWMU_OVERLOW_NO_CHANGE		         	_PWMU_OVERLOW_NO_CHANGE(PWMU_NAME)
#define PWMV_OVERLOW_NO_CHANGE		         	_PWMU_OVERLOW_NO_CHANGE(PWMV_NAME)
#define PWMW_OVERLOW_NO_CHANGE		         	_PWMU_OVERLOW_NO_CHANGE(PWMW_NAME)	

#define __PWMU_CC_MATCH_INVERT(name)       		name##_CC_MATCH_INVERT
#define _PWMU_CC_MATCH_INVERT(name)       		__PWMU_CC_MATCH_INVERT(name)
#define PWMU_CC_MATCH_INVERT		         	_PWMU_CC_MATCH_INVERT(PWMU_NAME)
#define PWMV_CC_MATCH_INVERT		         	_PWMU_CC_MATCH_INVERT(PWMV_NAME)
#define PWMW_CC_MATCH_INVERT		         	_PWMU_CC_MATCH_INVERT(PWMW_NAME)	
	
#define __PWMU_CMD_START(name)       			name##_CMD_START
#define _PWMU_CMD_START(name)        			__PWMU_CMD_START(name)
#define PWMU_CMD_START		         			_PWMU_CMD_START(PWMU_NAME)
#define PWMV_CMD_START		         			_PWMU_CMD_START(PWMV_NAME)
#define PWMW_CMD_START		         			_PWMU_CMD_START(PWMW_NAME)	
	
#define __PWMU_WriteCounter(name)       		name##_WriteCounter
#define _PWMU_WriteCounter(name)        		__PWMU_WriteCounter(name)
#define PWMU_WriteCounter(value)            	_PWMU_WriteCounter(PWMU_NAME)(value)
#define PWMV_WriteCounter(value)            	_PWMU_WriteCounter(PWMV_NAME)(value)
#define PWMW_WriteCounter(value)            	_PWMU_WriteCounter(PWMW_NAME)(value)	
	
#define __PWMU_WriteCompareBuf(name)       		name##_WriteCompareBuf
#define _PWMU_WriteCompareBuf(name)        		__PWMU_WriteCompareBuf(name)
#define PWMU_WriteCompareBuf(value)            	_PWMU_WriteCompareBuf(PWMU_NAME)(value)
#define PWMV_WriteCompareBuf(value)            	_PWMU_WriteCompareBuf(PWMV_NAME)(value)
#define PWMW_WriteCompareBuf(value)            	_PWMU_WriteCompareBuf(PWMW_NAME)(value)	

#define __PWMU_ReadPeriod(name)       			name##_ReadPeriod
#define _PWMU_ReadPeriod(name)        			__PWMU_ReadPeriod(name)
#define PWMU_ReadPeriod()            			_PWMU_ReadPeriod(PWMU_NAME)()
#define PWMV_ReadPeriod()            			_PWMU_ReadPeriod(PWMV_NAME)()
#define PWMW_ReadPeriod()            			_PWMU_ReadPeriod(PWMW_NAME)()	

#define __PWMU_ReadCompare(name)       			name##_ReadCompare
#define _PWMU_ReadCompare(name)        			__PWMU_ReadCompare(name)
#define PWMU_ReadCompare()            			_PWMU_ReadCompare(PWMU_NAME)()
#define PWMV_ReadCompare()            			_PWMU_ReadCompare(PWMV_NAME)()
#define PWMW_ReadCompare()            			_PWMU_ReadCompare(PWMW_NAME)()	
	
#define __PWMU_TriggerCommand(name)       		name##_TriggerCommand
#define _PWMU_TriggerCommand(name)       		__PWMU_TriggerCommand(name)
#define PWMU_TriggerCommand(value1, value2)   	_PWMU_TriggerCommand(PWMU_NAME)(value1, value2)
	
#define __PWMCtrlReg_Write(name)       			name##_Write
#define _PWMCtrlReg_Write(name)        			__PWMCtrlReg_Write(name)
#define PWMCtrlReg_Write(value)            		_PWMCtrlReg_Write(PWMREG_NAME)(value)

/*PWM Period*/
uint16 pwmPeriod = 0;
/*PWM Compare Values*/
uint16 pwmCmp[3];

/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_PWMStart											  *
*																			  *
*   Description: Init three TCPWM modules PWMU PWMV PWMW					  *
*																			  *				
*   Paramters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_PWMStart()
{
    /* start 3 phases PWM	*/
    PWMU_Start();
	PWMU_SetPWMMode(PWMU_UNDERFLOW_SET| PWMU_OVERLOW_NO_CHANGE| PWMU_CC_MATCH_INVERT);
    PWMV_Start();
	PWMV_SetPWMMode(PWMV_UNDERFLOW_SET| PWMV_OVERLOW_NO_CHANGE| PWMV_CC_MATCH_INVERT);
    PWMW_Start();
	PWMW_SetPWMMode(PWMW_UNDERFLOW_SET| PWMW_OVERLOW_NO_CHANGE| PWMW_CC_MATCH_INVERT);
    /* clear counter value after start PWM	*/
    PWMU_WriteCounter(0);
    PWMV_WriteCounter(0);
    PWMW_WriteCounter(0);
    
    /* get the initial duty cycles	*/
    pwmPeriod = PWMU_ReadPeriod();
    pwmCmp[0] = PWMU_ReadCompare();
    pwmCmp[1] = PWMV_ReadCompare();
    pwmCmp[2] = PWMW_ReadCompare();
      
    /* disable GPIO output as default, enable later based on firmware	*/
    PWMCtrlReg_Write(0x00);
    /* synchronized start three PWMs */
    PWMU_TriggerCommand(0x07, PWMU_CMD_START); 
}
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_PWMStop											  *
*																			  *
*   Description: Disable three TCPWM modules PWMU PWMV PWMW					  *
*																			  *				
*   Paramters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_PWMStop()
{
    PWMU_Stop();
    PWMV_Stop();
    PWMW_Stop();
}
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_PWMOutputEnable									  *
*																			  *
*   Description: Enable PWM Outputs					    					  *
*																			  *				
*   Paramters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_PWMOutputEnable()
{
    /* enable GPIO output	*/
    PWMCtrlReg_Write(0x01);       
}
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_PWMOutputDisable									  *
*																			  *
*   Description: Disable PWM Outputs					  					  *
*																			  *				
*   Paramters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_PWMOutputDisable()
{
    /* disable GPIO output	*/
    PWMCtrlReg_Write(0x00);
}
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_PWMOutputUpdate									  *
*																			  *
*   Description: Update	PWM dutys and handle the 0% and 100% cases			  *
*				 															  *				
*   Paramters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_PWMOutputUpdate()
{
	/* handle the 0% and 100% for PWM-U */
	PWMU_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMU_UNDERFLOW_SET| PWMU_OVERLOW_NO_CHANGE| PWMU_CC_MATCH_INVERT));
	if(pwmCmp[0] == 0)
	{
		pwmCmp[0] = pwmPeriod + 1;		
		PWMU_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMU_UNDERFLOW_CLEAR| PWMU_OVERLOW_NO_CHANGE| PWMU_CC_MATCH_INVERT));
	}
	else if(pwmCmp[0] == pwmPeriod)
	{
		pwmCmp[0] = pwmPeriod + 1;
	}
	/* handle the 0% and 100% for PWM-V */
	PWMV_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMV_UNDERFLOW_SET| PWMV_OVERLOW_NO_CHANGE| PWMV_CC_MATCH_INVERT));
	if(pwmCmp[1] == 0)
	{
		pwmCmp[1] = pwmPeriod + 1;		
		PWMV_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMV_UNDERFLOW_CLEAR| PWMV_OVERLOW_NO_CHANGE| PWMV_CC_MATCH_INVERT));
	}
	else if(pwmCmp[1] == pwmPeriod)
	{
		pwmCmp[1] = pwmPeriod + 1;
	}
	
	/* handle the 0% and 100% for PWM-W  */
	PWMW_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMW_UNDERFLOW_SET| PWMW_OVERLOW_NO_CHANGE| PWMW_CC_MATCH_INVERT));
	if(pwmCmp[2] == 0)
	{
		pwmCmp[2] = pwmPeriod + 1;		
		PWMW_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMW_UNDERFLOW_CLEAR| PWMW_OVERLOW_NO_CHANGE| PWMW_CC_MATCH_INVERT));
	}
	else if(pwmCmp[2] == pwmPeriod)
	{
		pwmCmp[2] = pwmPeriod + 1;
	}
    /* Update new duty value into compare buffer registers	*/
    PWMU_WriteCompareBuf(pwmCmp[0]);
    PWMV_WriteCompareBuf(pwmCmp[1]);
    PWMW_WriteCompareBuf(pwmCmp[2]); 
}



/* [] END OF FILE */
