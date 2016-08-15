/*******************************************************************************
*
* Filename:             Startup.c
* Owner :               Bob Hu (bobh@cypress.com)
*
* Version:              V1.0 
* Description:          Sensorless BLDC Motor Control
*                       This file contains function exectued during motor startup
*                       stage. Once BLDC roration enter normal runnning, functions
*                       in this file are not executed.
*                       
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
#include "Startup.h"
#include "MotorRun.h"
#include "BLDCController.h"

/******************************************************************************
 * Global variables definition
 * ----------------------------------------------------------------------------
 * These varialbes should be populated to other modules. Header file contain 
 * the extern statement for these variables.
 ******************************************************************************/ 

/******************************************************************************
 * Local Macro definition
 * ----------------------------------------------------------------------------
 * These Macro are only used in this module. These Macro should not be populated
 * to other modules.
 ******************************************************************************/

/******************************************************************************
 * Local variables definition
 * ----------------------------------------------------------------------------
 * These varialbes are only used in this module. These varialbes should not be 
 * populated to other modules.
 ******************************************************************************/
/* timeout flag for zero crossing checking in freerun stage */
static uint8 timeoutFlag = TRUE;

typedef enum _Startup_Stage_T 
{
    START_STAGE,
    DEC_VOLT_STAGE,
    FREE_RUN_STAGE,
    ACCELERATION_STAGE,
    FAIL_STAGE
}Startup_Stage_T;

/*******************************************************************************
* Function Name: Preposition
********************************************************************************
* Summary:
*   align rotor to desired position with pre-defined duty and time 
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void Preposition(void)
{
    /* counter for pre-position stage */
    static uint32  prepositionTicker = 0;
    
    /* Set dutycycle of preposition */
    dutyCycle = BLDC_Config.prepositionDuty;
    PWM_Drive_WriteCompare1(dutyCycle);  
    
    SectorCtrl_Write(1);
    /* Delay 100mS */
    if(prepositionTicker >= BLDC_Config.prepositionTime)
    {  
        /* Set dutycycle of Freerun */
        dutyCycle = BLDC_Config.startDuty;
        /* Change status */
        runState  = FREERUNNING;    
        /* Initialize parameters for freerun */        
        timeoutFlag = TRUE;
        /* reset preposition ticker */        
        prepositionTicker = 0;
    }
    else
    {
        prepositionTicker++;
    }
}

/*******************************************************************************
* Function Name: FreeRun
********************************************************************************
* Summary:
*   Detect initial zerocross point, sampling the BEMF signal based on the output
*   of comparator have impact pulse.
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void FreeRun(void)
{      
    /* pointer of startup time table */
    static uint16 freerunCounter = 0;
    /* checking timer in free run stage */
    static uint32  checkPeriod = 1;
    /* flag to enable zero-crossing check */
    static uint8 checkEnable = FALSE;
    /* detected zero crossong count */
    static uint8 zeroCrossingCount = 0;
    /* state for startup procedure */
    static Startup_Stage_T startupStage = START_STAGE;
    /* counter to store how many communation happens in specific period */
    static uint8 resetCount = 0;    
    static uint16 commutateCount;
    
    uint16 curCount = 0;     
    uint16 dutyLocal = dutyCycle;
    
    if(timeoutFlag == TRUE)
	 {   
        /* reset timeoutFlag after a detect timeout happens */
	    timeoutFlag = FALSE;         
        /* execute one commutation in open loop */
	    Commutating();
        /* store the commutation timestamp */
        commutateCount = Counter_Spd_ReadCounter();
        		       
        /* reset counter for zero-crossing after desired commutation happens */
		if(resetCount >= ZC_COUNT_RESET_THRESHOLD)
		{
			resetCount = 0;
			zeroCrossingCount = 0;
		}
        resetCount++;  
        
        /* Have enough ZC count and ready to switch to normal run stage */
        if((zeroCrossingCount >= ZC_CHECKING_STABLE_THRESHOLD) && (resetCount <= 5))
		{           
            /* reset all internal flags for next execution of startup freerun */
            startupStage = START_STAGE;
            resetCount = 0;
			zeroCrossingCount = 0;			
            freerunCounter = 0;
            checkEnable = FALSE;
            timeoutFlag = TRUE;
            BLDC_Config.accStageWait = ACC_STAGE_EXEC_COUNT;
            
            /* switch main state machine to normal run state */
            runState = NORMALRUN;
            /* intial the first commutate period based on the last communtation in freerun stage */
            BLDC_Control.zeroCrossPeriod = checkPeriod;           
            /* set flag to indicated that motor runs in normal run stage */
            BLDC_Control.inNormalRun = TRUE; 
                                  
            /* get timestamp for current commutation */
            BLDC_Control.commutateStamp = commutateCount;
            /* down counter, avoid a unexpected interrupt generation */
            Counter_Spd_WriteCompare(Counter_Spd_ReadCounter() + 10);          
           
            return;
		}
        
        /* state machine for startup */
        switch(startupStage)
		{
		case START_STAGE:
            /* --------------------------------------------------------------------------------------
             * in this state, the voltage on motor winding is fixed, the rotation speed is also fixed
             * this state intents to make motor start to run with a big enough start voltage 
             * do not detect zero crossing in this stage 
             * --------------------------------------------------------------------------------------*/
            checkEnable = FALSE;
            dutyLocal = BLDC_Config.startDuty;
            checkPeriod = BLDC_Config.startCheckPeriod;
                        
			if(freerunCounter >= BLDC_Config.startStageWait)
			{
                /* swithc stage for startup process */
				startupStage = DEC_VOLT_STAGE;
                /* reset internal counter */
				freerunCounter = 0;
			}
			break;
            
		case DEC_VOLT_STAGE:
            /* --------------------------------------------------------------------------------------
             * in this state, the voltage on motor winding is decreased, the rotation speed is still 
             * fixed this state intents to decrease the voltage to prevent motor winding from burn
             * do not detect zero crossing in this stage 
             * --------------------------------------------------------------------------------------*/       
			checkEnable = TRUE;
            /* only decrease duty cycle when reach the end of interval */
            if(freerunCounter >= BLDC_Config.decStageInterval)
			{
				dutyLocal--;
				freerunCounter = 0;
                /* increaset checking speed */
                checkPeriod -= 3;
			}
            
			if(dutyLocal <= BLDC_Config.freerunDuty)
            {
				startupStage = FREE_RUN_STAGE;
                freerunCounter = 0;
            }
            
			break;
            
		case FREE_RUN_STAGE:
            /* --------------------------------------------------------------------------------------
             * in this state, the voltage on motor winding is set with free run voltage, which may 
             * lower than start voltage, because the start voltage need to overcome the big inertia
             * at heavy load status. The rotation speed is still fixed. 
             * start to detect zero crossing in this stage 
             * --------------------------------------------------------------------------------------*/            
			checkEnable = TRUE;
            dutyLocal = BLDC_Config.freerunDuty;
            checkPeriod = BLDC_Config.startCheckPeriod;
            
			if(freerunCounter >= BLDC_Config.freerunStageWait)
			{
				startupStage = ACCELERATION_STAGE;
				freerunCounter = 0;
			}
			break;
            
		case ACCELERATION_STAGE:
            /* --------------------------------------------------------------------------------------
             * in this state, the voltage on motor winding is increased for higher speed. The rotation
             * speed is also increased.
             * --------------------------------------------------------------------------------------*/             
			checkEnable = TRUE;
            if(freerunCounter >= BLDC_Config.accStageInterval)
			{
				freerunCounter = 0;
				dutyLocal += BLDC_Config.accDutyStep;
				checkPeriod -= BLDC_Config.accTimeStep;
				BLDC_Config.accStageWait--;
			}
			
			if(BLDC_Config.accStageWait == 0)
			{
				startupStage = FAIL_STAGE;
				freerunCounter = 0;
			}
			break;
	
		case FAIL_STAGE:
        default:
            runState = FREERUNFAILED;
            BLDC_Control.errorCode = FREERUN_ERROR;
			break;
		}
        
        dutyCycle = dutyLocal;
        freerunCounter++;
	}
	   
	/* get current count from down counter */
    curCount = Counter_Spd_ReadCounter();
    /* since it is down counter, the commutate count should be larger than current count */
	if((uint16)(commutateCount - curCount) < checkPeriod)
	{
	    if(checkEnable)
		{
            /* check zero-crossing event, if yes, zeroCrossingIsValid will be true */
		    if(CheckZeroCrossing() == TRUE)
			{
                /* increase zero-crossing counter */
                zeroCrossingCount++;
                /* initialize the last zero-crossing time stamp */
                BLDC_Control.zeroCrossStamp = curCount;
                /* Exit from zero crossing detection */
			    checkEnable = FALSE;                  
			}
		}
	}	
	else
	{
        /* If timeout, execute next forcing commutation */
	    timeoutFlag =TRUE;	
	} 
}

/* [] END OF FILE */
