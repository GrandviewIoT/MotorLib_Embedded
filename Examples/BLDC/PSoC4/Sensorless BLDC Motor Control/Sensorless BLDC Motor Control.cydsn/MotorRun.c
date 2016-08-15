/*******************************************************************************
*
* Filename:             MotorRun.c
* Owner :               Bob Hu (bobh@cypress.com)
*
* Version:              V1.0 
* Description:          Sensorless BLDC Motor Control
*                       This file contains function to keep motor normal rotation,
*                       including CheckZeroCrossing, Commutating and three ISRs to 
*                       handler commutation, zero-crossing and speed sampling
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
#include "parameters.h"
#include "MotorRun.h"
#include "Startup.h"
#include "BLDCController.h"

/******************************************************************************
 * Global variables definition
 * ----------------------------------------------------------------------------
 * These varialbes should be populated to other modules. Header file contain 
 * the extern statement for these variables.
 ******************************************************************************/
/* skip ckecking count for freewheeling pulse*/
static uint8 flywheelDelay = 0;                         

/******************************************************************************
 * Local Macro definition
 * ----------------------------------------------------------------------------
 * These Macro are only used in this module. These Macro should not be populated
 * to other modules.
 ******************************************************************************/
#define ZC_LOW_LEVEL                        (uint8)(0x00)
#define ZC_HIHG_LEVEL                       (uint8)(0x01)

/* up to 1.2 times of MIN_SPEED_COUNT, which enable PID works to get speed error */
#define MAX_SPEED_COUNT                     ((uint16)((SECOND_PER_MINUTE * SPEED_MEASURE_CLOCK_FREQ) / (MOTOR_POLE_PAIR_NUM  * MIN_SPEED_REF_RPM))*1.2)

/******************************************************************************
 * Local variables definition
 * ----------------------------------------------------------------------------
 * These varialbes are only used in this module. These varialbes should not be 
 * populated to other modules.
 ******************************************************************************/

/*******************************************************************************
* Function Name: CheckZeroCrossing
********************************************************************************
* Summary:
* The CheckZeroCrossing function checking BEMF zero-crossing piont by read 
* output of comparator. 
*
* Parameters:  
*  void:  
*
* Return: 
*  unsigned char
*
*******************************************************************************/
uint8 CheckZeroCrossing(void)
{
    uint8 zeroCrossingIsDetect = FALSE;
    uint8 levelStatus = ZC_LOW_LEVEL; 
    static uint8 preLevelStatus = ZC_LOW_LEVEL; 
    uint8 debounceCount = 0;
    uint8 i = 0;
    
    flywheelDelay++;	
    /* Wait for eleminating pulse error by diode flywheel */    
    if(flywheelDelay >= BLDC_Config.zcCheckSkipCount) 
	{
        if(BLDC_Control.checkFallingEdge)   /* desired to check falling edge */
        {
            levelStatus = Status_Comp_Read();
            /* Current is low and prevous one is high, then a falling edge is detected */
            if ((levelStatus == ZC_LOW_LEVEL) && (preLevelStatus == ZC_HIHG_LEVEL)) 
            {
                for(i = 0; i < 3; i++)
                {
                    levelStatus = Status_Comp_Read();
                    if(levelStatus == ZC_LOW_LEVEL)
                        debounceCount++;
                    else
                        debounceCount--;
                }
                if(debounceCount == 3)
                {
                    zeroCrossingIsDetect = TRUE;  
                }
            } 
        }
        else                                /* desired to check rising edge */
        { 
            levelStatus = Status_Comp_Read();
            /* Current is high and prevous one is low, then a rising edge is detected */
            if ((levelStatus == ZC_HIHG_LEVEL) && (preLevelStatus == ZC_LOW_LEVEL))
            {
                for(i = 0; i < 3; i++)
                {
                    levelStatus = Status_Comp_Read();
                    if(levelStatus == ZC_HIHG_LEVEL)
                        debounceCount++;
                    else
                        debounceCount--;
                }
                if(debounceCount == 3)
                {
                    zeroCrossingIsDetect = TRUE;  
                } 
            }         
        }
        /* store current value */
        preLevelStatus = levelStatus;
   }
    else                                     /* skip zero-crossing due to freewheeling pulse */
    { 
        if(BLDC_Control.checkFallingEdge) 
		{
            preLevelStatus = ZC_HIHG_LEVEL;
		}
		else
		{
            preLevelStatus = ZC_LOW_LEVEL;
		}		
    }

   return zeroCrossingIsDetect;    
}

/*******************************************************************************
* Function Name: Commutating
********************************************************************************
* Summary:
* The Commutating function commutating BLDC motor and switch BEMF sample channels. 
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void Commutating(void)
{	
    /* get current sector number */
    uint8 sector = BLDC_Control.sector;

#if (DEBUG_COMMUTATE_PULSE_GEN_ENABLE)  
    /* set TP_Commutate HIHG */
    CtrlReg_Debug_Control ^= (DEBUG_COMMUTATE_PULSE_GEN_MASK);
#endif  

    /* reset delay counter used in skipping flywheel pulse for zero crossing detection */
    flywheelDelay =0;  
    /* !! set sector for LUT output control to complete comuutation !! */
    SectorCtrl_Write(sector);   

    /* update new duty cycle to PWM component */
    PWM_Drive_WriteCompare1(dutyCycle);
    
    /* set desired edge detected in zero-crossing detectioin based on sector */
    if((sector & 0x01) == 0x01)
    {
        BLDC_Control.checkFallingEdge = TRUE;
    }
    else
    {
        BLDC_Control.checkFallingEdge = FALSE; 
    }
    
    /* adjust desired edge detected in zero-crossing detectioinC for clockwise rotation */
    if(BLDC_Config.direction == CLOCK)
    {
        /* invert the flag */
        BLDC_Control.checkFallingEdge ^= TRUE;
    } 
         		 
    /* sample different BEMF input channel */ 
    if((sector == 1 ) || (sector == 4))
    {
	   AMux_1_FastSelect(1);      /* detect zero crossing on BEMF2 */
    }
    else if ((sector == 2) || (sector == 5)) 
    {
       AMux_1_FastSelect(2);      /* detect zero crossing on BEMF3 */
    }
    else if ((sector == 3) || (sector == 6)) 
    {
       AMux_1_FastSelect(0);      /* detect zero crossing on BEMF1 */
    }
    else
    {
        AMux_1_FastSelect(0);      /* detect zero crossing on BEMF1 */
    }
        
    /* One direction */
    if(BLDC_Config.direction == CLOCK)
    {
        sector--;
        if(sector < 1) 
            sector = 6;
    }
    /* Inverse direction */
    else
    {
        sector++;
        if(sector > 6)
	        sector = 1;      
    }	 
    BLDC_Control.sector = sector;   
}

/*******************************************************************************
* Function Name: PWM_Drive_ISR
********************************************************************************
* Summary:
* Critical ISR for PWM drive
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
CY_ISR(PWM_Drive_ISR)
{
    uint16 curTimeStamep = 0;  
    uint16 curPeriod = 0;
    
    static uint16 pidExecTicker = 0;
    static uint16 speedRefUpdateTicker = 0;    
  
    /* in preposition stage, calling Prepositon function to align rotor to desired position */
    if(runState==PREPOSITION)	   	   
    {	   
        /* reset internal tick counter */
        pidExecTicker = 0;
        speedRefUpdateTicker = 0;
        /* execute preposition function */
		Preposition();        
    }	
    else if(runState==FREERUNNING)  /* open loop running for free running stage */
    {
        FreeRun();
    }
	/* after free running ,system goes to normal run stage if no error happens */   
    if (runState == NORMALRUN)
    {    
        /* For protection - if longer than setting threshold, stop the motor */
        curTimeStamep = Counter_Spd_ReadCounter();
        if(((uint16)(BLDC_Control.commutateStamp - curTimeStamep)) > ZC_CHECK_ERROR_THRESHOLD)
        {
#if (DEBUG_GENERAL_TP_OUTPUT_ENABLE)
            CtrlReg_Debug_Control |= (DEBUG_GENERAL_TP_MASK);
#endif 
            /* Stop the motor */
            runState = ERRORSTOP;	
            BLDC_Control.errorCode = ZC_CHECK_ERROR;
#if (DEBUG_GENERAL_TP_OUTPUT_ENABLE)
            CtrlReg_Debug_Control &= ~(DEBUG_GENERAL_TP_MASK);
#endif 
        } 
                
        /* firmware checking for zero-crossing point */
        if(CheckZeroCrossing() == TRUE)
        {
            /* Calculate period between two ZC events */
            curTimeStamep = Counter_Spd_ReadCounter();
            /* calculation duration between two zero crossing point */
            curPeriod = BLDC_Control.zeroCrossStamp - curTimeStamep;            
            
            /* Get Avg value by a 0.5/0.5 FIR filter */
            BLDC_Control.zeroCrossPeriod = (BLDC_Control.zeroCrossPeriod >> 1) + (curPeriod >> 1); 
            /* Delay 30 degree, than commutate.  this function can be  found in  check30degreepassed.c */
            BLDC_Control.delayTime = BLDC_Control.zeroCrossPeriod >> 1;  
            
            /* store current zeroCrossStamp */
            BLDC_Control.zeroCrossStamp = curTimeStamep;
            /* if compare value is equal to 0x0 or 0xFFFF, the compare signal fail to generate */            
            Counter_Spd_WriteCompare((uint16)(curTimeStamep - BLDC_Control.delayTime + 6));
            /* re-enable commutation ISR to process next coming commutation INT */
            isr_commutate_ClearPending();
    		isr_commutate_Enable();
            
            /* disable PWM interrupt to save CPU load */
            isr_pwm_Disable();  
        }
        
        pidExecTicker++;
        if(pidExecTicker == SPEED_PID_EXEC_INTERVAL)
        {
            needRunSpeedPID = TRUE;
            pidExecTicker = 0;
        }
        
        speedRefUpdateTicker++;
        if(speedRefUpdateTicker == SPEED_REF_UPDATE_INTERVAL)
        {
            needUpdateSpeedRef = TRUE;
            speedRefUpdateTicker = 0;
        }
    }        
                    
    /* clear interrupt status of PWM component to enable next interrupt generation */
    PWM_Drive_GetInterruptSource();
}

/*******************************************************************************
* Function Name: Commutate_ISR
********************************************************************************
* Summary:
* Critical ISR for commutation 
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
CY_ISR(Commutate_ISR)
{ 
    /* commutation execution */    
    Commutating();            

    /* store timestamp for current commutation event */
    BLDC_Control.commutateStamp = Counter_Spd_ReadCounter();
	
    /* disable commutation isr to avoid unexpected commutation event */
    isr_commutate_Disable();
	
	/* re-enable ISR for pwm signal to detect motor stall event */
    isr_pwm_ClearPending();
    isr_pwm_Enable(); 
}

/*******************************************************************************
* Function Name: Speed_ISR
********************************************************************************
* Summary:
* Critical ISR for speed measurement 
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
CY_ISR(Speed_ISR)
{
    static uint16 preSpdCapture = 0;
    uint16 tmp;
    uint16 speedCntLocal = 0;
    /* get current capture count */
    tmp = Counter_Spd_ReadCapture();
    speedCntLocal = (uint16)(preSpdCapture - tmp);
    preSpdCapture = tmp;

    /* detect if motor is stopped */
    if(speedCntLocal > MAX_SPEED_COUNT) 
        speedCntLocal = MAX_SPEED_COUNT; 
    speedCurCount = (speedCurCount >> 2) + (speedCurCount >> 1) + (speedCntLocal >> 2);
    
    /* clear interrupt status of Counter component to enable next interrupt generation */                
    Counter_Spd_ReadStatusRegister();
}

/* [] END OF FILE */
