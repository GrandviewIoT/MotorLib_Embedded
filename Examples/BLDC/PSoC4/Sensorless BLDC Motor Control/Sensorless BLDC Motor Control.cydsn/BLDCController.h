/*******************************************************************************
*
* Filename:             BLDCController.h
* Owner :               Bob Hu (bobh@cypress.com)
*
* Version:              V1.0 
* Description:          Sensorless BLDC Motor Control
*                       Header file for BLDCController.c, define the constant value and 
*                       function declaration
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
#ifndef __BLDC_CONTROLLER_H__
#define __BLDC_CONTROLLER_H__

/******************************************************************************
 * Header file including
 ******************************************************************************/ 
#include <cytypes.h>
#include "Parameters.h"
#include "Debug.h"    

/******************************************************************************
 * Macro declaration   
 ******************************************************************************/    
#ifndef TRUE
#define TRUE                                ((uint8)(0x01u))
#endif  

#ifndef FALSE
#define FALSE                               ((uint8)(0x00u))
#endif  

/******************************************************************************
 * Structure/Enum type declaration   
 ******************************************************************************/   
/* enum type for error code */
typedef enum _Error_Code
{
    NO_ERROR,                               /* no error happens */
    ZC_CHECK_ERROR,                         /* zero-crossing detection failure */
    OV_ERROR,                               /* over voltage happens */
    UV_ERROR,                               /* under voltage happens */
    FREERUN_ERROR,                          /* freerun stage fails, fail to enter close loop */ 
    ANY_ERROR,                              /* for any unknown error */
    ERROR_SIZE                              /* varialbe to store count of error types */
}Error_Code_T;

/* Structure type to store control varialbe during BLDC roration */
typedef struct _Run_Control
{
    uint8           runFlag;                /* flag to start motor running */
    Error_Code_T    errorCode;              /* error code for motor running */
    
    uint8           sector;                 /* one of 6 sector of BLDC control */ 
    uint8           checkFallingEdge;       /* If check fall edge of BEMF */ 
	uint8           inNormalRun;            /* flag to indicate if motor works in normal run stage*/ 
    
    uint16          speedMeasuredRpm;       /* speed measured by counter, in RPM uint */
    uint16          speedRefRpm;            /* current speed reference, may be smaller than speedGivenRpm */
    uint16          speedGivenRpm;          /* given speed reference which is desired to achevie  */
    
    uint16          commutateStamp;         /* Time stamp of latest commucation event */
    uint16          zeroCrossStamp;         /* Previoius Time stamp of previous commucation event */ 
    uint16          zeroCrossPeriod;        /* Period between two commucation event */ 
	uint16          delayTime;	            /* Delay time between zero-cross and commutation */ 
    
    int32           pidOutput;              /* PID output */
    int32           pidSpeedErr;            /* speed count error in speed PID */
}BLDC_Control_T;

/* enum type for BLDC running state */
typedef enum
{
    STOPPED         = 0x01 ,                /* motor stop state */
    FREERUNNING     = 0x02 ,                /* freerun state, open loop during motor startup */
    NORMALRUN       = 0x03 ,                /* normal run state with sesorless control after BEMF is detected */
    ERRORSTOP       = 0x04 ,                /* error state if error happens in normal run  */
    FREERUNFAILED   = 0x05 ,                /* error state if freerun stage failed */
    PREPOSITION     = 0x06 ,                /* error state if freerun stage failed */
    OCERROR         = 0x07 ,                /* error state if over current happens */
} Motor_Stage_T;

/******************************************************************************
 * Global variables declaration   
 ******************************************************************************/    
extern BLDC_Control_T   BLDC_Control;              /* Structure varialbe for control BLDC running */
extern BLDC_Config_T    BLDC_Config;               /* Structure varialbe to store motor parameter */
extern Motor_Stage_T   runState;                   /* enum variable to store running state */

extern uint16 dutyCycle;                           /* duty cycle value applied on driving PWM */
extern uint16 speedRefCount;                       /* given speed reference */
extern uint16 speedCurCount;                       /* measured rotation speed */
extern uint8  needRunSpeedPID;                     /* flag to sync with PWM ticker ISR to run speed PID */
extern uint8  needUpdateSpeedRef;                  /* flag to sync with PWM ticker ISR to update speed reference */
/******************************************************************************
 * Global function declaration   
 ******************************************************************************/  
extern void BLDC_ParameterInit(void);
extern void BLDC_ControllerInit(void);
extern void BLDC_Start(void);
extern void BLDC_Stop(void);
extern void BLDC_Run(void);
extern void BLDC_UpdateSpeedRef(uint16 speed);
extern void BLDC_SpeedCloseLoop(void);
/******************************************************************************
 * End of declaration   
 ******************************************************************************/  

#endif  /* __BLDC_CONTROLLER_H__ */
/* [] END OF FILE */

