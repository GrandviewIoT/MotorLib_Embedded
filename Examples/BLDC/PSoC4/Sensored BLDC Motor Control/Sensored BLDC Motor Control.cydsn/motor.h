/*******************************************************************************
*
* Filename:             motor.h
* Owner :               Rod Wang (rowa@cypress.com)
*
* Version:              V1.0 
* Description:          This file contains the function prototypes and constants used in
*                       the motor parameter.  
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

#ifndef _MOTOR_H_
#define _MOTOR_H_
 
#include <cytypes.h>   

/* define TRUE and FALSE if not defined previously */
#ifndef FALSE
#define FALSE           (uint8) 0
#endif

#ifndef TRUE
#define TRUE            (uint8) 1
#endif 

/* macro definition for rotation direction */
#define CLOCK_WISE              (uint8) 0
#define COUNTER_CLOCK_WISE      (uint8) 1

/* Status specific*/
#define STATUS_RUN              (uint8) 0x01
#define STATUS_STOP             (uint8) (STATUS_RUN + 1)
#define STATUS_ERROR            (uint8) (STATUS_STOP + 1)

#define FREQ_CAPTURE_CLK        ((uint32) (100000))
#define MOTOR_POLE_PAIRS        ((uint32) (4))

               /* Command from UI */
typedef struct
{
    uint8  run; /* Let motor running*/
} UI_CMD;

               /* Data passed between UI and MC controller */
typedef struct
{ 
    uint8   Dir;         /* Direction */
   
    uint16  speedRpm;    /* Actural motor speed */
    uint16  speedRpmRef; /* motor speed command value */
   
    uint16  kp;          /* Proportional coeffient of PID */
    uint16  ki;          /* Integral coeffient of PID */
   
    uint16  maxSpeedRpm; /* Motor parameters */
    uint16  minSpeedRpm; /* Motor parameters */ 
    uint8   polePairs;   /* POLE PAIRS       */
    uint8   maxCurr;     /* Over current threshold */   
} UI_DATA;


/*To be defined*/
typedef enum _Error {
                      no_error  = 0,
                      overCur   = 1,
                      hallError = 2,
                      lowVolt   = 3,
                      highVolt  = 4
                    }     Error_T;

extern Error_T   errorState; 
extern UI_CMD    UI_Cmd;
extern UI_DATA   UI_Data;
extern uint16    speedRef;
extern uint16    speedCur;

extern uint16    preSpeedCur;
extern uint16    preCntCaptur;

extern uint16    pwmCnt;
extern uint16    pwmCntLocal;
extern uint8     dutyCycle;
extern uint8     stateSys;

/* Functions */
void Init_UI_FW(void);
void Init_HW(void);
void Init_UI_HW(void);
    
#endif
