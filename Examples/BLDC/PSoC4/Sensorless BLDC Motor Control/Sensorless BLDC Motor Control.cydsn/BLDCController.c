/*******************************************************************************
*
* Filename:             BLDCController.c
* Owner :               Bob Hu (bobh@cypress.com)
*
* Version:              V1.0 
* Description:          Sensorless BLDC Motor Control
*                       This file contains high-level function to control the BLDC
*                       motor, such as BLDC_Start, BLDC_Stop, BLDC_Init, and so on.
*                       The low level control algorithm is implemented in seperate
*                       files.
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
#include "Parameters.h"
#include "MotorRun.h"
#include "Control.h"
#include "BLDCController.h"


/******************************************************************************
 * Global variables definition
 * ----------------------------------------------------------------------------
 * These varialbes should be populated to other modules. Header file contain 
 * the extern statement for these variables.
 ******************************************************************************/ 
uint16  dutyCycle = 0;                      /* duty cycle value applied on driving PWM */
uint16  speedRefCount = 0;                  /* given speed reference */
uint16  speedCurCount = 0;                  /* measured rotation speed */
uint8   needRunSpeedPID = FALSE;            /* flag to sync with PWM ticker ISR to run speed PID */
uint8   needUpdateSpeedRef = FALSE;                 /* flag to sync with PWM ticker ISR to update speed reference */

BLDC_Control_T   BLDC_Control;              /* Structure varialbe for control BLDC running */
BLDC_Config_T    BLDC_Config;               /* Structure varialbe to store motor parameter */
Motor_Stage_T    runState = STOPPED;        /* enum variable to store running state */

/******************************************************************************
 * Local Macro definition
 * ----------------------------------------------------------------------------
 * These Macro are only used in this module. These Macro should not be populated
 * to other modules.
 ******************************************************************************/
#define BLDC_CONTROL_INIT_CODE              {\
    BLDC_Control.runFlag = FALSE;\
    BLDC_Control.errorCode = NO_ERROR;\
    BLDC_Control.sector = 1;\
    BLDC_Control.checkFallingEdge = FALSE;\
    BLDC_Control.inNormalRun = FALSE;\
    BLDC_Control.speedMeasuredRpm = 0;\
    BLDC_Control.speedRefRpm = 0;\
    BLDC_Control.speedGivenRpm = 0;\
    BLDC_Control.commutateStamp = 0;\
    BLDC_Control.zeroCrossStamp = 0;\
    BLDC_Control.zeroCrossPeriod = 0;\
    BLDC_Control.delayTime = 0;\
    BLDC_Control.pidOutput = 0;\
    BLDC_Control.pidSpeedErr = 0;\
}

/******************************************************************************
 * Local variables definition
 * ----------------------------------------------------------------------------
 * These varialbes are only used in this module. These varialbes should not be 
 * populated to other modules.
 ******************************************************************************/
static uint8   restartFlag = TRUE;          /* flag to indicate restart operation after stop */
static uint8   speedCloseEnable = FALSE;    /* enable flag to enter speed close loop */

/*******************************************************************************
* Function Name: BLDC_ParameterInit
********************************************************************************
* Summary:
*   a) Configure motor parameters based on connected motor;
*   b) Initialize parameters used in control algorithm based on macros defined
*      in Parameters.h
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void  BLDC_ParameterInit (void)
{
       /* set motor pole-pair number */
    BLDC_Config.polePairNumber = MOTOR_POLE_PAIR_NUM;
    
       /* motor rotation direction, options are CLOCK and COUNTER_CLOCK */
    BLDC_Config.direction = CLOCK;    // default = CLOCKWISE

       /* set speed reference in RPM units */
    BLDC_Config.initSpeedRefRpm = INIT_SPEED_REF_RPM;
    
    BLDC_Config.kp = KP_INITIAL_VALUE;  // setup kp for PID control
    BLDC_Config.ki = KI_INITIAL_VALUE;  // setup ki for PID control
    
    BLDC_Config.startDuty        = START_PWM_DUTY;
    BLDC_Config.freerunDuty      = FREERUN_PWM_DUTY;
    BLDC_Config.startCheckPeriod = START_ZC_CHECK_PERIOD;
    BLDC_Config.startStageWait   = START_STAGE_WAIT_COUNT;
    BLDC_Config.decStageInterval = DEC_STAGE_INTERVAL;
    BLDC_Config.freerunStageWait = FREERUN_STAGE_WAIT_COUNT;
    BLDC_Config.accStageInterval = ACC_STAGE_INTERVAL;
    BLDC_Config.accStageWait     = ACC_STAGE_EXEC_COUNT;
    BLDC_Config.accDutyStep      = ACC_STAGE_DUTY_STEP;
    BLDC_Config.accTimeStep      = ACC_STAGE_TIME_STEP;
    
    BLDC_Config.prepositionTime  = PREPOSITION_WAIT_TIME;
    BLDC_Config.prepositionDuty  = PREPOSITION_PWM_DUTY;
    
    BLDC_Config.speedCloseLoopWait = SPEED_CLOSE_LOOP_DELAY;
    BLDC_Config.zcCheckSkipCount   = ZC_CHECK_SKIP_COUNT;    
}

/*******************************************************************************
* Function Name: BLDC_ControllerInit
********************************************************************************
* Summary:
*   a) Configure motor parameters used in rotation;
*   b) Initialize PSoC peripherals
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void  BLDC_ControllerInit (void)
{
    /**************************************************************************
     * PSoC peripheral initialization 
     **************************************************************************/
    
       /* PWM Initialization */
    PWM_Drive_Start();	

       /* set half of minimum duty cycle for BEMF detection sync trigger generation */
    PWM_Drive_WriteCompare2(MIN_PWM_DUTY >> 1);

       /* Speed Counter Initialization */
    Counter_Spd_Start();
    
       /* Amux_1 initialization */
    AMux_1_Start();
    AMux_1_Select(0);  
    
    /* following modules are used for over current protection */    
    
       /* start BEMF detection comparator */
    BEMF_Comp_Start();

       /* set ISR for internal tick counter, enable as default */
    isr_pwm_StartEx(PWM_Drive_ISR);
 
       /* set ISR for speed measurement, enable as default */
    isr_spd_measure_StartEx(Speed_ISR);

       /* set ISR for commutation process, disable as default */
    isr_commutate_StartEx(Commutate_ISR);
    isr_commutate_Disable();
    
    /**************************************************************************
     * internal control flags initialization 
     **************************************************************************/
       /* set default value f or BLDC_Control structure */ 
    BLDC_CONTROL_INIT_CODE;
    
       /* set initial value for internal control flags */
    restartFlag = TRUE;
    runState    = STOPPED;
    
    SectorCtrl_Write (0x00);
}

/*******************************************************************************
* Function Name: BLDC_Start
********************************************************************************
* Summary:
*   Start motor rotation
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void  BLDC_Start (void)
{
       /* reset control parameters and status flags */
    runState    = PREPOSITION;  
    restartFlag = FALSE; 
    
       /* set default value f or BLDC_Control structure */ 
    BLDC_CONTROL_INIT_CODE;    
   
       /* reset motor speed reference with initial value */
    BLDC_Control.speedGivenRpm = BLDC_Config.initSpeedRefRpm;
    
       /* clear all pending interrupt and re-enable PWM interrupt processing */
    isr_pwm_ClearPending();
    isr_pwm_Enable();
}

/*******************************************************************************
* Function Name: BLDC_Stop
********************************************************************************
* Summary:
*   Stop motor rotation
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void BLDC_Stop()
{
    runState = STOPPED;

       /* disable PWM periodical ISR */
    isr_pwm_Disable();
    
       /* disable commutation interrupt generation */
    isr_commutate_Disable();

       /* Diable PWM output */
    SectorCtrl_Write (0x00);

       /* Prepare for next re-start */
    restartFlag = TRUE;

       /* reset enable flag for entering speed close loop */
    speedCloseEnable = FALSE;
}


/*******************************************************************************
* Function Name: BLDC_Run
********************************************************************************
* Summary:
*   Main process function for BLDC motor rotation, including state switch among 
*   Startup, normal run and error state. The speed PID for close loop control is 
*   also executed here.
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void  BLDC_Run (void)
{   
   switch(runState)
    {
    case STOPPED:
           /* check if start motor is running */ 
        if ((BLDC_Control.errorCode == NO_ERROR) && (BLDC_Control.runFlag == TRUE) && (restartFlag == TRUE))
           {     // is not running, so start it up
             BLDC_Start();
           }
        break;
    
    case PREPOSITION:
    case FREERUNNING:
           /* stop the motor running if not entering the normal run stage due to startup failure */
        if (BLDC_Control.runFlag == FALSE)
           {
             BLDC_Stop();                
           }        
        break;

    case NORMALRUN:
           /* execute speed close loop if it is enabeed */
        if (needRunSpeedPID == TRUE)      /* if 255(0xFF) ticks happens */
           {   
             BLDC_SpeedCloseLoop();
             needRunSpeedPID = FALSE;
           }
		
        if (needUpdateSpeedRef == TRUE)
           {            
               /* Convert speed from timer counter valut into Rpm value*/
            BLDC_Control.speedMeasuredRpm = SECOND_PER_MINUTE / BLDC_Config.polePairNumber;
            BLDC_Control.speedMeasuredRpm = BLDC_Control.speedMeasuredRpm * SPEED_MEASURE_CLOCK_FREQ / speedCurCount;
               /* update speed reference based on specific curve */
            BLDC_UpdateSpeedRef(BLDC_Control.speedGivenRpm); 
               /* reset speed reference update flag */
            needUpdateSpeedRef = FALSE;
           }
        
        if (BLDC_Control.runFlag == FALSE)
           {            
             BLDC_Stop();                
           }
        
        break;
        
    case ERRORSTOP:
        BLDC_Control.runFlag = FALSE;  
        BLDC_Stop(); 
        
        
#if (DEBUG_AUTO_CLEAR_ERROR_STATE_ENABLE)        
        BLDC_Control.errorCode = NO_ERROR;
#else
          /* in error state, the LED2 blinks at 1Hz until reset button is pressed */
        while(1)
          {            
            switch (BLDC_Control.errorCode)
             {           
                case ANY_ERROR:
                case ZC_CHECK_ERROR:  
                case OV_ERROR:
                case UV_ERROR:                    
                case FREERUN_ERROR:        
                 {
                    uint8 i = 0;                    
                    for(i = 0; i < BLDC_Control.errorCode; i++)
                    {
                        /* LED on */
                        CtrlReg_Debug_Control &= ~(DEBUG_STATUS_LED_MASK);
                        CyDelay(200);
                        /* LED off */
                        CtrlReg_Debug_Control |= (DEBUG_STATUS_LED_MASK);
                        CyDelay(200);
                    }
                    CyDelay(2000 - 100 * BLDC_Control.errorCode);
                    break;
                 } 
                
                case NO_ERROR:
                default:
                    break;
             }
         }
#endif    
         
    default:
        BLDC_Stop();
        BLDC_Control.runFlag = FALSE;
        break;
    }
}


/*******************************************************************************
* Function Name: BLDC_UpdateSpeedRef
********************************************************************************
* Summary:
*   Set speed reference based on input parameter. The speed reference should in 
*   the correct speed range decided by setting in BLDC_Config structure
*
* Parameters:  
*  uint16 speed: new speed reference
*
* Return: 
*  void
*
*******************************************************************************/
void  BLDC_UpdateSpeedRef (uint16 speed)
{    
    uint16 refDelta = 0;
    uint16 localSpeedRef = BLDC_Control.speedRefRpm;

    if (speed > localSpeedRef)
      {
        refDelta = speed - localSpeedRef;
        if (refDelta > MAX_SPEED_REF_UPDATE_STEP)
           {
             localSpeedRef += MAX_SPEED_REF_UPDATE_STEP;                        
           }
          else
           {
             localSpeedRef += refDelta; 
           }
        if (localSpeedRef > MAX_SPEED_REF_RPM)
            localSpeedRef = MAX_SPEED_REF_RPM;
      }
     else
      {
        refDelta = localSpeedRef - speed;
        if (refDelta > MAX_SPEED_REF_UPDATE_STEP)
          {
            if (localSpeedRef > MAX_SPEED_REF_UPDATE_STEP)
                localSpeedRef -= MAX_SPEED_REF_UPDATE_STEP;            
            else
                localSpeedRef = 0;
          }
         else
          {
            if (localSpeedRef > refDelta)
                localSpeedRef -= refDelta;            
                else localSpeedRef = 0;
          }
        if (localSpeedRef < MIN_SPEED_REF_RPM)
            localSpeedRef = MIN_SPEED_REF_RPM;
      }
    
    BLDC_Control.speedRefRpm = localSpeedRef;
}


void  BLDC_SpeedCloseLoop (void)
{
    static uint8 speedCloseDelay = 0;   /* delay counter to enter speed close loop */ 

    if (speedCloseEnable == FALSE)
      {
        speedCloseDelay++;
        if (speedCloseDelay > BLDC_Config.speedCloseLoopWait)
          {
            speedCloseEnable = TRUE;
            speedCloseDelay  = 0;

               /* update current speed for initilization */
            BLDC_Control.speedMeasuredRpm = SECOND_PER_MINUTE / BLDC_Config.polePairNumber;
            BLDC_Control.speedMeasuredRpm = BLDC_Control.speedMeasuredRpm * SPEED_MEASURE_CLOCK_FREQ / speedCurCount;

               /* update current speed into reference speed */
            BLDC_Control.speedRefRpm = BLDC_Control.speedMeasuredRpm;

               /* set initial speed reference to control strucutre */
            BLDC_Control.speedGivenRpm = BLDC_Config.initSpeedRefRpm;

               /* set current duty cycle value as initial value of PID result */
            BLDC_Control.pidOutput  = dutyCycle << PI_RANGE_SCALE;
            BLDC_Control.pidOutput -= LIMIT_HALF_RANGE;
          }
      }
     else
      {
        /**********************************************************************
         *  update speed reference from RPM speed reference, 
         *  result is converted to desired valued for speed measure counter
         *  equation:
         *      RPM = SECOND_PER_MINUTE * F_ELECTRICAL / POLE_PAIRS_NUM
         *      SPEED_REF_COUNT = SPEED_MEASURE_CLOCK_FREQ / F_ELECTRICAL
         **********************************************************************/                
        speedRefCount = SECOND_PER_MINUTE / BLDC_Config.polePairNumber;
        speedRefCount = speedRefCount * SPEED_MEASURE_CLOCK_FREQ / BLDC_Control.speedRefRpm;                             
        
#if (DEBUG_SPEED_CLOSE_ENABLE)                
        /* Speed PID close loop control */ 
        SpeedPID();
        CalcPWMDuty();
#endif       
     } 
}
/* [] END OF FILE */
