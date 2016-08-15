//*******1*********2*********3*********4*********5*********6*********7**********
//
//                           MotorLib_Bldc_FOC.c
//
// Motor Control Library:  High Level support for FOC
//                         BLDC support.
//
// This is a portable code library, designed to work across a number of
// processors, including:   TI Tiva Cortex-M
//                          TI MSP432 with Cortex-M core
//                          TI MSP430 F5529, FR5969, FR6989, FR5994
//                          STM32 F3, F4, L4, L7 Cortex-M series
//
// Low level detailed support is provided by the associated MCU platform's
//     MotorLib_LL_xxxx_yyyy.c module, where xxxx = Chip type and
//     yyyyy = MCU type, e.g. MotorLib_LL_L6230_STM32.c

/*******************************************************************************
*
* Filename:             foc.c
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
#include "math.h"
#include "Cymc.h"

#define ADC_IA_CHAN             (uint8) (0)
#define ADC_IB_CHAN             (uint8) (1)
#define ADC_VBUS_CHAN           (uint8) (2)
#define ADC_VR_CHAN             (uint8) (3)

#define ALARMCURRENT            (int16)(MAX_PHASE_CURRENT*ADC_RESOLUTION*SENSING_RESISTOR*OPAMP_GAIN/VDD)
#define MIN_SPEED_STEP          _FQ(0.02)
#define SLOW_REGULATION         _FQ(0.1)
#define MIDDLE_REGULATION       _FQ(0.2)
#define FAST_REGULATION         _FQ(0.4)

#define VBUS_LOW                 20                     /* 15V */
#define VBUS_HIGH                30                     /* 30V */
#define SEC_COUNTER             5000                    /* 500ms */

uint16 speedLoopCount = 1; 
                                /* phase current U, V in q15 format */
q15_t  phaseCurrentU  = 0, phaseCurrentV  = 0;    
q15_t  phaseCurrentU0 = 0, phaseCurrentV0 = 0;
                                /* current angle in q15 format */
q15_t  curAngle       = 0;
                                /* seconds counter*/
uint16 pwmCounter     = 0;
uint8  secFlag        = 0;


/****************************************************************************************
*                                                                                          *
*   Function:    updateSpeedrefFromVR                                                      *
*                                                                                          *
*   Description: update speed reference                                                 *
*                                                                                          *                
*   Paramters:   None                                                                    *
*                                                                                            *
*   Returns:     None                                                                      *        
*                                                                                          *        
****************************************************************************************/
void  updateSpeedrefFromVR()
{
    q15_t  temp;
    q15_t  refTemp;
    
    if (motor.runState != M_SPEEDLOOP)
        return;
    
    refTemp = adcRawData[ADC_VR_CHAN] << 3;
    refTemp = Cymc_GFL_Mul_Q15(refTemp,_FQ(1.1));

    temp    = refTemp - motor.speedRef;
    
    if (temp < 0)
        temp = -temp;
    if (temp > MIN_SPEED_STEP)
        motor.speedRef = refTemp;
    temp = motor.speedRef - rampUp.output;
    
    if (temp < 0)
       temp = -temp;          // flip from negative to positive

    if (temp > SLOW_REGULATION)
      {
        rampUp.delayPrescaler = SLOW_RAMPUP_DELAY;
      }
     else if(temp > MIDDLE_REGULATION)
      {
        rampUp.delayPrescaler = MIDDLE_RAMPUP_DELAY;
      }
     else if(temp > FAST_REGULATION)
      {
        rampUp.delayPrescaler = FAST_RAMPUP_DELAY;
      }
     else
      {
        rampUp.delayPrescaler = DEFAULT_RAMPUP_DELAY;
      }

    if (motor.speedRef > SPEEDREF_MAX)
        motor.speedRef = SPEEDREF_MAX;
        else if (motor.speedRef < SPEEDREF_MIN)
                motor.speedRef = SPEEDREF_MIN;
    
}


/****************************************************************************************
*                                                                                          *
*   Function:    phaseCurrentOverProtect                                                  *
*                                                                                          *
*   Description: phase current over protection                                             *
*                                                                                          *                
*   Paramters:   None                                                                    *
*                                                                                            *
*   Returns:     None                                                                      *        
*                                                                                          *        
****************************************************************************************/
void phaseCurrentOverProtect()
{
    if (phaseCurrentU > ALARMCURRENT  || phaseCurrentU < -ALARMCURRENT)
      {
        errState |= ERR_OVER_CURRENT;
        Cymc_MAL_MotorStop();
      }
    if (phaseCurrentV > ALARMCURRENT || phaseCurrentV < -ALARMCURRENT )
      {
        Cymc_MAL_MotorStop();
        errState |= ERR_OVER_CURRENT;
      }
}


/****************************************************************************************
*                                                                                      *
*   Function:    VbusProtect                                                           *
*                                                                                      *
*   Description: Under and Over Voltage protection                                     *
*                                                                                      *                
*   Paramters:   None                                                                  *
*                                                                                      *
*   Returns:     None                                                                  *        
*                                                                                      *        
****************************************************************************************/
void VbusProtect()
{
    uint16 vBus = 0;

        // Read in Vbus voltage.
        // VDD is in mV units, so divide by 1000 to convert to V units
    vBus = ((uint32) adcRawData[ADC_VBUS_CHAN]  * VDD * DC_BUS_SCALER) / (1000 * ADC_RESOLUTION); 

    if (vBus < VBUS_LOW)
      {
        Cymc_MAL_MotorStop();
        errState |= ERR_UNDER_VOLTAGE;
      }
     else if(vBus > VBUS_HIGH)
      {
        Cymc_MAL_MotorStop();
        errState |= ERR_OVER_VOLTAGE;
      }
}


/****************************************************************************************
*                                                                                          *
*   Function:    FOC_MainLoop_ISR                                                          *
*                                                                                          *
*   Description: PWM main ISR, implement base FOC flow                                    *
*                                                                                          *                
*   Paramters:   None                                                                    *
*                                                                                            *
*   Returns:     None                                                                      *        
*                                                                                          *        
****************************************************************************************/
CY_ISR(FOC_MainLoop_ISR)
{    
    /* system ticker, every count indicates 1ms */
    pwmCounter++;
    if (pwmCounter >= SEC_COUNTER)
      {
        pwmCounter = 0;
        secFlag = 1;
      }  
    
        /* Read ADC and Get phase current */
    Cymc_HAL_ADCReadSample();
    Cymc_BCL_GetADCOffset(adcRawData[ADC_IA_CHAN], adcRawData[ADC_IB_CHAN]);    
    
    phaseCurrentU  = adcRawData[ADC_IA_CHAN];
    phaseCurrentV  = ((adcOffsetU + adcOffsetV + adcOffsetV)) - adcRawData[ADC_IB_CHAN] - adcRawData[ADC_IA_CHAN];
    phaseCurrentU  = (phaseCurrentU - adcOffsetU);
    phaseCurrentV  = (phaseCurrentV - adcOffsetV);
    phaseCurrentOverProtect();
    phaseCurrentU0 = phaseCurrentU << 3;
    phaseCurrentV0 = phaseCurrentV << 3;
    
        /* Clark Transformation uvw->αβ        */
    clarkTrans.inU = phaseCurrentU0 << 2;  /* Phase U current.  */
    clarkTrans.inV = phaseCurrentV0 << 2;  /* Phase V current.  */    
    Cymc_BCL_Clark(&clarkTrans); 

        /* Speed Ramp Up */
    rampUp.RefSpeed = motor.speedRef;    
    Cymc_BCL_RampUp(&rampUp);
    if (rampUp.output < _FQ(0.2))
        motor.vqRef = (rampUp.output>>1) + _FQ(0.06);
        
        /*Switch to Speed Loop*/
    if (motor.runState == M_OPENLOOP)
      {
        pidId.iOut = 0;
        pidIq.iOut = motor.vqRef;
        if (rampUp.output >= motor.speedRef - SPEED_MIN_STEP)
          {
            motor.runState = M_SPEEDLOOP;
            if (motor.runState != M_SPEEDLOOP || motor.runState != M_CURRENTLOOP)
              {
                pidId.iOut = 0;
                pidIq.iOut = motor.vqRef;
              }
          }
      }

         /* Position Calculation via SMO observer */
    smoEst.Vbus     = _FQ(1.0);
    smoEst.speedRef = rampUp.output;
    smoEst.volAlpha = invParkTrans.outAlpha;
    smoEst.volBeta  = invClarkTrans.InBeta;
    smoEst.curAlpha = clarkTrans.outAlpha;
    smoEst.curBeta  = clarkTrans.outBeta;
    Cymc_ACL_SMOCal(&smoEst);    

        /*angle given*/
    if (motor.runState == M_SPEEDLOOP)
      {
        curAngle = smoEst.Theta;
      }
     else
      {        
            /* Angle Generator For Open Loop */
        angleGen.speed = rampUp.output;
        Cymc_BCL_AngleGenerator (&angleGen);
    
        curAngle = angleGen.angle;
        pidSpeed.iOut = 0;
      }
    
       /* Speed Calculation */
    if (motor.runState != M_STOP)
      {
        speedEst.curTheta = curAngle;
        Cymc_BCL_SpeedCal (&speedEst);
        motor.speedRPM   = speedEst.estimatedSpeedRpm;

            /* speed value cannot be negative */
        if (motor.speedRPM < 0)
            motor.speedRPM = 0;
        if ((uint16)motor.speedRPM > SPEED_RPM_MAX)
            motor.speedRPM = SPEED_RPM_MAX;
      }
     else
      {
        speedEst.estimatedSpeed = 0;
        motor.speedRPM = 0;
      }
    
       /* Park Transformation αβ->dq        */
    parkTrans.inAlpha = clarkTrans.outAlpha;
    parkTrans.inBeta  = clarkTrans.outBeta;
    Cymc_GFL_SinCosCal_Q15 (curAngle, &parkTrans.inSin,  &parkTrans.inCos);
    Cymc_BCL_Park (&parkTrans);
    
       /* Speed PID Controller        */
    if (speedLoopCount == speedLoopPrescaler)
      {
        pidSpeed.ref = rampUp.output; /* motor.speedRef; */
        pidSpeed.fbk = speedEst.estimatedSpeed;
        Cymc_GFL_PICal (&pidSpeed);
        speedLoopCount    = 0;
        updateSpeedrefFromVR();
      }
    speedLoopCount++; 
    
      /* Iq PID Controller        */
    if(motor.runState == M_SPEEDLOOP)
      {    
        pidIq.ref = pidSpeed.out;
      }
     else
      {
        pidIq.ref = motor.vqRef;
      }
    pidIq.fbk = parkTrans.outQ;
    Cymc_GFL_PICal(&pidIq);
    
       /* Id PID Controller        */
    pidId.ref = motor.vdRef; 
    pidId.fbk = parkTrans.outD;
    Cymc_GFL_PICal(&pidId);
    
       /* Inverse Park Transformation dq->αβ    */
    if(motor.runState == M_OPENLOOP || motor.runState == M_STOP)
      {
        invParkTrans.inD = motor.vdRef;
        invParkTrans.inQ = motor.vqRef;
      }
     else 
      {
        invParkTrans.inD = pidId.out;
        invParkTrans.inQ = pidIq.out;
      }
    invParkTrans.inSin = parkTrans.inSin;
    invParkTrans.inCos = parkTrans.inCos;
    Cymc_BCL_InvPark(&invParkTrans);
    
       /* Inverse Clark Transformation αβ->uvw        */
    invClarkTrans.InAlpha = invParkTrans.outAlpha;
    invClarkTrans.InBeta  = invParkTrans.outBeta;
    Cymc_BCL_InvClark(&invClarkTrans);
    
       /* SVPWM Calculation            */
    svpwmCal.Uu = invClarkTrans.outU;
    svpwmCal.Uv = invClarkTrans.outV;
    svpwmCal.Uw = invClarkTrans.outW;
    Cymc_BCL_SVPWM(&svpwmCal);
    
        /* Update PWM Duty        */
    pwmCmp[0] = svpwmCal.CmpU;
    pwmCmp[1] = svpwmCal.CmpV;
    pwmCmp[2] = svpwmCal.CmpW;
    Cymc_HAL_PWMOutputUpdate();
}


/****************************************************************************************
*                                          ISR                                          *
*   Function:    Ibus_Over_ISR                                                          *
*                                                                                       *
*   Description: comprator ISR     for bus current over protection                      *
*                                                                                       *                
*   Paramters:   None                                                                   *
*                                                                                       *
*   Returns:     None                                                                   *        
*                                                                                       *        
****************************************************************************************/
    uint32   IsrTimes = 0;

CY_ISR(Ibus_Over_ISR)
{
    LPComp_IbusPt_ClearInterrupt (LPComp_IbusPt_INTR);

    IsrTimes++;

    if (LPComp_IbusPt_GetCompare())
      {
        errState |= ERR_OVER_CURRENT;
        Cymc_MAL_MotorStop();
        LPComp_IbusPt_ClearInterrupt (LPComp_IbusPt_INTR);
      }

    LPComp_IbusPt_ClearInterrupt (LPComp_IbusPt_INTR);
}

