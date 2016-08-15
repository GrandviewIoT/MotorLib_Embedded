/*******************************************************************************
*
* Filename:             userinterface.c
* Owner :               Rod Wang (rowa@cypress.com)
*
* Version:              V1.0 
* Description:          This file contains function to detect whether start-stop 
*                       swith was pressed. 
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
#include "motor.h"
#include "getvalue.h"
#include "userinterface.h"


/******************************************************************************
 * Global variables definition
 * ----------------------------------------------------------------------------
 * These varialbes should be populated to other modules. Header file contain 
 * the extern statement for these variables.
 ******************************************************************************/ 

/* press status for button SW2, default is BUTTON_OFF */
uint8 btnStatus[BTN_COUNT];

/******************************************************************************
 * Local Macro definition
 * ----------------------------------------------------------------------------
 * These Macro are only used in this module. These Macro should not be populated
 * to other modules.
 ******************************************************************************/
#define BTN_LOW_LEVEL                       (uint8)(0x00)
#define BTN_HIHG_LEVEL                      (uint8)(0x01)

/* this macro define the initial level when starting to detect button pressing */
#define BTN_OFF_LEVEL                       BTN_HIHG_LEVEL
/* this macro defines the desired level when button is pressed */
#define BTN_ON_LEVEL                        BTN_LOW_LEVEL

/* debounce count for button glitch filter */
#define BTN_GLITCH_FILTER_ACTIVE_CNT        (uint8)(15)
#define BTN_GLITCH_FILTER_DISCARD_CNT       (uint8)(5)
#define BTN_GLITCH_FILTER_INIT_CNT          (uint8)(10)

/******************************************************************************
 * Local variables definition
 * ----------------------------------------------------------------------------
 * These varialbes are only used in this module. These varialbes should not be 
 * populated to other modules.
 ******************************************************************************/
typedef struct _Btn_Status_T
{
    uint8   preBtnStatus;                   /* varialbe to store last status of button level  */
    uint8   glitchFilter;                   /* glitch filter counter for button pressing */
    uint8   btnIsDetectFlag;                 /* flag to store button pressing event */ 
}Btn_Status_T;

static Btn_Status_T btnArray[BTN_COUNT];

static uint8 bcpTxBuffer[32];

/*******************************************************************************
* Function Name: Button_Init
********************************************************************************
* Summary:
*   Initialize button status structure to detect pressing event
*
* Parameters:  
*   void
*
* Return: 
*   void
*
*******************************************************************************/
void ButtonInit(void)
{
    uint8 i = 0;
    
    for(i = 0; i < BTN_COUNT; i++)
    {
        btnArray[i].preBtnStatus = BTN_OFF_LEVEL;
        btnArray[i].glitchFilter = BTN_GLITCH_FILTER_INIT_CNT;
        btnArray[i].btnIsDetectFlag = FALSE;
    }
}

/*******************************************************************************
* Function Name: ButtonPressDetect
********************************************************************************
* Summary:
*   Detect button status for pressing event
*
* Parameters:  
*   void
*
* Return: 
*   void
*
*******************************************************************************/
void ButtonPressDetect(uint8 btnStatus, Btn_Status_T* btnArrayPtr, uint8* btnStatusPtr)
{
    uint8 curBtnStatus = btnStatus;    /* get current button level status */
    if(btnArrayPtr->btnIsDetectFlag == FALSE)          
    {
        /* detect button pressing event */
        btnArrayPtr->btnIsDetectFlag = ((curBtnStatus == BTN_ON_LEVEL) && 
                                       (curBtnStatus ^ btnArrayPtr->preBtnStatus)) ? TRUE : FALSE;
    }
    else
    {
        if(curBtnStatus == BTN_ON_LEVEL)   /* button keeps in desired level status */
        {            
            /*  detect if glitch filter counter value is larger than pre-defined threshold */
            if(btnArrayPtr->glitchFilter > BTN_GLITCH_FILTER_ACTIVE_CNT)
            {
                /* succeed in button pressing detection, invert button ON/OFF status */
                *btnStatusPtr = *btnStatusPtr ^ BUTTON_ON;
                /* reset glitch filter coutner */
                btnArrayPtr->glitchFilter = BTN_GLITCH_FILTER_INIT_CNT;
                /* clear detection flag */
                btnArrayPtr->btnIsDetectFlag = FALSE;
            }
            else
            {
                /* increase glitch filter coutner */
                btnArrayPtr->glitchFilter++; 
            }
        }
        else
        {
            if(btnArrayPtr->glitchFilter < BTN_GLITCH_FILTER_DISCARD_CNT)
            {
                /* keep buttons status unchanged, reset glitch filter coutner */
                btnArrayPtr->glitchFilter = BTN_GLITCH_FILTER_INIT_CNT;
                /* clear detection flag */
                btnArrayPtr->btnIsDetectFlag = FALSE;
            }
            else
            {
                 /* decrease glitch filter coutner */
                btnArrayPtr->glitchFilter--; 
            }
        }
    }
    /* update last button status with current status */
    btnArrayPtr->preBtnStatus = curBtnStatus;
}

/*******************************************************************************
* Function Name: ButtonProcess
********************************************************************************
* Summary:   
* This funtion detects the press action of button.
*
* Parameters: None 
*  
* Return: None
*
*******************************************************************************/
void ButtonProcess(void)
{
    ButtonPressDetect(SW2_Read(), &btnArray[0], &btnStatus[0]);
    if(btnStatus[0] == BUTTON_ON)
    {
        UI_Cmd.run = TRUE;    
        stateSys = STATUS_RUN;
    }
    else
    {
        UI_Cmd.run = FALSE;     
        stateSys = STATUS_STOP;
    }
}

/*******************************************************************************
* Function Name: UpdateStatusStart
********************************************************************************
* Summary:   
* This funtion initialize the motor, and ready to start.
*
* Parameters: None 
*  
* Return: None
*
*******************************************************************************/
void UpdateStatusInit(void)
{
    UI_Cmd.run = FALSE;
    stateSys = STATUS_STOP;    
}

/*******************************************************************************
* Function Name: UpdateStatusError
********************************************************************************
* Summary:   
* This funtion stop the motor and update error flag.
*
* Parameters: None 
*  
* Return: None
*
*******************************************************************************/
void UpdateStatusError()
{    
    stateSys = STATUS_ERROR;    
}

/****************************************************************************************
*                                                                             
*   Function:    BCPPoll                                                        
*                                                                             
*   Description: BCPPoll routine                                                   
*             Format:  RX8 [h=55] @0speed @1speed [t=AA]                      
*   Paramters:   None                                                          
*                                                                                 
*   Returns:     None                                                                 
*                                                                                   
****************************************************************************************/

void BCPPoll()
{
    uint8 index = 0;
    
    if(UART_BCP_SpiUartGetTxBufferSize())
       return;
    
    /* package header */
    bcpTxBuffer[index++] = 0x55;
    
    /* construct BCP data pacakge with speed value, MSB first */    
    
    /* current measured speed */
    bcpTxBuffer[index++] = (uint8)((UI_Data.speedRpm & 0xFF00) >> 8);
    bcpTxBuffer[index++] = (uint8)(UI_Data.speedRpm & 0x00FF); 
    /* speed reference */
    bcpTxBuffer[index++] = (uint8)((UI_Data.speedRpmRef & 0xFF00) >> 8);
    bcpTxBuffer[index++] = (uint8)(UI_Data.speedRpmRef & 0x00FF);   
   
    
    /* package tail */
    bcpTxBuffer[index++] = 0xAA;
    
    UART_BCP_SpiUartPutArray(bcpTxBuffer, index);
}
/* [] END OF FILE */

