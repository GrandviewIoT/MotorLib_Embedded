/*******************************************************************************
*
* Filename:             interface.c
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
#include "interface.h"
#include "foc.h"


#define LEVEL_HIGH      1
#define LEVEL_LOW       0

static uint8 keyCur[BUTTON_COUNT] = {0};
static uint8 keyPrev[BUTTON_COUNT] = {0};
uint8 keyState[BUTTON_COUNT] = {0};
static uint8 bcpTxBuffer[32];
/****************************************************************************************
*                                                                                          *
*   Function:    key_poll                                                                  *
*                                                                                          *
*   Description: key poll routine                                                       *
*                                                                                          *                
*   Paramters:   None                                                                    *
*                                                                                            *
*   Returns:     None                                                                      *        
*                                                                                          *        
****************************************************************************************/
void key_poll()
{
    uint8 i = 0;
    /* read button status */
    keyCur[SW2_IDX] = SW2_Read();
    
    for(i = 0; i < BUTTON_COUNT; i++)
    {       
        if(keyPrev[i] == LEVEL_LOW && keyCur[i] == LEVEL_HIGH)
        {
//            keyCur[i] = SW2_Read();
            keyState[i] = KEY_PRESSED;
            CyDelay(2);
        }
        if(keyPrev[i] == LEVEL_HIGH && keyCur[0] == LEVEL_LOW && keyState[0] == KEY_PRESSED)    
        {
            keyState[i] = KEY_RELEASED;            
        }
        
        keyPrev[i] = keyCur[i]; 
    } 
}
/****************************************************************************************
*                                                                                          *
*   Function:    BCPPoll                                                                  *
*                                                                                          *
*   Description: BCPPoll routine                                                           *
*                 Format:  RX8 [h=55] @0speed @1speed @0currentU @1currentU [t=AA]       *                
*   Paramters:   None                                                                    *
*                                                                                            *
*   Returns:     None                                                                      *        
*                                                                                          *        
****************************************************************************************/

void BCPPoll()
{
    uint8 index = 0;
    int32 temp;
    if(UART_BCP_SpiUartGetTxBufferSize())
        return;
    bcpTxBuffer[index++] = 0x55;
    temp = motor.speedRPM;
    bcpTxBuffer[index++] = (temp>>8)&0xFF;
    bcpTxBuffer[index++] = (temp)&0xFF;
    temp = Cymc_GFL_Mul_Q15(motor.speedRef,motor.ratedSpeedRPM);
    bcpTxBuffer[index++] = (temp>>8)&0xFF;
    bcpTxBuffer[index++] = (temp)&0xFF;
    bcpTxBuffer[index++] = 0xAA;
    UART_BCP_SpiUartPutArray(bcpTxBuffer, index);
    
}
/* [] END OF FILE */
