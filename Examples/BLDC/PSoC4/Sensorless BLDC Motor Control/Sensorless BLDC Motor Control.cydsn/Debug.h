/*******************************************************************************
*
* Filename:             Debug.h
* Owner :               Bob Hu (bobh@cypress.com)
*
* Version:              V1.0 
* Description:          Sensorless BLDC Motor Control
*                       Header file to define flags used in debug purpose, for example, 
*                       flag to disable speed PID close loop, etc.
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
#ifndef _DEBUG_H_
#define _DEBUG_H_
    
/******************************************************************************
 * Macro declaration   
 ******************************************************************************/                                 
/* macro definition for mask used in CtrlReg_Debug regisiter */
#define DEBUG_STATUS_LED_MASK                       (uint8)(0x01)
#define DEBUG_STATUS_LED_SEL_MASK                   (uint8)(0x01 << 1)    
#define DEBUG_COMMUTATE_PULSE_GEN_MASK              (uint8)(0x01 << 2)    
#define DEBUG_GENERAL_TP_MASK                       (uint8)(0x01 << 3)  
  
/* macro definition for debug purpose, set "1" to enable corresponding debug features */
    
/* flag to enable/disable speed close loop PID */
#define DEBUG_SPEED_CLOSE_ENABLE                    1   
/* flag to enable/disable pulse generation when commutation on P0.1 */
#define DEBUG_COMMUTATE_PULSE_GEN_ENABLE            0    
/* flag to enable/disable zero-crossing detection signal output */
#define DEBUG_ZC_DETECTION_OUTPUT_ENABLE            0    
/* flag to enable/disable general TP output */
#define DEBUG_GENERAL_TP_OUTPUT_ENABLE              0   

/* flag to enable/disable auto-clear error state , disable for release version */
#define DEBUG_AUTO_CLEAR_ERROR_STATE_ENABLE         0

#endif  /* _DEBUG_H_ */    
/* [] END OF FILE */
