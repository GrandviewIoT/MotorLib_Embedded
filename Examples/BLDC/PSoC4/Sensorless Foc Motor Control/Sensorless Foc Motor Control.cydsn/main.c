/*******************************************************************************
*
* Filename:             main.c
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
#include "MotorLib_Api.h"            // pull in MotorLib common definitions
#include "Board_Support_Api.h"       // pull in defs for Board Support

void  main ()
{
        /*====================================================================*
         * motor initialization                                               *
         *====================================================================*/
         /* Opamp Init */
    Opamp_A_Start();
    Opamp_B_Start();    

         /* SAR ADC Init */
    Cymc_HAL_ADCStart();    

         /* TCPWM Init */
    Cymc_HAL_PWMOutputDisable();
    Cymc_HAL_PWMStart();    

         /* Motor Paramters Init */
    Cymc_MAL_MotorInit();

         /* PWM Main ISR Init    */
    PWM_MainLoop_ISR_StartEx (FOC_MainLoop_ISR);

        /*====================================================================*
         * protection initialization                                          *
         *====================================================================*/
        /* Lpcomp and IDAC Init For Bus Current Protection */
    LPComp_IbusPt_Start();
    IDAC_IbusPt_Start();
    IDAC_IbusPt_SetValue (0x80);
    isr_Ibus_Over_StartEx (Ibus_Over_ISR);    

        /*====================================================================*
         * user interface initialization                                      *
         *====================================================================*/
         /* default LED display if OFF */
    LED2_Write(LED_OFF);

        /* UART Init */
    UART_BCP_Start();
    
        /* delay for a while to make DCbus voltage stable, otherwise
        ** it may trigger an under voltage fault */
    board_delay_ms (100);

    board_enable_global_interrupts();  // enable global interrupts
    
    for (;;)
      {         
        VbusProtect();     /* check if Vbus error happens */
        if (errState)
          {
                /* stop motor if it is still running */
            if (motor.runState != M_STOP)
               Cymc_MAL_MotorStop();

                /* blink LED in 1Hz if error happens */                
            if (secFlag)
              {
                secFlag = 0;
                LED2_Write (LED2_Read() ^ 0x01);
              }
          }
         else
          {            
            key_poll();        
            if (keyState[SW2_IDX] == KEY_RELEASED)
              {        
                    /* start the motor if it stops */
                if (motor.runState == M_STOP)
                  {
                    if (motor.runState == M_STOP && errState == 0)
                      {
                        Cymc_MAL_MotorStart();
                        LED2_Write (LED_ON);
                      }
                  }
                 else
                  {
                    Cymc_MAL_MotorStop();
                    LED2_Write(LED_OFF);
                  }
                keyState[SW2_IDX] = KEY_IDLE;
              }
            
            if (motor.runState != M_STOP)
              {
                  /* only send data out when motor is running */            
                BCPPoll();            
              }
          }    
      }
}

/* [] END OF FILE */
