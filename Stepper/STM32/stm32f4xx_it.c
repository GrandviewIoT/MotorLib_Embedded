/*******************************************************************************
  * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/stm32f4xx_it.c 
  * @author  IPC Rennes
  * @version V1.6.0
  * @date    February 03, 2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
*******************************************************************************/

#include <stdint.h>                        // pull in standard int16_t, etc defs
#include <stdbool.h>
#include "MotorLib_Api.h"                 // pull in Motor Control API defs

#include "stm32f4xx_it.h"
//#include "motorcontrol.h"

                //-------------------------------------------------------------
                //                  Encoder support
                //-------------------------------------------------------------
   uint32_t  encoder_Ph_C_LEFT_rupts  = 0;  // count of interrupts this interval
   uint32_t  encoder_Ph_B_RIGHT_rupts = 0;  // counts are reset at each motor_compute_rps_speed()

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern TIM_HandleTypeDef hTimPwm1;
extern TIM_HandleTypeDef hTimPwm2;
extern TIM_HandleTypeDef hTimPwm3;

void  EXTI15_10_IRQHandler (void);        // ISRs
void  TIM2_IRQHandler (void);
void  TIM3_IRQHandler (void);
void  TIM4_IRQHandler (void);

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}


/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}


/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**************************************************************************
  *                             SYSTICK   ISR
  *
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
*************************************************************************/
void  SysTick_Handler (void)
{
    HAL_IncTick();
}


/*************************************************************************
  * @brief  This function handles interrupt for External lines 10 to 15
  * @param  None
  * @retval None                               FAULT / FLAG pin interrupt
*************************************************************************/
void  EXTI15_10_IRQHandler (void)
{
    HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_10);
}


/**************************************************************************
  * @brief  This function handles TIM2 PWM interrupt request.
  * @param  None                                        CCR: TIM_CHANNEL_2
  * @retval None                                         Motor 2  /  PWM2
*************************************************************************/
void  TIM2_IRQHandler (void)
{
    HAL_TIM_IRQHandler (&hTimPwm2);
}

/**************************************************************************
  * @brief  This function handles TIM3 PWM interrupt request.
  * @param  None                                        CCR: TIM_CHANNEL_2
  * @retval None                                         Motor 1  /  PWM1
*************************************************************************/
void  TIM3_IRQHandler (void)
{
    HAL_TIM_IRQHandler (&hTimPwm1);
}

/**************************************************************************
  * @brief  This function handles TIM4 PWM interrupt request.
  * @param  None                                        CCR: TIM_CHANNEL_3
  * @retval None                                         Motor 3  /  PWM3
*************************************************************************/
void  TIM4_IRQHandler (void)
{
    HAL_TIM_IRQHandler (&hTimPwm3);
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
