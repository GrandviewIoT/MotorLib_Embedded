
/*******************************************************************************
  *
  *                                 stm32f4xx_hal_msp.c
  *
  *
  * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/
  * @author  IPC Rennes
  * @version V1.6.0
  * @date    February 03, 2016
  * @brief   HAL MSP module.
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*******************************************************************************/

#include <stdint.h>                        // pull in standard int16_t, etc defs
#include <stdbool.h>
#include "MotorLib_Api.h"                  // pull in Motor Control API defs

//#include "main.h"

extern void BSP_MotorControl_StepClockHandler(uint8_t deviceId);
extern void BSP_MotorControl_FlagInterruptHandler(void);

/*****************************************************************************
  *
  *                       SPI MSP Initialization
  *
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param[in] hspi SPI handle pointer
  * @retval None
*****************************************************************************/

void  HAL_SPI_MspInit (SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  if(hspi->Instance == SPIx)
  {
       /*##-1- Enable peripherals and GPIO Clocks #############################*/
       /* Enable GPIO TX/RX clock */
    SPIx_SCK_GPIO_CLK_ENABLE();
    SPIx_MISO_GPIO_CLK_ENABLE();
    SPIx_MOSI_GPIO_CLK_ENABLE();
       /* Enable SPI clock */
    SPIx_CLK_ENABLE();

       /*##-2- Configure peripheral GPIO ######################################*/
       /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_MEDIUM;
    GPIO_InitStruct.Alternate = SPIx_SCK_AF;

    HAL_GPIO_Init (SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

       /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_MISO_PIN;
    GPIO_InitStruct.Alternate = SPIx_MISO_AF;
    HAL_GPIO_Init (SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

       /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_MOSI_PIN;
    GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
    HAL_GPIO_Init (SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);
  }
}

/**
  * @brief SPI MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO configuration to its default state
  * @param[in] hspi SPI handle pointer
  * @retval None
  */
void  HAL_SPI_MspDeInit (SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance == SPIx)
  {
       /*##-1- Reset peripherals ##########################################*/
    SPIx_FORCE_RESET();
    SPIx_RELEASE_RESET();

       /*##-2- Disable peripherals and GPIO Clocks ########################*/
       /* Configure SPI SCK as alternate function  */
    HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
       /* Configure SPI MISO as alternate function  */
    HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN);
       /* Configure SPI MOSI as alternate function  */
    HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);
  }
}


/*****************************************************************************
  *
  *                            PWM  Initialization
  *
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
*****************************************************************************/
void  HAL_TIM_PWM_MspInit (TIM_HandleTypeDef* htim_pwm)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if (htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1)
   {
       /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1_CLCK_ENABLE();

       /* GPIO configuration */
    GPIO_InitStruct.Pin       = BSP_MOTOR_CONTROL_BOARD_PWM_1_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM1;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_1_PORT, &GPIO_InitStruct);

       /* Set Interrupt Group Priority of Timer Interrupt*/
    HAL_NVIC_SetPriority(BSP_MOTOR_CONTROL_BOARD_PWM1_IRQn, 4, 0);

       /* Enable the timer global Interrupt */
    HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_PWM1_IRQn);
   }
  else if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2)
   {
       /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2_CLCK_ENABLE();

       /* GPIO configuration */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM2;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_2_PORT, &GPIO_InitStruct);

       /* Set Interrupt Group Priority of Timer Interrupt*/
    HAL_NVIC_SetPriority(BSP_MOTOR_CONTROL_BOARD_PWM2_IRQn, 4, 0);

       /* Enable the timer2 global Interrupt */
    HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_PWM2_IRQn);

   }
  else if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3)
   {
       /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3_CLCK_ENABLE();

    /* GPIO configuration */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT, &GPIO_InitStruct);

       /* Set Interrupt Group Priority of Timer Interrupt*/
    HAL_NVIC_SetPriority(BSP_MOTOR_CONTROL_BOARD_PWM3_IRQn, 3, 0);

       /* Enable the timer global Interrupt */
    HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_PWM3_IRQn);
   }
}

/**
  * @brief PWM MSP De-Initialization
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void  HAL_TIM_PWM_MspDeInit (TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1)
  {
       /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1_CLCK_DISABLE();

       /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PWM_1_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_1_PIN);

  }
  else if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2)
  {
       /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2_CLCK_DISABLE();

       /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PWM_2_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_2_PIN);

  }
  else if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3)
  {
       /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3_CLCK_DISABLE();

       /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN);
  }
}


/*****************************************************************************
  *
  *                             PWM  ISR  Callback
  *
  * @param[in] htim PWM handle pointer
  * @retval None
*****************************************************************************/
    extern   MOTOR_BLOCK   motor_blk [];
 
void  HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
{
    MOTOR_BLOCK   *mtr_blk;

  if ((htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1) && (htim->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM1))
    { 
      mtr_blk = &motor_blk[0];
      if (mtr_blk->mtr_motion_state != INACTIVE)
         {
           BSP_MotorControl_StepClockHandler (0);
         }
    }
  if ((htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2) && (htim->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM2))
   {
     mtr_blk = &motor_blk[1];
     if (mtr_blk->mtr_motion_state != INACTIVE)
        {
          BSP_MotorControl_StepClockHandler (1);
        }
   }

  if ((htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3) && (htim->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM3))
   {
     mtr_blk = &motor_blk[2];
     HAL_GPIO_TogglePin (BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN);
     if ((mtr_blk->mtr_motion_state != INACTIVE) &&
        (HAL_GPIO_ReadPin(BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN) == GPIO_PIN_SET))
        {
          BSP_MotorControl_StepClockHandler (2);
        }
   }
}


/*****************************************************************************
  *
  *                        External Line Callback  ISR          FLAG/FAULT pin
  *
  * @param[in] GPIO_Pin pin number
  * @retval None
*****************************************************************************/
void  HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BSP_MOTOR_CONTROL_BOARD_FLAG_PIN)
     {
       BSP_MotorControl_FlagInterruptHandler();
     }
 }

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
