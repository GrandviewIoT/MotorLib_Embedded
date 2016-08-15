
//******************************************************************************
//
//                     BDC_motor_ctl_ISRs_STM32F4.c
//
// Motor control code Interrupt Service Handlers (ADC, GPIOs, ...) for MSP432.
// History:
//   05/11/16 -Carved off as separate module. Duquaine
//******************************************************************************

#include "MotorLib_Api.h"                     // pull in common definitions

void  DMA2_Stream0_IRQHandler (void);          // ADC DMA ISR - all channels
void  ADC1_DMA_IRQHandler (void);              // ADC only (no-DMA) ISR


   int       g_adc_current_sense_A = 0;  // L6206 "A" side  current sense
   int       g_adc_current_sense_B = 0;  // L6206 "B" side  current sense
   int       g_adc_speed_raw_value = 0;  // speed value from ADC input

   uint32_t  encoder_Ph_C_LEFT_rupts  = 0;  // count of interrupts this interval
   uint32_t  encoder_Ph_B_RIGHT_rupts = 0;  // counts are reset at each motor_compute_rps_speed()
                                         // call, which occurs every 50 ms
   uint32_t  g_adc_Status;
   uint16_t  ADC0_Seq0_Conversion_Values[4]; //ADC values from latest conversion

   extern ADC_HandleTypeDef    ADC_InitStruct;


//******************************************************************************
//                            ADC  DMA  ISR
//
// We get our ADC results when ssociated ADC's associated DMA signals completion
//******************************************************************************
void  DMA2_Stream0_IRQHandler (void)
{
        // Copy ADC sensor values to Motor globals
    g_adc_current_sense_A = ADC0_Seq0_Conversion_Values [0]; // L6206 "A" side
    g_adc_current_sense_B = ADC0_Seq0_Conversion_Values [1]; // L6206 "B" side
    g_adc_speed_raw_value = ADC0_Seq0_Conversion_Values [2]; // Breadboard potentiometer speed control

    HAL_DMA_IRQHandler (ADC_InitStruct.DMA_Handle);  // Call post rupt cleanup
}


//******************************************************************************
//                           ENCODER   GPIOs   ISR
//
//                   GPIO Pin   Interrupts  ISR                   Hall Encoders
//
//      Hall encoders are attached to PA8 and PA9         (Arduino D7 and D8)
//******************************************************************************
/**
  * EXTI line detection callbacks
  * GPIO_Pin: Specifies the pins connected EXTI 9_5 lines
  *
  * Is invoked by STM32's HAL GPIO interrupt handler
  */
void  HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == HALL_ENCODER_LEFT_PIN)
     {
       encoder_Ph_C_LEFT_rupts++;    // count the left wheel encoder interrupts
     }
  if (GPIO_Pin == HALL_ENCODER_RIGHT_PIN)
     {
       encoder_Ph_B_RIGHT_rupts++;   // count the right wheel encoder interrupts
     }
}

//*****************************************************************************
