
//******************************************************************************
//
//                     BLDC_motor_ctl_ISRs_STM32F4.c
//
// Motor control code Interrupt Service Handlers (ADC, GPIOs, ...) for MSP432.
// History:
//   05/11/16 -Carved off as separate module. Duquaine
//******************************************************************************

#include "MotorLib_Api.h"                       // pull in common definitions

void  DMA2_Stream0_IRQHandler (void);           // ADC DMA ISR - all channels
void  ADC1_DMA_IRQHandler (void);               // ADC only (no-DMA) ISR

extern CALLBACK_RTN   _g_ADC_callback_ptr;      // in MotorLib_Basic.c

   uint32_t  encoder_LEFT_rupts    = 0;  // count of interrupts this interval
   uint32_t  encoder_RIGHT_rupts   = 0;  // counts are reset at each motor_compute_rps_speed()
                                         // call, which occurs every 50 ms

                 //----------------------------
                 //  Encoder related I/O vars
                 //----------------------------
    uint32_t  encoder_Ph_A_rupts       = 0;
    uint32_t  encoder_Ph_B_RIGHT_rupts = 0; // count of interrupts this interval
    uint32_t  encoder_Ph_C_LEFT_rupts  = 0; // counts are reset at each motor_compute_rps_speed()
                                            // call, which occurs every 50 ms

// int       g_adc_current_sense_A = 0;  // L6206 "A" side  current sense
// int       g_adc_current_sense_B = 0;  // L6206 "B" side  current sense
// int       g_adc_speed_raw_value = 0;  // speed value from ADC input

                 //----------------------------------------------------------
                 //                      ADC  Value
                 // local cache of ADC values. Acts like a ping-pong buffer,
                 // so main() can ask for data on demkand, rather than this
                 // logic asynchronously overriding the data in middle of main() usage
                 //----------------------------------------------------------
   uint32_t   g_adc_Status;
   uint16_t   ADC0_Seq0_Conversion_Values[8]; //ADC values from latest conversion
                                                // Saved values (ping-pong)
    uint16_t  _g_adc_BEMF_A;                    // raw ADC A14 value
    uint16_t  _g_adc_BEMF_B;                    // raw ADC A13 value
    uint16_t  _g_adc_BEMF_C;                    // raw ADC A11 value
    uint16_t  _g_adc_VBUS;                      // raw ADC A9  value
    uint16_t  _g_adc_current_sense_A;           // raw ADC A8  value
    uint16_t  _g_adc_current_sense_B;           // raw ADC A6  value
    uint16_t  _g_adc_current_sense_C;           // raw ADC A1  value
    uint16_t  _g_adc_Speed_Control_value;       // raw ADC value

    uint32_t  _g_motor_port_rupt_flags = 0;     // GPIO Port 3 interrupt flags
    uint32_t  _g_hall_port_rupt_flags  = 0;     // GPIO Port 4 interrupt flags
    int       _g_hall_index            = 0;     // resulting Hall Index


    uint32_t  _g_ADC_interrupts          = 0;   // DEBUG COUNTERs
    uint32_t  _g_GPIO_Fault_interrupts   = 0;
    uint32_t  _g_GPIO_Encoder_interrupts = 0;
    uint32_t  _g_commute_timer_rupts     = 0;

   extern ADC_HandleTypeDef    ADC_InitStruct;


//******************************************************************************
//                            ADC  DMA  ISR
//
// We get our ADC results when ssociated ADC's associated DMA signals completion
//******************************************************************************
void  DMA2_Stream0_IRQHandler (void)
{
_g_ADC_interrupts++;                                  // UPDATE DEBUG COUNTERS

        // Copy ADC sensor values to Motor globals
    _g_adc_BEMF_A          = ADC0_Seq0_Conversion_Values [0]; // BEMF phase A
    _g_adc_BEMF_B          = ADC0_Seq0_Conversion_Values [1]; // BEMF phase B
    _g_adc_BEMF_C          = ADC0_Seq0_Conversion_Values [2]; // BEMF phase C
    _g_adc_VBUS            = ADC0_Seq0_Conversion_Values [3]; // VBUS
    _g_adc_current_sense_A = ADC0_Seq0_Conversion_Values [4]; // current Phase A
    _g_adc_current_sense_B = ADC0_Seq0_Conversion_Values [5]; // current Phase B
    _g_adc_current_sense_C = ADC0_Seq0_Conversion_Values [6]; // current Phase C
    _g_adc_Speed_Control_value = ADC0_Seq0_Conversion_Values [7]; // Board potentiometer speed control

    HAL_DMA_IRQHandler (ADC_InitStruct.DMA_Handle);  // Call post rupt cleanup
}


//******************************************************************************
//                           ENCODER   GPIOs   ISR
//
//                   GPIO Pin   Interrupts  ISR                   Hall Encoders
//
//      Hall encoders are attached to PA8 and PA9         (Arduino D7 and D8)
//******************************************************************************
void  Encoder_GPIO_EXTI_Callback (uint16_t GPIO_Pin);
void  L6230_FlagInterruptHandler(void);

/**
  * EXTI line detection callbacks
  * GPIO_Pin: Specifies the pins connected EXTI 9_5 lines
  *
  * Is invoked by STM32's HAL GPIO interrupt handler via HAL_GPIO_EXTI_Callback()
  * in MotorLib_LL_L6230.c
  */
void  Encoder_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
      // 07/29/16 - This is being called instead of the ISR in
  if (GPIO_Pin == HALL_ENCODER_A_PIN)
     {
       encoder_LEFT_rupts++;     // count the left wheel encoder interrupts
     }
  if (GPIO_Pin == HALL_ENCODER_B_PIN)
     {
       encoder_RIGHT_rupts++;    // count the right wheel encoder interrupts
     }

// edded 06/29/16
    if (GPIO_Pin == GPIO_PIN_10)                    // Fault_Flag  pin PA10 / D2
     {
       L6230_FlagInterruptHandler();
     }
}

//*****************************************************************************
