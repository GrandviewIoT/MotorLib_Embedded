
//******************************************************************************
//
//                            BLDC_motor_ctl_ISRs_MSP432DRV8848.c
//
// Motor control code Interrupt Service Handlers (ADC, GPIOs, ...) for MSP432.
//
// History:
//   05/16/16 - Created. Duquaine
//******************************************************************************

#include "BLDC_motor_ctl_api.h"                   // pull in common definitions


    CALLBACK_RTN   _g_ADC_callback_ptr = 0L;

    int       g_adc_current_VREF_raw_value = 0; // DRV8848 value for VREF from ADC
///int        g_adc_speed_raw_value = 0;        // speed value from ADC input


                 //----------------------------
                 //  Encoder related I/O vars
                 //----------------------------
    uint32_t  encoder_Ph_A_rupts       = 0;
    uint32_t  encoder_Ph_B_RIGHT_rupts = 0; // count of interrupts this interval
    uint32_t  encoder_Ph_C_LEFT_rupts  = 0; // counts are reset at each motor_compute_rps_speed()
                                            // call, which occurs every 50 ms

                 //----------------------------------------------------------
                 // local cache of ADC values. Acts like a ping-pong buffer,
                 // so main() can ask for data on demkand, rather than this
                 // logic asynchronously overriding the data in middle of main() usage
                 //----------------------------------------------------------
    uint16_t  _g_adc_Phase_A_Voltage_Sense;     // raw ADC A14 value
    uint16_t  _g_adc_Phase_B_Voltage_Sense;     // raw ADC A13 value
    uint16_t  _g_adc_Phase_C_Voltage_Sense;     // raw ADC A11 value
    uint16_t  _g_adc_Bus_Voltage_Sense;         // raw ADC A9  value
    uint16_t  _g_adc_Phase_A_Current_Sense;     // raw ADC A8  value
    uint16_t  _g_adc_Phase_B_Current_Sense;     // raw ADC A6  value
    uint16_t  _g_adc_Phase_C_Current_Sense;     // raw ADC A1  value
    uint16_t  _g_adc_Speed_Control_Sense;       // raw ADC value

    uint32_t  _g_motor_port_rupt_flags = 0;     // GPIO Port 3 interrupt flags
    uint32_t  _g_hall_port_rupt_flags  = 0;     // GPIO Port 4 interrupt flags
    int       _g_hall_index            = 0;     // resulting Hall Index

                 //-----------------------
                 //  SPI related I/O data
                 //-----------------------
    uint16_t  _g_spi_io_operation      = 0;
    uint16_t  _g_spi_io_status         = 0;   // 0 = no I/O, 1 = Busy I/O, 2 = I/O Complete
    uint16_t  _g_spi_length            = 0;
    char      *_g_spi_write_buf        = 0L;
    char      *_g_spi_read_buf         = 0L;


    uint32_t  _g_ADC_interrupts          = 0;   // DEBUG COUNTERs
    uint32_t  _g_GPIO_Fault_interrupts   = 0;
    uint32_t  _g_GPIO_Encoder_interrupts = 0;
    uint32_t  _g_commute_timer_rupts     = 0;
    uint32_t  _g_spi_RX_rupts            = 0;
    uint32_t  _g_spi_TX_rupts            = 0;


//******************************************************************************
//                            ADC    ISR
// 
//******************************************************************************
void  ADC14_IRQHandler (void)
{
    _g_adc_Phase_A_Voltage_Sense = ADC14MEM0;   // save ADC conversion values
    _g_adc_Phase_B_Voltage_Sense = ADC14MEM1;
    _g_adc_Phase_C_Voltage_Sense = ADC14MEM2;
    _g_adc_Bus_Voltage_Sense     = ADC14MEM3;
    _g_adc_Phase_A_Current_Sense = ADC14MEM4;
    _g_adc_Phase_B_Current_Sense = ADC14MEM5;
    _g_adc_Phase_C_Current_Sense = ADC14MEM6;

#if defined(SPEED_CONTROL_ADC_CHANNEL)
          // Breadboard potentiometer speed control
    _g_adc_Speed_Control_Sense   = ADC14MEM7;
    if (_g_ADC_callback_ptr != 0L)
       {        // invoke user-defined callback to pass back Speed ADC value
          (*_g_ADC_callback_ptr) (_g_adc_Speed_Control_Sense);
       }
#endif

    _g_ADC_interrupts++;             // Update DEBUG counter

}


//******************************************************************************
//                           nFAULT   GPIOs   ISR
//
//                   GPIO Pin   Interrupts  ISR                   nFAULT
//
//            nFAULT is attached to P3.2 (on J1-3)
//******************************************************************************
void  PORT3_IRQHandler (void)
{
    _g_GPIO_Fault_interrupts++;            // Update DEBUG counter

    _g_motor_port_rupt_flags = P3IV;       // save copy of Interrupt flags DEBUG

    if (_g_motor_port_rupt_flags & P3IV__P3IFG2)   // interrupt on P3.2  nFAULT
       {     // We got a FAULT - immediately shutoff motor and DRV8305 !!!
         _g_mtrdrvr->motor_fault_shutdown();
       }
}


//******************************************************************************
//                           ENCODER   GPIOs   ISR
//
//                   GPIO Pin   Interrupts  ISR                   Hall Encoders
//
//       Hall Sensor   -   Phase A   J1-5     P4.1  GPIO rupt
//       Hall Sensor Right Phase B   J1-6     P4.3  GPIO rupt
//       Hall Sensor Left  Phase C   J1-8     P4.6  GPIO rupt
//******************************************************************************
void  PORT4_IRQHandler (void)
{
   // 04/05/16 - WORKS !

    _g_GPIO_Encoder_interrupts++;            // Update DEBUG counter

       // Context save interrupt flag before calling interrupt vector.
       // Reading interrupt vector generator will automatically clear IFG flag
//  _g_hall_port_rupt_flags = __even_in_range(P4IV, P4IV_16);    // MSP430 only
    _g_hall_port_rupt_flags = P4IV;       // save copy of Interrupt flags DEBUG

    _g_hall_index = 0;
    switch (_g_hall_port_rupt_flags)
      {
              // Vector  P4IV__NONE: // No Interrupt pending
          case P4IV__NONE:
              break;

              // Vector  P4IV__P4IFG1:               P4.1 - Phase A encoder
          case P4IV__P4IFG1:         // P4IV_2:
              encoder_Ph_A_rupts++; // count the right wheel encoder interrupts
              break;           // counts reset at next motor_compute_rps_speed()

              // Vector  P4IV__P4IFG3:               P4.3 - Ph B / RIGHT encoder
          case P4IV__P4IFG3:         // P4IV_8:
              encoder_Ph_B_RIGHT_rupts++; // count the right wheel encoder interrupts
              break;           // counts reset at next motor_compute_rps_speed()

              // Vector  P4IV__P4IFG6:               P4.6 - Ph C / LEFT encoder
          case  P4IV__P4IFG6:         // P4IV_14:
              encoder_Ph_C_LEFT_rupts++;   // count the left wheel encoder interrupts
              break;           // counts reset at next motor_compute_rps_speed()

              /* Default case */
          default:
              break;
      }

    _g_hall_index = 0;              // generate resulting HALL sensor index
    if (READ_HALL_SENSOR_C)
       _g_hall_index  = 0b0001;     // 0x01
    if (READ_HALL_SENSOR_B)
       _g_hall_index |= 0b0010;     // 0x02
    if (READ_HALL_SENSOR_C)
       _g_hall_index |= 0b0100;     // 0x04

}


//******************************************************************************
//
//                UCB0 (EUSCIB0)     SPI    ISR
//
//******************************************************************************

void  EUSCIB0_IRQHandler (void)
{
   uint8_t   data_byte;

    if (UCB0IFG & UCTXIFG)
       {    // we got a TxBE transmit interrupt - see if have any more to send
         _g_spi_TX_rupts++;                        // Update DEBUG counter
         if (_g_spi_length != 0)
            {     // yes, see if read or write
              if ( _g_spi_io_operation != SPI_READ)
                 data_byte = *_g_spi_write_buf++;
                 else data_byte = 0x00;            // send dummy byte for READs
              UCB0TXBUF = data_byte;               // send data to slave
              _g_spi_length--;                     // deduct # bytes left
            }
         if (_g_spi_length == 0)
            {     // length = 0, so shutoff TX interrupts
              UCB0IE &= ~UCTXIE;
            }
         UCB0IFG &= ~UCTXIFG;                      // clear TX rupt
       }

    if (UCA3IFG & UCRXIFG)
       {    // we got a RX complete interrupt - process it
         _g_spi_RX_rupts++;                        // Update DEBUG counter
         data_byte = UCB0RXBUF;                    // read and save rcvd RX
         if ( _g_spi_io_operation != SPI_WRITE)
            {      // save the received data into caller buf
              *_g_spi_read_buf = data_byte;
              _g_spi_read_buf++;                   // step to next byte in buf
            }
              // we already deducted length during send, so
              // now see if length = 0. If so, post the operation complete
         if (_g_spi_length == 0)
            { _g_spi_io_status = SPI_IO_COMPLETE;  // yes, post operation complete
              DEASSERT_CS();                       // and shutoff Chip Select
            }
         UCB0IFG &= ~UCRXIFG;                      // clear RX rupt
       }


#if LATER
    uint16_t  _g_io_operation   = 0; // e.g. SPI_READ, SPI_WRITE, SPI_WRITE_READ
    uint16_t  _g_spi_length     = 0;
    char      *_g_spi_write_buf = 0L;
    char      *_g_spi_read_buf  = 0L;

    if (UCA3IFG & UCTXIFG)
      {
        UCA3TXBUF = TXData;                  // Transmit next character
        UCA3IE &= ~UCTXIE;

        while ( ! (UCA3IFG&UCRXIFG))  ;

        RXData = UCA3RXBUF;
        UCA3IFG &= ~UCRXIFG;
      }


    if (UCA3IFG & UCRXIFG)
    {
      while (!(UCA3IFG & UCTXIFG));           // USCI_A3 TX buffer ready?
      UCA3TXBUF = UCA3RXBUF;                  // Echo received data
    }

    DEASSERT_CS();                            // turn off CS pin


//#define UCB0IFG                   (HWREG16(0x4000202C)) /* eUSCI_Bx Interrupt Flag Register */
//#define UCB0IV                    (HWREG16(0x4000202E)) /* eUSCI_Bx Interrupt Vector Register */
//#define UCB0STAT                  (HWREG8_L(UCB0STATW)) /* eUSCI_Bx Status */

   TA1CCTL1 &= ~CCIFG;                   // turn off the interrupt flag

         // call higher level handler (in motor_ctl_lib) to process
   motor_commutation_update();
#endif

}


//******************************************************************************
//
//                Timer A1    COMMUTATION TIMER    ISR
//
//******************************************************************************

void  TA1_N_IRQHandler (void)
{
   _g_commute_timer_rupts++;             // Update DEBUG counter

   TA1CCTL1 &= ~CCIFG;                   // turn off the interrupt flag

         // call higher level handler (in motor_ctl_lib) to process
   motor_commutation_update();

// TA1CCR1 += 32768;   // ??? needed ???     YES - WHAT A TURD !!!   05/22/16
   if (_g_mtr_blk != 0L)
      TA1CCR1 = _g_mtr_blk->mtr_commute_time_ticks; // relead value for next pass (how DUMB !)

}

//*****************************************************************************
