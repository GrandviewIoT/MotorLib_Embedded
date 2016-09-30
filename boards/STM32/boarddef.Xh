
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                                 boarddef.h                             STM32
//
//
//
// Top level module for Board dependent definitions for Tiva, C2000, and MSP430
//
// Contains APIs for common routines implemented for every board, and invokes
// #include for the appropriate board_NNNNN.h file, based on MCU type.
//
// History:
//   xx/xx/xx - Created.
//   12/14/14 - Renamed from board.h to boarddef.h, so we do not accidentally
//              pick up some of the board.h defs that TI has in their projects.
//
//
// MCU Hardware supported:
// -----------------------
//    Nucleo  F4_01     =  256 K Flash  /  32 K SRAM
//
//
//   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
// Copyright (C) 2014 Grandview Systems
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// History:
//   12/01/14 - Created as a generic high-level include, to select correct defs.
// -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
//
// The MIT License (MIT)
//
// Copyright (c) 2014-2015 Wayne Duquaine / Grandview Systems
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//******************************************************************************

#ifndef __BOARDDEF_H__
#define __BOARDDEF_H__
                            //----------------------------------------
#define  STM32_MCU          // generic to denote this is a STM32 MCU
                            //----------------------------------------

              //------------------------------------------------------
              //------------------------------------------------------
              //      pull in HAL API includes for All Boards
              //------------------------------------------------------
              //------------------------------------------------------
#include "board_STM32.h"      // pull in HAL Board specific pin/port defs


extern const  GPIO_TypeDef *_g_gpio_base[];    // SEPARATE LIB ISSUE !!! ??? WVD

#if (USES_RTOS)
            // RTOS_conf.h file will provide mappings      TBD next pass using FREERTOS
#endif

#if (USES_MEMS_ENV)
#ifndef USES_I2C
#define  USES_I2C        // ensure I2C turned on for MEMS Sensors
#endif
#endif

              //****************************************************************
              //   Generic board level APIs used for Nucleo/Discovery support
              //
              //  These board_xxx calls are used by the code to provide
              //  "low level" cross platform APIs for key GPIO, ADC, PWM,
              //  and Timer functions.
              //****************************************************************

                  //-----------------------
                  // generic APIs
                  //-----------------------
int   setup (void);
void  main_loop (void);

                  //-----------------------
                  //  Startup & Delay APIs
                  //-----------------------
void  board_init (long mcu_clock_rate,int option_flags); // Function Prototypes
void  board_busy_wait_usec (long usec_delay);            // Delay of 1 us units
void  board_delay_ms (long msec_delay);                  // Delay of 1 ms units
void  board_disable_global_interrupts (void);
void  board_enable_global_interrupts (void);
void  board_error_handler (void);
long  board_frequency_to_period_ticks (long frequency);


#ifndef ADC_CB_EVENT_HANDLER
typedef  void (*P_EVENT_HANDLER)(void *pValue);
typedef  void (*ADC_CB_EVENT_HANDLER)(void *pCbParm, uint16_t *channel_results,
                int num_channels, int flags);
typedef  void (*I2C_CB_EVENT_HANDLER)(void *pCbParm, int rupt_id, int status);
typedef  void (*SPI_CB_EVENT_HANDLER)(void *pCbParm, int rupt_id, int status);
typedef  void (*TMR_CB_EVENT_HANDLER)(void *pCbParm, int rupt_id);
typedef  void (*UART_CB_EVENT_HANDLER)(void *pCbParm, int rupt_id, int status);
typedef  void (*IO_CB_EVENT_HANDLER)(void *pCbParm, int rupt_id, int status);
#endif


                  //-----------------
                  //  ADC APIs
                  //-----------------
int  board_adc_init (unsigned int adc_module_id, uint32_t clock_rate,
                     int trigger_type, int flags);
int  board_adc_config_channel (unsigned int adc_module_id, int channel_num,
                               int sequencer, int step_num,
                               int last,  int flags);
int  board_adc_check_conversions_completed (unsigned int adc_module_id, int sequencer);
int  board_adc_disable (unsigned int adc_module_id, int sequencer);
int  board_adc_enable (unsigned int adc_module_id, int sequencer);
void board_adc_get_app_trigger_masks (unsigned int adc_module_id,
                                      uint16_t *app_trigger_tmr_mmsmask,
                                      uint16_t *app_trigger_user_api_id);
int  board_adc_get_results (unsigned int adc_module_id, int sequencer,
                            uint16_t  channel_results[]);
int  board_adc_get_resolution (unsigned int module_id);
int  board_adc_set_callback (unsigned int adc_module_id, ADC_CB_EVENT_HANDLER callback_function, void *callback_parm);
int  board_adc_set_resolution (unsigned int module_id, int bit_resolution);
int  board_adc_user_trigger_start (unsigned int adc_module_id, int sequencer);


                  //-----------------
                  //  DAC APIs          STM32:  F0_72, F3_34, L0_53, L1_52
                  //-----------------           not on F1_03, F4_01
int  board_dac_init (unsigned int dac_module_id, uint32_t clock_rate, int flags);
int  board_dac_config_channel (unsigned int dac_module_id, int channel_num,
                               int trigger_type,
                               long sps_frequency, int flags);
int  board_dac_check_conversions_completed (unsigned int dac_module_id, int channel_id);
int  board_dac_clear_conversions_completed (unsigned int dac_module_id, int channel_num);
int  board_dac_enable_channel (unsigned int dac_module_id,  int channel_id, int flags);
int  board_dac_disable_channel (unsigned int dac_module_id, int channel_id, int flags);
int  board_dac_gen_sample_table (unsigned int wave_type, short *table_buf, int num_steps);
int  board_dac_set_sample_table (unsigned int module_id, int channel, short *table_buf, int num_steps);
void board_dac_get_app_trigger_masks (unsigned int dac_module_id,
                                      uint16_t *app_trigger_tmr_mmsmask,
                                      uint16_t *app_trigger_user_api_id);


                  //-----------------
                  //  GPIO APIs
                  //-----------------
void  board_gpio_init (void);
int   board_gpio_pin_config (unsigned int pin_id, int dir, int pull);
int   board_gpio_pin_config_pinmux (unsigned int pin_id,
                                    int dir, int pull, int alt_func);
int   board_gpio_pin_lookup (unsigned int pin_id, GPIO_TypeDef **GPIO_port, uint32_t *GPIO_pin);
uint16_t  board_gpio_read_port (unsigned int gpio_port_num, uint16_t pins_mask, int flags);
uint16_t  board_gpio_read_pin (unsigned int pin_id, int flag);
int   board_gpio_toggle_pin (unsigned int pin_id);
int   board_gpio_write_pin (unsigned int pin_id, int in_out_flag);
int   board_irq_pin_config (unsigned int pin_id,
                            int rise_fall, int pullup, uint32_t irq_vector_num,
                            unsigned long priority);
int   board_irq_pin_disable (unsigned int pin_id,
                             uint32_t irq_vector_num, int clear_pending_rupts);
int   board_irq_pin_enable (unsigned int pin_id,
                            uint32_t irq_vector_num, int clear_pending_rupts);


                  //-----------------
                  //  I2C APIs
                  //-----------------
//int  board_i2c_init (int i2c_module, int master_slave, int my_local_address,
//                     long baud_rate, int flags, I2C_HandleTypeDef *caller_hi2c_ptr);
int  board_i2c_init (unsigned int i2c_module_id, int scl_pin_id, int sda_pin_id,
                     int master_slave, int my_local_address,
                     long i2c_baud_timing,  int flags,
                     I2C_HandleTypeDef *caller_hi2c_ptr);
int  board_i2c_ALTFUNC_lookup (unsigned int module_id, int scl_gpio_pin, uint32_t *scl_AltFuncId,
                               int sda_gpio_pin, uint32_t *sda_AltFuncId);
int  board_i2c_check_io_completed (unsigned int i2c_module, int flags);
I2C_HandleTypeDef *board_i2c_get_handle (unsigned int i2c_module_lid);
int  board_i2c_set_callback (unsigned int i2c_module_id,
                             I2C_CB_EVENT_HANDLER callback_function,
                             void *callback_parm);
int  board_i2c_set_max_timeout (unsigned int i2c_module_id, uint32_t max_timeout);
int  board_i2c_read (unsigned int i2c_module_id, int slave_addr,
                     uint8_t *receive_buffer, int max_buf_length, int flags);
int  board_i2c_read_header_data (unsigned int i2c_module_lid, int slave_addr,
                           uint16_t header_buffer, int hbuf_length,
                           uint8_t  *receive_buffer, int rbuf_length,int flags);
int  board_i2c_write (unsigned int i2c_module_id, int slave_addr,
                      uint8_t *transmit_buffer, int buf_length, int flags);
int  board_i2c_write_read (unsigned int i2c_module_id, int slave_addr,
                           uint16_t header_buffer,   int hbuf_length,
//  future                 uint8_t *transmit_buffer, int xbuf_length,
                           uint8_t *receive_buffer,  int rbuf_length, int flags);
int  board_i2c_write_header_data (unsigned int i2c_module_id, int slave_addr,
                                  uint16_t header_buffer,    int hbuf_length,
///   future                      uint8_t  *header_buffer,   int hbuf_length,
                                  uint8_t  *transmit_buffer, int xbuf_length,
                                  int flags);
void board_i2c_dma_init (void);
void board_i2c_IRQ_Handler (int i2c_interrupt_number);
void board_i2c_ERRIRQ_Handler (int i2c_interrupt_number);

int  board_i2c_nack (unsigned int i2c_module);
int  board_i2c_stop (unsigned int i2c_module);


                  //-----------------------
                  //   SPI   APIs
                  //-----------------------
int  board_spi_init (unsigned int spi_module_id,
                     int sclk_pin_id, int miso_pin_id, int mosi_pin_id,
                     int master_slave, int spi_mode,
                     int baud_rate_scalar,  int flags,
                     SPI_HandleTypeDef *ptr_Caller_SpiHdl);  // extended support
int  board_spi_check_io_completed (unsigned int module_id, int flags);
SPI_HandleTypeDef * board_spi_get_handle (unsigned int spi_module_id);
int  board_spi_read (unsigned int spi_module_id, uint8_t *receive_buffer,
                     int max_buf_length, int flags);
int  board_spi_set_callback (unsigned int spi_module_id,
                             SPI_CB_EVENT_HANDLER callback_function,
                             void *callback_parm);
int  board_spi_set_max_timeout (unsigned int spi_module_id, uint32_t max_timeout);
int  board_spi_write (unsigned int spi_module_id, uint8_t *transmit_buffer,
                      int buf_length, int flags);
int  board_spi_write_read (unsigned int spi_module_id, uint8_t *transmit_buffer,
                           uint8_t *receive_buffer, int buf_length,
                           int flags);
void  board_spi_dma_init (void);
void  board_spi_IRQ_Handler (int spi_interrupt_number);



                  //-----------------------
                  //   Timer / PWM   APIs
                  //-----------------------
int  board_timerpwm_init (unsigned int module_id, int counter_type, long period_value,
                          int timer_clock_source, int flags,
                          TIM_HandleTypeDef *ptr_Caller_TimHdl);  // extended support
int  board_timerpwm_check_completed (unsigned int module_id, int check_mask, int reset_flags);
int  board_timerpwm_config_channel (unsigned int module_id, int channel_id, long initial_duty, int mode, int flags);
int  board_timerpwm_config_channel_pair (unsigned int module_id, int channelA_id, int channelB_id, int flags);
int  board_timerpwm_config_trigger_mode (unsigned int module_id, int adc_dac_module_id,
                                         int trig_type, int flags);
int  board_timerpwm_disable (unsigned int module_id, int flags);
int  board_timerpwm_enable_CCR_input (unsigned int module_id, int channel_id, long initial_duty, int flags);
int  board_timerpwm_enable_CCR_output (unsigned int module_id, int channel_id,
                           long initial_duty, int action_flags, int extended_flags);
int  board_timerpwm_enable (unsigned int module_id, int interrupt_flags);
long board_timerpwm_get_current_value (unsigned int module_id);
long board_timerpwm_get_CCR_capture_value (unsigned int module_id, int CCR_channel_num);
long board_timerpwm_get_duty_cycle (unsigned int modgen_id, int chan_id);
long board_timerpwm_get_period (unsigned int module_id);
TIM_HandleTypeDef * board_timerpwm_get_handle (unsigned int module_id);
int  board_timerpwm_reset_CCR_output (unsigned int module_id, int chan_id, int flags);
int  board_timerpwm_set_callback (unsigned int module_id, TMR_CB_EVENT_HANDLER callback_function, void *callback_parm);
int  board_timerpwm_set_channel_output (unsigned int module_id, int channel_id, int output_mode, int flags);
int  board_timerpwm_set_dead_time (unsigned int module_id, int rising_edge, int falling_edge);
int  board_timerpwm_set_duty_cycle (unsigned int modgen_id, int chan_num, long duty_cycle, int flags);
int  board_timerpwm_set_period (unsigned int module_id, long new_period_value, int flags);
int  board_timerpwm_set_phase (unsigned int module_id, int channel_id, long phase_offset);
int  board_timerpwm_set_prescalar (unsigned int module_id, long prescalar_val, int flags);

                  //------------------
                  //  Sysclocks  APIs
                  //------------------
void  board_stop_WDT (void);
void  board_system_clock_config (long mcu_clock_hz, int option_flags);
long  board_system_clock_get_frequency (void);
long  board_sys_IO_clock_get_frequency (void);
void  board_systick_timer_config (void);
unsigned long  board_systick_timer_get_value (void);


                  //-----------------
                  //  UART  APIs
                  //-----------------
//int   board_uart_init (int mod_id, long baud_rate, int flags);
int  board_uart_init (unsigned int module_id, int tx_gpio_pin_id,
                      int rx_gpio_pin_id, long baud_rate, int flags);
int  board_uart_ALTFUNC_lookup (unsigned int module_id, int tx_gpio_pin, uint32_t *tx_AltFuncId,
                                int rx_gpio_pin, uint32_t *rx_AltFuncId);
int  board_uart_console_read_fdx (unsigned int module_id, uint8_t *user_buf, int max_buf_len,
                                  int pause_time, int max_wait_time, int flags);
int  board_uart_get_char (unsigned int mod_id, int flags, int max_wait_time);  // 0 = no wait
int  board_uart_rx_data_check (unsigned int mod_id);
int  board_uart_rx_flush (unsigned int module_id, int flags);
int  board_uart_read_bytes (unsigned int mod_id, uint8_t *read_buf, int buf_max_length, int flags);
int  board_uart_read_text_line (unsigned int mod_id, char *read_buf, int buf_length, int flags);
int  board_uart_set_callback (unsigned int module_id,
                              UART_CB_EVENT_HANDLER callback_function,
                              void *callback_parm);
////int  board_uart_set_echoplex (int module_id, int on_off_flag);
int  board_uart_set_max_timeout (unsigned int module_id, uint32_t max_timeout);
int  board_uart_write_char (unsigned int mod_id,  char outchar, int flags);
int  board_uart_write_bytes (unsigned int mod_id, uint8_t *bytebuf, int buf_len, int flags);
int  board_uart_write_string (unsigned int mod_id, char *outstr, int flags);


                  //-------------------------
                  //  UNIQUE-ID / CRC  APIs
                  //-------------------------
void  board_unique_crcid_init (unsigned long seed, int flags);
unsigned long  board_unique_crcid_compute (void *in_buf, int in_buf_length,
                                           int flags_32_8);

                  //-------------------------------
                  //  VTIMER (Virtual Timer)  APIs
                  //-------------------------------
int  board_vtimer_start (unsigned int vtimer_id, uint32_t timer_duration_millis,
                         P_EVENT_HANDLER callback_function,  void *callback_parm);
int  board_vtimer_completed (unsigned int vtimer_id);
int  board_vtimer_reset (unsigned int vtimer_id);
void board_vtimer_check_expiration (uint32_t gsystick_millisecs);



//******************************************************************************
//                    GLOBAL  CONTANTS  USED  WITH  BOARD  APIs
//******************************************************************************

              //-------------------------------
              //   Internal Error Codes
              //-------------------------------
   //--------------------------------------------------------------------
   // supply TCP/UDP errno codes that are not normally included in STM32 defs
   //
   // Normally, the errno codes are positive values (9/11/22/43/110), BUT
   //   - W5200 normal reply codes map over some of these values
   //   - Ditto SimpleLink.
   // So to avoid amibuity, we changed these to negative values instead.
   //--------------------------------------------------------------------
#define EBADF                               -9
#define EAGAIN                             -11       // try again (later)
#define EINVAL                             -22
#define EPIPE                              -32
#define ETIMEDOUT                         -110


   // Maximum Timeout values for Interrupts/flags waiting loops.
   // These timeouts are not based on accurate values, they just meant
   // to guarantee that the application will not remain stuck if the
   // SPI communication is corrupted. You may modify these timeout values
   // depending on CPU frequency and application conditions (interrupt
   // routines ...)
#define SPIx_TIMEOUT_MAX          ((uint32_t) 0x1000)


// The magic number that resides at the end of the TX/RX buffer (1 byte after
// the allocated size) for the purpose of detection of the overrun. The location
// of the memory where the magic number resides shall never be written. In case
// it is written - the overrun occurred and either receive function or send
// function will stuck forever.
#define  CC3000_BUFFER_MAGIC_NUMBER (0xDE)


extern unsigned char  wlan_tx_buffer[];


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//                            Common  Shield  Information
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

   //-----------------------------------------------------------------
   // supply defs that not included as part of Wiznet W5200/W5500
   //-----------------------------------------------------------------
#if defined(USES_W5200)  ||  defined(USES_W5500)

#ifndef USES_SPI
#define  USES_SPI                 /* ensure SPI dependency turned on */
#endif

struct timeval
 {
     unsigned long  tv_sec;
     unsigned long  tv_usec;
 };

    // Need to add special logic to W5200 Handler to emulate FD_SET and SELECT
    // by setting IR2/IMR regs to interrupt on recv of disconnect situs


#define FD_SET                              WIZ_FD_SET
#define FD_CLR                              WIZ_FD_CLR
#define FD_ISSET                            WIZ_FD_ISSET
#define FD_ZERO                             WIZ_FD_ZERO
#define fd_set                              WIZ_FdSet_t

typedef struct WizFdSet_t                      /* Select socket array */
{
   uint32_t     fd_array;    /* Bit map of SOCKET Descriptors (Max 8 sockets) */
///uint32t      fd_array[(SL_FD_SETSIZE + 31)/32]; /* Bit map of SOCKET Descriptors */
} WIZ_FdSet_t;


/*
    Select's WIZ_FdSet_t SET function
    Sets current socket descriptor on WIZ_FdSet_t container
*/
void  WIZ_FD_SET (short fd, WIZ_FdSet_t *fdset);

/*
    Select's WIZ_FdSet_t CLR function
    Clears current socket descriptor on WIZ_FdSet_t container
*/
void  WIZ_FD_CLR (short fd, WIZ_FdSet_t *fdset);


/*
    Select's WIZ_FdSet_t ISSET function
    Checks if current socket descriptor is set (TRUE/FALSE)

    Returns TRUE if set, FALSE if unset
*/
short  WIZ_FD_ISSET (short fd, WIZ_FdSet_t *fdset);

/*
    Select's WIZ_FdSet_t ZERO function
    Clears all socket descriptors from WIZ_FdSet_t
*/
void  WIZ_FD_ZERO (WIZ_FdSet_t *fdset);
//#define  WIZ_FD_ZERO(WIZ_FdSet_t *fdset)  (*fdset = 0)

#define  fd_set   WIZ_FdSet_t

#endif                       //  defined(USES_W5200)



/*******************************************************************************
*                 WizNet5200 Wired Ethernet Shield   (Seeed Studio, Radio Shack)
*
*                     WizNet5200 Ethernet usage with Nucleo Boards
*
*   The Seeed Studio shield, in keeping with the newer Arduino UNO design,
*   sends its SPI signals out the 6-pin ICSP header, located at the bottom of
*   the shield.  _None_ of the Nucleo boards support the 6-pin UNO ICSP/SPI
*   header. (Note that the ISCP SPI signals are _not_ routed over to
*   the standard Arduino SPI pins D13-D15).
*
*   So we must jumper the wires from the 6-pin ICSP header over to a set of
*   SPI pins on the Nucleo. To help avoid SPI conflicts with some of the other
*   Arduino shields and STmicro X-Nucleo shields, we route the ISCP SPI
*   wires over to the Nucleo's "Morpho" connector pins, and depending upon
*   the Nucleo board, route it to either the SPI-3 pins, or the SPI-2 pins.
*
*   SPI-3 pins (PC10-PC12, upper left corner of the left CN7 Morpho connector),
*   are used for the following Nucleo boards:
*                  F303RE     F401RE     L152RE     L476RG
*
*   SPI-2 pins (PC2-PC3 lower left corner of of the left CN7 Morpho connector,
*   and PB13 on the right CN10 connector pin 30) are used for the following
*   Nucleo boards:
*                  F072RB     F091RC     L053R8
*
*   SPI-2 pins (PB13-PB15 on the right CN10 Morpho connector pins 26/28/30),
*   are used for the following Nucleo board:
*                  F103RB
*
*   For the Wiznet W52000 (Seeed Studio) shield, the following pins are used,
*   based on the type of Nucleo board:
*
*                         F303RE / L152RE
*                         F401RE / L476RG
*    Function               Morpho  pin      Arduino Pin
*    ----------            -------------    -------------
*    SPI SCLK   (SPI3)         PC10          ICSP # 3       Red     Upper left
*    SPI MISO                  PC11          ICSP # 1       Yellow  Upper Right
*    SPI MOSI                  PC12          ICSP # 4       Green   2nd Left
*    SPI CS for W5200 default  PB6           Digital # 10
*    SPI CS for W5200 conflict PA10          Digital #  2   Alternate pin used
*    IRQ                       PB3           NRST           (on Power header)
*    CS for SD Card            PB5           Digital #  4   (Optional)
*
*                         F072RB / F091RC
*                         L053R8
*    Function               Morpho  pin      Arduino Pin
*    ----------            -------------    -------------
*    SPI SCLK   (SPI2)         PB13          ICSP # 3
*    SPI MISO                  PC2           ICSP # 1
*    SPI MOSI                  PC3           ICSP # 4
*    SPI CS for W5200 default  PB6           Digital # 10
*    SPI CS for W5200 conflict PA10          Digital #  2   Alternate pin used
*    IRQ                       PB3           NRST           (on Power header)
*    CS for SD Card            PB5           Digital #  4   (Optional)
*
*                             F103RB
*    Function               Morpho  pin      Arduino Pin
*    ----------            -------------    -------------
*    SPI SCLK   (SPI2)         PB13          ICSP # 3
*    SPI MISO                  PB14          ICSP # 1
*    SPI MOSI                  PB15          ICSP # 4
*    SPI CS for W5200 default  PB6           Digital # 10
*    SPI CS for W5200 conflict PA10          Digital #  2   Alternate pin used
*    IRQ                       PB3           NRST           (on Power header)
*    CS for SD Card            PB5           Digital #  4   (Optional)
*******************************************************************************/


/*******************************************************************************
*                        CC3000 usage with Nucleo Boards
*
*   On the CC3000 shield, the following pins are used:
*
*                      STM32_F3_02R8                                      CHANGE
*     Function              pin        Arduino Pin  (Right side -
*     ----------       -------------   ------------  all digital pins)
*     SPI SCLK  (SPI2)     PB13        Digital # 13
*     SPI MISO             PB14        Digital # 12
*     SPI MOSI             PB15        Digital # 11
*     SPI CS for CC3000    PB6         Digital # 10
*     VBAT_ENABLE          PB4         Digital #  5
*     IRQ                  PB3         Digital #  3
*     CS for SD Card       PB5         Digital #  4   (Optional)
*
*     SYSTICK -  Interrupt enabled
*     IRQ     -  EXTI-3 interrupt enabled
*     SPI2    -  Interrupt enabled   Needed ???
*     DMA1-Channel4 = SPI RX (MISO) - Interrupt enabled
*     DMA1-Channel5 = SPI TX (MOS1) - Interrupt enabled
*
*  Key Notes:
*    - When CC3000 ENABLE is raised, this causes a CC3000 reset/initialization
*      sequence to occur. The CC3000 signals this by dropping its IRQ pin
*      LOW on reset start, and raises the IRQ pin back HIGH when the reset/init
*      sequence is complete.
*    - The STM32 must wait until this sequence is complete before issuing any
*      SPI commands to the CC3000.
*******************************************************************************/

#endif                          //  __BOARDDEF_H__

//*****************************************************************************
