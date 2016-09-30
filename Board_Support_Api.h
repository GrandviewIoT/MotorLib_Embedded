
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                              Board_Support_Api.h
//
// Common API call definitions used for common Board Support library.
//
// History:
//   05/16/16 - Created. Duquaine
//
// -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
//
// The MIT License (MIT)
//
// Copyright (c) 2016 Wayne Duquaine / Grandview Systems
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

#ifndef _BOARD_SUPPORT_API_H_
#define   _BOARD_SUPPORT_API_H_

#include <stdint.h>              // pull in standard int16_t, etc defs
#include <stdbool.h>

              // valid options for option_flags on board_init()
#define  NO_SYSTICK_ENABLE  1    // do not auto-enable Systick during board_init

void  board_init (long mcu_clock_rate, int option_flags);
void  board_delay_ms (long wait_time);
void  board_disable_global_interrupts (void);
void  board_enable_global_interrupts (void);
long  board_system_clock_get_frequency (void);


#endif                          // _BOARD_SUPPORT_API_H_

//*****************************************************************************
