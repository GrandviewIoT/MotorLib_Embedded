
                       //  WVD MODIFIED


/*******************************************************************************
  * @file    l6474.h
  * @author  IPC Rennes
  * @version V1.6.0
  * @date    February 03, 2016
  * @brief   Header for L6474 driver (fully integrated microstepping motor driver)
  * @note    (C) COPYRIGHT 2014 STMicroelectronics
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
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __L6474_H
#define __L6474_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "l6474_target_config.h"
//#include "motor.h"     // WVD CHANGE 06/23/16

/** @addtogroup BSP
  * @{
  */

/** @addtogroup L6474
  * @{
  */

/* Exported Constants --------------------------------------------------------*/

/** @defgroup L6474_Exported_Constants L6474 Exported Constants
  * @{
  */

                         /// Maximum frequency of the PWMs in Hz
#define L6474_MAX_PWM_FREQ   (10000)

                         /// Minimum frequency of the PWMs in Hz
#define L6474_MIN_PWM_FREQ   (2)

/// Current FW version
#define L6474_FW_VERSION (6)

/// L6474 max number of bytes of command & arguments to set a parameter
#define L6474_CMD_ARG_MAX_NB_BYTES              (4)

/// L6474 command + argument bytes number for GET_STATUS command
#define L6474_CMD_ARG_NB_BYTES_GET_STATUS       (1)

/// L6474 response bytes number
#define L6474_RSP_NB_BYTES_GET_STATUS           (2)

/// L6474 max absolute position
#define L6474_MAX_POSITION (int32_t)(0x001FFFFF)

/// L6474 min absolute position
#define L6474_MIN_POSITION (int32_t)(0xFFE00000)

/// L6474 torque regulation DAC reference current max value
#define L6474_TVAL_MAX_VALUE          (4000)

/// L6474 TOFF_MIN and TON_MIN max value in us
#define L6474_TOFF_TON_MIN_MAX_VALUE          (64)

/// L6474 Overcurrent detection threshold max value in mA (OCD_TH register)
#define L6474_OCD_TH_MAX_VALUE          (6000)

  /**
  * @}
  */

/* Exported Types  -------------------------------------------------------*/

/** @defgroup L6474_Exported_Types L6474 Exported Types
  * @{
  */

/** @defgroup L6474_ABS_POS_Mask L6474 ABS_POS Mask
  * @{
  */
/// masks for ABS_POS register
typedef enum {
  L6474_ABS_POS_VALUE_MASK        = ((uint32_t)0x003FFFFF),
  L6474_ABS_POS_SIGN_BIT_MASK     = ((uint32_t)0x00200000)
} L6474_ABS_POS_Mask_t;
/**
  * @}
  */

/** @defgroup L6474_EL_POS_Mask L6474 EL_POS Mask
  * @{
  */
/// masks for EL_POS register
typedef enum {
  L6474_ELPOS_STEP_MASK       = ((uint16_t)0x180),
  L6474_ELPOS_MICROSTEP_MASK  = ((uint16_t)0x07F)
} L6474_ELPOS_Mask_t;
/**
  * @}
  */

/** @defgroup L6474_Fast_Decay_Time_Options  L6474 Fast Decay Time Options
  * @{
  */
///TOFF_FAST values for T_FAST register
typedef enum {
  L6474_TOFF_FAST_2us = ((uint8_t) 0x00 << 4),
  L6474_TOFF_FAST_4us = ((uint8_t) 0x01 << 4),
  L6474_TOFF_FAST_6us = ((uint8_t) 0x02 << 4),
  L6474_TOFF_FAST_8us = ((uint8_t) 0x03 << 4),
  L6474_TOFF_FAST_10us = ((uint8_t) 0x04 << 4),
  L6474_TOFF_FAST_12us = ((uint8_t) 0x05 << 4),
  L6474_TOFF_FAST_14us = ((uint8_t) 0x06 << 4),
  L6474_TOFF_FAST_16us = ((uint8_t) 0x07 << 4),
  L6474_TOFF_FAST_18us = ((uint8_t) 0x08 << 4),
  L6474_TOFF_FAST_20us = ((uint8_t) 0x09 << 4),
  L6474_TOFF_FAST_22us = ((uint8_t) 0x0A << 4),
  L6474_TOFF_FAST_24us = ((uint8_t) 0x0B << 4),
  L6474_TOFF_FAST_26us = ((uint8_t) 0x0C << 4),
  L6474_TOFF_FAST_28us = ((uint8_t) 0x0D << 4),
  L6474_TOFF_FAST_30us = ((uint8_t) 0x0E << 4),
  L6474_TOFF_FAST_32us = ((uint8_t) 0x0F << 4)
} L6474_TOFF_FAST_t;
/**
  * @}
  */

/** @defgroup L6474_Fall_Step_Time_Options  L6474 Fall Step Time Options
  * @{
  */
///FAST_STEP values for T_FAST register
typedef enum {
  L6474_FAST_STEP_2us = ((uint8_t) 0x00),
  L6474_FAST_STEP_4us = ((uint8_t) 0x01),
  L6474_FAST_STEP_6us = ((uint8_t) 0x02),
  L6474_FAST_STEP_8us = ((uint8_t) 0x03),
  L6474_FAST_STEP_10us = ((uint8_t) 0x04),
  L6474_FAST_STEP_12us = ((uint8_t) 0x05),
  L6474_FAST_STEP_14us = ((uint8_t) 0x06),
  L6474_FAST_STEP_16us = ((uint8_t) 0x07),
  L6474_FAST_STEP_18us = ((uint8_t) 0x08),
  L6474_FAST_STEP_20us = ((uint8_t) 0x09),
  L6474_FAST_STEP_22us = ((uint8_t) 0x0A),
  L6474_FAST_STEP_24us = ((uint8_t) 0x0B),
  L6474_FAST_STEP_26us = ((uint8_t) 0x0C),
  L6474_FAST_STEP_28us = ((uint8_t) 0x0D),
  L6474_FAST_STEP_30us = ((uint8_t) 0x0E),
  L6474_FAST_STEP_32us = ((uint8_t) 0x0F)
} L6474_FAST_STEP_t;
/**
  * @}
  */

/** @defgroup L6474_Overcurrent_Threshold_options L6474 Overcurrent Threshold options
  * @{
  */
///OCD_TH register
typedef enum {
  L6474_OCD_TH_375mA  = ((uint8_t) 0x00),
  L6474_OCD_TH_750mA  = ((uint8_t) 0x01),
  L6474_OCD_TH_1125mA = ((uint8_t) 0x02),
  L6474_OCD_TH_1500mA = ((uint8_t) 0x03),
  L6474_OCD_TH_1875mA = ((uint8_t) 0x04),
  L6474_OCD_TH_2250mA = ((uint8_t) 0x05),
  L6474_OCD_TH_2625mA = ((uint8_t) 0x06),
  L6474_OCD_TH_3000mA = ((uint8_t) 0x07),
  L6474_OCD_TH_3375mA = ((uint8_t) 0x08),
  L6474_OCD_TH_3750mA = ((uint8_t) 0x09),
  L6474_OCD_TH_4125mA = ((uint8_t) 0x0A),
  L6474_OCD_TH_4500mA = ((uint8_t) 0x0B),
  L6474_OCD_TH_4875mA = ((uint8_t) 0x0C),
  L6474_OCD_TH_5250mA = ((uint8_t) 0x0D),
  L6474_OCD_TH_5625mA = ((uint8_t) 0x0E),
  L6474_OCD_TH_6000mA = ((uint8_t) 0x0F)
} L6474_OCD_TH_t;
/**
  * @}
  */

/** @defgroup L6474_STEP_MODE_Register_Masks  L6474 STEP_MODE Register Masks
  * @{
  */
///STEP_MODE register
typedef enum {
  L6474_STEP_MODE_STEP_SEL = ((uint8_t) 0x07),
  L6474_STEP_MODE_SYNC_SEL = ((uint8_t) 0x70)
} L6474_STEP_MODE_Masks_t;
/**
  * @}
  */

/** @defgroup L6474_STEP_SEL_Options_For_STEP_MODE_Register L6474 STEP_SEL Options For STEP_MODE Register
  * @{
  */
///STEP_SEL field of STEP_MODE register
typedef enum {
  L6474_STEP_SEL_1    = ((uint8_t) 0x08),  //full step
  L6474_STEP_SEL_1_2  = ((uint8_t) 0x09),  //half step
  L6474_STEP_SEL_1_4  = ((uint8_t) 0x0A),  //1/4 microstep
  L6474_STEP_SEL_1_8  = ((uint8_t) 0x0B),  //1/8 microstep
  L6474_STEP_SEL_1_16 = ((uint8_t) 0x0C)   //1/16 microstep
} L6474_STEP_SEL_t;
/**
  * @}
  */

/** @defgroup L6474_SYNC_SEL_Options_For_STEP_MODE_Register L6474 SYNC_SEL Options For STEP_MODE Register
  * @{
  */
///SYNC_SEL field of STEP_MODE register
typedef enum {
  L6474_SYNC_SEL_1_2    = ((uint8_t) 0x80),
  L6474_SYNC_SEL_1      = ((uint8_t) 0x90),
  L6474_SYNC_SEL_2      = ((uint8_t) 0xA0),
  L6474_SYNC_SEL_4      = ((uint8_t) 0xB0),
  L6474_SYNC_SEL_8      = ((uint8_t) 0xC0),
  L6474_SYNC_SEL_UNUSED = ((uint8_t) 0xD0)
} L6474_SYNC_SEL_t;
/**
  * @}
  */

/** @defgroup L6474_ALARM_EN_Register_Options L6474 ALARM_EN Register Options
  * @{
  */
///ALARM_EN register
typedef enum {
  L6474_ALARM_EN_OVERCURRENT      = ((uint8_t) 0x01),
  L6474_ALARM_EN_THERMAL_SHUTDOWN = ((uint8_t) 0x02),
  L6474_ALARM_EN_THERMAL_WARNING  = ((uint8_t) 0x04),
  L6474_ALARM_EN_UNDERVOLTAGE     = ((uint8_t) 0x08),
  L6474_ALARM_EN_SW_TURN_ON       = ((uint8_t) 0x40),
  L6474_ALARM_EN_WRONG_NPERF_CMD  = ((uint8_t) 0x80)
} L6474_ALARM_EN_t;
/**
  * @}
  */

/** @defgroup L6474_CONFIG_Register_Masks L6474 CONFIG Register Masks
  * @{
  */
///CONFIG register
typedef enum {
  L6474_CONFIG_OSC_SEL  = ((uint16_t) 0x0007),
  L6474_CONFIG_EXT_CLK  = ((uint16_t) 0x0008),
  L6474_CONFIG_EN_TQREG = ((uint16_t) 0x0020),
  L6474_CONFIG_OC_SD    = ((uint16_t) 0x0080),
  L6474_CONFIG_POW_SR   = ((uint16_t) 0x0300),
  L6474_CONFIG_TOFF      = ((uint16_t) 0x7C00)
} L6474_CONFIG_Masks_t;
/**
  * @}
  */

/** @defgroup L6474_Clock_Source_Options_For_CONFIG_Register L6474 Clock Source Options For CONFIG Register
  * @{
  */
///Clock source option for CONFIG register
typedef enum {
  L6474_CONFIG_INT_16MHZ = ((uint16_t) 0x0000),
  L6474_CONFIG_INT_16MHZ_OSCOUT_2MHZ   = ((uint16_t) 0x0008),
  L6474_CONFIG_INT_16MHZ_OSCOUT_4MHZ   = ((uint16_t) 0x0009),
  L6474_CONFIG_INT_16MHZ_OSCOUT_8MHZ   = ((uint16_t) 0x000A),
  L6474_CONFIG_INT_16MHZ_OSCOUT_16MHZ  = ((uint16_t) 0x000B),
  L6474_CONFIG_EXT_8MHZ_XTAL_DRIVE     = ((uint16_t) 0x0004),
  L6474_CONFIG_EXT_16MHZ_XTAL_DRIVE    = ((uint16_t) 0x0005),
  L6474_CONFIG_EXT_24MHZ_XTAL_DRIVE    = ((uint16_t) 0x0006),
  L6474_CONFIG_EXT_32MHZ_XTAL_DRIVE    = ((uint16_t) 0x0007),
  L6474_CONFIG_EXT_8MHZ_OSCOUT_INVERT  = ((uint16_t) 0x000C),
  L6474_CONFIG_EXT_16MHZ_OSCOUT_INVERT = ((uint16_t) 0x000D),
  L6474_CONFIG_EXT_24MHZ_OSCOUT_INVERT = ((uint16_t) 0x000E),
  L6474_CONFIG_EXT_32MHZ_OSCOUT_INVERT = ((uint16_t) 0x000F)
} L6474_CONFIG_OSC_MGMT_t;
/**
  * @}
  */

/** @defgroup L6474_External_Torque_Regulation_Options_For_CONFIG_Register L6474 External Torque Regulation Options For CONFIG Register
  * @{
  */
///External Torque regulation options for CONFIG register
typedef enum {
  L6474_CONFIG_EN_TQREG_TVAL_USED = ((uint16_t) 0x0000),
  L6474_CONFIG_EN_TQREG_ADC_OUT = ((uint16_t) 0x0020)
} L6474_CONFIG_EN_TQREG_t;
/**
  * @}
  */

/** @defgroup L6474_Over_Current_Shutdown_Options_For_CONFIG_Register L6474 Over Current Shutdown Options For CONFIG Register
  * @{
  */
///Over Current Shutdown options for CONFIG register
typedef enum {
  L6474_CONFIG_OC_SD_DISABLE = ((uint16_t) 0x0000),
  L6474_CONFIG_OC_SD_ENABLE  = ((uint16_t) 0x0080)
} L6474_CONFIG_OC_SD_t;
/**
  * @}
  */

/** @defgroup L6474_Power_Bridge_Output_Slew_Rate_Options L6474 Power Bridge Output Slew Rate Options
  * @{
  */
/// POW_SR values for CONFIG register
typedef enum {
  L6474_CONFIG_SR_320V_us    =((uint16_t)0x0000),
  L6474_CONFIG_SR_075V_us    =((uint16_t)0x0100),
  L6474_CONFIG_SR_110V_us    =((uint16_t)0x0200),
  L6474_CONFIG_SR_260V_us    =((uint16_t)0x0300)
} L6474_CONFIG_POW_SR_t;
/**
  * @}
  */

/** @defgroup L6474_Off_Time_Options L6474 Off Time Options
  * @{
  */
/// TOFF values for CONFIG register
typedef enum {
  L6474_CONFIG_TOFF_004us   = (((uint16_t) 0x01) << 10),
  L6474_CONFIG_TOFF_008us   = (((uint16_t) 0x02) << 10),
  L6474_CONFIG_TOFF_012us  = (((uint16_t) 0x03) << 10),
  L6474_CONFIG_TOFF_016us  = (((uint16_t) 0x04) << 10),
  L6474_CONFIG_TOFF_020us  = (((uint16_t) 0x05) << 10),
  L6474_CONFIG_TOFF_024us  = (((uint16_t) 0x06) << 10),
  L6474_CONFIG_TOFF_028us  = (((uint16_t) 0x07) << 10),
  L6474_CONFIG_TOFF_032us  = (((uint16_t) 0x08) << 10),
  L6474_CONFIG_TOFF_036us  = (((uint16_t) 0x09) << 10),
  L6474_CONFIG_TOFF_040us  = (((uint16_t) 0x0A) << 10),
  L6474_CONFIG_TOFF_044us  = (((uint16_t) 0x0B) << 10),
  L6474_CONFIG_TOFF_048us  = (((uint16_t) 0x0C) << 10),
  L6474_CONFIG_TOFF_052us  = (((uint16_t) 0x0D) << 10),
  L6474_CONFIG_TOFF_056us  = (((uint16_t) 0x0E) << 10),
  L6474_CONFIG_TOFF_060us  = (((uint16_t) 0x0F) << 10),
  L6474_CONFIG_TOFF_064us  = (((uint16_t) 0x10) << 10),
  L6474_CONFIG_TOFF_068us  = (((uint16_t) 0x11) << 10),
  L6474_CONFIG_TOFF_072us  = (((uint16_t) 0x12) << 10),
  L6474_CONFIG_TOFF_076us  = (((uint16_t) 0x13) << 10),
  L6474_CONFIG_TOFF_080us  = (((uint16_t) 0x14) << 10),
  L6474_CONFIG_TOFF_084us  = (((uint16_t) 0x15) << 10),
  L6474_CONFIG_TOFF_088us  = (((uint16_t) 0x16) << 10),
  L6474_CONFIG_TOFF_092us  = (((uint16_t) 0x17) << 10),
  L6474_CONFIG_TOFF_096us  = (((uint16_t) 0x18) << 10),
  L6474_CONFIG_TOFF_100us = (((uint16_t) 0x19) << 10),
  L6474_CONFIG_TOFF_104us = (((uint16_t) 0x1A) << 10),
  L6474_CONFIG_TOFF_108us = (((uint16_t) 0x1B) << 10),
  L6474_CONFIG_TOFF_112us = (((uint16_t) 0x1C) << 10),
  L6474_CONFIG_TOFF_116us = (((uint16_t) 0x1D) << 10),
  L6474_CONFIG_TOFF_120us = (((uint16_t) 0x1E) << 10),
  L6474_CONFIG_TOFF_124us = (((uint16_t) 0x1F) << 10)
} L6474_CONFIG_TOFF_t;
/**
  * @}
  */

/** @defgroup L6474_STATUS_Register_Bit_Masks L6474 STATUS Register Bit Masks
  * @{
  */
///STATUS Register Bit Masks
typedef enum {
  L6474_STATUS_HIZ         = (((uint16_t) 0x0001)),
  L6474_STATUS_DIR         = (((uint16_t) 0x0010)),
  L6474_STATUS_NOTPERF_CMD = (((uint16_t) 0x0080)),
  L6474_STATUS_WRONG_CMD   = (((uint16_t) 0x0100)),
  L6474_STATUS_UVLO        = (((uint16_t) 0x0200)),
  L6474_STATUS_TH_WRN      = (((uint16_t) 0x0400)),
  L6474_STATUS_TH_SD       = (((uint16_t) 0x0800)),
  L6474_STATUS_OCD         = (((uint16_t) 0x1000))
} L6474_STATUS_Masks_t;
/**
  * @}
  */

/** @defgroup L6474_Direction_Field_Of_STATUS_Register L6474 Direction Field Of STATUS Register
  * @{
  */
///Diretion field of STATUS register
typedef enum {
  L6474_STATUS_DIR_FORWARD = (((uint16_t) 0x0001) << 4),
  L6474_STATUS_DIR_REVERSE = (((uint16_t) 0x0000) << 4)
} L6474_STATUS_DIR_t;
/**
  * @}
  */

/** @defgroup L6474_Internal_Register_Addresses L6474 Internal Register Addresses
  * @{
  */
/// Internal L6474 register addresses
typedef enum {
  L6474_ABS_POS        = ((uint8_t) 0x01),
  L6474_EL_POS         = ((uint8_t) 0x02),
  L6474_MARK           = ((uint8_t) 0x03),
  L6474_RESERVED_REG01 = ((uint8_t) 0x04),
  L6474_RESERVED_REG02 = ((uint8_t) 0x05),
  L6474_RESERVED_REG03 = ((uint8_t) 0x06),
  L6474_RESERVED_REG04 = ((uint8_t) 0x07),
  L6474_RESERVED_REG05 = ((uint8_t) 0x08),
  L6474_RESERVED_REG06 = ((uint8_t) 0x15),
  L6474_TVAL           = ((uint8_t) 0x09),
  L6474_RESERVED_REG07 = ((uint8_t) 0x0A),
  L6474_RESERVED_REG08 = ((uint8_t) 0x0B),
  L6474_RESERVED_REG09 = ((uint8_t) 0x0C),
  L6474_RESERVED_REG10 = ((uint8_t) 0x0D),
  L6474_T_FAST         = ((uint8_t) 0x0E),
  L6474_TON_MIN        = ((uint8_t) 0x0F),
  L6474_TOFF_MIN       = ((uint8_t) 0x10),
  L6474_RESERVED_REG11 = ((uint8_t) 0x11),
  L6474_ADC_OUT        = ((uint8_t) 0x12),
  L6474_OCD_TH         = ((uint8_t) 0x13),
  L6474_RESERVED_REG12 = ((uint8_t) 0x14),
  L6474_STEP_MODE      = ((uint8_t) 0x16),
  L6474_ALARM_EN       = ((uint8_t) 0x17),
  L6474_CONFIG         = ((uint8_t) 0x18),
  L6474_STATUS         = ((uint8_t) 0x19),
  L6474_RESERVED_REG13 = ((uint8_t) 0x1A),
  L6474_RESERVED_REG14 = ((uint8_t) 0x1B),
  L6474_INEXISTENT_REG = ((uint8_t) 0x1F)
} L6474_Registers_t;
/**
  * @}
  */

/** @defgroup L6474_Command_Set L6474 Command Set
  * @{
  */
/// L6474 command set
typedef enum {
  L6474_NOP           = ((uint8_t) 0x00),
  L6474_SET_PARAM     = ((uint8_t) 0x00),
  L6474_GET_PARAM     = ((uint8_t) 0x20),
  L6474_ENABLE        = ((uint8_t) 0xB8),
  L6474_DISABLE       = ((uint8_t) 0xA8),
  L6474_GET_STATUS    = ((uint8_t) 0xD0),
  L6474_RESERVED_CMD1 = ((uint8_t) 0xEB),
  L6474_RESERVED_CMD2 = ((uint8_t) 0xF8)
} L6474_Commands_t;
/**
  * @}
  */


/** @defgroup Device_Commands Device Commands
  * @{
  */
/// Device commands
typedef enum {
  RUN_CMD,
  MOVE_CMD,
  SOFT_STOP_CMD,
  NO_CMD
} deviceCommand_t;
/**
  * @}
  */


/** @defgroup L6474_Initialization_Structure L6474 Initialization Structure
  * @{
  */
/// Motor driver initialization structure definition
typedef struct
{
    /// Acceleration rate in step/s2. Range: (0..+inf).
    uint16_t acceleration_step_s2;
    /// Deceleration rate in step/s2. Range: (0..+inf).
    uint16_t deceleration_step_s2;
    /// Maximum speed in step/s. Range: (30..10000].
    uint16_t maximum_speed_step_s;
     ///Minimum speed in step/s. Range: [30..10000).
    uint16_t minimum_speed_step_s;
     ///Torque regulation current in mA. Range: 31.25mA to 4000mA.
    float torque_regulation_current_mA;
     ///Overcurrent threshold in mA Range: 374mA to 6000mA.
    float overcurrent_threshold;
    ///Overcurrent shutwdown (OC_SD field of CONFIG register).
    L6474_CONFIG_OC_SD_t overcurrent_shutwdown;
     /// Torque regulation method (EN_TQREG field of CONFIG register).
    L6474_CONFIG_EN_TQREG_t torque_regulation_method;
     /// Step selection (STEP_SEL field of STEP_MODE register).
    L6474_STEP_SEL_t step_selection;
     /// Sync selection (SYNC_SEL field of STEP_MODE register).
    L6474_SYNC_SEL_t sync_selection;
     /// Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
    L6474_FAST_STEP_t fall_time;
     /// Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
    L6474_TOFF_FAST_t maximum_fast_decay_time;
     /// Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
    float minimum_ON_time_us;
     /// Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
    float minimum_OFF_time_us;
     /// Target Swicthing Period (field TOFF of CONFIG register).
    L6474_CONFIG_TOFF_t target_swicthing_period;
     /// Slew rate (POW_SR field of CONFIG register).
    L6474_CONFIG_POW_SR_t slew_rate;
     /// Clock setting (OSC_CLK_SEL field of CONFIG register).
    L6474_CONFIG_OSC_MGMT_t clock;
     /// Alarm (ALARM_EN register).
    uint32_t alarm;
} L6474_Init_t;

/**
  * @}
  */

/** @defgroup Device_Parameters Device Parameters
  * @{
  */
#if defined(NO_LONGER_USED)
                         //------------------------------------------------------------
                         //                       motor_blk
                         //                  Motor Block Structure
                         //
                         // There is one MOTOR_BLOCK per motor in the system.
                         // It contains all the key parameters for running the motor.
                         //------------------------------------------------------------
typedef struct {
                         // Position in steps that the motor reported from a ABS_POS cmd
    volatile int32_t   mtr_reported_position;
                         // Position in steps at the start of the goto or move commands
    volatile int32_t   currentPosition;
                         // number of steps to perform for the GOTO or MOVE commands
    volatile uint32_t  stepsToTake;
                         // Position in steps at the START of the goto or move commands
    volatile int32_t   startPosition;
                         // Position in steps at the END of the goto or move commands
    volatile int32_t   endPosition;
                         // number of steps performed from the begining of the goto or the move command
    volatile uint32_t  relativePosition;
                         // position in steps at the end of the accelerating phase
    volatile uint32_t  endAccelPosition;
                         // position in steps at the start of the decelerating phase
    volatile uint32_t  startDecelPosition;

                         // acceleration in pps^2    (pulses per second squared)
    volatile uint16_t  acceleration;
                         // deceleration in pps^2
    volatile uint16_t  deceleration;
                         // max speed in pps (speed to use for GOTO or MOVE command)
    volatile uint16_t  maxSpeed;
                         // min speed in pps
    volatile uint16_t  minSpeed;
                         // current speed in pps
    volatile uint16_t  speed;
                         // accumulator used to store speed increase smaller than 1 pps
    volatile uint32_t  accu;

    volatile uint8_t   mtr_homed_flag;         // 1 denotes motor was physically or logically homed.

    volatile motorState_t    motionState;      // Current State of the device
    volatile deviceCommand_t commandExecuted;  // command under execution
    volatile motorDir_t      direction;        // FORWARD or BACKWARD direction
} MOTOR_BLOCK_L6474;
#endif

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/


/** @defgroup L6474_Exported_Functions L6474 Exported Functions
  * @{
  */

/** @defgroup Device_Control_Functions Device Control Functions
  * @{
  */


                       //  WVD MODIFIED

void     L6474_AttachErrorHandler(void (*callback)(uint16_t));  //Attach a user callback to the error handler
void     L6474_AttachFlagInterrupt(void (*callback)(void));     //Attach a user callback to the flag Interrupt
void     L6474_Init(void* pInit);                               //Start the L6474 library
uint16_t L6474_GetAcceleration(MOTOR_BLOCK *mtr_blk);       //Return the acceleration in pps^2
uint16_t L6474_GetCurrentSpeed(MOTOR_BLOCK *mtr_blk);       //Return the current speed in pps
uint16_t L6474_GetDeceleration(MOTOR_BLOCK *mtr_blk);       //Return the deceleration in pps^2
//motorState_t L6474_GetDeviceState(MOTOR_BLOCK *mtr_blk);    //Return the device state

//motorDrv_t * L6474_GetMotorHandle(void);                   //Return handle of the motor driver handle
MOTOR_HANDLERS * L6474_GetMotorHandle(void);                 //Return handle of the motor driver handle
uint8_t  L6474_GetFwVersion(void);                           //Return the FW version
int32_t  L6474_GetMark(MOTOR_BLOCK *mtr_blk);                //Return the mark position
uint16_t L6474_GetMaxSpeed(MOTOR_BLOCK *mtr_blk);           //Return the max speed in pps
uint16_t L6474_GetMinSpeed(MOTOR_BLOCK *mtr_blk);           //Return the min speed in pps
int32_t  L6474_GetPosition(MOTOR_BLOCK *mtr_blk);            //Return the ABS_POSITION (32b signed)
void     L6474_GoHome(MOTOR_BLOCK *mtr_blk);                    //Move to the home position
void     L6474_GoMark(MOTOR_BLOCK *mtr_blk);                    //Move to the Mark position
void     L6474_GoTo(MOTOR_BLOCK *mtr_blk, int32_t targetPosition);  //Go to the specified position
void     L6474_HardStop(MOTOR_BLOCK *mtr_blk);                  //Stop the motor
void     L6474_HizStop(MOTOR_BLOCK *mtr_blk) ;                  //Stop the motor and disable the power bridge
//void     L6474_Move(MOTOR_BLOCK *mtr_blk,                       //Move the motor of the specified number of steps
//                  motorDir_t direction, uint32_t stepCount);
uint16_t L6474_ReadId(void);                                //Read Id to get driver instance
void     L6474_ResetAllDevices(void);                           //Reset all L6474 devices
//void     L6474_Run(MOTOR_BLOCK *mtr_blk, motorDir_t direction); //Run the motor
bool     L6474_SetAcceleration(MOTOR_BLOCK *mtr_blk,uint16_t newAcc); //Set the acceleration in pps^2
bool     L6474_SetDeceleration(MOTOR_BLOCK *mtr_blk,uint16_t newDec); //Set the deceleration in pps^2
void     L6474_SetHome(MOTOR_BLOCK *mtr_blk);                         //Set current position to be the home position
void     L6474_SetMark(MOTOR_BLOCK *mtr_blk);                         //Set current position to be the Markposition
bool     L6474_SetMaxSpeed(MOTOR_BLOCK *mtr_blk,uint16_t newMaxSpeed); //Set the max speed in pps
bool     L6474_SetMinSpeed(MOTOR_BLOCK *mtr_blk,uint16_t newMinSpeed); //Set the min speed in pps
bool     L6474_SetNumMotors(uint8_t nbDevices);                        //Set the number of devices
bool     L6474_SoftStop(MOTOR_BLOCK *mtr_blk);                         //Progressively stops the motor
void     L6474_WaitWhileActive(MOTOR_BLOCK *mtr_blk);                  //Wait for the device state becomes Inactive

/** @defgroup L6474_Control_Functions L6474 Control Functions
  * @{
  */
void     L6474_CmdDisable(MOTOR_BLOCK *mtr_blk);          //Send the L6474_DISABLE command
void     L6474_CmdEnable(MOTOR_BLOCK *mtr_blk);           //Send the L6474_ENABLE command
uint32_t L6474_CmdGetParam(MOTOR_BLOCK *mtr_blk,          //Send the L6474_GET_PARAM command
                           uint32_t param);
uint16_t L6474_CmdGetStatus(MOTOR_BLOCK *mtr_blk);        // Send the L6474_GET_STATUS command
void     L6474_CmdNop(MOTOR_BLOCK *mtr_blk);              //Send the L6474_NOP command
void     L6474_CmdSetParam(MOTOR_BLOCK *mtr_blk,          //Send the L6474_SET_PARAM command
                       uint32_t param, uint32_t value);
float    L6474_GetAnalogValue(MOTOR_BLOCK *mtr_blk, uint32_t param); //Get parameters (the value is formatted from register value to float)
uint16_t L6474_ReadStatusRegister(MOTOR_BLOCK *mtr_blk);  // Read the L6474_STATUS register without
                                                          // clearing the flags
void     L6474_Reset(MOTOR_BLOCK *mtr_blk);               //Set the L6474 reset pin
void     L6474_ReleaseReset(MOTOR_BLOCK *mtr_blk);        //Release the L6474 reset pin
void     L6474_SelectStepMode(MOTOR_BLOCK *mtr_blk,       // Step mode selection
                              int stepMod);
void     L6474_SetDirection(MOTOR_BLOCK *mtr_blk,         //Set the L6474 direction pin
                            int direction);
bool     L6474_SetAnalogValue(MOTOR_BLOCK *mtr_blk, uint32_t param, float value); //Set the parameters by making a conversion to register format

/**
  * @}
  */

/** @defgroup MotorControl_Board_Linked_Functions MotorControl Board Linked Functions
  * @{
  */
                   //--------------------------------------
                   //        Board sepcific calls
                   //--------------------------------------
///Delay of the requested number of milliseconds
extern void L6474_Board_Delay(uint32_t delay);
///Enable Irq
extern void L6474_Board_EnableIrq(void);
///Disable Irq
extern void L6474_Board_DisableIrq(void);
///Initialise GPIOs used for L6474s
extern void L6474_Board_GpioInit(uint8_t motor_id);
///Set PWM1 frequency and start it
extern void L6474_Board_Pwm1SetFreq(uint16_t newFreq);
///Set PWM2 frequency and start it
extern void L6474_Board_Pwm2SetFreq(uint16_t newFreq);
///Set PWM3 frequency and start it
extern void L6474_Board_Pwm3SetFreq(uint16_t newFreq);
///Init the PWM of the specified device
extern void L6474_Board_PwmInit(uint8_t motor_id);
///Stop the PWM of the specified device
extern void L6474_Board_PwmStop(uint8_t motor_id);
///Reset the L6474 reset pin
extern void L6474_Board_ReleaseReset(uint8_t motor_id);
///Set the L6474 reset pin
extern void L6474_Board_Reset(uint8_t motor_id);
///Set direction GPIO
extern void L6474_Board_SetDirectionGpio(uint8_t motor_id, uint8_t gpioState);
///Initialise the SPI used for L6474s
extern uint8_t L6474_Board_SpiInit(void);
///Write bytes to the L6474s via SPI
extern uint8_t L6474_Board_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t nbDevices);
/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
  }
#endif

#endif /* #ifndef __L6474_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
