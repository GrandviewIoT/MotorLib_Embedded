/**
 ******************************************************************************
 * @file    MC_SixStep_param.h
 * @author  System lab - Automation and Motion control team
 * @version V1.0.0
 * @date    06-July-2015
 * @brief   This header file provides all parameters to driver a motor with 6Step
            library
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/** @addtogroup MIDDLEWARES     MIDDLEWARES 
  * @brief  Middlewares Layer
  * @{ 
*/

/** @addtogroup MC_6-STEP_LIB       MC_6-STEP_LIB 
  * @brief  Motor Control driver
  * @{ 
*/

/** @defgroup Main_Motor_parameters    Main_Motor_parameters
  *  @{
    * @brief All motor parameters for 6Step driving
*/
 
/* **************************************************************************** 
 ==============================================================================   
                       ###### BASIC PARAMETERS ######
 ============================================================================== 
**************************************************************************** */   
#define NUM_POLE_PAIRS                       7      /*!< Number of Motor Pole pairs */
#define DIRECTION                            0      /*!< Set motor direction CW = 0 and CCW = 1*/ 
#define TARGET_SPEED                      3000      /*!< Target speed in closed loop control */ 


/* **************************************************************************** 
 ==============================================================================   
                       ###### ADVANCED PARAMETERS ######
 ============================================================================== 
**************************************************************************** */

#if defined(USES_L6230)
                        //-----------------------------
                        //  L6230 Parameters
                        //-----------------------------
/*!< ********************* Open loop control *********************************/
#define STARTUP_CURRENT_REFERENCE         2000      /*!< StartUP Currente Reference (2000 = 2.2A)*/
#define ACC                             600000      /*!< Mechanical acceleration rate (setting available in manual mode, LOAD_TYPE = 0) */ 
#define MINIMUM_ACC                        500      /*!< Mechanical acceleration rate for BIG load application */ 
#define NUMBER_OF_STEPS                  20000      /*!< Number of elements for motor start-UP (max value 65535)*/ 
#define TIME_FOR_ALIGN                     500      /*!< Time for alignment (msec)*/ 
#define BUTTON_DELAY                      1000      /*!< Delay time to enable push button for new command (1 = 1msec)*/ 
#define NUMBER_ZCR                          12      /*!< Number of zero crossing event during the startup for closed loop control begin */   
/*!< ********************* Closed Loop control *********************************/
#define SPEED_LOOP_TIME                      1      /*!< Speed Loop time (1 = 1msec) */  
#define KP_GAIN                           8000      /*!< Kp parameter for PI regulator */
#define KI_GAIN                             50      /*!< Ki parameter for PI regulator */   
#define KP_DIV                            4096      /*!< Kp parameter divider for PI regulator */
#define KI_DIV                            4096      /*!< Ki parameter divider for PI regulator */   
#define LOWER_OUT_LIMIT                    120      /*!< Low Out value of PI regulator */      
#define UPPER_OUT_LIMIT                   2000      /*!< High Out value of PI regulator */    
#define MAX_POT_SPEED                    10000      /*!< Maximum Speed regulated by potentiometer */
#define MIN_POT_SPEED                     1500      /*!< Minimum Speed regulated by potentiometer */
#define VAL_POT_SPEED_DIV                    2      /*!< Validation potentiometer speed divider */
#define INITIAL_DEMAGN_DELAY                10      /*!< Initial value for delay time during startup for Bemf detection */

#define SYNCHRONOUS_RECTIFICATION           0       /*!< Only allowed for L6398, else is always 0 */

/*!< Zero Crossissing parameters */
#define BEMF_THRSLD_DOWN                    200     /*!< Zero Crossing threshold */  
#define BEMF_THRSLD_UP                      200     /*!< Zero Crossing threshold */ 

/*!< Speed filtering parameters */
#define FILTER_DEEP                         20      /*!< Number of bits for digital filter */  
#define HFBUFFERSIZE                        10
#define ADC_SPEED_TH                        82      /*!<Fixed treshold to change the target speed (t.b.f) */ 

/*!< Motor stall detection parameters */
#define BEMF_CONSEC_DOWN_MAX                10     /*!< Maximum value of BEMF Consecutive Threshold Falling Crossings Counter in closed loop */       
#define BEMF_CNT_EVENT_MAX                 100     /*!< Maximum number of BEMF Counter in open loop*/       

/*!< Debug pin */
#define GPIO_ZERO_CROSS                      1     /*!< Enable (1) the GPIO toggling for zero crossing detection */  
#define GPIO_COMM                            1     /*!< Enable (1) the GPIO toggling for commutation */  
#endif                      // defined(USE_L6230)


#if defined(L6398)
                            //-----------------------------
                            //  L6398 Parameters
                            //-----------------------------
/*!< ********************* Open loop control *********************************/
#define STARTUP_CURRENT_REFERENCE         1000      /*!< StartUP Currente Reference */
#define ACC                              10000      /*!< Mechanical acceleration rate */ 
#define MINIMUM_ACC                        500      /*!< Mechanical acceleration rate for BIG load application */ 
#define NUMBER_OF_STEPS                  20000      /*!< Number of elements for motor start-UP (max value 65535)*/ 
#define TIME_FOR_ALIGN                     100      /*!< Time for alignment (msec)*/ 
#define BUTTON_DELAY                      1000      /*!< Delay time to enable push button for new command (1 = 1msec)*/ 
#define NUMBER_ZCR                          12      /*!< Number of zero crossing event during the startup for closed loop control begin */   

/*!< ********************* Closed Loop control - CURRENT MODE *********************************/
#define SPEED_LOOP_TIME                      1      /*!< Speed Loop time (1 = 1msec) */  
#define KP_GAIN                           6000      /*!< Kp parameter for PI regulator */
#define KI_GAIN                            150      /*!< Ki parameter for PI regulator */   
#define KP_DIV                            8192      /*!< Kp parameter divider for PI regulator */
#define KI_DIV                            8192      /*!< Ki parameter divider for PI regulator */   
#define LOWER_OUT_LIMIT                    100      /*!< Low Out value of PI regulator */      
#define UPPER_OUT_LIMIT                  16384      /*!< High Out value of PI regulator */    
#define MAX_POT_SPEED                     6000      /*!< Maximum Speed regulated by potentiometer */
#define MIN_POT_SPEED                     1000      /*!< Minimum Speed regulated by potentiometer */
#define VAL_POT_SPEED_DIV                    2      /*!< Validation potentiometer speed divider */
#define INITIAL_DEMAGN_DELAY                 2      /*!< Initial value for delay time during startup for Bemf detection */

/*!< ********************* Closed Loop control - VOLTAGE MODE *********************************/
#define KP_GAIN_VM                          100     /*!< Kp parameter for PI regulator */
#define KI_GAIN_VM                           30     /*!< Ki parameter for PI regulator */   
#define KP_DIV_VM                          8192     /*!< Kp parameter divider for PI regulator */
#define KI_DIV_VM                          8192     /*!< Ki parameter divider for PI regulator */   
#define LOWER_OUT_LIMIT_VM                  400     /*!< Low Out value of PI regulator */      
#define UPPER_OUT_LIMIT_VM                 1799     /*!< High Out value of PI regulator */   
#define DUTY_CYCLE_INIT_VALUE               199     /*!< Initial duty cycle value during startup */          

/*!< *********** Syncronous rectification for high frequency leg ****************/
#define SYNCHRONOUS_RECTIFICATION           1       /*!< Enable (1) or Disable (0) the complementary output of high frequency leg for synchronous rectification */

/*!< Zero Crossissing parameters */
#define BEMF_THRSLD_DOWN                    80      /*!< Zero Crossing threshold */  
#define BEMF_THRSLD_UP                      80      /*!< Zero Crossing threshold */ 

/*!< Speed filtering parameters */
#define FILTER_DEEP                         20      /*!< Number of bits for digital filter */  
#define HFBUFFERSIZE                        10
#define ADC_SPEED_TH                        82      /*!<Fixed treshold to change the target speed (t.b.f) */ 

/*!< Motor stall detection parameters */
#define BEMF_CONSEC_DOWN_MAX                10      /*!< Maximum value of BEMF Consecutive Threshold Falling Crossings Counter in closed loop */       
#define BEMF_CNT_EVENT_MAX                 100      /*!< Maximum number of BEMF Counter in open loop*/       

/*!< Debug pin */
#define GPIO_ZERO_CROSS                      1      /*!< Enable (1) the GPIO toggling for zero crossing detection */  
#define GPIO_COMM                            1      /*!< Enable (1) the GPIO toggling for commutation */  
#endif                      // defined(L6398)


                            //-----------------------------
                            //    Demo mode parameters
                            //-----------------------------
#define DEMO_START_TIME                   5000     /*!< Time (msec) to keep the motor in run mode */  
#define DEMO_STOP_TIME                    2000     /*!< Time (msec) to keep the motor in stop mode */  

/*!< Look UP table for dynamic demagn control of speed  */  
#define DEMAGN_VAL_1                         1     /*!< Look UP table for dynamic demagn control for speed into (10000,12000] or [-12000,-10000) range */  
#define DEMAGN_VAL_2                         2     /*!< Look UP table for dynamic demagn control for speed into ( 7800,10000] or [-10000,- 7800) range */  
#define DEMAGN_VAL_3                         3     /*!< Look UP table for dynamic demagn control for speed into ( 6400, 7800] or [- 7800,- 6400) range */  
#define DEMAGN_VAL_4                         4     /*!< Look UP table for dynamic demagn control for speed into ( 5400, 6400] or [- 6400,- 5400) range */    
#define DEMAGN_VAL_5                         5     /*!< Look UP table for dynamic demagn control for speed into ( 4650, 5400] or [- 5400,- 4650) range */  
#define DEMAGN_VAL_6                         6     /*!< Look UP table for dynamic demagn control for speed into ( 4100, 4650] or [- 4650,- 4100) range */  
#define DEMAGN_VAL_7                         7     /*!< Look UP table for dynamic demagn control for speed into ( 3650, 4100] or [- 4100,- 3650) range */  
#define DEMAGN_VAL_8                         8     /*!< Look UP table for dynamic demagn control for speed into ( 3300, 3650] or [- 3650,- 3300) range */    
#define DEMAGN_VAL_9                         9     /*!< Look UP table for dynamic demagn control for speed into ( 2600, 3300] or [- 3300,- 2600) range */  
#define DEMAGN_VAL_10                       10     /*!< Look UP table for dynamic demagn control for speed into ( 1800, 2600] or [- 2600,- 1800) range */  
#define DEMAGN_VAL_11                       11     /*!< Look UP table for dynamic demagn control for speed into ( 1500, 1800] or [- 1800,- 1500) range */  
#define DEMAGN_VAL_12                       12     /*!< Look UP table for dynamic demagn control for speed into ( 1300, 1500] or [- 1500,- 1300) range */  
#define DEMAGN_VAL_13                       13     /*!< Look UP table for dynamic demagn control for speed into ( 1000, 1300] or [- 1300,- 1000) range */  
#define DEMAGN_VAL_14                       14     /*!< Look UP table for dynamic demagn control for speed into [  500, 1000] or [- 1000,-  500] range */  

#define TRUE                                 1     /*!< Define TRUE */  
#define FALSE                                0     /*!< Define FALSE */
