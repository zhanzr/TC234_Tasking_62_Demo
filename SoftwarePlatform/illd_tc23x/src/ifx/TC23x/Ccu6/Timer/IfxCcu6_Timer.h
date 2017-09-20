/**
 * \file IfxCcu6_Timer.h
 * \brief CCU6 TIMER details
 * \ingroup IfxLld_Ccu6
 *
 * \version iLLD_1_0_0_11_0
 * \copyright Copyright (c) 2013 Infineon Technologies AG. All rights reserved.
 *
 *
 *                                 IMPORTANT NOTICE
 *
 *
 * Infineon Technologies AG (Infineon) is supplying this file for use
 * exclusively with Infineon's microcontroller products. This file can be freely
 * distributed within development tools that are supporting such microcontroller
 * products.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *
 * \defgroup IfxLld_Ccu6_Timer_Usage How to use the CCU6 TIMER Interface driver?
 * \ingroup IfxLld_Ccu6_Timer
 *
 * The TIMER interface driver provides a default CCU6 configuration for runnung the selected timer block.
 *
 * User can select the various configuration options that hardware allows
 *
 * In the following sections it will be described, how to integrate the driver into the application framework.
 *
 *
 * \section IfxLld_Ccu6_Timer_Preparation Preparation
 *
 *
 * \subsection IfxLld_Ccu6_Timer_Include Include Files
 *
 * Include following header file into your C code:
 *
 * \code
 *     #include <Ccu6/Timer/IfxCcu6_Timer.h>
 * \endcode
 *
 * \subsection IfxLld_Ccu6_Timer_Variables Variables
 *
 * Declare the TIMER handle as global variable in your C code:
 *
 * \code
 *     // used globally
 *     static IfxCcu6_Timer timer;
 * \endcode
 *
 * \subsection IfxLld_Ccu6_Timer_Interrupt Interrupt Handler Installation
 *
 * See also \ref IfxLld_Cpu_Irq_Usage
 *
 * Define priority for the Interrrupt handler. This is normally done in the Ifx_IntPrioDef.h file:
 *
 * \code
 *     // priorities are normally defined in Ifx_IntPrioDef.h
 *     #define IFX_INTPRIO_CCU6 1
 * \endcode
 *
 * Add the interrupt service routine to your C code.
 *
 * \code
 *     IFX_INTERRUPT(ccu60ISR_Timer, 0, IFX_INTPRIO_CCU6)
 *     {
 *         //user code
 *     }
 * \endcode
 *
 * Finally install the interrupt handlers in your initialisation function:
 *
 * \code
 *     // install interrupt handlers
 *     IfxCpu_Irq_installInterruptHandler(&ccu60ISR_Timer, IFX_INTPRIO_CCU6);
 *     IfxCpu_enableInterrupts();
 * \endcode
 *
 * \subsection IfxLld_Ccu6_Timer_Init Module Initialisation
 *
 * The module initialisation can be done in the same function. Here an example:
 *
 * \code
 *     // create configuration
 *     IfxCcu6_Timer_Config timerConfig;
 *     IfxCcu6_Timer_initModuleConfig(&timerConfig, &MODULE_CCU60);
 *
 *     // configure the frequency of the timer in case of internal start
 *     // this frequency will be set for the timer block selected later
 *     timerConfig.base.frequency = 400000;
 *
 *     // configure the period of the timer
 *     timerConfig.base.period = 100;
 *
 *     // configure the waiting time in case of delayed T13 start in sync with T12
 *     timerConfig.base.waitingTime = 0;
 *
 *     // select the timer that needs to be started
 *     timerConfig.timer = IfxCcu6_TimerId_t12;
 *
 *     // select the synchronous operation if both timers need to be start at the same time
 *     // previous selection of timer block can be ignored in this mode
 *     timerConfig.synchronousOperation = TRUE;
 *
 *     // configure the clock for internal mode
 *     timerConfig.clock.t12ExtClockEnabled   = FALSE;
 *     timerConfig.clock.t12ExtClockInput     = NULL_PTR;
 *     timerConfig.clock.t12countingInputMode = IfxCcu6_CountingInputMode_internal;
 *
 *     // configure the selcted timer block
 *     timerConfig.timer12.countMode     = IfxCcu6_T12CountMode_edgeAligned;
 *     timerConfig.timer12.counterValue     = 0;
 *
 *     // configure the interrupts
 *     timerConfig.interrupt.interruptSource = IfxCcu6_InterruptSource_t12PeriodMatch;
 *     timerConfig.interrupt.serviceRequest  = IfxCcu6_ServiceRequest_2;
 *     timerConfig.interrupt.priority        = IFX_INTRPRIO_CCU6;
 *     timerConfig.interrupt.typeOfService   = IfxSrc_Tos_cpu0;
 *
 *     // configure input and output triggers
 *     timerConfig.trigger.t12ExtInputTrigger   = IfxCcu60_T12HRB_P00_7_IN;
 *     timerConfig.trigger.t13ExtInputTrigger   = NULL_PTR;
 *     timerConfig.trigger.extInputTriggerMode  = IfxCcu6_ExternalTriggerMode_risingEdge;
 *     timerConfig.trigger.t13InSyncWithT12     = FALSE;
 *
 *     // initialize the module
 *     IfxCcu6_Timer_initModule(&timer, &timerConfig);
 * \endcode
 *
 *
 * The Timer is ready for use now!
 *
 *
 * \section IfxLld_Ccu6_Timer_Control Control
 *
 *
 * The TIMER driver provides simple to use timer Control functions
 *
 * This means: In addition to start and stop a single timer, you can start and stop both the timer blocks at the same time
 * you can also run the timer in single shot mode and finally you an manually make the timer count each step
 *
 * Start the timer
 *
 * \code
 *     IfxCcu6_Timer_start(&timer);
 * \endcode
 *
 * Stop the timer
 *
 * \code
 *     IfxCcu6_Timer_stop(&timer);
 * \endcode
 *
 * Start both the timers synchronously, when the synchronousOperation mode is selected in the configuration
 *
 * \code
 *     IfxCcu6_Timer_synchronousStart(&timer);
 * \endcode
 *
 * Stop both the timers synchronously, when the synchronousOperation mode is selected in the configuration
 *
 * \code
 *     IfxCcu6_Timer_synchronousStop(&timer);
 * \endcode
 *
 * Start the selected timer in single shot mode
 *
 * \code
 *     IfxCcu6_Timer_startSingleShotMode(&timer);
 * \endcode
 *
 * Make the timer count each step manually
 *
 * \code
 *     uint8 i;
 *     for (i=0; i<50; i++)
 *     {
 *         IfxCcu6_Timer_countOneStep(&timer);
 *     }
 * \endcode
 *
 * \defgroup IfxLld_Ccu6_Timer TIMER Interface driver
 * \ingroup IfxLld_Ccu6
 * \defgroup IfxLld_Ccu6_Timer_DataStructures Data Structures
 * \ingroup IfxLld_Ccu6_Timer
 * \defgroup IfxLld_Ccu6_Timer_Module_Initialize_Functions Module Initialize Functions
 * \ingroup IfxLld_Ccu6_Timer
 * \defgroup IfxLld_Ccu6_Timer_Timer_Control_Functions Timer Control Functions
 * \ingroup IfxLld_Ccu6_Timer
 */

#ifndef IFXCCU6_TIMER_H
#define IFXCCU6_TIMER_H 1

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "Ccu6/Std/IfxCcu6.h"
#include "If/Ccu6If/Timer.h"

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

/** \addtogroup IfxLld_Ccu6_Timer_DataStructures
 * \{ */
/** \brief Structure for clock configuration
 */
typedef struct
{
    boolean                   t12ExtClockEnabled;       /**< \brief Timer 12 external clock enable / disable selection */
    IfxCcu6_T12hr_In         *t12ExtClockInput;         /**< \brief External input signal selection for timer 12 */
    IfxCcu6_CountingInputMode t12countingInputMode;     /**< \brief Input event leading to a counting action of the timer T12 */
    boolean                   t13ExtClockEnabled;       /**< \brief Timer 13 external clock enable / disable selection */
    IfxCcu6_T13hr_In         *t13ExtClockInput;         /**< \brief External input signal selection for timer 13 */
    IfxCcu6_CountingInputMode t13countingInputMode;     /**< \brief Input event leading to a counting action of the timer T13 */
} IfxCcu6_Timer_Clock;

/** \brief Structure for interrupt configuration
 */
typedef struct
{
    IfxCcu6_InterruptSource source;             /**< \brief Interrupt source selection */
    IfxCcu6_ServiceRequest  serviceRequest;     /**< \brief Selection of service request outputs */
    uint16                  priority;           /**< \brief Interrupt priority */
    IfxSrc_Tos              typeOfService;      /**< \brief type of interrupt service */
} IfxCcu6_Timer_InterruptConfig;

/** \brief Structure for Timer 12
 */
typedef struct
{
    IfxCcu6_T12CountMode countMode;        /**< \brief Operating mode of Timer 12 */
    uint16               counterValue;     /**< \brief 16-bit counter value of Timer12 */
} IfxCcu6_Timer_Timer12;

/** \brief Structure for Timer 13
 */
typedef struct
{
    uint16                      counterValue;         /**< \brief 16-bit counter value of Timer13 */
    IfxCcu6_T13TriggerEvent     t12SyncEvent;         /**< \brief T12 sync trigger event to start T13 */
    IfxCcu6_T13TriggerDirection t12SyncDirection;     /**< \brief Additional information to control trigger event selection */
} IfxCcu6_Timer_Timer13;

/** \brief Configuration structure for external triggers
 */
typedef struct
{
    IfxCcu6_T12hr_In           *t12ExtInputTrigger;      /**< \brief External input signal selection for timer 12 */
    IfxCcu6_T13hr_In           *t13ExtInputTrigger;      /**< \brief External input signal selection for timer 13 */
    IfxCcu6_ExternalTriggerMode extInputTriggerMode;     /**< \brief Event of signal T1xHR that can set the run bit T1xR by HW */
    boolean                     t13InSyncWithT12;        /**< \brief Selection of Timer 13 start in sync with T12 */
} IfxCcu6_Timer_TriggerConfig;

/** \} */

/** \brief Structure for CCU6 output pin configuration
 */
typedef struct
{
    const IfxCcu6_T12hr_In *t12hr;              /**< \brief T12HR input signal */
    const IfxCcu6_T13hr_In *t13hr;              /**< \brief T13HR input signal */
    IfxPort_InputMode       t1xhrInputMode;     /**< \brief The T1xHR pin input mode which should be configured */
} IfxCcu6_Timer_Pins;

/** \addtogroup IfxLld_Ccu6_Timer_DataStructures
 * \{ */
/** \brief Module handle
 */
typedef struct
{
    Timer                       base;        /**< \brief Base Timer object */
    Ifx_CCU6                   *ccu6;        /**< \brief Pointer to the base of CCU6 registers */
    IfxCcu6_TimerId             timer;       /**< \brief Timer number (T12 / T13) */
    IfxCcu6_Timer_TriggerConfig trigger;     /**< \brief Structure for trigger configuration */
} IfxCcu6_Timer;

/** \brief Configuration structure of the module
 */
typedef struct
{
    Timer_Config                  base;                     /**< \brief Base configuration */
    Ifx_CCU6                     *ccu6;                     /**< \brief Pointer to the base of CCU6 registers */
    IfxCcu6_TimerId               timer;                    /**< \brief Timer number (T12 / T13) */
    boolean                       synchronousOperation;     /**< \brief Synchronous operation selection (starting / stopping both the timers at the same time) */
    IfxCcu6_Timer_Clock           clock;                    /**< \brief Structure for clock configuration */
    IfxCcu6_Timer_Timer12         timer12;                  /**< \brief Structure for Timer 12 */
    IfxCcu6_Timer_Timer13         timer13;                  /**< \brief Structure for Timer 13 */
    IfxCcu6_Timer_InterruptConfig interrupt1;               /**< \brief Structure for first interrupt configuration */
    IfxCcu6_Timer_InterruptConfig interrupt2;               /**< \brief Structure for second interrupt configuration */
    IfxCcu6_Timer_InterruptConfig interrupt3;               /**< \brief Structure for third interrupt configuration */
    IfxCcu6_Timer_InterruptConfig interrupt4;               /**< \brief Structure for fourth interrupt configuration */
    IfxCcu6_Timer_TriggerConfig   trigger;                  /**< \brief Structure for trigger configuration */
    IfxCcu6_Timer_Pins           *pins;                     /**< \brief Structure for CCU6 output pin configuration */
} IfxCcu6_Timer_Config;

/** \} */

/** \addtogroup IfxLld_Ccu6_Timer_Module_Initialize_Functions
 * \{ */

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief Initialises the module with default configuration
 * \param timer Module handle
 * \param config Configuration structure of the module
 * \return None
 *
 * A coding example can be found in \ref IfxLld_Ccu6_Timer_Usage
 *
 */
IFX_EXTERN void IfxCcu6_Timer_initModule(IfxCcu6_Timer *timer, const IfxCcu6_Timer_Config *config);

/** \brief Fills the config structure with default values
 * \param config Configuration structure of the module
 * \param ccu6 Pointer to the base of CCU6 registers
 * \return None
 *
 * A coding example can be found in \ref IfxLld_Ccu6_Timer_Usage
 *
 */
IFX_EXTERN void IfxCcu6_Timer_initModuleConfig(IfxCcu6_Timer_Config *config, Ifx_CCU6 *ccu6);

/** \} */

/** \addtogroup IfxLld_Ccu6_Timer_Timer_Control_Functions
 * \{ */

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief Counts the timer one step
 * \param timer Module handle
 * \return None
 *
 * A coding example can be found in \ref IfxLld_Ccu6_Timer_Usage
 *
 */
IFX_EXTERN void IfxCcu6_Timer_countOneStep(IfxCcu6_Timer *timer);

/** \brief Starts the timer
 * \param timer Module handle
 * \return None
 *
 * A coding example can be found in \ref IfxLld_Ccu6_Timer_Usage
 *
 */
IFX_EXTERN void IfxCcu6_Timer_start(IfxCcu6_Timer *timer);

/** \brief Starts the single shot mode of the timer
 * \param timer Module handle
 * \return None
 *
 * A coding example can be found in \ref IfxLld_Ccu6_Timer_Usage
 *
 */
IFX_EXTERN void IfxCcu6_Timer_startSingleShotMode(IfxCcu6_Timer *timer);

/** \brief Stops the timer
 * \param timer Module handle
 * \return None
 *
 * A coding example can be found in \ref IfxLld_Ccu6_Timer_Usage
 *
 */
IFX_EXTERN void IfxCcu6_Timer_stop(IfxCcu6_Timer *timer);

/** \brief Starts both the timers together
 * \param timer Module handle
 * \return None
 *
 * A coding example can be found in \ref IfxLld_Ccu6_Timer_Usage
 *
 */
IFX_EXTERN void IfxCcu6_Timer_synchronousStart(IfxCcu6_Timer *timer);

/** \brief Starts both the timers together
 * \param timer Module handle
 * \return None
 *
 * A coding example can be found in \ref IfxLld_Ccu6_Timer_Usage
 *
 */
IFX_EXTERN void IfxCcu6_Timer_synchronousStop(IfxCcu6_Timer *timer);

/** \} */

#endif /* IFXCCU6_TIMER_H */
