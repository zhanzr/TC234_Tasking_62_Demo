/**
 * \file IfxCcu6_PwmBc.c
 * \brief CCU6 PWMBC details
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
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "IfxCcu6_PwmBc.h"

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

uint32 IfxCcu6_PwmBc_getMotorSpeed(IfxCcu6_PwmBc *pwmBc)
{
    uint32  currentTime, previousTime;
    float32 speed;

    currentTime  = IfxCcu6_getCaptureRegisterValue(pwmBc->ccu6, IfxCcu6_T12Channel_0);
    previousTime = IfxCcu6_getCaptureShadowRegisterValue(pwmBc->ccu6, IfxCcu6_T12Channel_0);

    // TODO -- FIX IT --
    speed = (currentTime - previousTime) * (6 / (pwmBc->base.t13Frequency));
    speed = speed * 60 * 100000;    //convertion to RPM (minutes),
    return speed;
}


void IfxCcu6_PwmBc_initModule(IfxCcu6_PwmBc *pwmBc, const IfxCcu6_PwmBc_Config *config)
{
    Ifx_CCU6 *ccu6SFR = config->ccu6; // pointer to CCU6 registers
    pwmBc->ccu6 = ccu6SFR;            // adding register pointer to module handler

    /* -- hardware module initialisation -- */

    // enable module if it hasn't been enabled by any other interface //
    if (IfxCcu6_isModuleEnabled(ccu6SFR) == FALSE)
    {
        IfxCcu6_enableModule(ccu6SFR);
    }

    /* -- Timer13 initialisation -- */

    // enable Timer13 if it hasn't been enabled by any other interface //
    if (IfxCcu6_getTimerAvailabilityStatus(ccu6SFR, IfxCcu6_TimerId_t13) == FALSE)
    {
        IfxCcu6_enableTimer(ccu6SFR, IfxCcu6_TimerId_t13);
    }

    // clock initialisation //
    pwmBc->base.t13Frequency = IfxCcu6_setT13Frequency(ccu6SFR, config->base.t13Frequency, config->base.t13Period);

    // duty cycle initialisation //

    IfxCcu6_setT13CounterValue(ccu6SFR, config->timer13.counterValue);

    IfxCcu6_setT13CompareValue(ccu6SFR, config->timer13.compareValue);

    // if Timer 13 start is in sync with Timer 12 //

    if (config->trigger.t13InSyncWithT12)
    {
        IfxCcu6_setT13TriggerEventMode(ccu6SFR, config->timer13.t12SyncEvent);
        IfxCcu6_setT13TriggerEventDirection(ccu6SFR, config->timer13.t12SyncDirection);
    }

    /* -- Timer12 initialisation -- */

    // enable Timer12 if it hasn't been enabled by any other interface //
    if (IfxCcu6_getTimerAvailabilityStatus(ccu6SFR, IfxCcu6_TimerId_t12) == FALSE)
    {
        IfxCcu6_enableTimer(ccu6SFR, IfxCcu6_TimerId_t12);
    }

    // clock initialisation //

    IfxCcu6_setT12Frequency(ccu6SFR, config->base.t12Frequency, config->base.t12Period, config->timer12.countMode);

    // duty cycle initialisation //

    IfxCcu6_setT12CounterValue(ccu6SFR, config->timer12.counterValue);

    // hall effect noise filter initialisation
    if (config->base.noiseFilter)
    {
        IfxCcu6_setDeadTimeValue(ccu6SFR, (uint8)config->base.noiseFilter);
        IfxCcu6_enableDeadTime(ccu6SFR, IfxCcu6_T12Channel_0);
        IfxCcu6_disableDelayBypass(ccu6SFR);
    }

    // channel initialisation for hall sensor mode ( for Brush Less DC motor)
    IfxCcu6_setT12ChannelMode(ccu6SFR, IfxCcu6_T12Channel_0, IfxCcu6_T12ChannelMode_hallSensor);
    IfxCcu6_setT12ChannelMode(ccu6SFR, IfxCcu6_T12Channel_1, IfxCcu6_T12ChannelMode_hallSensor);
    IfxCcu6_setT12ChannelMode(ccu6SFR, IfxCcu6_T12Channel_2, IfxCcu6_T12ChannelMode_hallSensor);

    // phase delay initialisation
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, (uint16)config->base.phaseDelay);

    // multichannel control / output pattern control initialisation
    IfxCcu6_setMultiChannelSwitchingMode(ccu6SFR, config->multiChannelControl.switchingSelect);
    IfxCcu6_setMultiChannelSwitchingSync(ccu6SFR, config->multiChannelControl.switchingSync);

    /* -- output path initialisation -- */

    //enable Timer13 modulation output path for all T12 channel outs//

    IfxCcu6_enableModulationOutput(ccu6SFR, IfxCcu6_TimerId_t13, IfxCcu6_ChannelOut_cc0);
    IfxCcu6_enableModulationOutput(ccu6SFR, IfxCcu6_TimerId_t13, IfxCcu6_ChannelOut_cc1);
    IfxCcu6_enableModulationOutput(ccu6SFR, IfxCcu6_TimerId_t13, IfxCcu6_ChannelOut_cc2);

    if (config->pins->cout63 != NULL_PTR)
    {
        IfxCcu6_enableModulationOutput(ccu6SFR, IfxCcu6_TimerId_t13, IfxCcu6_ChannelOut_cout3);
    }

    // output passive logic configuration //
    //TODO check correct polarity:

    IfxCcu6_setOutputPassiveState(ccu6SFR, IfxCcu6_ChannelOut_cc0, config->base.activeState);
    IfxCcu6_setOutputPassiveState(ccu6SFR, IfxCcu6_ChannelOut_cc1, config->base.activeState);
    IfxCcu6_setOutputPassiveState(ccu6SFR, IfxCcu6_ChannelOut_cc2, config->base.activeState);

    if (config->pins->cout63 != NULL_PTR)

    {
        IfxCcu6_setOutputPassiveState(ccu6SFR, IfxCcu6_ChannelOut_cout3, config->base.activeState);
    }

    /* -- Pin mapping -- */

    const IfxCcu6_PwmBc_Pins *pins = config->pins;

    if (pins != NULL_PTR)
    {
        // modulation output pins

        IfxCcu6_Cc60_Out *cc60Out = pins->cc60Out;

        if (cc60Out != NULL_PTR)
        {
            IfxCcu6_initCc60OutPin(cc60Out, pins->outputMode, pins->pinDriver);
        }

        IfxCcu6_Cc61_Out *cc61Out = pins->cc61Out;

        if (cc61Out != NULL_PTR)
        {
            IfxCcu6_initCc61OutPin(cc61Out, pins->outputMode, pins->pinDriver);
        }

        IfxCcu6_Cc62_Out *cc62Out = pins->cc62Out;

        if (cc62Out != NULL_PTR)
        {
            IfxCcu6_initCc62OutPin(cc62Out, pins->outputMode, pins->pinDriver);
        }

        IfxCcu6_Cout60_Out *cout60 = pins->cout60;

        if (cout60 != NULL_PTR)
        {
            IfxCcu6_initCout60Pin(cout60, pins->outputMode, pins->pinDriver);
        }

        IfxCcu6_Cout61_Out *cout61 = pins->cout61;

        if (cout61 != NULL_PTR)
        {
            IfxCcu6_initCout61Pin(cout61, pins->outputMode, pins->pinDriver);
        }

        IfxCcu6_Cout62_Out *cout62 = pins->cout62;

        if (cout62 != NULL_PTR)
        {
            IfxCcu6_initCout62Pin(cout62, pins->outputMode, pins->pinDriver);
        }

        IfxCcu6_Cout63_Out *cout63 = pins->cout63;

        if (cout63 != NULL_PTR)
        {
            IfxCcu6_initCout63Pin(cout63, pins->outputMode, pins->pinDriver);
        }

        // hall sensor input pins

        IfxCcu6_Ccpos0_In *ccpos0 = pins->ccpos0;

        if (ccpos0 != NULL_PTR)
        {
            IfxCcu6_initCcpos0Pin(ccpos0, pins->inputMode);
        }

        IfxCcu6_Ccpos1_In *ccpos1 = pins->ccpos1;

        if (ccpos1 != NULL_PTR)
        {
            IfxCcu6_initCcpos1Pin(ccpos1, pins->inputMode);
        }

        IfxCcu6_Ccpos2_In *ccpos2 = pins->ccpos2;

        if (ccpos2 != NULL_PTR)
        {
            IfxCcu6_initCcpos2Pin(ccpos2, pins->inputMode);
        }

        IfxCcu6_T12hr_In *t12hr = pins->t12hr;

        if (t12hr != NULL_PTR)
        {
            IfxCcu6_initT12hrPin(t12hr, pins->t1xhrInputMode);
        }

        IfxCcu6_T13hr_In *t13hr = pins->t13hr;

        if (t13hr != NULL_PTR)
        {
            IfxCcu6_initT13hrPin(t13hr, pins->t1xhrInputMode);
        }
    }

    /* -- hall pattern configuration (when to start the hall pattern evaluation) -- */

    IfxCcu6_setHallSensorTriggerMode(ccu6SFR, config->hallSyncEvent);

    /* -- interrupt initialisation -- */

    if (config->interrupt1.priority > 0)
    {
        IfxCcu6_enableInterrupt(config->ccu6, config->interrupt1.source);
        IfxCcu6_routeInterruptNode(config->ccu6, config->interrupt1.source, config->interrupt1.serviceRequest);

        volatile Ifx_SRC_SRCR *src;
        src = IfxCcu6_getSrcAddress(config->ccu6, config->interrupt1.serviceRequest);
        IfxSrc_init(src, config->interrupt1.typeOfService, config->interrupt1.priority);
        IfxSrc_enable(src);
    }

    if (config->interrupt2.priority > 0)
    {
        IfxCcu6_enableInterrupt(config->ccu6, config->interrupt2.source);
        IfxCcu6_routeInterruptNode(config->ccu6, config->interrupt2.source, config->interrupt2.serviceRequest);

        volatile Ifx_SRC_SRCR *src;
        src = IfxCcu6_getSrcAddress(config->ccu6, config->interrupt2.serviceRequest);
        IfxSrc_init(src, config->interrupt2.typeOfService, config->interrupt2.priority);
        IfxSrc_enable(src);
    }

    if (config->interrupt3.priority > 0)
    {
        IfxCcu6_enableInterrupt(config->ccu6, config->interrupt3.source);
        IfxCcu6_routeInterruptNode(config->ccu6, config->interrupt3.source, config->interrupt3.serviceRequest);

        volatile Ifx_SRC_SRCR *src;
        src = IfxCcu6_getSrcAddress(config->ccu6, config->interrupt3.serviceRequest);
        IfxSrc_init(src, config->interrupt3.typeOfService, config->interrupt3.priority);
        IfxSrc_enable(src);
    }

    if (config->interrupt4.priority > 0)
    {
        IfxCcu6_enableInterrupt(config->ccu6, config->interrupt4.source);
        IfxCcu6_routeInterruptNode(config->ccu6, config->interrupt4.source, config->interrupt4.serviceRequest);

        volatile Ifx_SRC_SRCR *src;
        src = IfxCcu6_getSrcAddress(config->ccu6, config->interrupt4.serviceRequest);
        IfxSrc_init(src, config->interrupt4.typeOfService, config->interrupt4.priority);
        IfxSrc_enable(src);
    }

    /* -- output trigger initialisation --*/

    if (config->trigger.outputTriggerEnabled)
    {
        IfxCcu6_connectTrigger(ccu6SFR, config->trigger.outputLine, config->trigger.outputTrigger);
    }

    pwmBc->trigger          = config->trigger;
    pwmBc->hallPatternIndex = 0;

#if IFX_CFG_USE_STANDARD_INTERFACE
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, (uint32)pwmBc == ((uint32)&pwmBc->base));
    pwmBc->base.functions.start             = (PwmBc_Start) & IfxCcu6_PwmBc_start;
    pwmBc->base.functions.stop              = (PwmBc_Stop) & IfxCcu6_PwmBc_stop;
    pwmBc->base.functions.updateHallPattern = (PwmBc_UpdateHallPattern) & IfxCcu6_PwmBc_updateHallPattern;
    pwmBc->base.functions.getMotorSpeed     = (PwmBc_GetMotorSpeed) & IfxCcu6_PwmBc_getMotorSpeed;
#endif
}


void IfxCcu6_PwmBc_initModuleConfig(IfxCcu6_PwmBc_Config *config, Ifx_CCU6 *ccu6)
{
    const IfxCcu6_PwmBc_Config defaultConfig = {
        .ccu6              = NULL_PTR, // will be initialized below

        .base.t12Frequency = 400000,
        .base.t13Frequency = 400000,
        .base.t12Period    = 100,
        .base.t13Period    = 100,
        .base.phaseDelay   = 20,
        .base.noiseFilter  = 10,
        .base.activeState  = Ifx_ActiveState_high,

        .timer12           = {
            .countMode    = IfxCcu6_T12CountMode_edgeAligned,
            .counterValue = 0,
        },

        .timer13                    = {
            .counterValue     = 0,
            .compareValue     = 0,
            .t12SyncEvent     = IfxCcu6_T13TriggerEvent_noAction,
            .t12SyncDirection = IfxCcu6_T13TriggerDirection_noAction,
        },

        .hallSyncEvent       = IfxCcu6_HallSensorTriggerMode_t13PM,

        .hallPatternIndex    = 0,

        .multiChannelControl = {
            .switchingSelect = IfxCcu6_MultiChannelSwitchingSelect_t12Channel1CompareMatch,
            .switchingSync   = IfxCcu6_MultiChannelSwitchingSync_t13ZeroMatch,
        },

        .pins       = NULL_PTR,

        .interrupt1 = {
            .source         = IfxCcu6_InterruptSource_t12PeriodMatch,
            .serviceRequest = IfxCcu6_ServiceRequest_0,
            .priority       = 0,                // interrupt priority 0
            .typeOfService  = IfxSrc_Tos_cpu0,  // type of service CPU0
        },

        .interrupt2                 = {
            .source         = IfxCcu6_InterruptSource_t13PeriodMatch,
            .serviceRequest = IfxCcu6_ServiceRequest_1,
            .priority       = 0,                // interrupt priority 0
            .typeOfService  = IfxSrc_Tos_cpu0,  // type of service CPU0
        },

        .interrupt3                 = {
            .source         = IfxCcu6_InterruptSource_t12OneMatch,
            .serviceRequest = IfxCcu6_ServiceRequest_2,
            .priority       = 0,                // interrupt priority 0
            .typeOfService  = IfxSrc_Tos_cpu0,  // type of service CPU0
        },

        .interrupt4                 = {
            .source         = IfxCcu6_InterruptSource_trap,
            .serviceRequest = IfxCcu6_ServiceRequest_3,
            .priority       = 0,                // interrupt priority 0
            .typeOfService  = IfxSrc_Tos_cpu0,  // type of service CPU0
        },

        .trigger                    = {
            .t12ExtInputTrigger     = NULL_PTR,
            .t12ExtInputTriggerMode = IfxCcu6_ExternalTriggerMode_risingEdge,
            .t13ExtInputTrigger     = NULL_PTR,
            .t13ExtInputTriggerMode = IfxCcu6_ExternalTriggerMode_risingEdge,
            .t13InSyncWithT12       = FALSE,

            .outputTriggerEnabled   = TRUE,
            .outputLine             = IfxCcu6_TrigOut_0,
            .outputTrigger          = IfxCcu6_TrigSel_cout63,
        },
    };

    /* Default Configuration */
    *config = defaultConfig;

    /* take over module pointer */
    config->ccu6 = ccu6;
}


void IfxCcu6_PwmBc_start(IfxCcu6_PwmBc *pwmBc)
{
    // enable shadow transfers
    IfxCcu6_enableShadowTransfer(pwmBc->ccu6, TRUE, TRUE);

    // internal start
    if ((pwmBc->trigger.t13ExtInputTrigger == NULL_PTR) && (pwmBc->trigger.t12ExtInputTrigger == NULL_PTR))
    {
        // if T13 start is in sync with T12 start the T12
        if (pwmBc->trigger.t13InSyncWithT12)
        {
            IfxCcu6_startTimer(pwmBc->ccu6, TRUE, FALSE);
        }
        else    // start both timers
        {
            IfxCcu6_startTimer(pwmBc->ccu6, TRUE, TRUE);
        }
    }

    // external start
    else
    {
        // if T13 start is not in sync with T12
        if (!(pwmBc->trigger.t13InSyncWithT12))
        {
            // T13 external start if selected
            if (pwmBc->trigger.t13ExtInputTrigger != NULL_PTR)
            {
                IfxCcu6_setExternalRunMode(pwmBc->ccu6, IfxCcu6_TimerId_t13, pwmBc->trigger.t13ExtInputTriggerMode);
                IfxCcu6_setT13InputSignal(pwmBc->ccu6, pwmBc->trigger.t13ExtInputTrigger);
            }
            else // internal start of T13 alone
            {
                IfxCcu6_startTimer(pwmBc->ccu6, FALSE, TRUE);
            }
        }
        else
        {}

        // T12 external start if selected
        if (pwmBc->trigger.t12ExtInputTrigger != NULL_PTR)
        {
            IfxCcu6_setExternalRunMode(pwmBc->ccu6, IfxCcu6_TimerId_t12, pwmBc->trigger.t12ExtInputTriggerMode);
            IfxCcu6_setT12InputSignal(pwmBc->ccu6, pwmBc->trigger.t12ExtInputTrigger);
        }
        else // internal start of T12 alone
        {
            IfxCcu6_startTimer(pwmBc->ccu6, TRUE, FALSE);
        }
    }
}


void IfxCcu6_PwmBc_stop(IfxCcu6_PwmBc *pwmBc)
{
    IfxCcu6_disableShadowTransfer(pwmBc->ccu6, TRUE, TRUE);

    // if T13 start is in sync with T12, remove sync trigger
    if (pwmBc->trigger.t13InSyncWithT12)
    {
        IfxCcu6_setT13TriggerEventMode(pwmBc->ccu6, IfxCcu6_T13TriggerEvent_noAction);
        IfxCcu6_setT13TriggerEventDirection(pwmBc->ccu6, IfxCcu6_T13TriggerDirection_noAction);
    }

    // remove external triggers if any
    if ((pwmBc->trigger.t13ExtInputTrigger != NULL_PTR) || (pwmBc->trigger.t12ExtInputTrigger != NULL_PTR))
    {
        if (pwmBc->trigger.t13ExtInputTrigger != NULL_PTR)
        {
            IfxCcu6_setExternalRunMode(pwmBc->ccu6, IfxCcu6_TimerId_t13, IfxCcu6_ExternalTriggerMode_disable);
        }
        else
        {
            IfxCcu6_setExternalRunMode(pwmBc->ccu6, IfxCcu6_TimerId_t12, IfxCcu6_ExternalTriggerMode_disable);
        }
    }
    else
    {}

    // stop both timers
    IfxCcu6_stopTimer(pwmBc->ccu6, TRUE, TRUE);

    //disable Timer13 modulation output path for all T12 channel outs//

    IfxCcu6_disableModulationOutput(pwmBc->ccu6, IfxCcu6_TimerId_t13, IfxCcu6_ChannelOut_cc0);
    IfxCcu6_disableModulationOutput(pwmBc->ccu6, IfxCcu6_TimerId_t13, IfxCcu6_ChannelOut_cc1);
    IfxCcu6_disableModulationOutput(pwmBc->ccu6, IfxCcu6_TimerId_t13, IfxCcu6_ChannelOut_cc2);
    IfxCcu6_disableModulationOutput(pwmBc->ccu6, IfxCcu6_TimerId_t13, IfxCcu6_ChannelOut_cout3);
}


void IfxCcu6_PwmBc_updateHallPattern(IfxCcu6_PwmBc *pwmBc, uint8 controlTable[6][3])
{
    uint8 index = pwmBc->hallPatternIndex;

    IfxCcu6_updateHallPattern(pwmBc->ccu6, controlTable[index][0], controlTable[index][1], controlTable[index][2]);

    index++;

    if (index >= 6)
    {
        index = 0;
    }

    pwmBc->hallPatternIndex = index;
}
