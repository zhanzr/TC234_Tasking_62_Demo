/**
 * \file IfxGtm_Tom_Timer.c
 * \brief GTM TIMER details
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
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "IfxGtm_Tom_Timer.h"
#include "_Utilities/Ifx_Assert.h"
#include "IfxGtm_bf.h"
#include "stddef.h"

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

boolean IfxGtm_Tom_Timer_acknowledgeTimerIrq(IfxGtm_Tom_Timer *driver)
{
    boolean event;

    event = IfxGtm_Tom_Ch_isZeroNotification(driver->tom, driver->timerChannel);

    if (event)
    {
        IfxGtm_Tom_Ch_clearZeroNotification(driver->tom, driver->timerChannel);
    }
    else
    {}

    return event;
}


boolean IfxGtm_Tom_Timer_acknowledgeTriggerIrq(IfxGtm_Tom_Timer *driver)
{
    boolean event;

    event = IfxGtm_Tom_Ch_isOneNotification(driver->tom, driver->triggerChannel);

    if (event)
    {
        IfxGtm_Tom_Ch_clearOneNotification(driver->tom, driver->triggerChannel);
    }
    else
    {}

    return event;
}


void IfxGtm_Tom_Timer_addToChannelMask(IfxGtm_Tom_Timer *driver, IfxGtm_Tom_Ch channel)
{
    if (driver->timerChannel <= IfxGtm_Tom_Ch_7)
    {
        if (channel <= IfxGtm_Tom_Ch_7)
        {
            driver->channelsMask[0]                 |= 1 << channel;
            driver->tgcGlobalControlDisableUpdate[0] = IfxGtm_Tom_Tgc_buildFeature(0, driver->channelsMask[0], IFX_GTM_TOM_TGC0_GLB_CTRL_UPEN_CTRL0_OFF);
            driver->tgcGlobalControlApplyUpdate[0]   = IfxGtm_Tom_Tgc_buildFeature(driver->channelsMask[0], 0, IFX_GTM_TOM_TGC0_GLB_CTRL_UPEN_CTRL0_OFF);
        }
        else
        {
            driver->channelsMask[1]                 |= 1 << (channel - IfxGtm_Tom_Ch_8);
            driver->tgcGlobalControlDisableUpdate[1] = IfxGtm_Tom_Tgc_buildFeature(0, driver->channelsMask[1], IFX_GTM_TOM_TGC0_GLB_CTRL_UPEN_CTRL0_OFF);
            driver->tgcGlobalControlApplyUpdate[1]   = IfxGtm_Tom_Tgc_buildFeature(driver->channelsMask[1], 0, IFX_GTM_TOM_TGC0_GLB_CTRL_UPEN_CTRL0_OFF);
        }
    }
    else
    {
        driver->channelsMask[0]                 |= 1 << (channel - IfxGtm_Tom_Ch_8);
        driver->tgcGlobalControlDisableUpdate[0] = IfxGtm_Tom_Tgc_buildFeature(0, driver->channelsMask[0], IFX_GTM_TOM_TGC0_GLB_CTRL_UPEN_CTRL0_OFF);
        driver->tgcGlobalControlApplyUpdate[0]   = IfxGtm_Tom_Tgc_buildFeature(driver->channelsMask[0], 0, IFX_GTM_TOM_TGC0_GLB_CTRL_UPEN_CTRL0_OFF);
    }
}


void IfxGtm_Tom_Timer_applyUpdate(IfxGtm_Tom_Timer *driver)
{
    IfxGtm_Tom_Tgc_writeGlobalControl(driver->tgc[0], driver->tgcGlobalControlApplyUpdate[0]);

    if (driver->tgc[1])
    {
        IfxGtm_Tom_Tgc_writeGlobalControl(driver->tgc[1], driver->tgcGlobalControlApplyUpdate[1]);  /* Note: Write of 0 value has no effect */
    }
}


void IfxGtm_Tom_Timer_disableUpdate(IfxGtm_Tom_Timer *driver)
{
    IfxGtm_Tom_Tgc_writeGlobalControl(driver->tgc[0], driver->tgcGlobalControlDisableUpdate[0]);

    if (driver->tgc[1])
    {
        IfxGtm_Tom_Tgc_writeGlobalControl(driver->tgc[1], driver->tgcGlobalControlDisableUpdate[1]); /* Note: Write of 0 value has no effect */
    }
}


float32 IfxGtm_Tom_Timer_getFrequency(IfxGtm_Tom_Timer *driver)
{
    return 1.0 / IfxStdIf_Timer_tickToS(driver->base.clockFreq, driver->base.period);
}


float32 IfxGtm_Tom_Timer_getInputFrequency(IfxGtm_Tom_Timer *driver)
{
    return driver->base.clockFreq;
}


Ifx_TimerValue IfxGtm_Tom_Timer_getOffset(IfxGtm_Tom_Timer *driver)
{
    return driver->offset;
}


Ifx_TimerValue IfxGtm_Tom_Timer_getPeriod(IfxGtm_Tom_Timer *driver)
{
    return driver->base.period;
}


volatile uint32 *IfxGtm_Tom_Timer_getPointer(IfxGtm_Tom_Timer *driver)
{
    return IfxGtm_Tom_Ch_getTimerPointer(driver->tom, driver->timerChannel);
}


float32 IfxGtm_Tom_Timer_getResolution(IfxGtm_Tom_Timer *driver)
{
    return 1.0 / driver->base.clockFreq;
}


Ifx_TimerValue IfxGtm_Tom_Timer_getTrigger(IfxGtm_Tom_Timer *driver)
{
    return IfxGtm_Tom_Ch_getCompareOne(driver->tom, driver->triggerChannel) - 1;
}


volatile uint32 *IfxGtm_Tom_Timer_getTriggerPointer(IfxGtm_Tom_Timer *driver)
{
    return IfxGtm_Tom_Ch_getCompareOnePointer(driver->tom, driver->triggerChannel);
}


boolean IfxGtm_Tom_Timer_init(IfxGtm_Tom_Timer *driver, const IfxGtm_Tom_Timer_Config *config)
{
    boolean                result = TRUE;
    IfxGtm_Tom_Timer_Base *base   = &driver->base;
    uint16                 maskShift;

    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config->base.countDir == IfxStdIf_Timer_CountDir_up);    /* only this mode is supported */

    driver->gtm          = config->gtm;
    driver->tomIndex     = config->tom;
    driver->tom          = &config->gtm->TOM[config->tom];
    driver->timerChannel = config->timerChannel;

    base->triggerEnabled = config->base.trigger.enabled;

    if (base->triggerEnabled)
    {
        driver->triggerChannel = config->triggerOut->channel;
    }
    else
    {
        driver->triggerChannel = driver->timerChannel; // Set to timer channel to disable its use
    }

    if (config->timerChannel <= 7)
    {
        driver->tgc[0] = IfxGtm_Tom_Ch_getTgcPointer(driver->tom, 0);
        driver->tgc[1] = IfxGtm_Tom_Ch_getTgcPointer(driver->tom, 1);
    }
    else
    {
        driver->tgc[0] = IfxGtm_Tom_Ch_getTgcPointer(driver->tom, 1);
        driver->tgc[1] = NULL_PTR; /* NOTE currently no concatenation between TOMs */
    }

    driver->channelsMask[1]                  = 0;
    driver->tgcGlobalControlApplyUpdate[1]   = 0;
    driver->tgcGlobalControlDisableUpdate[1] = 0;

    /* Initialize the timer part */
    /* FIXME add IfxGtm_Tom_Ch_configurePwmMode() and use it */
    IfxGtm_Tom_Ch_setClockSource(driver->tom, driver->timerChannel, config->clock);
    IfxGtm_Tom_Ch_setTriggerOutput(driver->tom, driver->timerChannel, IfxGtm_Tom_Ch_OutputTrigger_generate);

    IfxGtm_Tom_Timer_updateInputFrequency(driver);

    if ((config->base.minResolution > 0) && ((1.0 / base->clockFreq) > config->base.minResolution))
    {
        result = FALSE;
        IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);
    }

    result        &= IfxGtm_Tom_Timer_setFrequency(driver, config->base.frequency);
    driver->offset = IfxStdIf_Timer_sToTick(driver->base.clockFreq, 1.0 / config->base.frequency * config->base.startOffset);

    /* check that driver->offset is not more than 16 bits */
    if (driver->offset <= 0xFFFF)
    {
        IfxGtm_Tom_Ch_setCounterValue(driver->tom, driver->timerChannel, driver->offset);
    }
    else
    {
        result = FALSE;
        IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);
    }

    /* Initialize the trigger part */
    maskShift = (config->timerChannel <= 7) ? 0 : 8;
    IfxGtm_Tom_Timer_addToChannelMask(driver, driver->timerChannel);

    if (base->triggerEnabled)
    {
        IfxGtm_Tom_Ch triggerChannel     = driver->triggerChannel;
        uint16        triggerChannelMask = 1 << (triggerChannel - maskShift);
        /* TO DO: enable the trigger to be not in the same TGC group as the timer */

        IfxGtm_Tom_Ch_setSignalLevel(driver->tom, triggerChannel, config->base.trigger.risingEdgeAtPeriod ? Ifx_ActiveState_high : Ifx_ActiveState_low);
        IfxGtm_Tom_Ch_setCounterValue(driver->tom, triggerChannel, driver->offset);

        if (triggerChannel != driver->timerChannel)
        {
            /* FIXME add IfxGtm_Tom_Ch_configurePwmMode() and use it */
            IfxGtm_Tom_Ch_setResetSource(driver->tom, triggerChannel, IfxGtm_Tom_Ch_ResetEvent_onTrigger);
            IfxGtm_Tom_Ch_setClockSource(driver->tom, triggerChannel, config->clock);
            IfxGtm_Tom_Ch_setTriggerOutput(driver->tom, triggerChannel, IfxGtm_Tom_Ch_OutputTrigger_forward);
            IfxGtm_Tom_Tgc_enableChannels(driver->tgc[0], triggerChannelMask, 0, FALSE);
            IfxGtm_Tom_Timer_addToChannelMask(driver, driver->triggerChannel);
        }

        /* Signal must go out of the GTM even if the port outpout is not enabled */
        IfxGtm_Tom_Tgc_enableChannelsOutput(driver->tgc[0], triggerChannelMask, 0, FALSE);

        if (config->base.trigger.outputEnabled)
        {
            /* Initialize the port */
            IfxGtm_PinMap_setTomTout(config->triggerOut, config->base.trigger.outputMode, config->base.trigger.outputDriver);
        }

        result &= IfxGtm_Tom_Timer_setTrigger(driver, config->base.trigger.triggerPoint);
    }

    /* Interrupt configuration */
    {
        volatile Ifx_SRC_SRCR *src;
        boolean                timerHasIrq   = config->base.isrPriority > 0;
        boolean                triggerHasIrq = (config->base.trigger.isrPriority > 0) && base->triggerEnabled;

        if (driver->triggerChannel == driver->timerChannel)
        {
            IfxGtm_Tom_Ch_setNotification(driver->tom, driver->timerChannel, timerHasIrq ? config->irqModeTimer : config->irqModeTrigger, timerHasIrq, triggerHasIrq);
            src = IfxGtm_Tom_Ch_getSrcPointer(driver->gtm, config->tom, driver->timerChannel);
            IfxSrc_init(src, timerHasIrq ? config->base.isrProvider : config->base.trigger.isrProvider, timerHasIrq ? config->base.isrPriority : config->base.trigger.isrPriority);
            /* FIXME ADD warning on interrupt setting in case timer and trigger uses the same channels or different channels, and in case only timer or trigger or both generates interrupts */
            IfxSrc_enable(src);
        }
        else
        {
            IfxGtm_IrqMode irqMode = IfxGtm_IrqMode_pulseNotify;

            if (timerHasIrq)
            {
                IfxGtm_Tom_Ch_setNotification(driver->tom, driver->timerChannel, irqMode, TRUE, FALSE);
                src = IfxGtm_Tom_Ch_getSrcPointer(driver->gtm, config->tom, driver->timerChannel);
                IfxSrc_init(src, config->base.isrProvider, config->base.isrPriority);
                IfxSrc_enable(src);
            }

            if (triggerHasIrq)
            {
                IfxGtm_Tom_Ch_setNotification(driver->tom, driver->triggerChannel, irqMode, FALSE, TRUE);
                src = IfxGtm_Tom_Ch_getSrcPointer(driver->gtm, config->tom, driver->triggerChannel);
                IfxSrc_init(src, config->base.trigger.isrProvider, config->base.trigger.isrPriority);
                IfxSrc_enable(src);
            }
        }
    }

    /* Transfer the shadow registers */
    IfxGtm_Tom_Tgc_setChannelsForceUpdate(driver->tgc[0], driver->channelsMask[0], 0, 0, 0);
    IfxGtm_Tom_Tgc_trigger(driver->tgc[0]);
    IfxGtm_Tom_Tgc_setChannelsForceUpdate(driver->tgc[0], 0, driver->channelsMask[0], 0, 0);

    return result;
}


void IfxGtm_Tom_Timer_initConfig(IfxGtm_Tom_Timer_Config *config, Ifx_GTM *gtm)
{
    IfxStdIf_Timer_initConfig(&config->base);
    config->gtm            = gtm;
    config->tom            = IfxGtm_Tom_0;
    config->timerChannel   = IfxGtm_Tom_Ch_0;
    config->triggerOut     = NULL_PTR;
    config->clock          = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk0;
    config->base.countDir  = IfxStdIf_Timer_CountDir_up;
    config->irqModeTimer   = IfxGtm_IrqMode_level;
    config->irqModeTrigger = IfxGtm_IrqMode_level;
}


void IfxGtm_Tom_Timer_run(IfxGtm_Tom_Timer *driver)
{
    IfxGtm_Tom_Tgc_enableChannels(driver->tgc[0], driver->channelsMask[0], 0, TRUE);

    if (driver->tgc[1])
    {
        IfxGtm_Tom_Tgc_enableChannels(driver->tgc[1], driver->channelsMask[1], 0, TRUE); /* Note: Write of 0 value has no effect */
    }
}


boolean IfxGtm_Tom_Timer_setFrequency(IfxGtm_Tom_Timer *driver, float32 frequency)
{
    Ifx_TimerValue period = IfxStdIf_Timer_sToTick(driver->base.clockFreq, 1.0 / frequency);

    return IfxGtm_Tom_Timer_setPeriod(driver, period);
}


boolean IfxGtm_Tom_Timer_setPeriod(IfxGtm_Tom_Timer *driver, Ifx_TimerValue period)
{
    boolean result = TRUE;
    driver->base.period = period;

    /* check that period is not more that 16 bits */
    if (period <= 0xFFFF)
    {
        IfxGtm_Tom_Ch_setCompareZeroShadow(driver->tom, driver->timerChannel, period);

        if (driver->triggerChannel != driver->timerChannel)
        {
            IfxGtm_Tom_Ch_setCompareZeroShadow(driver->tom, driver->triggerChannel, period);
        }
    }
    else
    {
        result = FALSE;
        IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);
    }

    return result;
}


void IfxGtm_Tom_Timer_setSingleMode(IfxGtm_Tom_Timer *driver, boolean enabled)
{
    IfxGtm_Tom_Ch_setOneShotMode(driver->tom, driver->timerChannel, enabled);
}


boolean IfxGtm_Tom_Timer_setTrigger(IfxGtm_Tom_Timer *driver, Ifx_TimerValue triggerPoint)
{
    boolean result = TRUE;

    /* check that trigger point is not more that 16 bits */
    if (triggerPoint <= (0xFFFF - 1))
    {
        IfxGtm_Tom_Ch_setCompareOneShadow(driver->tom, driver->triggerChannel, triggerPoint + 1);
    }
    else
    {
        result = FALSE;
        IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);
    }

    return result;
}


boolean IfxGtm_Tom_Timer_stdIfTimerInit(IfxStdIf_Timer *stdif, IfxGtm_Tom_Timer *driver)
{
    /* *INDENT-OFF* Note: this file was indented manually by the author. */
    /* Set the API link */
    stdif->driver               = driver;
    stdif->getFrequency         =(IfxStdIf_Timer_GetFrequency        )&IfxGtm_Tom_Timer_getFrequency;
    stdif->getPeriod            =(IfxStdIf_Timer_GetPeriod           )&IfxGtm_Tom_Timer_getPeriod;
    stdif->getResolution        =(IfxStdIf_Timer_GetResolution       )&IfxGtm_Tom_Timer_getResolution;
    stdif->getTrigger           =(IfxStdIf_Timer_GetTrigger          )&IfxGtm_Tom_Timer_getTrigger;
    stdif->setFrequency         =(IfxStdIf_Timer_SetFrequency        )&IfxGtm_Tom_Timer_setFrequency;
    stdif->updateInputFrequency =(IfxStdIf_Timer_UpdateInputFrequency)&IfxGtm_Tom_Timer_updateInputFrequency;
    stdif->applyUpdate          =(IfxStdIf_Timer_ApplyUpdate         )&IfxGtm_Tom_Timer_applyUpdate;
    stdif->disableUpdate        =(IfxStdIf_Timer_DisableUpdate       )&IfxGtm_Tom_Timer_disableUpdate;
    stdif->getInputFrequency    =(IfxStdIf_Timer_GetInputFrequency   )&IfxGtm_Tom_Timer_getInputFrequency;
    stdif->run                  =(IfxStdIf_Timer_Run                 )&IfxGtm_Tom_Timer_run;
    stdif->setPeriod            =(IfxStdIf_Timer_SetPeriod           )&IfxGtm_Tom_Timer_setPeriod;
    stdif->setSingleMode        =(IfxStdIf_Timer_SetSingleMode       )&IfxGtm_Tom_Timer_setSingleMode;
    stdif->setTrigger           =(IfxStdIf_Timer_SetTrigger          )&IfxGtm_Tom_Timer_setTrigger;
    stdif->stop                 =(IfxStdIf_Timer_Stop                )&IfxGtm_Tom_Timer_stop;
    stdif->ackTimerIrq          =(IfxStdIf_Timer_AckTimerIrq         )&IfxGtm_Tom_Timer_acknowledgeTimerIrq;
    stdif->ackTriggerIrq        =(IfxStdIf_Timer_AckTriggerIrq       )&IfxGtm_Tom_Timer_acknowledgeTriggerIrq;
    /* *INDENT-ON* */

    return TRUE;
}


void IfxGtm_Tom_Timer_stop(IfxGtm_Tom_Timer *driver)
{
    IfxGtm_Tom_Tgc_enableChannels(driver->tgc[0], 0, driver->channelsMask[0], TRUE);

    if (driver->tgc[1])
    {
        IfxGtm_Tom_Tgc_enableChannels(driver->tgc[1], 0, driver->channelsMask[1], TRUE); /* Note: Write of 0 value has no effect */
    }
}


void IfxGtm_Tom_Timer_updateInputFrequency(IfxGtm_Tom_Timer *driver)
{
    driver->base.clockFreq = IfxGtm_Tom_Ch_getClockFrequency(driver->gtm, driver->tom, driver->timerChannel);
}
