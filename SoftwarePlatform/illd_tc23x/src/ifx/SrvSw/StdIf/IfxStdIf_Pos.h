/**
 * \file IfxStdIf_Pos.h
 * \brief Standard interface: Position interface
 * \ingroup IfxStdIf
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
 * \defgroup library_srvsw_stdif_posif Standard interface: Position interface
 * \ingroup library_srvsw_stdif
 *
 * The standard interface position interface (IfxStdIf_Pos) abstract the hardware used for position interface feature like encoder, hall, resolver, ... It provide, after proper initialization an hardware
 * Independent way to interact with the position sensor like getting position, speed, direction, ...
 *
 * The figure below shows the standard position interface.
 *
 * \image html "stdif_PosIf.png" "Standard position interface"
 *
 * This interface defines the following features:
 * -
 *
 *
 */

#ifndef IFXSTDIF_POSIF_H_
#define IFXSTDIF_POSIF_H_ 1

#include "Cpu/Std/Ifx_Types.h"
#include "IfxStdIf.h"

/** \brief Output event enable / disable */
typedef enum
{
    IfxStdIf_Pos_MotionType_rotating,             /**< \brief Rotating sensor */
    IfxStdIf_Pos_MotionType_linear                /**< \brief Linear sensor */
} IfxStdIf_Pos_MotionType;

/** \brief Output event enable / disable */
typedef enum
{
    IfxStdIf_Pos_ResolutionFactor_oneFold  = 1,             /**< \brief Default, no multipluication factor */
    IfxStdIf_Pos_ResolutionFactor_twoFold  = 2,             /**< \brief 2-fold resolution. Valid for encoder */
    IfxStdIf_Pos_ResolutionFactor_fourFold = 4              /**< \brief 4-fold resolution. Valid for encoder */
} IfxStdIf_Pos_ResolutionFactor;

/** \brief Position sensor Types */
typedef enum
{
    IfxStdIf_Pos_SensorType_encoder,
    IfxStdIf_Pos_SensorType_hall,
    IfxStdIf_Pos_SensorType_resolver,
    IfxStdIf_Pos_SensorType_angletrk,
    IfxStdIf_Pos_SensorType_igmr,
    IfxStdIf_Pos_SensorType_virtual
} IfxStdIf_Pos_SensorType;

/** \brief Position sensor direction definition */
typedef enum
{
    IfxStdIf_Pos_Dir_forward,         /**< \brief Forward direction. For rotating position sensor, forward is clockwise rotation */
    IfxStdIf_Pos_Dir_backward,        /**< \brief Backward direction. For rotating position sensor, fackward is counter-clockwise rotation */
    IfxStdIf_Pos_Dir_unknown          /**< \brief Unknown direction */
} IfxStdIf_Pos_Dir;

/** \brief Position sensor status definition */
typedef union
{
    uint32 status;                      /**< \brief Global status access */
    struct
    {
        uint32 notSynchronised : 1;     /**< \brief Sensor is not synchronized */
        uint32 signalLoss : 1;          /**< \brief Loss of signal error */
        uint32 signalDegradation : 1;   /**< \brief Signal degradation warning */
        uint32 trackingLoss : 1;        /**< \brief Tracking loss error */
        uint32 commError : 1;           /**< \brief Communication error*/
    } B;                                /**< \brief Bitfielf status access */
} IfxStdIf_Pos_Status;

typedef sint32 IfxStdIf_Pos_RawAngle;

/** \brief Forward declaration */
typedef struct IfxStdIf_Pos_ IfxStdIf_Pos;

/** \brief Return the position, inclusive turns
 *
 * Return the sensor position in rad for rotating sensor inclusive turns, or in m for linear sensors.
 * For linear sensor the effect is the same as IfxStdIf_Pos_GetPosition.
 *
 * \param driver Pointer to the interface driver object
 * \return Return the position inclusive turns in rad
 */
typedef float32 (*IfxStdIf_Pos_GetAbsolutePosition)(IfxStdIf_InterfaceDriver driver);

/** \brief Handles the zero interrupt
 * \param driver Pointer to the interface driver object
 * \return none
 */
typedef void (*IfxStdIf_Pos_OnZeroIrq)(IfxStdIf_InterfaceDriver driver);

/** \brief Return the raw position sensor offset
 *
 * \param driver Pointer to the interface driver object
 * \return Return the raw position sensor offset sensor in ticks
 */
typedef sint32 (*IfxStdIf_Pos_GetOffset)(IfxStdIf_InterfaceDriver driver);

/** \brief Return the position
 *
 * Return the sensor position in rad for rotating sensor or in m for linear sensors.
 * For rotating sensor, the position is always between 0 and 2*IFX_PI.
 *
 * \param driver Pointer to the interface driver object
 * \return Return the position in rad or m
 */
typedef float32 (*IfxStdIf_Pos_GetPosition)(IfxStdIf_InterfaceDriver driver);

/** \brief Return the direction
 *
 * Return the sensor direction.
 *
 * \param driver Pointer to the interface driver object
 * \return Return the direction
 */
typedef IfxStdIf_Pos_Dir (*IfxStdIf_Pos_GetDirection)(IfxStdIf_InterfaceDriver driver);

/** \brief Return the sensor status
 *
 * \param driver Pointer to the interface driver object
 * \return Return the sensor status
 */
typedef IfxStdIf_Pos_Status (*IfxStdIf_Pos_GetFault)(IfxStdIf_InterfaceDriver driver);

/** \brief Return the period per rotation setting
 *
 * \param driver Pointer to the interface driver object
 * \return Return the period per rotation setting
 */
typedef uint16 (*IfxStdIf_Pos_GetPeriodPerRotation)(IfxStdIf_InterfaceDriver driver);

/** \brief Return the raw position in ticks
 *
 * Return the sensor raw position in ticks.
 *
 * \param driver Pointer to the interface driver object
 * \return Return the raw position in ticks
 */
typedef sint32 (*IfxStdIf_Pos_GetRawPosition)(IfxStdIf_InterfaceDriver driver);

/** \brief Get the update period
 * \param driver Pointer to the interface driver object
 * \return Return the update period in s
 */
typedef float32 (*IfxStdIf_Pos_GetRefreshPeriod)(IfxStdIf_InterfaceDriver driver);

/** \brief Get the resolution
 * \param driver Pointer to the interface driver object
 * \return Return the resolution
 */
typedef sint32 (*IfxStdIf_Pos_GetResolution)(IfxStdIf_InterfaceDriver driver);

/** \brief Get the sensor type
 * \param driver Pointer to the interface driver object
 * \return Return the sensor type
 */
typedef IfxStdIf_Pos_SensorType (*IfxStdIf_Pos_GetSensorType)(IfxStdIf_InterfaceDriver driver);

/** \brief Return the speed
 * \param driver Pointer to the interface driver object
 * \return Return the speed in rad/s or m/s
 */
typedef float32 (*IfxStdIf_Pos_GetSpeed)(IfxStdIf_InterfaceDriver driver);

/** \brief Return position in turn
 * \param driver Pointer to the interface driver object
 * \return Return position in turn
 */
typedef sint32 (*IfxStdIf_Pos_GetTurn)(IfxStdIf_InterfaceDriver driver);

/** \brief Refresh the status
 *
 *  Update the driver state like speed, position, status, taking into account the updatePeriod
 *
 * \param driver Pointer to the interface driver object
 * \return None
 */
typedef void (*IfxStdIf_Pos_Update)(IfxStdIf_InterfaceDriver driver);

/** \brief Reset the driver
 *
 *  Clear faults, reset speed and position to 0
 *
 * \param driver Pointer to the interface driver object
 * \return None
 */
typedef void (*IfxStdIf_Pos_Reset)(IfxStdIf_InterfaceDriver driver);

/** \brief Reset the driver fault
 *
 *  Clear faults
 *
 * \param driver Pointer to the interface driver object
 * \return None
 */
typedef void (*IfxStdIf_Pos_ResetFaults)(IfxStdIf_InterfaceDriver driver);

/** \brief Set the sensor offset
 * \param driver Pointer to the interface driver object
 * \param offset Offset in sensor ticks
 * \return None
 */
typedef void (*IfxStdIf_Pos_SetOffset)(IfxStdIf_InterfaceDriver driver, sint32 offset);

/** \brief Set the sensor position (virtual sensor)
 * \param driver Pointer to the interface driver object
 * \param position Position sensor rad
 * \return None
 */
typedef void (*IfxStdIf_Pos_SetPosition)(IfxStdIf_InterfaceDriver driver, float32 position);
/** \brief Set the sensor position (virtual sensor)
 * \param driver Pointer to the interface driver object
 * \param position Position sensor ticks
 * \return None
 */
typedef void (*IfxStdIf_Pos_SetRawPosition)(IfxStdIf_InterfaceDriver driver, sint32 position);

/** \brief Set the sensor speed (virtual sensor)
 * \param driver Pointer to the interface driver object
 * \param speed speed in rad/s
 * \return None
 */
typedef void (*IfxStdIf_Pos_SetSpeed)(IfxStdIf_InterfaceDriver driver, float32 speed);

/** \brief Set the update period
 * \param driver Pointer to the interface driver object
 * \param updatePeriod Refresh period in s
 * \return None
 */
typedef void (*IfxStdIf_Pos_SetRefreshPeriod)(IfxStdIf_InterfaceDriver driver, float32 updatePeriod);

/** \brief Standard interface object
 */
struct IfxStdIf_Pos_
{
    IfxStdIf_InterfaceDriver          driver;               /**< \brief Interface driver object                  */
    IfxStdIf_Pos_OnZeroIrq            onZeroIrq;            /**< \brief \see IfxStdIf_Pos_OnZeroIrq           */
    IfxStdIf_Pos_GetAbsolutePosition  getAbsolutePosition;  /**< \brief Return the absolute position     */
    IfxStdIf_Pos_GetOffset            getOffset;            /**< \brief \see IfxStdIf_Pos_GetOffset           */
    IfxStdIf_Pos_GetPosition          getPosition;          /**< \brief \see IfxStdIf_Pos_GetPosition           */
    IfxStdIf_Pos_GetDirection         getDirection;         /**< \brief \see IfxStdIf_Pos_GetDirection           */
    IfxStdIf_Pos_GetFault             getFault;             /**< \brief \see IfxStdIf_Pos_GetFault           */
    IfxStdIf_Pos_GetRawPosition       getRawPosition;       /**< \brief \see IfxStdIf_Pos_GetRawPosition           */
    IfxStdIf_Pos_GetPeriodPerRotation getPeriodPerRotation; /**< \brief \see IfxStdIf_Pos_GetPeriodPerRotation           */
    IfxStdIf_Pos_GetRefreshPeriod     getRefreshPeriod;     /**< \brief \see IfxStdIf_Pos_GetRefreshPeriod           */
    IfxStdIf_Pos_GetResolution        getResolution;        /**< \brief \see IfxStdIf_Pos_GetResolution           */
    IfxStdIf_Pos_GetSensorType        getSensorType;        /**< \brief \see IfxStdIf_Pos_GetSensorType           */
    IfxStdIf_Pos_GetTurn              getTurn;              /**< \brief \see IfxStdIf_Pos_GetTurn           */
    IfxStdIf_Pos_Reset                reset;                /**< \brief \see IfxStdIf_Pos_Reset           */
    IfxStdIf_Pos_ResetFaults          resetFaults;          /**< \brief \see IfxStdIf_Pos_ResetFaults           */
    IfxStdIf_Pos_GetSpeed             getSpeed;             /**< \brief \see IfxStdIf_Pos_GetSpeed             */
    IfxStdIf_Pos_Update               update;               /**< \brief \see IfxStdIf_Pos_Update           */
    IfxStdIf_Pos_SetOffset            setOffset;            /**< \brief \see IfxStdIf_Pos_SetOffset           */
    IfxStdIf_Pos_SetPosition          setPosition;          /**< \brief \see IfxStdIf_Pos_SetPosition           */
    IfxStdIf_Pos_SetRawPosition       setRawPosition;       /**< \brief \see IfxStdIf_Pos_SetRawPosition           */
    IfxStdIf_Pos_SetSpeed             setSpeed;             /**< \brief \see IfxStdIf_Pos_SetSpeed           */
    IfxStdIf_Pos_SetRefreshPeriod     setRefreshPeriod;     /**< \brief \see IfxStdIf_Pos_SetRefreshPeriod           */
};

/** \brief Position interface configuration */
typedef struct
{
    sint32                        offset;                    /**< \brief Position sensor offset */
    boolean                       reversed;                  /**< \brief If true, the sensor direction is reversed */
    sint32                        resolution;                /**< \brief Sensor resolution. For encoder with 1024 pulse per revolution, the value should be 1024 */
    uint16                        periodPerRotation;         /**< \brief Number of period per rotation. Is usually 1 for encoder */
    IfxStdIf_Pos_ResolutionFactor resolutionFactor;          /**< \brief Resolution multiplier for encoder interface, valid is 2, 4. */
    float32                       updatePeriod;              /**< \brief period in seconds, at which the application calls IfxStdIf_Pos_update() */
    float32                       speedModeThreshold;        /**< \brief Speed threshold used for the speed calculation mode. For encoder, above the threshold the pulse count mode is used, below the threshold, the time delta is used */
    float32                       minSpeed;                  /**< \brief Absolute minimal allowed speed. below speed is recognized as 0rad/s */
    float32                       maxSpeed;                  /**< \brief Absolute maximal allowed speed. Above speed is recognized as error */
    boolean                       speedFilterEnabled;        /**< \brief Enable / disable the speed low pass filter */
    float32                       speedFilerCutOffFrequency; /**< \brief Speed low pass filter cut off frequency */
} IfxStdIf_Pos_Config;

/** \addtogroup library_srvsw_stdif_posif
 *  \{
 */

/** \copydoc IfxStdIf_Pos_OnZeroIrq */
IFX_INLINE void IfxStdIf_Pos_onZeroIrq(IfxStdIf_Pos *stdIf)
{
    stdIf->onZeroIrq(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_GetAbsolutePosition */
IFX_INLINE float32 IfxStdIf_Pos_getAbsolutePosition(IfxStdIf_Pos *stdIf)
{
    return stdIf->getAbsolutePosition(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_GetFault */
IFX_INLINE IfxStdIf_Pos_Status IfxStdIf_Pos_getFault(IfxStdIf_Pos *stdIf)
{
    return stdIf->getFault(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_GetOffset */
IFX_INLINE sint32 IfxStdIf_Pos_getOffset(IfxStdIf_Pos *stdIf)
{
    return stdIf->getOffset(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_GetPosition */
IFX_INLINE float32 IfxStdIf_Pos_getPosition(IfxStdIf_Pos *stdIf)
{
    return stdIf->getPosition(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_GetDirection */
IFX_INLINE IfxStdIf_Pos_Dir IfxStdIf_Pos_getDirection(IfxStdIf_Pos *stdIf)
{
    return stdIf->getDirection(stdIf->driver);
}


IFX_INLINE uint16 IfxStdIf_Pos_getPeriodPerRotation(IfxStdIf_Pos *stdIf)
{
    return stdIf->getPeriodPerRotation(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_GetRawPosition */
IFX_INLINE sint32 IfxStdIf_Pos_getRawPosition(IfxStdIf_Pos *stdIf)
{
    return stdIf->getRawPosition(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_GetRefreshPeriod */
IFX_INLINE float32 IfxStdIf_Pos_getRefreshPeriod(IfxStdIf_Pos *stdIf)
{
    return stdIf->getRefreshPeriod(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_GetResolution */
IFX_INLINE sint32 IfxStdIf_Pos_getResolution(IfxStdIf_Pos *stdIf)
{
    return stdIf->getResolution(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_GetTurn */
IFX_INLINE sint32 IfxStdIf_Pos_getTurn(IfxStdIf_Pos *stdIf)
{
    return stdIf->getTurn(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_GetSensorType */
IFX_INLINE IfxStdIf_Pos_SensorType IfxStdIf_Pos_getSensorType(IfxStdIf_Pos *stdIf)
{
    return stdIf->getSensorType(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_GetSpeed */
IFX_INLINE float32 IfxStdIf_Pos_getSpeed(IfxStdIf_Pos *stdIf)
{
    return stdIf->getSpeed(stdIf->driver);
}


/** Check whether the sensor is faulty */
IFX_INLINE boolean IfxStdIf_Pos_isFault(IfxStdIf_Pos *stdIf)
{
    return IfxStdIf_Pos_getFault(stdIf).status != 0;
}


/** \copydoc IfxStdIf_Pos_Update */
IFX_INLINE void IfxStdIf_Pos_update(IfxStdIf_Pos *stdIf)
{
    stdIf->update(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_Reset */
IFX_INLINE void IfxStdIf_Pos_reset(IfxStdIf_Pos *stdIf)
{
    stdIf->reset(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_ResetFaults */
IFX_INLINE void IfxStdIf_Pos_resetFaults(IfxStdIf_Pos *stdIf)
{
    stdIf->resetFaults(stdIf->driver);
}


/** \copydoc IfxStdIf_Pos_SetOffset */
IFX_INLINE void IfxStdIf_Pos_setOffset(IfxStdIf_Pos *stdIf, sint32 offset)
{
    stdIf->setOffset(stdIf->driver, offset);
}


/** \copydoc IfxStdIf_Pos_SetPosition */
IFX_INLINE void IfxStdIf_Pos_setPosition(IfxStdIf_Pos *stdIf, float32 position)
{
    stdIf->setPosition(stdIf->driver, position);
}


/** \copydoc IfxStdIf_Pos_SetRawPosition */
IFX_INLINE void IfxStdIf_Pos_setRawPosition(IfxStdIf_Pos *stdIf, sint32 position)
{
    stdIf->setRawPosition(stdIf->driver, position);
}


/** \copydoc IfxStdIf_Pos_SetSpeed */
IFX_INLINE void IfxStdIf_Pos_setSpeed(IfxStdIf_Pos *stdIf, float32 speed)
{
    stdIf->setSpeed(stdIf->driver, speed);
}


/** \copydoc IfxStdIf_Pos_SetRefreshPeriod */
IFX_INLINE void IfxStdIf_Pos_setRefreshPeriod(IfxStdIf_Pos *stdIf, float32 updatePeriod)
{
    stdIf->setRefreshPeriod(stdIf->driver, updatePeriod);
}


/** \} */

/** \brief Converts from rad/s to rpm
 *
 * \param speed Specifies the speed in rad/s.
 *
 * \return returns the converted speed in rpm.
 * \see IfxStdIf_Pos_rpmToRads()
 */
IFX_INLINE float32 IfxStdIf_Pos_radsToRpm(float32 speed)
{
    return (60.0 / (2.0 * IFX_PI)) * speed;
}


/** \brief Converts from rpm to rad/s
 *
 * \param speed Specifies the speed in rpm.
 *
 * \return returns the converted speed in rad/s.
 * \see IfxStdIf_Pos_radsToRpm()
 */
IFX_INLINE float32 IfxStdIf_Pos_rpmToRads(float32 speed)
{
    return speed * ((2.0 * IFX_PI) / 60.0);
}


/** Initialize the configuration structure to default
 *
 * \param config Position interface configuration. This parameter is initialized by the function
 *
 */
IFX_EXTERN void IfxStdIf_Pos_initConfig(IfxStdIf_Pos_Config *config);

#endif /* IFXSTDIF_POSIF_H_ */
