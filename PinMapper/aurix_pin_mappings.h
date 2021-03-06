/*
 * Generated by TASKING Pin Mapper Tool for AURIX
 * - device  : TC23x
 * - package : TQFP100
 */

#ifndef AURIX_PIN_MAPPINGS_H_
#define AURIX_PIN_MAPPINGS_H_

/* iLLD library include files */

#include <Port/Io/IfxPort_Io.h>

#include <_PinMap/IfxPort_PinMap.h>

#include <_Reg/IfxAsclin_bf.h>
#include <_Reg/IfxAsclin_reg.h>
#include <_Reg/IfxScu_bf.h>
#include <_Reg/IfxScu_reg.h>


/* iLLD driver modules */

#define IFXCFG_MODULE_IFX_ASCLIN0
#define IFXCFG_MODULE_IFX_SCU0


/* iLLD peripheral driver I/O connections */

// aschs0
#define IFXCFG_ASCLIN0_RX                       IfxAsclin0_RXA_P14_1_IN
#define IFXCFG_ASCLIN0_RX_MODE                  IfxPort_InputMode_noPullDevice
#define IFXCFG_ASCLIN0_RX_PAD_DRIVER            IfxPort_PadDriver_cmosAutomotiveSpeed1
#define IFXCFG_ASCLIN0_TX                       IfxAsclin0_TX_P14_0_OUT
#define IFXCFG_ASCLIN0_TX_MODE                  IfxPort_OutputMode_pushPull
#define IFXCFG_ASCLIN0_TX_PAD_DRIVER            IfxPort_PadDriver_cmosAutomotiveSpeed4

// evr1v
#define IFXCFG_SCU0_MODE                        IfxScu_HWCFG0DCLDO_P14_6_IN
#define IFXCFG_SCU0_MODE_MODE                   IfxPort_InputMode_pullUp
#define IFXCFG_SCU0_MODE_PAD_DRIVER             IfxPort_PadDriver_cmosAutomotiveSpeed1

// scu
#define IFXCFG_SCU0_HWCFG3                      IfxScu_HWCFG3_P14_3_IN
#define IFXCFG_SCU0_HWCFG3_MODE                 IfxPort_InputMode_pullUp
#define IFXCFG_SCU0_HWCFG3_PAD_DRIVER           IfxPort_PadDriver_cmosAutomotiveSpeed1
#define IFXCFG_SCU0_HWCFG4                      IfxScu_HWCFG4_P10_5_IN
#define IFXCFG_SCU0_HWCFG4_MODE                 IfxPort_InputMode_noPullDevice
#define IFXCFG_SCU0_HWCFG4_PAD_DRIVER           IfxPort_PadDriver_cmosAutomotiveSpeed1
#define IFXCFG_SCU0_HWCFG5                      IfxScu_HWCFG5_P10_6_IN
#define IFXCFG_SCU0_HWCFG5_MODE                 IfxPort_InputMode_noPullDevice
#define IFXCFG_SCU0_HWCFG5_PAD_DRIVER           IfxPort_PadDriver_cmosAutomotiveSpeed1


/* Symbolic names for GPIO ports */

// p13_0
#define IFXCFG_PORT_GPIO2                       IfxPort_P13_0
#define IFXCFG_PORT_GPIO2_MODE                  IfxPort_Mode_outputOpenDrainGeneral
#define IFXCFG_PORT_GPIO2_PAD_DRIVER            IfxPort_PadDriver_cmosAutomotiveSpeed4

// p13_1
#define IFXCFG_PORT_GPIO1                       IfxPort_P13_1
#define IFXCFG_PORT_GPIO1_MODE                  IfxPort_Mode_outputOpenDrainGeneral
#define IFXCFG_PORT_GPIO1_PAD_DRIVER            IfxPort_PadDriver_cmosAutomotiveSpeed4

// p13_2
#define IFXCFG_PORT_GPIO3                       IfxPort_P13_2
#define IFXCFG_PORT_GPIO3_MODE                  IfxPort_Mode_outputOpenDrainGeneral
#define IFXCFG_PORT_GPIO3_PAD_DRIVER            IfxPort_PadDriver_cmosAutomotiveSpeed4

// p13_3
#define IFXCFG_PORT_GPIO4                       IfxPort_P13_3
#define IFXCFG_PORT_GPIO4_MODE                  IfxPort_Mode_outputOpenDrainGeneral
#define IFXCFG_PORT_GPIO4_PAD_DRIVER            IfxPort_PadDriver_cmosAutomotiveSpeed4


/* Generic port I/O configuration */

#define IFXCFG_P13_0_IO_CONFIG                  { &IFXCFG_PORT_GPIO2, IFXCFG_PORT_GPIO2_MODE, IFXCFG_PORT_GPIO2_PAD_DRIVER }
#define IFXCFG_P13_1_IO_CONFIG                  { &IFXCFG_PORT_GPIO1, IFXCFG_PORT_GPIO1_MODE, IFXCFG_PORT_GPIO1_PAD_DRIVER }
#define IFXCFG_P13_2_IO_CONFIG                  { &IFXCFG_PORT_GPIO3, IFXCFG_PORT_GPIO3_MODE, IFXCFG_PORT_GPIO3_PAD_DRIVER }
#define IFXCFG_P13_3_IO_CONFIG                  { &IFXCFG_PORT_GPIO4, IFXCFG_PORT_GPIO4_MODE, IFXCFG_PORT_GPIO4_PAD_DRIVER }


/* Initialization routines */

extern void gpio_init_pins(void);


#endif /* AURIX_PIN_MAPPINGS_H_ */
