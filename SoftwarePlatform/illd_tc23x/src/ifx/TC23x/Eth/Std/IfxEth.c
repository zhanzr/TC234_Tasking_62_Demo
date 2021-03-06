/**
 * \file IfxEth.c
 * \brief ETH  basic functionality
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

#include "IfxEth.h"

/******************************************************************************/
/*-----------------------Exported Variables/Constants-------------------------*/
/******************************************************************************/

uint8              IfxEth_rxBuffer[IFXETH_MAX_RX_BUFFERS][IFXETH_RTX_BUFFER_SIZE];

IfxEth_RxDescrList IfxEth_rxDescr;

uint8              IfxEth_txBuffer[IFXETH_MAX_TX_BUFFERS][IFXETH_RTX_BUFFER_SIZE];

IfxEth_TxDescrList IfxEth_txDescr;

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

void IfxEth_enableModule(void)
{
    {
        uint16 l_TempVar = IfxScuWdt_getCpuWatchdogPassword();
        IfxScuWdt_clearCpuEndinit(l_TempVar);
        ETH_CLC.U = 0;
        IfxScuWdt_setCpuEndinit(l_TempVar);
    }
}


void IfxEth_freeReceiveBuffer(IfxEth *eth)
{
    IfxEth_RxDescr *descr = IfxEth_getActualRxDescriptor(eth);
    IfxEth_RxDescr_release(descr);
    IfxEth_shuffleRxDescriptor(eth);
}


void *IfxEth_getReceiveBuffer(IfxEth *eth)
{
    void           *result = 0;
    IfxEth_RxDescr *descr;

    if (IfxEth_isRxDataAvailable(eth))
    {
        eth->rxCount++;
        descr  = IfxEth_getActualRxDescriptor(eth);
        result = (void *)(descr->RDES2.U);
    }

    IfxEth_wakeupReceiver(eth);

    return result;
}


void *IfxEth_getTransmitBuffer(IfxEth *eth)
{
    void           *buffer = NULL_PTR;
    IfxEth_TxDescr *descr  = IfxEth_getActualTxDescriptor(eth);

    // check descriptor / buffer is free.
    if (descr->TDES0.A.OWN == 0)
    {
        buffer = ((void *)descr->TDES2.U);
    }

    return buffer;
}


void IfxEth_init(IfxEth *eth, const IfxEth_Config *config)
{
    eth->ethSfr = config->ethSfr;

#ifndef _WIN32
    IfxEth_enableModule();

    if (config->phyInterfaceMode == IfxEth_PhyInterfaceMode_rmii)
    {
        IfxEth_setupRmiiOutputPins(eth, config->rmiiPins);
        IfxEth_setupRmiiInputPins(eth, config->rmiiPins);
    }
    else
    {
        IfxEth_setupMiiOutputPins(eth, config->miiPins);
        IfxEth_setupMiiInputPins(eth, config->miiPins);
    }

#endif

    IfxEth_resetModule();

    /* select the Phy Interface Mode */
    IfxEth_setPhyInterfaceMode(eth, config->phyInterfaceMode);
    IfxEth_applySoftwareReset(eth);

    /* wait until reset is finished or timeout. */
    {
        uint32 timeout = 0;

        while ((IfxEth_isSoftwareResetDone(eth) == 0) && (timeout < IFXETH_MAX_TIMEOUT_VALUE))
        {
            timeout++;
        }
    }

    /* configure bus mode */
    {
        Ifx_ETH_BUS_MODE busMode;
        busMode.U      = ETH_BUS_MODE.U;
        busMode.B.DSL  = 0; /* descriptor skip length in ring mode */
        busMode.B.ATDS = 0; /* alternate descriptor size: 0 => 4 DWORDS, 1 => 8 DWORDS */
        busMode.B.DA   = 0; /* 0 = weighted round-robin, 1 = fixed priority */

        ETH_BUS_MODE.U = busMode.U;
    }

    /* configure ETH MAC */
    {
        Ifx_ETH_MAC_CONFIGURATION ethMacCfg;
        ethMacCfg.U        = ETH_MAC_CONFIGURATION.U;

        ethMacCfg.B.PRELEN = 0;      /* 7 bytes preamble                                                                          */
        //  ethMacCfg.B.RE = 0;     /* disable receiver                                                                           */
        //  ethMacCfg.B.TE = 0;     /* disable transmitter                                                                        */
        ethMacCfg.B.DC          = 0; /* Deferral Check                                                                            */
        //  ethMacCfg.B.BL = 0;     /* Backoff Limit                                                                              */
        ethMacCfg.B.ACS         = 1; /* Automatic Pad/CRC stripping--less than 1536 bytes                                         */
        //  ethMacCfg.B.DR = 0;     /* Disable Retry                                                                              */
        ethMacCfg.B.IPC         = 0; /* checksum offload                                                                          */
        ethMacCfg.B.DM          = 1; /* Duplex Mode: 0=Half Duplex, 1=Full duplex                                                 */
        ethMacCfg.B.LM          = 0; /* Loopback Mode                                                                             */
        //  ethMacCfg.B.DO = 0;     /* Disable Receive Own                                                                        */
        ethMacCfg.B.FES         = 1; /* Speed 0->10mbps 1->100mbps                                                                */
        ethMacCfg.B.PS          = 1; /* port select 10/100mbps            */
        ethMacCfg.B.IFG         = 0; /* Inter Frame Gap - gap between frames = 96 bit times                                       */
        ethMacCfg.B.JE          = 0; /* Jumbo Frame Enable - no jumbo frames                                                      */
        ethMacCfg.B.JD          = 0; /* Jabber Disable - cut of transmission after 2,048 data bytes.                              */
        ethMacCfg.B.WD          = 0; /* Watchdog Disable - cut off frame after 2,048 bytes.                                       */
        ethMacCfg.B.CST         = 1; /* CRC stripping - last four bytes are stripped and dropped to application.                  */
        ethMacCfg.B.TWOKPE      = 0; /* 2K Packets Enable - with JE=0 - all received frames of size > 1,518bytes are Giant frames.*/

        ETH_MAC_CONFIGURATION.U = ethMacCfg.U;
    }

    IfxEth_setMacAddress(eth, config->macAddress);

    /* setup MMC */
    ETH_MMC_CONTROL.B.CNTFREEZ = 1;         /* disable MMC counters - counters reset */

    /* setup GMAC */
    ETH_STATUS.U           = 0x0001e7ff;    /* reset all interrupt flag(s) */
    ETH_MAC_FRAME_FILTER.U = 0x00000010;    /*  Hash Unicast */

    ETH_INTERRUPT_ENABLE.U = 0x00010041;    /* enable tx & rx interrupts */

#ifndef _WIN32

    if (config->isrPriority)
    {
        IfxSrc_init(&SRC_ETH, config->isrProvider, config->isrPriority);
        IfxSrc_enable(&SRC_ETH);
    }

    if (config->phyInit != NULL_PTR)
    {
        config->phyInit();          /* init PHY (100Mbit, full duplex with RMII) */
    }

#endif

    eth->config   = *config;
    eth->error    = 0;
    eth->status.U = 0;
    eth->rxCount  = 0;
    eth->txCount  = 0;

    IfxEth_stopTransmitter(eth);

    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, sizeof(IfxEth_TxDescr) == (IFXETH_DESCR_SIZE * sizeof(uint32)));
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, sizeof(IfxEth_RxDescr) == (IFXETH_DESCR_SIZE * sizeof(uint32)));

    eth->rxDescr = config->rxDescr;
    eth->txDescr = config->txDescr;

    IfxEth_initReceiveDescriptors(eth);
    IfxEth_initTransmitDescriptors(eth);
}


void IfxEth_initConfig(IfxEth_Config *config, Ifx_ETH *ethSfr)
{
    const IfxEth_Config defaultConfig = {
        {0x00, 0x11, 0x22, 0x33, 0x44, 0x55},        /* MAC address */
        NULL_PTR,
        NULL_PTR,
        IfxEth_PhyInterfaceMode_rmii,                /* PHY Interfac mode RMII */
        NULL_PTR,                                    /* Pointer to the RMII pin config */
        NULL_PTR,                                    /* Pointer to the MII pin config */
        (Ifx_Priority)0,                             /* Interrupt serivce priority */
        IfxSrc_Tos_cpu0,                             /* Interrupt serivce provider */
        NULL_PTR,                                    /* Pointer to register base */
        &IfxEth_rxDescr,                             /* pointer to RX descriptor RAM */
        &IfxEth_txDescr,                             /* pointer to TX descriptor RAM */
    };

    *config        = defaultConfig;
    config->ethSfr = ethSfr;
}


void IfxEth_initReceiveDescriptors(IfxEth *eth)
{
    /* Use enhanced descriptors, because ETH is generally configured to precision time protocol. */
    int             i;
    IfxEth_RxDescr *descr = IfxEth_getBaseRxDescriptor(eth);

    eth->pRxDescr = descr;

    /* init descriptor chained mode */
    for (i = 0; i < IFXETH_MAX_RX_BUFFERS; i++)
    {
        descr->RDES0.U      = 0;
        descr->RDES0.A.OWN  = 1U;

        descr->RDES1.U      = 0;
        descr->RDES1.A.RCH  = 1U;
        descr->RDES1.A.RBS1 = (IFXETH_RTX_BUFFER_SIZE);

#if !IFXETH_RX_BUFFER_BY_USER
        IfxEth_RxDescr_setBuffer(descr, &(IfxEth_rxBuffer[i][0]));
#endif

        /* with RCH set, link to next descriptor address */
        descr->RDES3.U = (uint32)&(descr[1]);
        descr          = &descr[1];
    }

    /* correction for last descriptor */
    {
        descr = &descr[-1];

        /* indicate end of ring */
        descr->RDES1.A.RER = 1U;

        /* with RCH set, link to first descriptor address */
        eth->pRxDescr  = IfxEth_getBaseRxDescriptor(eth);
        descr->RDES3.U = (uint32)eth->pRxDescr;
    }

    eth->rxCount = 0;

    /* write descriptor list base address */
    IfxEth_setReceiveDescriptorAddress(&MODULE_ETH, IfxEth_getBaseRxDescriptor(eth));
}


void IfxEth_initTransmitDescriptors(IfxEth *eth)
{
    int             i;
    IfxEth_TxDescr *descr = IfxEth_getBaseTxDescriptor(eth);

    eth->pTxDescr = descr;

    /* Initialize chained descriptor mode */
    for (i = 0; i < IFXETH_MAX_TX_BUFFERS; i++)
    {
        descr->TDES0.U     = 0;
        descr->TDES0.A.IC  = 1U;
        descr->TDES0.A.FS  = 1U;
        descr->TDES0.A.LS  = 1U;
        descr->TDES0.A.TCH = 1U;

#if !IFXETH_TX_BUFFER_BY_USER
        IfxEth_TxDescr_setBuffer(descr, &(IfxEth_txBuffer[i][0]));
#endif

        /* with TCH set, TDES3 points to next descriptor */
        descr->TDES3.U = (uint32)&descr[1];
        descr          = &descr[1];
    }

    /* correction for last descriptor */
    {
        descr = &descr[-1];

        /* indicate end of ring */
        descr->TDES0.A.TER = 1U;

        /* with TCH set, TDES3 points to the first descriptor */
        eth->pTxDescr  = IfxEth_getBaseTxDescriptor(eth);
        descr->TDES3.U = (uint32)eth->pTxDescr;
    }

    eth->txCount = 0;

    /* write descriptor list base address */
    IfxEth_setTransmitDescriptorAddress(&MODULE_ETH, IfxEth_getBaseTxDescriptor(eth));
}


void IfxEth_readMacAddress(IfxEth *eth, uint8 *macAddress)
{
    (void)eth;
    *((uint32 *)macAddress)       = ETH_MAC_ADDRESS_G00_LOW.U;
    *((uint16 *)(&macAddress[4])) = (uint16)(ETH_MAC_ADDRESS_G00_HIGH.U & 0xFFFFU);
}


void IfxEth_resetModule(void)
{
    uint16 passwd = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(passwd);

    ETH_KRST0.B.RST = 1;            /* Only if both Kernel reset bits are set a reset is executed */
    ETH_KRST1.B.RST = 1;
    IfxScuWdt_setCpuEndinit(passwd);

    while (0 == ETH_KRST0.B.RSTSTAT)    /* Wait until reset is executed */
    {}

    IfxScuWdt_clearCpuEndinit(passwd);
    ETH_KRSTCLR.B.CLR = 1;          /* Clear Kernel reset status bit */

    IfxScuWdt_setCpuEndinit(passwd);
}


void IfxEth_sendTransmitBuffer(IfxEth *eth, uint16 len)
{
    IfxEth_TxDescr *descr = IfxEth_getActualTxDescriptor(eth);

    descr->TDES1.U     = len;      /* with TCH set, TBS1 is used for buffer size */
    descr->TDES0.A.OWN = 1U;       /* release to DMA */

    IfxEth_wakeupTransmitter(eth);
    IfxEth_shuffleTxDescriptor(eth);

    eth->txCount++;
}


void IfxEth_setAndSendTransmitBuffer(IfxEth *eth, void *buffer, uint16 len)
{
    IfxEth_TxDescr_setBuffer(IfxEth_getActualTxDescriptor(eth), buffer);
    IfxEth_sendTransmitBuffer(eth, len);
}


void IfxEth_setMacAddress(IfxEth *eth, const uint8 *macAddress)
{
    (void)eth;
    ETH_MAC_ADDRESS_G00_HIGH.U = 0
                                 | ((uint32)macAddress[4] << 0U)
                                 | ((uint32)macAddress[5] << 8U)
                                 | 0x80000000U;

    ETH_MAC_ADDRESS_G00_LOW.U = 0
                                | ((uint32)macAddress[0] << 0U)
                                | ((uint32)macAddress[1] << 8U)
                                | ((uint32)macAddress[2] << 16U)
                                | ((uint32)macAddress[3] << 24U)
    ;
}


void IfxEth_setupChecksumEngine(IfxEth *eth, IfxEth_ChecksumMode mode)
{
    int i;

    if (mode != IfxEth_ChecksumMode_bypass)
    {
        ETH_OPERATION_MODE.B.TSF    = 1U;
        ETH_OPERATION_MODE.B.DT     = 0U; /* 0 = drop TCP/IP frame with checksum error */
        ETH_MAC_CONFIGURATION.B.IPC = 1U; /* 1 = enable received IP frame checksum engine */

        IfxEth_TxDescr *descr = IfxEth_getBaseTxDescriptor(eth);

        for (i = 0; i < IFXETH_MAX_TX_BUFFERS; i++)
        {
            descr->TDES0.A.CIC = mode;
            descr              = IfxEth_TxDescr_getNext(descr);
        }
    }
}


void IfxEth_setupMiiInputPins(IfxEth *eth, const IfxEth_MiiPins *miiPins)
{
    (void)eth;

    IfxPort_InputMode mode       = IfxPort_InputMode_noPullDevice;
    IfxPort_PadDriver speedGrade = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    IfxEth_Crs_In    *crs        = miiPins->crs;
    IfxEth_Col_In    *col        = miiPins->col;
    IfxEth_Txclk_In  *txClk      = miiPins->txClk;
    IfxEth_Rxclk_In  *rxClk      = miiPins->rxClk;
    IfxEth_Rxdv_In   *rxDv       = miiPins->rxDv;
    IfxEth_Rxer_In   *rxEr       = miiPins->rxEr;
    IfxEth_Rxd_In    *rxd0       = miiPins->rxd0;
    IfxEth_Rxd_In    *rxd1       = miiPins->rxd1;
    IfxEth_Rxd_In    *rxd2       = miiPins->rxd2;
    IfxEth_Rxd_In    *rxd3       = miiPins->rxd3;

    ETH_GPCTL.B.ALTI1  = rxClk->select;
    ETH_GPCTL.B.ALTI2  = crs->select;
    ETH_GPCTL.B.ALTI3  = col->select;
    ETH_GPCTL.B.ALTI4  = rxDv->select;
    ETH_GPCTL.B.ALTI5  = rxEr->select;
    ETH_GPCTL.B.ALTI6  = rxd0->select;
    ETH_GPCTL.B.ALTI7  = rxd1->select;
    ETH_GPCTL.B.ALTI8  = rxd2->select;
    ETH_GPCTL.B.ALTI9  = rxd3->select;
    ETH_GPCTL.B.ALTI10 = txClk->select;

    {
        IfxPort_setPinModeInput(crs->pin.port, crs->pin.pinIndex, mode);
        IfxPort_setPinModeInput(col->pin.port, col->pin.pinIndex, mode);
        IfxPort_setPinModeInput(txClk->pin.port, txClk->pin.pinIndex, mode);
        IfxPort_setPinModeInput(rxClk->pin.port, rxClk->pin.pinIndex, mode);
        IfxPort_setPinModeInput(rxDv->pin.port, rxDv->pin.pinIndex, mode);
        IfxPort_setPinModeInput(rxEr->pin.port, rxEr->pin.pinIndex, mode);
        IfxPort_setPinModeInput(rxd0->pin.port, rxd0->pin.pinIndex, mode);
        IfxPort_setPinModeInput(rxd1->pin.port, rxd1->pin.pinIndex, mode);
        IfxPort_setPinModeInput(rxd2->pin.port, rxd2->pin.pinIndex, mode);
        IfxPort_setPinModeInput(rxd3->pin.port, rxd3->pin.pinIndex, mode);

        IfxPort_setPinPadDriver(crs->pin.port, crs->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(col->pin.port, col->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(txClk->pin.port, txClk->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(rxClk->pin.port, rxClk->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(rxDv->pin.port, rxDv->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(rxEr->pin.port, rxEr->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(rxd0->pin.port, rxd0->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(rxd1->pin.port, rxd1->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(rxd2->pin.port, rxd2->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(rxd3->pin.port, rxd3->pin.pinIndex, speedGrade);
    }
}


void IfxEth_setupMiiOutputPins(IfxEth *eth, const IfxEth_MiiPins *miiPins)
{
    IfxPort_OutputMode mode       = IfxPort_OutputMode_pushPull;
    IfxPort_PadDriver  speedGrade = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    IfxEth_Txen_Out   *txEn       = miiPins->txEn;
    IfxEth_Txer_Out   *txEr       = miiPins->txEr;
    IfxEth_Txd_Out    *txd0       = miiPins->txd0;
    IfxEth_Txd_Out    *txd1       = miiPins->txd1;
    IfxEth_Txd_Out    *txd2       = miiPins->txd2;
    IfxEth_Txd_Out    *txd3       = miiPins->txd3;

    (void)eth;

    IfxPort_setPinPadDriver(txEn->pin.port, txEn->pin.pinIndex, speedGrade);
    IfxPort_setPinPadDriver(txEr->pin.port, txEr->pin.pinIndex, speedGrade);
    IfxPort_setPinPadDriver(txd0->pin.port, txd0->pin.pinIndex, speedGrade);
    IfxPort_setPinPadDriver(txd1->pin.port, txd1->pin.pinIndex, speedGrade);
    IfxPort_setPinPadDriver(txd2->pin.port, txd2->pin.pinIndex, speedGrade);
    IfxPort_setPinPadDriver(txd3->pin.port, txd3->pin.pinIndex, speedGrade);

    IfxPort_setPinModeOutput(txEn->pin.port, txEn->pin.pinIndex, mode, txEn->select);
    IfxPort_setPinModeOutput(txEr->pin.port, txEr->pin.pinIndex, mode, txEr->select);
    IfxPort_setPinModeOutput(txd0->pin.port, txd0->pin.pinIndex, mode, txd0->select);
    IfxPort_setPinModeOutput(txd1->pin.port, txd1->pin.pinIndex, mode, txd1->select);
    IfxPort_setPinModeOutput(txd2->pin.port, txd2->pin.pinIndex, mode, txd2->select);
    IfxPort_setPinModeOutput(txd3->pin.port, txd3->pin.pinIndex, mode, txd3->select);
}


void IfxEth_setupRmiiInputPins(IfxEth *eth, const IfxEth_RmiiPins *rmiiPins)
{
    (void)eth;

    ETH_GPCTL.B.ALTI0 = rmiiPins->mdio->inSelect;
    ETH_GPCTL.B.ALTI1 = rmiiPins->refClk->select;
    ETH_GPCTL.B.ALTI4 = rmiiPins->crsDiv->select;
    ETH_GPCTL.B.ALTI6 = rmiiPins->rxd0->select;
    ETH_GPCTL.B.ALTI7 = rmiiPins->rxd1->select;

    {
        IfxPort_InputMode mode       = IfxPort_InputMode_noPullDevice;
        IfxPort_PadDriver speedGrade = IfxPort_PadDriver_cmosAutomotiveSpeed1;

        IfxEth_Crsdv_In  *crsDiv     = rmiiPins->crsDiv;
        IfxEth_Refclk_In *refClk     = rmiiPins->refClk;
        IfxEth_Rxd_In    *rxd0       = rmiiPins->rxd0;
        IfxEth_Rxd_In    *rxd1       = rmiiPins->rxd1;

        IfxPort_setPinModeInput(crsDiv->pin.port, crsDiv->pin.pinIndex, mode);
        IfxPort_setPinModeInput(refClk->pin.port, refClk->pin.pinIndex, mode);
        IfxPort_setPinModeInput(rxd0->pin.port, rxd0->pin.pinIndex, mode);
        IfxPort_setPinModeInput(rxd1->pin.port, rxd1->pin.pinIndex, mode);

        IfxPort_setPinPadDriver(crsDiv->pin.port, crsDiv->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(refClk->pin.port, refClk->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(rxd0->pin.port, rxd0->pin.pinIndex, speedGrade);
        IfxPort_setPinPadDriver(rxd1->pin.port, rxd1->pin.pinIndex, speedGrade);
    }
}


void IfxEth_setupRmiiOutputPins(IfxEth *eth, const IfxEth_RmiiPins *rmiiPins)
{
    IfxPort_OutputMode mode       = IfxPort_OutputMode_pushPull;
    IfxPort_PadDriver  speedGrade = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    IfxEth_Mdc_Out    *mdc        = rmiiPins->mdc;
    IfxEth_Mdio_InOut *mdio       = rmiiPins->mdio;
    IfxEth_Txen_Out   *txen       = rmiiPins->txEn;
    IfxEth_Txd_Out    *txd0       = rmiiPins->txd0;
    IfxEth_Txd_Out    *txd1       = rmiiPins->txd1;

    (void)eth;

#if 0
    IfxPort_setPinPadDriver(mdc->pin.port, mdc->pin.pinIndex, speedGrade);
    IfxPort_setPinPadDriver(mdio->pin.port, mdio->pin.pinIndex, speedGrade);
#endif
    IfxPort_setPinPadDriver(txen->pin.port, txen->pin.pinIndex, speedGrade);
    IfxPort_setPinPadDriver(txd0->pin.port, txd0->pin.pinIndex, speedGrade);
    IfxPort_setPinPadDriver(txd1->pin.port, txd1->pin.pinIndex, speedGrade);

    IfxPort_setPinModeOutput(mdc->pin.port, mdc->pin.pinIndex, mode, mdc->select);
    IfxPort_setPinModeOutput(txen->pin.port, txen->pin.pinIndex, mode, txen->select);
    IfxPort_setPinModeOutput(txd0->pin.port, txd0->pin.pinIndex, mode, txd0->select);
    IfxPort_setPinModeOutput(txd1->pin.port, txd1->pin.pinIndex, mode, txd1->select);

    // For MDIO, when P21.1 is used it should be configured as output
    if ((mdio->pin.port == (&MODULE_P21)) && (mdio->pin.pinIndex == 1))
    {
        IfxPort_setPinModeOutput(mdio->pin.port, mdio->pin.pinIndex, mode, mdio->outSelect);
    }
}


void IfxEth_startReceiver(IfxEth *eth)
{
    (void)eth;

    // enable receiver and RX DMA
    ETH_OPERATION_MODE.B.SR    = 1;
    ETH_MAC_CONFIGURATION.B.RE = 1;
    ETH_RECEIVE_POLL_DEMAND.U  = 1;
}


void IfxEth_startTransmitter(IfxEth *eth)
{
    (void)eth;

    ETH_MAC_CONFIGURATION.B.TE = 1;
    ETH_OPERATION_MODE.B.ST    = 1;
    ETH_TRANSMIT_POLL_DEMAND.U = 1;
}


void IfxEth_stopTransmitter(IfxEth *eth)
{
    (void)eth;

    ETH_TRANSMIT_POLL_DEMAND.U = 0;
    ETH_OPERATION_MODE.B.ST    = 0;
    ETH_MAC_CONFIGURATION.B.TE = 0;
}


void IfxEth_wakeupReceiver(IfxEth *eth)
{
    eth->status.U = ETH_STATUS.U;

    // check if receiver suspended
    if (eth->status.U & (4U << IFX_ETH_STATUS_RS_OFF))
    {
        if (eth->status.B.RU)
        {
            ETH_STATUS.U = (IFX_ETH_STATUS_RU_MSK << IFX_ETH_STATUS_RU_OFF);
        }

        IfxEth_startReceiver(eth);
    }
}


void IfxEth_wakeupTransmitter(IfxEth *eth)
{
    eth->status.U = ETH_STATUS.U;

    // check if suspended
    if (eth->status.U & 0x00600000)
    {
        if (eth->status.B.TU)
        {
            // clear transmit unavailable and underflow flags
            ETH_STATUS.U = (IFX_ETH_STATUS_TU_MSK << IFX_ETH_STATUS_TU_OFF) |
                           (IFX_ETH_STATUS_UNF_MSK << IFX_ETH_STATUS_UNF_OFF);
        }

        IfxEth_startTransmitter(eth);
    }
}


void IfxEth_writeHeader(IfxEth *eth, uint8 *txBuffer, uint8 *destinationAddress, uint8 *sourceAddress, uint32 packetSize)
{
    (void)eth;
    uint32 i;

    /* Destination Address */
    for (i = 0; i < 6; i++)
    {
        *txBuffer++ = *destinationAddress++;
    }

    /* Source Address */
    for (i = 0; i < 6; i++)
    {
        *txBuffer++ = *sourceAddress++;
    }

    /* packet size */
    *txBuffer++ = (uint8)(packetSize / 256);
    *txBuffer   = (uint8)(packetSize % 256);
}
