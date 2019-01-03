/**
 * \file IfxQspi_SpiMaster.h
 * \brief QSPI SPIMASTER details
 * \ingroup IfxLld_Qspi
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
 * \defgroup IfxLld_Qspi_SpiMaster_Usage How to use the SPI Master Interface driver?
 * \ingroup IfxLld_Qspi
 *
 * The SPI Master interface driver provides a default QSPI configuration for a bidirectional serial communication of data words.
 *
 * Data transactions are buffered by the hardware based FIFOs. Incoming and outgoing data is transfered in background from/to the QSPI peripheral by interrupt service handlers, which are part of this driver as well. This allows a nonblocking communication without stalling the thread(s) from where data is sent and received.
 * Optionally Dma can be used for data transfers. Only the interrupt configuration and Module initialisation are different when dma is used.
 *
 * In the following sections it will be described, how to integrate the driver into the application framework.
 *
 * \section IfxLld_Qspi_SpiMaster_Preparation Preparation
 * \subsection IfxLld_Qspi_SpiMaster_Include Include Files
 *
 * Include following header file into your C code:
 * \code
 * #include <Qspi/SpiMaster/IfxQspi_SpiMaster.h>
 * \endcode
 *
 * \subsection IfxLld_Qspi_SpiMaster_Variables Variables
 *
 * Declare QSPI module and channel handles:
 *
 * \code
 * IfxQspi_SpiMaster spi;
 * IfxQspi_SpiMaster_Channel spiChannel;
 * \endcode
 *
 * In addition, declare global transmit and receive buffers for the data transfers:
 * \code
 * #define SPI_BUFFER_SIZE 8
 * uint8	spiTxBuffer[SPI_BUFFER_SIZE];
 * uint8	spiRxBuffer[SPI_BUFFER_SIZE];
 * \endcode
 *
 * \subsection IfxLld_Qspi_SpiMaster_Interrupt Interrupt Handler Installation (without dma use)
 *
 * See also \ref IfxLld_Cpu_Irq_Usage
 *
 * Define priorities for the Interrrupt handlers. This is normally done in the Ifx_IntPrioDef.h file:
 * \code
 * // priorities are normally defined in Ifx_IntPrioDef.h
 * #define IFX_INTPRIO_QSPI0_TX  1
 * #define IFX_INTPRIO_QSPI0_RX  2
 * #define IFX_INTPRIO_QSPI0_ER  5
 * \endcode
 *
 * Add the interrupt service routines to your C code. They have to call the QSPI interrupt handlers by passing the spi handle:
 * \code
 * IFX_INTERRUPT(qspi0TxISR, 0, IFX_INTPRIO_QSPI0_TX)
 * {
 *     IfxQspi_SpiMaster_isrTransmit(&spi);
 * }
 *
 * IFX_INTERRUPT(qspi0RxISR, 0, IFX_INTPRIO_QSPI0_RX)
 * {
 *     IfxQspi_SpiMaster_isrReceive(&spi);
 * }
 *
 * IFX_INTERRUPT(qspi0ErISR, 0, IFX_INTPRIO_QSPI0_ER)
 * {
 *     IfxQspi_SpiMaster_isrError(&spi);
 *
 *     // Process errors. Eg: parity Error is checked below
 *     IfxQspi_SpiMaster_Channel *chHandle   = IfxQspi_SpiMaster_activeChannel(&spi);
 *     if( chHandle->errorFlags.parityError == 1)
 *     {
 *         // Parity Error
 *     }
 * }
 * \endcode
 *
 * Finally install the interrupt handlers in your initialisation function:
 * \code
 *     // install interrupt handlers
 *     IfxCpu_Irq_installInterruptHandler(&qspi0TxISR, IFX_INTPRIO_QSPI0_TX);
 *     IfxCpu_Irq_installInterruptHandler(&qspi0RxISR, IFX_INTPRIO_QSPI0_RX);
 *     IfxCpu_Irq_installInterruptHandler(&qspi0ErISR, IFX_INTPRIO_QSPI0_ER);
 *     IfxCpu_enableInterrupts();
 * \endcode
 *
 * \subsection IfxLld_Qspi_SpiMaster_Interrupt_dma Interrupt Handler Installation (with dma use)
 *
 * See also \ref IfxLld_Cpu_Irq_Usage
 *
 * Define priorities for the Interrrupt handlers. This is normally done in the Ifx_IntPrioDef.h file:
 * \code
 * // qspi priorities
 * #define IFX_INTPRIO_QSPI0_TX  1 // DMA channel 1
 * #define IFX_INTPRIO_QSPI0_RX  2 // DMA channel 2
 * #define IFX_INTPRIO_QSPI0_ER  0x30
 *
 * // dma priorities
 * #define IFX_INTPRIO_DMA_CH1  10
 * #define IFX_INTPRIO_DMA_CH2  11
 * \endcode
 *
 * Add the interrupt service routines to your C code. They have to call the QSPI interrupt handlers by passing the spi handle:
 * \code
 * IFX_INTERRUPT(qspi0DmaTxISR, 0, IFX_INTPRIO_DMA_CH1 )
 * {
 *     IfxQspi_SpiMaster_isrDmaTransmit(&spi);
 * }
 *
 * IFX_INTERRUPT(qspi0DmaRxISR, 0, IFX_INTPRIO_DMA_CH2)
 * {
 *    IfxQspi_SpiMaster_isrDmaReceive(&spi);
 * }
 *
 * IFX_INTERRUPT(qspi0ErISR, 0, IFX_INTPRIO_QSPI0_ER)
 * {
 *     IfxQspi_SpiMaster_isrError(&spi);
 *
 *     // Process errors. Eg: parity Error is checked below
 *     IfxQspi_SpiMaster_Channel *chHandle   = IfxQspi_SpiMaster_activeChannel(&spi);
 *     if( chHandle->errorFlags.parityError == 1)
 *     {
 *         // Parity Error
 *     }
 * }
 * \endcode
 *
 * Finally install the interrupt handlers in your initialisation function:
 * \code
 *     // install interrupt handlers
 *     IfxCpu_Irq_installInterruptHandler(&qspi0DmaTxISR, IFX_INTPRIO_DMA_CH1);
 *     IfxCpu_Irq_installInterruptHandler(&qspi0DmaRxISR, IFX_INTPRIO_DMA_CH2);
 *     IfxCpu_Irq_installInterruptHandler(&qspi0ErISR,    IFX_INTPRIO_QSPI0_ER);
 *     IfxCpu_enableInterrupts();
 * \endcode
 *
 *
 * \subsection IfxLld_Qspi_SpiMaster_Init Module Initialisation (without dma use)
 *
 * The module initialisation can be done in the same function.
 *
 * Here an example for master mode:
 * \code
 *     // create module config
 *     IfxQspi_SpiMaster_Config spiMasterConfig;
 *     IfxQspi_SpiMaster_initModuleConfig(&spiMasterConfig, &MODULE_QSPI0);
 *
 *     // set the desired mode and maximum baudrate
 *     spiMasterConfig.base.mode             = SpiIf_Mode_master;
 *     spiMasterConfig.base.maximumBaudrate  = 10000000;
 *
 *     // ISR priorities and interrupt target
 *     spiMasterConfig.base.txPriority       = IFX_INTPRIO_QSPI0_TX;
 *     spiMasterConfig.base.rxPriority       = IFX_INTPRIO_QSPI0_RX;
 *     spiMasterConfig.base.erPriority       = IFX_INTPRIO_QSPI0_ER;
 *     spiMasterConfig.base.isrProvider      = IfxCpu_Irq_getTos(IfxCpu_getCoreIndex());
 *
 *     // pin configuration
 *     const IfxQspi_SpiMaster_Pins pins = {
 *         &IfxQspi0_SCLK_P20_11_OUT, IfxPort_OutputMode_pushPull, // SCLK
 *         &IfxQspi0_MTSR_P20_14_OUT, IfxPort_OutputMode_pushPull, // MTSR
 *         &IfxQspi0_MRSTA_P20_12_IN, IfxPort_InputMode_pullDown,  // MRST
 *         IfxPort_PadDriver_cmosAutomotiveSpeed3 // pad driver mode
 *     };
 *     spiMasterConfig.pins = &pins;
 *
 *     // initialize module
 *     //IfxQspi_SpiMaster spi; // defined globally
 *     IfxQspi_SpiMaster_initModule(&spi, &spiMasterConfig);
 * \endcode
 *
 * \subsection IfxLld_Qspi_SpiMaster_Init_dma Module Initialisation (with dma use)
 *
 * The module initialisation can be done in the same function.
 *
 * Here an example for master mode:
 * \code
 *     // create module config
 *     IfxQspi_SpiMaster_Config spiMasterConfig;
 *     IfxQspi_SpiMaster_initModuleConfig(&spiMasterConfig, &MODULE_QSPI0);
 *
 *     // set the desired mode and maximum baudrate
 *     spiMasterConfig.base.mode             = SpiIf_Mode_master;
 *     spiMasterConfig.base.maximumBaudrate  = 10000000;
 *
 *     // ISR priorities and interrupt target (with Dma usage)
 *     spiMasterConfig.base.txPriority       = IFX_INTPRIO_DMA_CH1;
 *     spiMasterConfig.base.rxPriority       = IFX_INTPRIO_DMA_CH2;
 *     spiMasterConfig.base.erPriority       = IFX_INTPRIO_QSPI0_ER;
 *
 *     // dma configuration.
 *     spiMasterConfig.dma.txDmaChannelId = IfxDma_ChannelId_1;
 *     spiMasterConfig.dma.rxDmaChannelId = IfxDma_ChannelId_2;
 *     spiMasterConfig.dma.useDma = 1;
 *
 *     // pin configuration
 *     const IfxQspi_SpiMaster_Pins pins = {
 *         &IfxQspi0_SCLK_P20_11_OUT, IfxPort_OutputMode_pushPull, // SCLK
 *         &IfxQspi0_MTSR_P20_14_OUT, IfxPort_OutputMode_pushPull, // MTSR
 *         &IfxQspi0_MRSTA_P20_12_IN, IfxPort_InputMode_pullDown,  // MRST
 *         IfxPort_PadDriver_cmosAutomotiveSpeed3 // pad driver mode
 *     };
 *     spiMasterConfig.pins = &Pins;
 *
 *
 *     // initialize module
 *     //IfxQspi_SpiMaster spi; // defined globally
 *     IfxQspi_SpiMaster_initModule(&spi, &spiMasterConfig);
 * \endcode
 *
 *
 * \subsection IfxLld_Qspi_SpiMaster_ChannelInit SPI Channel Initialisation
 *
 * After the module has been initialized, one or more SPI channels can be configured.
 * Each channel has a dedicated select line.
 *
 * Here an example for a SPI channel in master mode:
 * \code
 *     // create channel config
 *     IfxQspi_SpiMaster_ChannelConfig spiMasterChannelConfig;
 *     IfxQspi_SpiMaster_initChannelConfig(&spiMasterChannelConfig, &spi);
 *
 *     // set the baudrate for this channel
 *     spiMasterChannelConfig.base.baudrate = 5000000;
 *
 *     // select pin configuration
 *     const IfxQspi_SpiMaster_Output slsOutput = {
 *         &IfxQspi0_SLSO7_P33_5_OUT,
 *         IfxPort_OutputMode_pushPull,
 *         IfxPort_PadDriver_cmosAutomotiveSpeed1
 *     };
 *     spiMasterChannelConfig.sls.output = (IfxQspi_SpiMaster_Output)slsOutput;
 *
 *     // initialize channel
 *     //IfxQspi_SpiMaster_Channel spiChannel; // defined globally
 *     IfxQspi_SpiMaster_initChannel(&spiChannel, &spiMasterChannelConfig);
 * \endcode
 *
 * The QSPI is ready for use now!
 *
 *
 * \section IfxLld_Qspi_SpiMaster_DataTransfers Data Transfers
 *
 * In following examples we assume, that following buffers are declared globally:
 * \code
 * // declared somewhere globally
 * #define SPI_BUFFER_SIZE 8
 * uint8 spiTxBuffer[SPI_BUFFER_SIZE] = { 1, 2, 3, 4, 5, 6, 7, 8 };
 * uint8 spiRxBuffer[SPI_BUFFER_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0 };
 * \endcode
 *
 * Sending and Receiving a data stream:
 * \code
 *      int i=0;
 *     // wait until transfer of previous data stream is finished
 *     while( IfxQspi_SpiMaster_getStatus(&spiChannel) == SpiIf_Status_busy );
 *
 *     // send/receive new stream
 *     IfxQspi_SpiMaster_exchange(&spiChannel, &spiTxBuffer[i], &spiRxBuffer[i], SPI_BUFFER_SIZE);
 * \endcode
 *
 * Send only, discard received data:
 * \code
 *
 *     // wait until transfer of previous data stream is finished
 *     while( IfxQspi_SpiMaster_getStatus(&spiChannel) == SpiIf_Status_busy );
 *
 *     // send new stream
 *     IfxQspi_SpiMaster_exchange(&spiChannel, &spiTxBuffer[i], NULL_PTR, SPI_BUFFER_SIZE);
 * \endcode
 *
 * Receive only, send all-1
 * \code
 *     // wait until transfer of previous data stream is finished
 *     while( IfxQspi_SpiMaster_getStatus(&spiChannel) == SpiIf_Status_busy );
 *
 *     // receive new stream
 *     IfxQspi_SpiMaster_exchange(&spiChannel, NULL_PTR, &spiRxBuffer[i], SPI_BUFFER_SIZE);
 * \endcode
 *
 * \section IfxLld_Qspi_SpiMaster_PhaseTransition Phase transition and User Interrupt usage
 *
 * Phase transition and user defined interrupts are not configured internal to Driver.
 *
 * Example usage of Phase transition interrupt is given below
 * \code
 *     IfxCpu_Irq_installInterruptHandler(&qspi0PtISR,    IFX_INTPRIO_QSPI0_PT);
 * IfxQspi_pause(&MODULE_QSPI0);
 * IfxQspi_configPT1Event(&MODULE_QSPI0, IfxQspi_PhaseTransitionEvent_startOfFrame); // Configured for PT1 SOF event
 * IfxQspi_enablePT1Event(&MODULE_QSPI0,TRUE);
 * IfxQspi_run(&MODULE_QSPI0);
 * \endcode
 *
 * Add the interrupt service routines to your C code. They have to call the QSPI interrupt handlers by passing the spi handle:
 * \code
 * IFX_INTERRUPT(qspi0PtISR, 0, IFX_INTPRIO_QSPI0_PT )
 * {
 *     IfxQspi_PhaseTransitionEvent event;
 *     event = IfxQspi_SpiMaster_isrPhaseTransition(&spiMaster);
 *     // process event
 *     switch (event)
 *     {
 *       case IfxQspi_PhaseTransitionEvent_startOfFrame:
 *       //  SOF event
 *       break;
 *      // add other events if needed.
 *     }
 * }
 * \endcode
 *
 * \section IfxLld_Qspi_SpiMaster_XXLMode How to use XXL Mode with Dma
 *
 * The Qspi XXL mode is similar to Short mode with the exception to configure mode to xxl before calling IfxQspi_SpiMaster_initChannel() function. The remaining usage is similar to Short / ShortContinuous Mode (default one) with Dma
 *
 * \code
 * spiMasterChannelConfig.mode = IfxQspi_SpiMaster_Mode_xxl;
 * \endcode
 *
 *
 * \section IfxLld_Qspi_SpiMaster_LongMode How to use Long / Long Continuous Mode with Dma
 *
 * The qspi Long mode is similar to short mode Except for below additional changes.
 * 1. Transmit buffer need to be defined (this is in addition to source/TX data buffer. This buffer is used to store the DATA + CONFIG for dma usage).
 * 2. Configure the Spi Channel configuration mode (new one) to long or longContinuous mode
 * 3. Preparing Transmit buffer content for long/long Continuous mode
 *
 * Example Code is provided below.
 *
 * \subsection IfxLld_Qspi_SpiMaster_Include Include Files
 *
 * Include following header file into your C code:
 * \code
 * #include <Qspi/SpiMaster/IfxQspi_SpiMaster.h>
 * \endcode
 *
 * \subsection IfxLld_Qspi_SpiMaster_Variables Variables
 *
 * Declare QSPI module and channel handles:
 *
 * \code
 * IfxQspi_SpiMaster spi;
 * IfxQspi_SpiMaster_Channel spiChannel;
 * \endcode
 *
 * In addition, declare global transmit and receive buffers for the data transfers:
 * \code
 * #define SPI_BUFFER_SIZE    50
 * #define BACON_SIZE IFXQSPI_BACONSIZE(SPI_BUFFER_SIZE)
 * #define FIFO_SIZE IFXQSPI_FIFO32BITSIZE(SPI_BUFFER_SIZE)
 * // This should be 32 bit ; use addition of macros as shown
 * __attribute__ ((aligned(64))) uint32       spi0TxLongBuffer[BACON_SIZE + FIFO_SIZE];
 *
 * uint8	spiTxBuffer[SPI_BUFFER_SIZE];
 * uint8	spiRxBuffer[SPI_BUFFER_SIZE];
 * \endcode
 *
 * \subsection IfxLld_Qspi_SpiMaster_Init_dma Module Initialisation (with dma use)
 *
 * The module initialisation can be done in the same function.
 *
 * Here an example for master mode:
 * \code
 *     // create module config
 *     IfxQspi_SpiMaster_Config spiMasterConfig;
 *     IfxQspi_SpiMaster_initModuleConfig(&spiMasterConfig, &MODULE_QSPI0);
 *
 *     // set the desired mode and maximum baudrate
 *     spiMasterConfig.base.mode             = SpiIf_Mode_master;
 *     spiMasterConfig.base.maximumBaudrate  = 10000000;
 *
 *     // ISR priorities and interrupt target (with Dma usage)
 *     spiMasterConfig.base.txPriority       = IFX_INTPRIO_DMA_CH1;
 *     spiMasterConfig.base.rxPriority       = IFX_INTPRIO_DMA_CH2;
 *     spiMasterConfig.base.erPriority       = IFX_INTPRIO_QSPI0_ER;
 *
 *     // dma configuration.
 *     spiMasterConfig.dma.txDmaChannelId = IfxDma_ChannelId_1;
 *     spiMasterConfig.dma.rxDmaChannelId = IfxDma_ChannelId_2;
 *     spiMasterConfig.dma.useDma = 1;
 *
 *     // pin configuration
 *     const IfxQspi_SpiMaster_Pins pins = {
 *         &IfxQspi0_SCLK_P20_11_OUT, IfxPort_OutputMode_pushPull, // SCLK
 *         &IfxQspi0_MTSR_P20_14_OUT, IfxPort_OutputMode_pushPull, // MTSR
 *         &IfxQspi0_MRSTA_P20_12_IN, IfxPort_InputMode_pullDown,  // MRST
 *         IfxPort_PadDriver_cmosAutomotiveSpeed3 // pad driver mode
 *     };
 *     spiMasterConfig.pins = &Pins;
 *
 *
 *     // initialize module
 *     //IfxQspi_SpiMaster spi; // defined globally
 *     IfxQspi_SpiMaster_initModule(&spi, &spiMasterConfig);
 * \endcode
 *
 *
 * \subsection IfxLld_Qspi_SpiMaster_ChannelInit SPI Channel Initialisation
 *
 * After the module has been initialized, one or more SPI channels can be configured.
 * Each channel has a dedicated select line.
 *
 * Here an example for a SPI channel in master mode:
 * \code
 *     // create channel config
 *     IfxQspi_SpiMaster_ChannelConfig spiMasterChannelConfig;
 *     IfxQspi_SpiMaster_initChannelConfig(&spiMasterChannelConfig, &spi);
 *
 *     // set the baudrate for this channel
 *     spiMasterChannelConfig.base.baudrate = 5000000;
 *
 *     // select pin configuration
 *     const IfxQspi_SpiMaster_Output slsOutput = {
 *         &IfxQspi0_SLSO7_P33_5_OUT,
 *         IfxPort_OutputMode_pushPull,
 *         IfxPort_PadDriver_cmosAutomotiveSpeed1
 *     };
 *     spiMasterChannelConfig.sls.output = (IfxQspi_SpiMaster_Output)slsOutput;
 *
 *     spiMasterChannelConfig.mode = IfxQspi_SpiMaster_Mode_longContinuous; // Select Long Continuous Mode
 *
 *     // initialize channel
 *     //IfxQspi_SpiMaster_Channel spiChannel; // defined globally
 *     IfxQspi_SpiMaster_initChannel(&spiChannel, &spiMasterChannelConfig);
 * \endcode
 *
 * Sending and Receiving a Long data stream:
 * \code
 *      int i=0;
 *     // Pack data and Configuration to Transmit buffer (Only for Long / Long Continuous Mode(s))
 *     IfxQspi_SpiMaster_packLongModeBuffer(&spiMasterChannel, spi0TxBuffer, spi0TxLongBuffer, SPI_BUFFER_SIZE);
 *
 *     // wait until transfer of previous data stream is finished
 *     while( IfxQspi_SpiMaster_getStatus(&spiChannel) == SpiIf_Status_busy );
 *
 *     // send/receive new stream
 *     IfxQspi_SpiMaster_exchange(&spiChannel, &spiTxBuffer[i], &spiRxBuffer[i], SPI_BUFFER_SIZE);
 * \endcode
 *
 * \defgroup IfxLld_Qspi_SpiMaster SPI Master Driver
 * \ingroup IfxLld_Qspi
 * \defgroup IfxLld_Qspi_SpiMaster_DataStructures Data Structures
 * \ingroup IfxLld_Qspi_SpiMaster
 * \defgroup IfxLld_Qspi_SpiMaster_Module Module Functions
 * \ingroup IfxLld_Qspi_SpiMaster
 * \defgroup IfxLld_Qspi_SpiMaster_Channel Channel Functions
 * \ingroup IfxLld_Qspi_SpiMaster
 * \defgroup IfxLld_Qspi_SpiMaster_Support Support Functions
 * \ingroup IfxLld_Qspi_SpiMaster
 * \defgroup IfxLld_Qspi_SpiMaster_Com Communication
 * \ingroup IfxLld_Qspi_SpiMaster
 * \defgroup IfxLld_Qspi_SpiMaster_InterruptFunctions Interrupt Functions
 * \ingroup IfxLld_Qspi_SpiMaster
 * \defgroup IfxLld_Qspi_SpiMaster_DirectFifo Direct FIFO Access
 * \ingroup IfxLld_Qspi_SpiMaster
 */

#ifndef IFXQSPI_SPIMASTER_H
#define IFXQSPI_SPIMASTER_H 1

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "Cpu/Irq/IfxCpu_Irq.h"
#include "Dma/Dma/IfxDma_Dma.h"
#include "Qspi/Std/IfxQspi.h"
#include "Scu/Std/IfxScuWdt.h"

/******************************************************************************/
/*------------------------------Type Definitions------------------------------*/
/******************************************************************************/

typedef struct IfxQspi_SpiMaster_Channel_s IfxQspi_SpiMaster_Channel;

typedef void                             (*IfxQspi_SpiMaster_AutoSlso)(IfxQspi_SpiMaster_Channel *chHandle);

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

/** \addtogroup IfxLld_Qspi_SpiMaster_DataStructures
 * \{ */
typedef enum
{
    IfxQspi_SpiMaster_ChannelBasedCs_disabled = 0,  /**< \brief Slso will be low for complete frame */
    IfxQspi_SpiMaster_ChannelBasedCs_enabled  = 1   /**< \brief Slso will toggle with every byte */
} IfxQspi_SpiMaster_ChannelBasedCs;

typedef enum
{
    IfxQspi_SpiMaster_Mode_short           = 0,  /**< \brief Short Mode */
    IfxQspi_SpiMaster_Mode_long            = 1,  /**< \brief Long Mode */
    IfxQspi_SpiMaster_Mode_shortContinuous = 2,  /**< \brief Short Continous Mode */
    IfxQspi_SpiMaster_Mode_longContinuous  = 3,  /**< \brief Long Continous Mode */
    IfxQspi_SpiMaster_Mode_xxl             = 4   /**< \brief XXL Mode */
} IfxQspi_SpiMaster_Mode;

/** \} */

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

/** \addtogroup IfxLld_Qspi_SpiMaster_DataStructures
 * \{ */
/** \brief SLSI  pin configuration structure
 */
typedef struct
{
    const IfxQspi_Slsi_In *pin;      /**< \brief Pointer to SLSI in pin */
    IfxPort_InputMode      mode;     /**< \brief The SLSI pin input mode */
} IfxQspi_SpiMaster_Input;

/** \brief SLSO  pin configuration structure
 */
typedef struct
{
    const IfxQspi_Slso_Out *pin;        /**< \brief Pointer to SLSO out pin */
    IfxPort_OutputMode      mode;       /**< \brief The SLSO pin output mode */
    IfxPort_PadDriver       driver;     /**< \brief The pad driver mode which should be configured */
} IfxQspi_SpiMaster_Output;

/** \} */

/** \addtogroup IfxLld_Qspi_SpiMaster_DataStructures
 * \{ */
/** \brief Dma handle
 */
typedef struct
{
    IfxDma_Dma_Channel rxDmaChannel;       /**< \brief receive DMA channel handle */
    IfxDma_Dma_Channel txDmaChannel;       /**< \brief transmit DMA channel handle */
    IfxDma_ChannelId   rxDmaChannelId;     /**< \brief DMA channel no for the Spi recieve */
    IfxDma_ChannelId   txDmaChannelId;     /**< \brief DMA channel no for the Spi transmit */
    boolean            useDma;             /**< \brief use Dma for Data transfer/s */
} IfxQspi_SpiMaster_Dma;

/** \brief Dma configuration
 */
typedef struct
{
    IfxDma_ChannelId rxDmaChannelId;     /**< \brief DMA channel no for the Spi recieve */
    IfxDma_ChannelId txDmaChannelId;     /**< \brief DMA channel no for the Spi transmit */
    boolean          useDma;             /**< \brief use Dma for Data transfer/s */
} IfxQspi_SpiMaster_DmaConfig;

/** \brief Qspi Master Mode Error Flags
 */
typedef struct
{
    uint16 parityError : 1;            /**< \brief [0:0] Parity Error */
    uint16 configurationError : 1;     /**< \brief [1:1] Configuration Error */
    uint16 baudrateError : 1;          /**< \brief [2:2] baudrate Error */
    uint16 txFifoOverflowError : 1;    /**< \brief [3:3] TxFifo Overflow Error */
    uint16 txFifoUnderflowError : 1;   /**< \brief [4:4] TxFifo underflow Error */
    uint16 rxFifoOverflowError : 1;    /**< \brief [5:5] RxFifo Overflow Error */
    uint16 rxFifoUnderflowError : 1;   /**< \brief [6:6] RxFifo underflow Error */
    uint16 expectTimeoutError : 1;     /**< \brief [7:7] Expect Timeout Error */
} IfxQspi_SpiMaster_ErrorFlags;

/** \brief Union of Slave Select pins
 */
typedef union
{
    IfxQspi_SpiMaster_Input  input;      /**< \brief SLSI  pin configuration structure */
    IfxQspi_SpiMaster_Output output;     /**< \brief SLSO  pin configuration structure */
} IfxQspi_SpiMaster_InputOutput;

/** \brief Master pin IO configuration structure
 */
typedef struct
{
    const IfxQspi_Sclk_Out *sclk;          /**< \brief Pointer to SLCK out pin */
    IfxPort_OutputMode      sclkMode;      /**< \brief The SCLK pin output mode */
    const IfxQspi_Mtsr_Out *mtsr;          /**< \brief Pointer to MTSR out pin */
    IfxPort_OutputMode      mtsrMode;      /**< \brief The MTSR pin output mode */
    const IfxQspi_Mrst_In  *mrst;          /**< \brief Pointer to MRST in pin */
    IfxPort_InputMode       mrstMode;      /**< \brief The MRST pin input mode */
    IfxPort_PadDriver       pinDriver;     /**< \brief The pad driver mode which should be configured */
} IfxQspi_SpiMaster_Pins;

/** \} */

/** \addtogroup IfxLld_Qspi_SpiMaster_DataStructures
 * \{ */
/** \brief Module handle data structure
 */
typedef struct
{
    SpiIf                 base;     /**< \brief Module SPI interface handle */
    Ifx_QSPI             *qspi;     /**< \brief Pointer to QSPI module registers */
    IfxQspi_SpiMaster_Dma dma;      /**< \brief dma handle */
} IfxQspi_SpiMaster;

/** \brief Module Channel configuration structure
 */
typedef struct
{
    SpiIf_ChConfig                   base;               /**< \brief SPI interface channel configuration structure */
    IfxQspi_SpiMaster_InputOutput    sls;                /**< \brief Union of Slave Select pins */
    IfxQspi_SpiMaster_ChannelBasedCs channelBasedCs;     /**< \brief define the slso behaviour */
    IfxQspi_SpiMaster_Mode           mode;               /**< \brief Qspi Operating Mode */
} IfxQspi_SpiMaster_ChannelConfig;

/** \brief Module Channel handle structure
 */
struct IfxQspi_SpiMaster_Channel_s
{
    SpiIf_Ch                         base;                /**< \brief SPI interface channel handle structure */
    Ifx_QSPI_BACON                   bacon;               /**< \brief basic configuration register */
    IfxPort_Pin                      slso;                /**< \brief Defines SLSO pin */
    IfxQspi_SpiMaster_AutoSlso       activateSlso;        /**< \brief Specifies  function for Auto SLSO activation */
    IfxQspi_SpiMaster_AutoSlso       deactivateSlso;      /**< \brief Specifies  function for Auto SLSO deactivation */
    IfxQspi_ChannelId                channelId;           /**< \brief QSPI channel Number */
    Ifx_ActiveState                  slsoActiveState;     /**< \brief Specifies the current state of SLSO */
    uint8                            dataWidth;           /**< \brief Number of bits which will be written into the FIFO */
    boolean                          firstWrite;          /**< \brief Specifies whether the data id first write or not. */
    IfxQspi_SpiMaster_ChannelBasedCs channelBasedCs;      /**< \brief define the slso behaviour */
    IfxQspi_SpiMaster_Mode           mode;                /**< \brief Qspi Operating Mode */
    IfxQspi_SpiMaster_ErrorFlags     errorFlags;          /**< \brief Spi Master Error Flags */
};

/** \brief Module configuration structure
 */
typedef struct
{
    SpiIf_Config                  base;                           /**< \brief SPI interface configuration structure */
    Ifx_QSPI                     *qspi;                           /**< \brief Pointer to QSPI module registers */
    boolean                       allowSleepMode;                 /**< \brief Specifies module sleep mode */
    boolean                       pauseOnBaudrateSpikeErrors;     /**< \brief Specifies module pause on baudrate or spike errors */
    IfxQspi_PauseRunTransition    pauseRunTransition;             /**< \brief Specifies module run or pause mode */
    IfxQspi_TxFifoInt             txFifoThreshold;                /**< \brief Specifies the TXFIFO interrupt threshold */
    IfxQspi_RxFifoInt             rxFifoThreshold;                /**< \brief Specifies the RXFIFO interrupt threshold */
    const IfxQspi_SpiMaster_Pins *pins;                           /**< \brief structure for QSPI Master pins */
    IfxQspi_SpiMaster_DmaConfig   dma;                            /**< \brief Dma configuration */
} IfxQspi_SpiMaster_Config;

/** \} */

/** \addtogroup IfxLld_Qspi_SpiMaster_Module
 * \{ */

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief Initialises the module
 * \param handle Module handle
 * \param config Predefined configuration structure of the module
 * \return None
 *
 * Usage example: see \ref IfxLld_Qspi_SpiMaster_Usage
 *
 */
IFX_EXTERN void IfxQspi_SpiMaster_initModule(IfxQspi_SpiMaster *handle, const IfxQspi_SpiMaster_Config *config);

/** \brief Fills the config structure with default values
 * \param config Configuration structure which should be initialized.
 * \param qspi pointer to QSPI registers
 * \return None
 *
 * Usage example: see \ref IfxLld_Qspi_SpiMaster_Usage
 *
 */
IFX_EXTERN void IfxQspi_SpiMaster_initModuleConfig(IfxQspi_SpiMaster_Config *config, Ifx_QSPI *qspi);

/** \} */

/** \addtogroup IfxLld_Qspi_SpiMaster_Channel
 * \{ */

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief Initializes the channel
 * \param chHandle Module Channel handle
 * \param chConfig Channel configuration structure
 * \return Status of Channel (busy or ok or failure)
 *
 * Usage example: see \ref IfxLld_Qspi_SpiMaster_Usage
 *
 */
IFX_EXTERN SpiIf_Status IfxQspi_SpiMaster_initChannel(IfxQspi_SpiMaster_Channel *chHandle, const IfxQspi_SpiMaster_ChannelConfig *chConfig);

/** \brief Fills the config structure with default values
 * \param chConfig Configuration structure which should be initialized.
 * \param handle Module handle
 * \return None
 *
 * Usage example: see \ref IfxLld_Qspi_SpiMaster_Usage
 *
 */
IFX_EXTERN void IfxQspi_SpiMaster_initChannelConfig(IfxQspi_SpiMaster_ChannelConfig *chConfig, IfxQspi_SpiMaster *handle);

/** \} */

/** \addtogroup IfxLld_Qspi_SpiMaster_Com
 * \{ */

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief Exchanges data between source and data
 * \param chHandle Module Channel handle
 * \param src Source of data. Can be set to NULL_PTR if nothing to receive (transmit only)
 * \param dest Destination to which to be sent. Can be set to NULL_PTR if nothing to transmit (receive only) - in this case, all-1 will be sent.
 * \param count Number of data in pending
 * \return Status of exchange of data
 *
 * Usage example: see \ref IfxLld_Qspi_SpiMaster_Usage
 *
 */
IFX_EXTERN SpiIf_Status IfxQspi_SpiMaster_exchange(IfxQspi_SpiMaster_Channel *chHandle, const void *src, void *dest, Ifx_SizeT count);

/** \brief Gets the transmission status
 * \param chHandle Module Channel handle
 * \return Transmission status
 *
 * Usage example: see \ref IfxLld_Qspi_SpiMaster_Usage
 *
 */
IFX_EXTERN SpiIf_Status IfxQspi_SpiMaster_getStatus(IfxQspi_SpiMaster_Channel *chHandle);

/** \} */

/** \addtogroup IfxLld_Qspi_SpiMaster_InterruptFunctions
 * \{ */

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief Transmit interrupt handler
 * \param qspiHandle Module handle
 * \return None
 */
IFX_EXTERN void IfxQspi_SpiMaster_isrDmaReceive(IfxQspi_SpiMaster *qspiHandle);

/** \brief Transmit interrupt handler
 * \param qspiHandle Module handle
 * \return None
 */
IFX_EXTERN void IfxQspi_SpiMaster_isrDmaTransmit(IfxQspi_SpiMaster *qspiHandle);

/** \brief Error Interrupt handler
 * \param handle Module handle
 * \return None
 */
IFX_EXTERN void IfxQspi_SpiMaster_isrError(IfxQspi_SpiMaster *handle);

/** \brief Phase Transition interrupt handler
 * \param handle Module handle
 * \return Phase Transition Event
 */
IFX_EXTERN IfxQspi_PhaseTransitionEvent IfxQspi_SpiMaster_isrPhaseTransition(IfxQspi_SpiMaster *handle);

/** \brief Receive Interrupt handler
 * \param handle Module handle
 * \return None
 */
IFX_EXTERN void IfxQspi_SpiMaster_isrReceive(IfxQspi_SpiMaster *handle);

/** \brief Transmit interrupt handler
 * \param handle Module handle
 * \return None
 */
IFX_EXTERN void IfxQspi_SpiMaster_isrTransmit(IfxQspi_SpiMaster *handle);

/** \brief User defined interrupt handler
 * \param handle Module handle
 * \return User Event
 */
IFX_EXTERN IfxQspi_PhaseTransitionEvent IfxQspi_SpiMaster_isrUserDefined(IfxQspi_SpiMaster *handle);

/** \} */

/** \addtogroup IfxLld_Qspi_SpiMaster_DirectFifo
 * \{ */

/******************************************************************************/
/*-------------------------Inline Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief Reads data or status in RxFIFO
 * \param handle Module handle
 * \return Data or Status in RxFIFO
 */
IFX_INLINE uint32 IfxQspi_SpiMaster_readReceiveFifo(IfxQspi_SpiMaster *handle);

/** \brief Writes Basic configuration value to Tx FIFO
 * \param handle Module handle
 * \param baconVal Basic configuration value to be entered in TxFIFO
 * \return None
 */
IFX_INLINE void IfxQspi_SpiMaster_writeBasicConfiguration(IfxQspi_SpiMaster *handle, uint32 baconVal);

/** \brief Writes extended configuration of the channel
 * \param chHandle Module Channel handle
 * \param econVal Channel Timing configuration value
 * \return None
 */
IFX_INLINE void IfxQspi_SpiMaster_writeExtendedConfiguration(IfxQspi_SpiMaster_Channel *chHandle, uint32 econVal);

/** \brief Writes data and configuration mixed value to Tx FIFO
 * \param handle Module handle
 * \param mixEntryVal Mixed of Data and configuration
 * \return None
 */
IFX_INLINE void IfxQspi_SpiMaster_writeMixedDataConfiguration(IfxQspi_SpiMaster *handle, uint32 mixEntryVal);

/** \brief Writes the data to TxFIFO
 * \param chHandle Module Channel handle
 * \param data Data to be entered in Tx FIFO
 * \return None
 */
IFX_INLINE void IfxQspi_SpiMaster_writeTransmitFifo(IfxQspi_SpiMaster_Channel *chHandle, uint32 data);

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief pack data to Long mode buffer
 * \param chHandle Module Channel handle
 * \param data source Data buffer to be packed
 * \param longFifoBuffer destination buffer for data and configuration
 * \param dataLength size of data in source buffer (in bytes)
 * \return None
 */
IFX_EXTERN void IfxQspi_SpiMaster_packLongModeBuffer(IfxQspi_SpiMaster_Channel *chHandle, void *data, uint32 *longFifoBuffer, Ifx_SizeT dataLength);

/** \} */

/******************************************************************************/
/*-------------------------Global Function Prototypes-------------------------*/
/******************************************************************************/

/** \brief Gets the channel configuration
 * \param chHandle Module Channel handle
 * \return Channel configuration
 */
IFX_EXTERN SpiIf_ChConfig IfxQspi_SpiMaster_getChannelConfig(IfxQspi_SpiMaster_Channel *chHandle);

/** \brief Set the channel baudrate
 * \param chHandle Module Channel handle
 * \param baudrate Baudrate to be configured (in Baud)
 * \return Status of Channel (busy or ok or failure)
 */
IFX_EXTERN SpiIf_Status IfxQspi_SpiMaster_setChannelBaudrate(IfxQspi_SpiMaster_Channel *chHandle, float baudrate);

/******************************************************************************/
/*---------------------Inline Function Implementations------------------------*/
/******************************************************************************/

IFX_INLINE uint32 IfxQspi_SpiMaster_readReceiveFifo(IfxQspi_SpiMaster *handle)
{
    Ifx_QSPI *qspiSFR = handle->qspi;
    uint32    data    = IfxQspi_readReceiveFifo(qspiSFR);
    return data;
}


IFX_INLINE void IfxQspi_SpiMaster_writeBasicConfiguration(IfxQspi_SpiMaster *handle, uint32 baconVal)
{
    Ifx_QSPI *qspiSFR = handle->qspi;
    IfxQspi_writeBasicConfiguration(qspiSFR, baconVal);
}


IFX_INLINE void IfxQspi_SpiMaster_writeExtendedConfiguration(IfxQspi_SpiMaster_Channel *chHandle, uint32 econVal)
{
    IfxQspi_SpiMaster *handle = (IfxQspi_SpiMaster *)chHandle->base.driver;
    IfxQspi_writeExtendedConfiguration(handle->qspi, chHandle->channelId, econVal);
}


IFX_INLINE void IfxQspi_SpiMaster_writeMixedDataConfiguration(IfxQspi_SpiMaster *handle, uint32 mixEntryVal)
{
    Ifx_QSPI *qspiSFR = handle->qspi;
    IfxQspi_writeMixedDataTransmitFifo(qspiSFR, mixEntryVal);
}


IFX_INLINE void IfxQspi_SpiMaster_writeTransmitFifo(IfxQspi_SpiMaster_Channel *chHandle, uint32 data)
{
    IfxQspi_SpiMaster *handle = (IfxQspi_SpiMaster *)chHandle->base.driver;
    IfxQspi_writeTransmitFifo(handle->qspi, data);
}


#endif /* IFXQSPI_SPIMASTER_H */
