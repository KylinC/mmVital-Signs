/**
 *   @file  mss_main.c
 *
 *   @brief
 *     MSS main implementation of the Vital Signs Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/HwiP.h>

/* Demo Include Files */
#include "mss_mmw.h"
#include "mmw_messages.h"


/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_MCB    gMmwMssMCB;

/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/
/* CLI Init function */
extern void MmwDemo_CLIInit (void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions Prototype**************
 **************************************************************************/

/* Data path functions */
int32_t MmwDemo_mssDataPathConfig(void);
int32_t MmwDemo_mssDataPathStart(void);
int32_t MmwDemo_mssDataPathStop(void);

/* mmwave library call back fundtions */
void MmwDemo_mssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg);
void MmwDemo_mssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg);
static void MmwDemo_mssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg);
static void MmwDemo_mssMmwaveCloseCallbackFxn(void);

void MmwDemo_mssMmwaveStopcallbackFxn(void);
int32_t MmwDemo_mssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* MMW demo Task */
void MmwDemo_mssInitTask(UArg arg0, UArg arg1);
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);
void MmwDemo_mssCtrlPathTask(UArg arg0, UArg arg1);

/* external sleep function when in idle (used in .cfg file) */
void MmwDemo_sleep(void);

/* DSS to MSS exception signalling ISR */
static void MmwDemo_installDss2MssExceptionSignallingISR(void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Registered event function which is invoked when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always returns 0 [Continue passing the event to the peer domain]
 */
int32_t MmwDemo_mssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

#if 0
    System_printf ("Debug: BSS Event MsgId: %d [Sub Block Id: %d Sub Block Length: %d]\n",
                    msgId, sbId, sbLen);
#endif

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_CPUFAULT_EVT);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_ESMFAULT_EVT);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    rlRfInitComplete_t*  ptrRFInitCompleteMessage;
                    uint32_t            calibrationStatus;

                    /* Get the RF-Init completion message: */
                    ptrRFInitCompleteMessage = (rlRfInitComplete_t*)payload;
                    calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0xFFFU;

                    /* Display the calibration status: */
                    CLI_write ("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
                    break;
                }
                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    /* This event is not handled on MSS */
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    /* Increment the statistics for the number of failed reports */
                    gMmwMssMCB.stats.numFailedTimingReports++;

                    #if 0 
                    /* if something needs to be done then need to implement the function
                       to handle the event below in MmwDemo_mssCtrlPathTask()*/
                            
                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_MONITORING_REP_EVT);
                    #endif
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    /* Increment the statistics for the number of received calibration reports */
                    gMmwMssMCB.stats.numCalibrationReports++;

                    #if 0 
                    /* if something needs to be done then need to implement the function
                       to handle the event below in MmwDemo_mssCtrlPathTask()*/
                            
                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_CALIBRATION_REP_EVT);
                    #endif
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    /*Received Frame Stop async event from BSS.
                      No further action required on MSS as it will
                      wait for a message from DSS when it is done (MMWDEMO_DSS2MSS_STOPDONE)*/
                    break;
                }
                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked after the configuration
 *      has been used to configure the mmWave link and the BSS. This is applicable only for
 *      the XWR16xx. The BSS can be configured only by the MSS *or* DSS. The callback API is
 *      triggered on the remote execution domain (which did not configure the BSS)
 *
 *  @param[in]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg)
{
    /* For mmw Demo, mmwave_config() will always be called from MSS, 
       due to the fact CLI is running on MSS, hence this callback won't be called */

    gMmwMssMCB.stats.datapathConfigEvt ++;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been opened.
 *
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been closed.
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveCloseCallbackFxn(void)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been started. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibration configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg)
{
    /* Post an event to main data path task. 
       This function in only called when mmwave_start() is called on DSS */
    gMmwMssMCB.stats.datapathStartEvt ++;

    /* Post event to start is done */
    Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_COMPLETED_EVT);
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been stopped. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveStopCallbackFxn(void)
{
    /* Possible sceanarios:
       1. CLI sensorStop command triggers mmwave_stop() to be called from MSS 
       2. In case of Error, mmwave_stop() will be triggered either from MSS or DSS
     */
    gMmwMssMCB.stats.datapathStopEvt ++;
}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel 
 *
 *  @param[in]  message
 *      Pointer to the MMW demo message.  
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1 
 */
int32_t MmwDemo_mboxWrite(MmwDemo_message     * message)
{
    int32_t                  retVal = -1;
    
    retVal = Mailbox_write (gMmwMssMCB.peerMailbox, (uint8_t*)message, sizeof(MmwDemo_message));
    if (retVal == sizeof(MmwDemo_message))
    {
        retVal = 0;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from 
 *      Mailbox virtual channel.  
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_mboxReadTask(UArg arg0, UArg arg1)
{
    MmwDemo_message      message;
    int32_t              retVal = 0;
    uint32_t totalPacketLen;
    uint32_t numPaddingBytes;
    uint32_t itemIdx;

    /* wait for new message and process all the messages received from the peer */
    while(1)
    {
        Semaphore_pend(gMmwMssMCB.mboxSemHandle, BIOS_WAIT_FOREVER);
        
        /* Read the message from the peer mailbox: We are not trying to protect the read
         * from the peer mailbox because this is only being invoked from a single thread */
        retVal = Mailbox_read(gMmwMssMCB.peerMailbox, (uint8_t*)&message, sizeof(MmwDemo_message));
        if (retVal < 0)
        {
            /* Error: Unable to read the message. Setup the error code and return values */
            System_printf ("Error: Mailbox read failed [Error code %d]\n", retVal);
        }
        else if (retVal == 0)
        {
            /* We are done: There are no messages available from the peer execution domain. */
            continue;
        }
        else
        {
            /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
             * allow us to receive another message in the mailbox while we process the received message. */
            Mailbox_readFlush (gMmwMssMCB.peerMailbox);

            /* Process the received message: */
            switch (message.type)
            {
                case MMWDEMO_DSS2MSS_DETOBJ_READY:
                    /* Got detetced objectes , shipped out through UART */
                    /* Send header */
                    totalPacketLen = sizeof(MmwDemo_output_message_header);
                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                       (uint8_t*)&message.body.detObj.header,
                                       sizeof(MmwDemo_output_message_header));

                    /* Send TLVs */
                    for (itemIdx = 0;  itemIdx < message.body.detObj.header.numTLVs; itemIdx++)
                    {
                        UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                           (uint8_t*)&message.body.detObj.tlv[itemIdx],
                                           sizeof(MmwDemo_output_message_tl));
                        UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                           (uint8_t*)SOC_translateAddress(message.body.detObj.tlv[itemIdx].address,
                                                                          SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
                                           message.body.detObj.tlv[itemIdx].length);
                        totalPacketLen += sizeof(MmwDemo_output_message_tl) + message.body.detObj.tlv[itemIdx].length;
                    }

                    /* Send padding to make total packet length multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
                    numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (totalPacketLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
                    if (numPaddingBytes<MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
                    {
                        uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
                        /*DEBUG:*/ memset(&padding, 0xf, MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
                        UART_writePolling (gMmwMssMCB.loggingUartHandle,
                                            padding,
                                            numPaddingBytes);
                    }

                    /* Send a message to MSS to log the output data */
                    memset((void *)&message, 0, sizeof(MmwDemo_message));

                    message.type = MMWDEMO_MSS2DSS_DETOBJ_SHIPPED;

                    if (MmwDemo_mboxWrite(&message) != 0)
                    {
                        System_printf ("Error: Mailbox send message id=%d failed \n", message.type);
                    }

                    break;
                case MMWDEMO_DSS2MSS_STOPDONE:
                    /* Post event that stop is done */
                    Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_STOP_COMPLETED_EVT);
                break;
                case MMWDEMO_DSS2MSS_ASSERT_INFO:
                    /* Send the received DSS assert info through CLI */
                    CLI_write ("DSS Exception: %s, line %d.\n", message.body.assertInfo.file,
                        message.body.assertInfo.line);
                break;
                case MMWDEMO_DSS2MSS_ISR_INFO_ADDRESS:
                    gMmwMssMCB.dss2mssIsrInfoAddress = 
                        SOC_translateAddress(message.body.dss2mssISRinfoAddress,
                                             SOC_TranslateAddr_Dir_FROM_OTHER_CPU, NULL);
                    MmwDemo_installDss2MssExceptionSignallingISR();
                break;
                case MMWDEMO_DSS2MSS_MEASUREMENT_INFO:
                    /* Send the received DSS calibration info through CLI */
                    CLI_write ("compRangeBiasAndRxChanPhase");
                    CLI_write (" %.7f", message.body.compRxChanCfg.rangeBias);
                    int32_t i;
                    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
                    {
                        CLI_write (" %.5f", (float) message.body.compRxChanCfg.rxChPhaseComp[i].real/32768.);
                        CLI_write (" %.5f", (float) message.body.compRxChanCfg.rxChPhaseComp[i].imag/32768.);
                    }
                    CLI_write ("\n");
                break;
                default:
                {
                    /* Message not support */
                    System_printf ("Error: unsupport Mailbox message id=%d\n", message.type);
                    break;
                }
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      This function is a callback funciton that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received
 
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mboxCallback
(
    Mbox_Handle  handle,
    Mailbox_Type    peer
)
{
    /* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
     * the received message. */
    Semaphore_post (gMmwMssMCB.mboxSemHandle);
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Configuration on MSS. After received Configuration from
 *    CLI, this function will start the system configuration process, inclucing mmwaveLink, BSS
 *    and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathConfig(void)
{
    int32_t             errCode;
    MMWave_ErrorLevel   errorLevel;
    int16_t             mmWaveErrorCode;
    int16_t             subsysErrorCode;

    /* Has the mmWave module been opened? */
    if (gMmwMssMCB.isMMWaveOpen == false)
    {
        /* Get the open configuration from the CLI mmWave Extension */
        CLI_getMMWaveExtensionOpenConfig (&gMmwMssMCB.cfg.openCfg);

        /* Setup the calibration frequency:
         * TODO: Presently DFP does not support these for 68xx platform,
         * need to change when DFP is updated with the support */
        gMmwMssMCB.cfg.openCfg.freqLimitLow = 600U;
        gMmwMssMCB.cfg.openCfg.freqLimitHigh = 640U;

        /* Disable the frame start async event so that small chirp times
           are supported. If this event is enabled it will break real-time
           for small chirp times and cause 1D processing to crash
           due to lack of MIPS*/
        gMmwMssMCB.cfg.openCfg.disableFrameStartAsyncEvent = false;

        /* Enable frame stop async event so that we know when BSS has stopped*/
        gMmwMssMCB.cfg.openCfg.disableFrameStopAsyncEvent = false;

        /* No custom calibration: */
        gMmwMssMCB.cfg.openCfg.useCustomCalibration        = false;
        gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask = 0x0;
#ifndef SOC_XWR68XX_ES1 
        /* calibration monitoring base time unit
         * setting it to one frame duration as the demo doesnt support any 
         * monitoring related functionality
         */
        gMmwMssMCB.cfg.openCfg.calibMonTimeUnit            = 1;
#endif
        /* Open the mmWave module: */
        if (MMWave_open(gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.openCfg, NULL, &errCode) < 0)
        {
            /* Error: decode and Report the error */
            MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
            System_printf ("Error: mmWave Open failed [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);
            return -1;
        }

        /* mmWave module has been opened. */
        gMmwMssMCB.isMMWaveOpen = true;
    }
    else
    {
        /* openCfg related configurations like chCfg, lowPowerMode, adcCfg
         * are only used on the first sensor start. If they are different
         * on a subsequent sensor start, then generate a fatal error
         * so the user does not think that the new (changed) configuration
         * takes effect, the board needs to be reboot for the new
         * configuration to be applied.
         */
        MMWave_OpenCfg openCfg;

        CLI_getMMWaveExtensionOpenConfig (&openCfg);

        /* Initialize to same as in "if" part where open is done
         * to allow memory compare of structures to be used.
         * Note that even if structures may have holes, the memory
         * compare is o.k because CLI always stores the configurations
         * in the same global CLI structure which is copied over to the
         * one supplied by the application through the
         * CLI_getMMWaveExtensionOpenConfig API. Not using memcmp will
         * require individual field comparisons which is probably
         * more code size and cumbersome.
         */
		openCfg.freqLimitLow = 600U;
		openCfg.freqLimitHigh = 640U;
		openCfg.disableFrameStartAsyncEvent = false;
		openCfg.disableFrameStopAsyncEvent = false;

		/* No custom calibration: */
		gMmwMssMCB.cfg.openCfg.useCustomCalibration        = false;
		gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask = 0x0;
        /* Compare openCfg */
        if(memcmp((void *)&gMmwMssMCB.cfg.openCfg, (void *)&openCfg,
                          sizeof(MMWave_OpenCfg)) != 0)
        {
            /* Post event to CLI task that start failed so that CLI/GUI can be notified */
            Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_FAILED_EVT);

            MmwDemo_mssAssert(0);
        }
    }

    /* Get the control configuration from the CLI mmWave Extension */
    CLI_getMMWaveExtensionConfig (&gMmwMssMCB.cfg.ctrlCfg);

    /* Configure the mmWave module: */
    if (MMWave_config (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
    {
        System_printf ("Error: MMWDemoMSS mmWave Configuration failed [Error code %d]\n", errCode);
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Start on MSS. After received SensorStart command, MSS will 
 *    start all data path componets including mmwaveLink, BSS and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathStart(void)
{
    int32_t                 errCode;
    MMWave_CalibrationCfg   calibrationCfg;

    /* Initialize the calibration configuration: */
    memset ((void*)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

    /* Populate the calibration configuration: */
    calibrationCfg.dfeDataOutputMode                          = 
        gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start (gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        /* Error: Unable to start the mmWave control */
        System_printf ("Error: MMWDemoMSS mmWave Start failed [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: MMWDemoMSS mmWave Start succeeded \n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Start on MSS. After received SensorStart command, MSS will 
 *    start all data path componets including mmwaveLink, BSS and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathStop(void)
{
    int32_t             errCode;
    MMWave_ErrorLevel   errorLevel;
    int16_t             mmWaveErrorCode;
    int16_t             subsysErrorCode;
    int32_t             retVal = 0;

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_stop (gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error/Warning: Unable to stop the mmWave module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error: Set the return value to indicate error: */
            System_printf ("Error: MMWDemoMSS mmWave Stop failed [mmWave Error: %d Subsys: %d]\n", mmWaveErrorCode, subsysErrorCode);
            retVal = -1;
        }
        else
        {
			System_printf("Warning: MMWDemoMSS mmWave Stop failed [mmWave Error: %d Subsys: %d]\n", mmWaveErrorCode, subsysErrorCode);
            /* Warning: This is treated as a successful stop. */
        }
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gMmwMssMCB.ctrlHandle, &errCode) < 0)
            System_printf ("Error: mmWave control execution failed [Error code %d]\n", errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be invoked from the CLI or
 *      the Switch press to start the sensor. This sends an event to the
 *      sensor management task where the actual *start* procedure is
 *      implemented.
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_notifySensorStart(bool doReconfig)
{
    gMmwMssMCB.stats.cliSensorStartEvt ++;

    if (doReconfig) {
        /* Post sensorStart event to start reconfig and then start the sensor */
        Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_SENSORSTART_EVT);
    }
    else
    {
        /* Post frameStart event to skip reconfig and just start the sensor */
        Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_FRAMESTART_EVT);
    }
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be invoked from the CLI or
 *      the Switch press to start the sensor. This sends an event to the
 *      sensor management task where the actual *start* procedure is
 *      implemented.
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_notifySensorStop(void)
{
    gMmwMssMCB.stats.cliSensorStopEvt ++;

    /* Post sensorSTOP event to notify sensor stop command */
    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_SENSORSTOP_EVT);
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be used by the CLI to 
 *      pend for start complete (after MmwDemo_notifySensorStart is called)
 *
 *  @retval
 *      Not applicable
 */
int32_t MmwDemo_waitSensorStartComplete(void)
{
    UInt          event;
    int32_t       retVal;
    /* Pend on the START NOTIFY event */
    event = Event_pend(gMmwMssMCB.eventHandleNotify, 
                          Event_Id_NONE, 
                          MMWDEMO_DSS_START_COMPLETED_EVT | MMWDEMO_DSS_START_FAILED_EVT,
                          BIOS_WAIT_FOREVER);

    /************************************************************************
     * DSS event:: START notification
     ************************************************************************/
    if(event & MMWDEMO_DSS_START_COMPLETED_EVT)
    {
        /* Sensor has been started successfully */
        gMmwMssMCB.isSensorStarted = true;
        /* Turn on the LED */
		GPIO_write(SOC_XWR68XX_GPIO_2, 1U);
        retVal = 0;
    }
    else if(event & MMWDEMO_DSS_START_FAILED_EVT)
    {
        /* Sensor start failed */
        gMmwMssMCB.isSensorStarted = false;
        retVal = -1;
    } 
    else 
    {
        /* we should block forever till we get the events. If the desired event 
           didnt happen, then throw an assert */
        retVal = -1;
        MmwDemo_mssAssert(0);
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is a utility function which can be used by the CLI to 
 *      pend for stop complete (after MmwDemo_notifySensorStart is called)
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_waitSensorStopComplete(void)
{
    UInt          event;
    /* Pend on the START NOTIFY event */
    event = Event_pend(gMmwMssMCB.eventHandleNotify, 
                          Event_Id_NONE, 
                          MMWDEMO_DSS_STOP_COMPLETED_EVT,
                          BIOS_WAIT_FOREVER);

    /************************************************************************
     * DSS event:: STOP notification
     ************************************************************************/
    if(event & MMWDEMO_DSS_STOP_COMPLETED_EVT)
    {
        /* Sensor has been stopped successfully */
        gMmwMssMCB.isSensorStarted = false;
        
        /* Turn off the LED */
		GPIO_write(SOC_XWR68XX_GPIO_2, 0U);
        
        /* print for user */
        System_printf("Sensor has been stopped\n");
    }
    else {
        /* we should block forever till we get the event. If the desired event 
           didnt happen, then throw an assert */
        MmwDemo_mssAssert(0);
    }
}



/**
 *  @b Description
 *  @n
 *      Callback function invoked when the GPIO switch is pressed.
 *      This is invoked from interrupt context.
 *
 *  @param[in]  index
 *      GPIO index configured as input
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_switchPressFxn(unsigned int index)
{
    /* Was the sensor started? */
    if (gMmwMssMCB.isSensorStarted == true)
    {
        /* YES: We need to stop the sensor now */
        MmwDemo_notifySensorStop();
    }
    else
    {
        /* NO: We need to start the sensor now. */
        MmwDemo_notifySensorStart(true);
    }
}

/**
 *  @b Description
 *  @n
 *      The task is used to process data path events
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mssCtrlPathTask(UArg arg0, UArg arg1)
{
    UInt          event;

    /**********************************************************************
     * Setup the PINMUX:
     * - GPIO Input : Configure pin J13 as GPIO_1 input
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINJ13_PADAC, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINJ13_PADAC, SOC_XWR68XX_PINJ13_PADAC_GPIO_1);
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINK13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINK13_PADAZ, SOC_XWR68XX_PINK13_PADAZ_GPIO_2);

    /**********************************************************************
     * Setup the SW1 switch on the EVM connected to GPIO_1
     * - This is used as an input
     * - Enable interrupt to be notified on a switch press
     **********************************************************************/
	GPIO_setConfig(SOC_XWR68XX_GPIO_1, GPIO_CFG_INPUT | GPIO_CFG_IN_INT_RISING | GPIO_CFG_IN_INT_LOW);
	GPIO_setCallback(SOC_XWR68XX_GPIO_1, MmwDemo_switchPressFxn);
	GPIO_enableInt(SOC_XWR68XX_GPIO_1);

    /**********************************************************************
     * Setup the DS3 LED on the EVM connected to GPIO_2
     **********************************************************************/
	GPIO_setConfig(SOC_XWR68XX_GPIO_2, GPIO_CFG_OUTPUT);

    /* Data Path management task Main loop */
    while (1)
    {
        event = Event_pend(gMmwMssMCB.eventHandle, 
                          Event_Id_NONE, 
                          MMWDEMO_CLI_EVENTS | MMWDEMO_BSS_FAULT_EVENTS |
                          MMWDEMO_DSS_EXCEPTION_EVENTS,
                          BIOS_WAIT_FOREVER); 

        /************************************************************************
         * CLI event:: SensorStart
         ************************************************************************/

        if(event & MMWDEMO_CLI_SENSORSTART_EVT)
        {
            System_printf ("Debug: MMWDemoMSS Received CLI sensorStart Event\n");
        
            /* Setup the data path: */
            if(gMmwMssMCB.isSensorStarted==false)
            {
                if (MmwDemo_mssDataPathConfig () < 0) {
                    /* Post start failed event; error printing is done in function above */
                    Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_FAILED_EVT);
                    continue;
                }
                else
                {
                    /* start complete event is posted via DSS */
                }
            }
            else
            {
                /* Post start complete event as this is just a duplicate command */
                Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_COMPLETED_EVT);
                continue;
            }
        }

        /************************************************************************
         * CLI event:: SensorStop
         ************************************************************************/
        if(event & MMWDEMO_CLI_SENSORSTOP_EVT)
        {
            if(gMmwMssMCB.isSensorStarted==true)
            {
                if (MmwDemo_mssDataPathStop () < 0) {
                    /* do we need stop fail event; for now just mark the stop as completed */
                    Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_STOP_COMPLETED_EVT);
                    continue;
                }
                else
                {
                    /* DSS will post the stop completed event */
                }
            }
            else
            {
                /* Post stop complete event as this is just a duplicate command */
                Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_STOP_COMPLETED_EVT);
                continue;
            }
        }

        /************************************************************************
         * CLI event:: Framestart
         ************************************************************************/
        if(event & MMWDEMO_CLI_FRAMESTART_EVT)
        {
            /* error printing happens inside this function */
            if(gMmwMssMCB.isSensorStarted==false) 
            {
                if (MmwDemo_mssDataPathStart () < 0) {
                    /* Post start failed event; error printing is done in function above */
                    Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_FAILED_EVT);
                    continue;
                }
            }

            /* Post event to start is done */
            Event_post(gMmwMssMCB.eventHandleNotify, MMWDEMO_DSS_START_COMPLETED_EVT);
        }

        /************************************************************************
         * BSS event:: CPU fault
         ************************************************************************/
        if(event & MMWDEMO_BSS_CPUFAULT_EVT)
        {
            MmwDemo_mssAssert(0);
            break;
        }

        /************************************************************************
         * BSS event:: ESM fault
         ************************************************************************/
        if(event & MMWDEMO_BSS_ESMFAULT_EVT)
        {
            MmwDemo_mssAssert(0);
            break;
        }

        if(event & MMWDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT)
        {
            CLI_write ("DSS Chirp Processing Deadline Miss Exception.\n");
            DebugP_assert(0);
            break;
        }
        
        if(event & MMWDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT)
        {
            CLI_write ("DSS Frame Processing Deadline Miss Exception.\n");
            DebugP_assert(0);
            break;
        }
    }

    System_printf("Debug: VitalSignsDemoDSS Data path exit\n");
}

/**
 *  @b Description
 *  @n
 *      DSS to MSS ISR used for direct signalling of things like urgent exception
 *      events from DSS. Posts deadline miss events to @ref MmwDemo_mssCtrlPathTask.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_Dss2MssISR(uintptr_t arg)
{  
    switch(*(uint8_t*)gMmwMssMCB.dss2mssIsrInfoAddress)
    {
        case MMWDEMO_DSS2MSS_CHIRP_PROC_DEADLINE_MISS_EXCEPTION:
            Event_post(gMmwMssMCB.eventHandle, MMWDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT);
        break;
        
        case MMWDEMO_DSS2MSS_FRAME_PROC_DEADLINE_MISS_EXCEPTION:
            Event_post(gMmwMssMCB.eventHandle, MMWDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT);
        break;

        default:
            MmwDemo_mssAssert(0);
        break;
    }
}

/**
 *  @b Description
 *  @n
 *      Installs DSS to MSS Software Interrupt ISR for exception signalling.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_installDss2MssExceptionSignallingISR(void)
{
    HwiP_Params  hwiParams;
    volatile HwiP_Handle  hwiHandle;

    HwiP_Params_init(&hwiParams);
    hwiParams.name = "Dss2MssSwISR";
    hwiHandle = HwiP_create(MMWDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_MSS, 
                            MmwDemo_Dss2MssISR, &hwiParams);
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mssInitTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    UART_Params         uartParams;
    Task_Params         taskParams;
    Semaphore_Params    semParams;
    Mailbox_Config      mboxCfg;
    Error_Block         eb;

    /* Debug Message: */
    System_printf("Debug: VitalSignsDemoMSS Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/
    /* Pinmux setting */

    /* Setup the PINMUX to bring out the UART-1 */
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINN5_PADBE, SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX);
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINN4_PADBD, SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX);

    /* Setup the PINMUX to bring out the UART-3 */
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINF14_PADAJ, SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX);

    /* Setup the PINMUX to bring out the DSS UART */
	Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINP8_PADBM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
	Pinmux_Set_FuncSel(SOC_XWR68XX_PINP8_PADBM, SOC_XWR68XX_PINP8_PADBM_DSS_UART_TX);

    /* Initialize the UART */
    UART_init();

    /* Initialize the GPIO */
    GPIO_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /*****************************************************************************
     * Open & configure the drivers:
     *****************************************************************************/

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency  = gMmwMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate        = gMmwMssMCB.cfg.commandBaudRate;
    uartParams.isPinMuxDone    = 1U;

    /* Open the UART Instance */
    gMmwMssMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMssMCB.commandUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Command UART Instance\n");
        return;
    }

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMssMCB.cfg.loggingBaudRate;
    uartParams.isPinMuxDone   = 1U;

    /* Open the Logging UART Instance: */
    gMmwMssMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMssMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Logging UART Instance\n");
        return;
    }

    /*****************************************************************************
     * Creating communication channel between MSS & DSS
     *****************************************************************************/

    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    Semaphore_Params_init(&semParams);
    semParams.mode             = Semaphore_Mode_BINARY;
    gMmwMssMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &MmwDemo_mboxCallback;

    /* Initialization of Mailbox Virtual Channel  */
    gMmwMssMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_DSS, &mboxCfg, &errCode);
    if (gMmwMssMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }

    /* Create task to handle mailbox messges */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(MmwDemo_mboxReadTask, &taskParams, NULL);

    /*****************************************************************************
     * Create Event to handle mmwave callback and system datapath events 
     *****************************************************************************/
    /* Default instance configuration params */
    Error_init(&eb);
    gMmwMssMCB.eventHandle = Event_create(NULL, &eb);
    if (gMmwMssMCB.eventHandle == NULL) 
    {
        MmwDemo_mssAssert(0);
        return ;
    }
    
    Error_init(&eb);
    gMmwMssMCB.eventHandleNotify = Event_create(NULL, &eb);
    if (gMmwMssMCB.eventHandleNotify == NULL) 
    {
        MmwDemo_mssAssert(0);
        return ;
    }

    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Initialize the mmWave control init configuration */
    memset ((void*)&initCfg, 0 , sizeof(MMWave_InitCfg));

    /* Populate the init configuration for mmwave library: */
    initCfg.domain                      = MMWave_Domain_MSS;
    initCfg.socHandle                   = gMmwMssMCB.socHandle;
    initCfg.eventFxn                    = MmwDemo_mssMmwaveEventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver     = 1U;
    initCfg.linkCRCCfg.crcChannel       = CRC_Channel_CH1;
    initCfg.cfgMode                     = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode               = MMWave_ExecutionMode_COOPERATIVE;
    initCfg.cooperativeModeCfg.cfgFxn   = MmwDemo_mssMmwaveConfigCallbackFxn;
    initCfg.cooperativeModeCfg.openFxn  = MmwDemo_mssMmwaveOpenCallbackFxn;
    initCfg.cooperativeModeCfg.closeFxn = MmwDemo_mssMmwaveCloseCallbackFxn;
    initCfg.cooperativeModeCfg.startFxn = MmwDemo_mssMmwaveStartCallbackFxn;
    initCfg.cooperativeModeCfg.stopFxn  = MmwDemo_mssMmwaveStopCallbackFxn;

    /* Initialize and setup the mmWave Control module */
    gMmwMssMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwMssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf("Error: VitalSignsDemoMSS mmWave Control Initialization failed [Error code %d]\n", errCode);
        return;
    }
    System_printf("Debug: VitalSignsDemoMSS mmWave Control Initialization was successful\n");

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = MMWave_sync (gMmwMssMCB.ctrlHandle , &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            System_printf ("Error: MMWDemoMSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }  

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = 6;
    taskParams.stackSize = 3*1024;
    Task_create(MmwDemo_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Create a data path management task to handle data Path events
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = 2;
    taskParams.stackSize = 3*1024;
    Task_create(MmwDemo_mssCtrlPathTask, &taskParams, NULL);
    
    /*****************************************************************************
     * At this point, MSS and DSS are both up and synced. Configuration is ready to be sent.
     * Start CLI to get configuration from user
     *****************************************************************************/
    MmwDemo_CLIInit();

    /*****************************************************************************
     * Benchmarking Count init
     *****************************************************************************/
    /* Configure banchmark counter */
    Pmu_configureCounter(0, 0x11, FALSE);
    Pmu_startCounter(0);
   
    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the R4F using WFI (Wait For Interrupt) instruction. 
 *     When R4F has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The R4F will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" WFI ");
}

/**
 *  @b Description
 *  @n
 *      Send MSS assert information through CLI.
 */
void _MmwDemo_mssAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression) {
        CLI_write ("MSS Exception: %s, line %d.\n",file,line);
    }
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Vital Signs  Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params     taskParams;
    int32_t         errCode;
    SOC_Cfg         socCfg;

    /* Initialize the ESM: */
    ESM_init(0U); //dont clear errors as TI RTOS does it

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwMssMCB, 0, sizeof(MmwDemo_MCB));

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gMmwMssMCB.socHandle   = SOC_init (&socCfg, &errCode);
    if (gMmwMssMCB.socHandle  == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the DEMO configuration: */
    gMmwMssMCB.cfg.sysClockFrequency = MSS_SYS_VCLK;
    gMmwMssMCB.cfg.loggingBaudRate   = 921600;
    gMmwMssMCB.cfg.commandBaudRate   = 115200;

    /* Debug Message: */
    System_printf ("**********************************************\n");
    System_printf ("Debug: Launching the Vital-Signs Monitoring Demo\n");
    System_printf ("**********************************************\n");

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.priority = 3;
    Task_create(MmwDemo_mssInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}
