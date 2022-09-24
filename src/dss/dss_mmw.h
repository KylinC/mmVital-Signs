/**
 *   @file  dss_mmw.h
 *
 *   @brief
 *      This is the main header file for the Vital Signs Demo on DSS
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
#ifndef DSS_MMW_H
#define DSS_MMW_H

/* MMWAVE Driver Include Files */
#include <ti/common/mmwave_error.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/cbuff/cbuff.h>

/* BIOS/XDC Include Files */
#include <ti/sysbios/knl/Semaphore.h>

/* MMWAVE library Include Files */
#include <ti/control/mmwave/mmwave.h>
#include "mmwDemo_monitor.h"

/* Vital Signs Demo Include Files */
#include "mmw_config.h"
#include "dss_data_path.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief   mmwave Config event triggered from mmwave config call back function */
#define MMWDEMO_CONFIG_EVT                                  Event_Id_00

/*! @brief   mmwave Start event triggered from mmwave start call back function */
#define MMWDEMO_START_EVT                                   Event_Id_01

/*! @brief   BSS stop complete event  */
#define MMWDEMO_BSS_STOP_COMPLETE_EVT                       Event_Id_02

/*! @brief   Frame start interupt triggered event */
#define MMWDEMO_FRAMESTART_EVT                              Event_Id_03

/*! @brief   Chirp available interrupt triggered event */
#define MMWDEMO_CHIRP_EVT                                   Event_Id_04

/*! @brief   Stop complete event  */
#define MMWDEMO_STOP_COMPLETE_EVT                           Event_Id_05

/*! @brief   BSS frame trigger ready event */
#define MMWDEMO_BSS_FRAME_TRIGGER_READY_EVT                 Event_Id_06

/*! @brief   Monitoring report event */
#define MMWDEMO_BSS_MONITORING_REP_EVT                      Event_Id_07

/*! @brief   BSS Calibration report event */
#define MMWDEMO_BSS_CALIBRATION_REP_EVT                     Event_Id_08


/**
 * @brief
 *  Millimeter Wave Demo state
 *
 * @details
 *  The enumeration is used to hold the data path states for the
 *  Millimeter Wave demo
 */
typedef enum MmwDemo_DSS_STATE_e
{
    /*! @brief   State after data path is initialized */
    MmwDemo_DSS_STATE_INIT = 0,

    /*! @brief   State after data path is started */
    MmwDemo_DSS_STATE_STARTED,

    /*! @brief   State after data path is stopped */
    MmwDemo_DSS_STATE_STOPPED,

    /*! @brief   State after STOP request was received by DSP
                 but complete stop is on-going */
    MmwDemo_DSS_STATE_STOP_PENDING

}MmwDemo_DSS_STATE;

/**
 *  @b Description
 *  @n
 *      Structure stores meta information for detected objects.
 *
 *
 *  @details
 *      This structure holds the meta information for detected Objects.
 */
typedef struct MmwDemo_detOutputHdr_t
{
    /*! brief   Output buffer magic word */
    uint16_t    magicWord[4];

    /*! brief   Inter frame processing cycles */
    uint32_t    interFrameProcCycles;

    /*! brief   Noise Engergy */
    uint32_t    noiseEnergy;

    /*! brief   Number of detected Objects */
    uint32_t     numDetectedObj;
}MmwDemo_detOutputHdr;

/**
 * @brief
 *  Millimeter Wave Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_DSS_STATS_t
{
    /*! @brief   Counter which tracks the number of config events
                 The config event is triggered in mmwave config callback function
                 when remote sends configuration */
    uint8_t      configEvt;

    /*! @brief   Counter which tracks the number of open events
                 The open event is triggered in mmwave open callback function
                 when remote calls mmwave_open() */
    uint8_t      openEvt;

    /*! @brief   Counter which tracks the number of close events
                 The close event is triggered in mmwave close callback function
                 when remote calls mmwave_close() */
    uint8_t      closeEvt;

    /*! @brief   Counter which tracks the number of start events
                 The start event is triggered in mmwave start callback function
                 when remote calls mmwave_start() */
    uint8_t      startEvt;

    /*! @brief   Counter which tracks the number of stop events
                 The start event is triggered in mmwave stop callback function
                 when remote calls mmwave_stop() */
    uint8_t      stopEvt;

    /*! @brief   Counter which tracks the number of chirp interrupt skipped due to stopped state of sensor */
    uint32_t     chirpIntSkipCounter;

    /*! @brief   Counter which tracks the number of chirp interrupt skipped due to stopped state of sensor */
    uint32_t     frameIntSkipCounter;

    /*! @brief   Counter which tracks the number of chirp interrupt detected */
    uint32_t     chirpIntCounter;

    /*! @brief   Counter which tracks the number of frame start interrupt  detected */
    uint32_t     frameStartIntCounter;

    /*! @brief   Counter which tracks the number of chirp event detected
                 The chirp event is triggered in the ISR for chirp interrupt */
    uint32_t     chirpEvt;

    /*! @brief   Counter which tracks the number of frame start event detected
                 The frame start event is triggered in the ISR for frame start interrupt */
    uint32_t     frameStartEvt;

    /*! @brief   Counter which tracks the number of frames triggered in BSS detected
                 The frame trigger event is triggered in the mmwave async event callback function */
    uint32_t     frameTrigEvt;

    /*! @brief   Counter which tracks the number of Failed Timing Reports received from BSS  */
    uint32_t     numFailedTimingReports;

    /*! @brief   Counter which tracks the number of Calibration Report received from BSS  */
    uint32_t     numCalibrationReports;

    /*! @brief   Counter which tracks the number of times saving detected objects in
                 logging buffer is skipped */
    uint32_t     detObjLoggingSkip;

    /*! @brief   Counter which tracks the number of times saving detected objects in
                 logging buffer has an error */
    uint32_t     detObjLoggingErr;

    /*! @brief   Counter which tracks the number of sensor stop Async events from BSS  */
    uint32_t     bssStopAsyncEvt;
}MmwDemo_DSS_STATS;

/**
 * @brief
 *  Millimeter Wave Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_DSS_MCB_t
{
    MmwDemo_Cfg                 cfg;

    /*! @brief CLI related configuration */
    MmwDemo_CliCfg_t           cliCfg[RL_MAX_SUBFRAMES];

    /*! @brief   CLI related configuration common across all subframes */
    MmwDemo_CliCommonCfg_t      cliCommonCfg;

    /*! @brief   Handle to the SOC Module */
    SOC_Handle                  socHandle;

    /*! @brief   UART Logging Handle */
    UART_Handle                 loggingUartHandle;

    /*! @brief   This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*! @brief   Data Path object */
    MmwDemo_DSS_DataPathObj     dataPathObj[RL_MAX_SUBFRAMES];

    /*! @brief Sub-frame index into the dataPathObj */
    uint8_t subFrameIndx;

    /*! @brief actual number of sub-frames based on configuration */
    uint8_t numSubFrames;

    /*! @brief Data Path Context */
    MmwDemo_DSS_dataPathContext_t   dataPathContext;

    /*!@brief   Handle to the peer Mailbox */
    Mbox_Handle              peerMailbox;

    /*! @brief   Semaphore handle for the mailbox communication */
    Semaphore_Handle            mboxSemHandle;

    /*! @brief   DSS event handle */
    Event_Handle                eventHandle;

    /*! @brief   Handle to the SOC chirp interrupt listener Handle */
    SOC_SysIntListenerHandle    chirpIntHandle;

    /*! @brief   Handle to the SOC frame start interrupt listener Handle */
    SOC_SysIntListenerHandle    frameStartIntHandle;

    /*! @brief   Logging buffer flag */
    uint8_t                     loggingBufferAvailable;

    /*! @brief   mmw Demo state */
    MmwDemo_DSS_STATE           state;

    /*! @brief   mmw Demo statistics */
    MmwDemo_DSS_STATS           stats;

    /*! @brief   this structure is used to hold all the relevant information
         for the mmw demo LVDS stream*/
    //MmwDemo_LVDSStream_MCB_t    lvdsStream;

} MmwDemo_DSS_MCB;

extern void _MmwDemo_dssAssert(int32_t expression, const char *file, int32_t line);
#define MmwDemo_dssAssert(expression) {                                      \
                                    _MmwDemo_dssAssert(expression,           \
                                             __FILE__, __LINE__);         \
                                    DebugP_assert(expression);            \
                                   }


#ifdef __cplusplus
}
#endif

/**
 *  @b Description
 *  @n
 *    utility function that returns floor(log2(x)), used for computing
 *    log2 of variables that are known to be exact powers of 2
 *  @retval
 *      Not Applicable.
 */
static inline uint32_t MmwDemo_floorLog2(uint32_t x)
{
    return(30 - _norm(x));
}
#endif /* DSS_MMW_H */

