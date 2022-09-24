/**
 *   @file  mss_mmw.h
 *
 *   @brief
 *      This is the main header file for the VitalSigns Monitoring Demo
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

#ifndef MSS_MMW_DEMO_H
#define MSS_MMW_DEMO_H

#include <ti/common/mmwave_error.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>

#include <ti/sysbios/knl/Semaphore.h>

/* VitalSigns Demo Include Files */
#include "mmw_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief   sensor start CLI event */
#define MMWDEMO_CLI_SENSORSTART_EVT                     Event_Id_00

/*! @brief   sensor stop CLI  event */
#define MMWDEMO_CLI_SENSORSTOP_EVT                      Event_Id_01

/*! @brief   sensor frame start CLI  event */
#define MMWDEMO_CLI_FRAMESTART_EVT                      Event_Id_02

/*! @brief   BSS CPUFAULT event */
#define MMWDEMO_BSS_CPUFAULT_EVT                        Event_Id_03

/*! @brief   BSS ESMFAULT event */
#define MMWDEMO_BSS_ESMFAULT_EVT                        Event_Id_04

/*! @brief   Monitoring report event */
#define MMWDEMO_BSS_MONITORING_REP_EVT                  Event_Id_05

/*! @brief   BSS Calibration report event */
#define MMWDEMO_BSS_CALIBRATION_REP_EVT                 Event_Id_06

/*! @brief   start completed event from DSS/MSS */
#define MMWDEMO_DSS_START_COMPLETED_EVT                 Event_Id_07

/*! @brief   stop completed event from DSS */
#define MMWDEMO_DSS_STOP_COMPLETED_EVT                  Event_Id_08

/*! @brief   start failed event from DSS/MSS */
#define MMWDEMO_DSS_START_FAILED_EVT                    Event_Id_09

/*! @brief  DSS chirp processing deadline miss event */
#define MMWDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT        Event_Id_10

/*! @brief  DSS frame processing deadline miss event */
#define MMWDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT        Event_Id_11


/* All CLI events */
#define MMWDEMO_CLI_EVENTS                              (MMWDEMO_CLI_SENSORSTART_EVT |    \
                                                         MMWDEMO_CLI_SENSORSTOP_EVT |     \
                                                         MMWDEMO_CLI_FRAMESTART_EVT)

/* All BSS faults events */
#define MMWDEMO_BSS_FAULT_EVENTS                        (MMWDEMO_BSS_CPUFAULT_EVT |     \
                                                         MMWDEMO_BSS_ESMFAULT_EVT )
                                                         
/* All DSS exception events signalled directly through DSS2MSS SW1 ISR, these are 
   exception events that happen in the ISRs of DSS such as missing processing deadlines
   for chirp or frame for which DSS cannot use the mailbox path to communicate with MSS
   because of the urgent nature of the events. DSP will do a local assert in its
   ISR after signalling the exception to the MSS. */
#define MMWDEMO_DSS_EXCEPTION_EVENTS            (MMWDEMO_DSS_CHIRP_PROC_DEADLINE_MISS_EVT |   \
                                                 MMWDEMO_DSS_FRAME_PROC_DEADLINE_MISS_EVT )

/**
 * @brief
 *  Vital-Signs Monitoring Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  demo
 */
typedef struct MmwDemo_MSS_STATS_t
{
    /*! @brief   CLI event for sensorStart */
    uint8_t      cliSensorStartEvt;

    /*! @brief   CLI event for sensorStop */
    uint8_t      cliSensorStopEvt;

    /*! @brief   CLI event for frameStart */
    uint8_t      cliFrameStartEvt;

    /*! @brief   Counter which tracks the number of datapath config event detected
     *           The event is triggered in mmwave config callback function */
    uint8_t      datapathConfigEvt;

    /*! @brief   Counter which tracks the number of datapath start event  detected 
     *           The event is triggered in mmwave start callback function */
    uint8_t      datapathStartEvt;

    /*! @brief   Counter which tracks the number of datapath stop event detected 
     *           The event is triggered in mmwave stop callback function */
    uint8_t      datapathStopEvt;

    /*! @brief   Counter which tracks the number of failed calibration reports
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     numFailedTimingReports;

    /*! @brief   Counter which tracks the number of calibration reports received
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     numCalibrationReports;
}MmwDemo_MSS_STATS;








/**
 * @brief
 *  Vital Signs Monitoring Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  demo
 */
typedef struct MmwDemo_MCB_t
{
    /*! @brief   Configuration which is used to execute the demo */
    MmwDemo_Cfg                 cfg;

    /*! @brief   CLI related configuration */    
    MmwDemo_CliCfg_t            cliCfg[RL_MAX_SUBFRAMES];

    /*! @brief   CLI related configuration common across all subframes */
    MmwDemo_CliCommonCfg_t      cliCommonCfg;
 
    /*! * @brief   Handle to the SOC Module */
    SOC_Handle                  socHandle;

    /*! @brief   UART Logging Handle */
    UART_Handle                 loggingUartHandle;

    /*! @brief   UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;

    /*! @brief   This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*!@brief   Handle to the peer Mailbox */
    Mbox_Handle              peerMailbox;

    /*! @brief   Semaphore handle for the mailbox communication */
    Semaphore_Handle            mboxSemHandle;

    /*! @brief   MSS system event handle */
    Event_Handle                eventHandle;

    /*! @brief   MSS system event handle */
    Event_Handle                eventHandleNotify;

    /*! @brief   Handle to the SOC chirp interrupt listener Handle */
    SOC_SysIntListenerHandle    chirpIntHandle;

    /*! @brief   Handle to the SOC frame start interrupt listener Handle */
    SOC_SysIntListenerHandle    frameStartIntHandle;

    /*! @brief   Current status of the sensor */
    bool                         isSensorStarted;

    /*! @brief   Has the mmWave module been opened? */
    bool                        isMMWaveOpen;

    /*! @brief   mmw Demo stats */
    MmwDemo_MSS_STATS           stats;
    
    /*! @brief DSS to MSS Isr Info Address */
    uint32_t                   dss2mssIsrInfoAddress;


} MmwDemo_MCB;





/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern int32_t MmwDemo_mssDataPathConfig(void);
extern int32_t MmwDemo_mssDataPathStart(void);
extern int32_t MmwDemo_mssDataPathStop(void);

/* Sensor Management Module Exported API */
extern void MmwDemo_notifySensorStart(bool doReconfig);
extern void MmwDemo_notifySensorStop(void);
extern int32_t MmwDemo_waitSensorStartComplete(void);
extern void MmwDemo_waitSensorStopComplete(void);

extern void _MmwDemo_mssAssert(int32_t expression, const char *file, int32_t line);
#define MmwDemo_mssAssert(expression) {                                      \
                                         _MmwDemo_mssAssert(expression,      \
                                                  __FILE__, __LINE__);         \
                                         DebugP_assert(expression);             \
                                        }


#ifdef __cplusplus
}
#endif

#endif /* MSS_MMW_DEMO_H */

