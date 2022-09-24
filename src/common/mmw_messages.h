/**
 *   @file  mmw_messages.h
 *
 *   @brief
 *      This is the main header file for the Vital Signs Demo
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
#ifndef MMW_MESSAGES_H
#define MMW_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mmw_output.h"
#include "mmw_config.h"


/**
 * @brief
 *  Message types used in Millimeter Wave Demo for Mailbox communication
 * between MSS and DSS.
 *
 * @details
 *  The enum is used to hold all the messages types used for Mailbox communication
 * between MSS and DSS in mmw Demo.
 */
typedef enum MmwDemo_message_type_e
{
    /*! @brief   message types for MSS to DSS communication */
    MMWDEMO_MSS2DSS_GUIMON_CFG = 0xFEED0001,
    MMWDEMO_MSS2DSS_CFAR_RANGE_CFG,
    MMWDEMO_MSS2DSS_CFAR_DOPPLER_CFG,
    MMWDEMO_MSS2DSS_PEAK_GROUPING_CFG,
    MMWDEMO_MSS2DSS_MULTI_OBJ_BEAM_FORM,
    MMWDEMO_MSS2DSS_CALIB_DC_RANGE_SIG,
    MMWDEMO_MSS2DSS_DETOBJ_SHIPPED,
    MMWDEMO_MSS2DSS_SET_DATALOGGER,
    MMWDEMO_MSS2DSS_ADCBUFCFG,
    MMWDEMO_MSS2DSS_EXTENDED_MAX_VELOCITY,
    MMWDEMO_MSS2DSS_CLUTTER_REMOVAL,
    MMWDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE,
    MMWDEMO_MSS2DSS_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE,
    MMWDEMO_MSS2DSS_BPM_CFG,
    MMWDEMO_MSS2DSS_NEAR_FIELD_CFG,
    MMWDEMO_MSS2DSS_CQ_SATURATION_MONITOR,
    MMWDEMO_MSS2DSS_LVDSSTREAM_CFG,
    MMWDEMO_MSS2DSS_CQ_SIGIMG_MONITOR,
    MMWDEMO_MSS2DSS_ANALOG_MONITOR,
    MMWDEMO_MSS2DSS_VITALSIGNS_MEASUREMENT_PARAMS,
    MMWDEMO_MSS2DSS_VITALSIGNS_MOTION_DETECTION,
    MMWDEMO_VITALSIGNS_GUIMON_CFG,

    /*! @brief   message types for DSS to MSS communication */
    MMWDEMO_DSS2MSS_CONFIGDONE = 0xFEED0100,
    MMWDEMO_DSS2MSS_DETOBJ_READY,
    MMWDEMO_DSS2MSS_STOPDONE,
    MMWDEMO_DSS2MSS_ASSERT_INFO,
    MMWDEMO_DSS2MSS_ISR_INFO_ADDRESS,
    MMWDEMO_DSS2MSS_MEASUREMENT_INFO

}MmwDemo_message_type;


/*! @brief Software interrupt number used by DSS to signal exception from DSS to MSS */
#define MMWDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS    1
/*! @brief Software interrupt ID on MSS corresponding to
           @ref MMWDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_DSS */
#define MMWDEMO_DSS2MSS_EXCEPTION_SIGNALLING_SW_INT_MSS    SOC_XWR68XX_MSS_DSS2MSS_SW1_INT

/** @defgroup DSS_TO_MSS_EXCEPTION_IDS DSS to MSS Exception IDs
 *
 * @brief
 *  Exception ID definitions for DSS to MSS urgent exception signalling through
 *  software interrupt.
 *
 @{ */

 /*! @brief   DSS to MSS chirp processing deadline miss exception ID */
#define MMWDEMO_DSS2MSS_CHIRP_PROC_DEADLINE_MISS_EXCEPTION 0

/*! @brief   DSS to MSS frame processing deadline miss exception ID */
#define MMWDEMO_DSS2MSS_FRAME_PROC_DEADLINE_MISS_EXCEPTION 1

/** @}*/ /* end defgroup DSS_TO_MSS_EXCEPTION_IDS */

/**
 * @brief
 *  TLV part of the message from DSS to MSS on data path detection information.
 *
 * @details
 *  The structure describes the message item
 */
typedef struct MmwDemo_msgTlv_t
{
    /*! @brief   Payload type */
    uint32_t    type;

    /*! @brief   Length in bytes */
    uint32_t    length;

    /*! @brief Address of the payload */
    uint32_t   address;
} MmwDemo_msgTlv;

/**
 * @brief
 *  Message for reporting detection information from data path to MSS
 *
 * @details
 *  The structure defines the message body for detection information from from data path to MSS.
 */
typedef struct MmwDemo_detObjMsg_t
{
    /*! @brief Header of the detection information message */
    MmwDemo_output_message_header header;

    /*! @brief TLVs of the detection information */
    MmwDemo_msgTlv   tlv[MMWDEMO_OUTPUT_MSG_MAX];
} MmwDemo_detInfoMsg;

#define MMWDEMO_MAX_FILE_NAME_SIZE 128
/**
 * @brief
 *  Message for reporting DSS assertion information
 *
 * @details
 *  The structure defines the message body for the information
 *  on a DSS exception that should be forwarded to the MSS.
 */
typedef struct MmwDemo_dssAssertInfoMsg_t
{
    /*! @brief file name */
    char     file[MMWDEMO_MAX_FILE_NAME_SIZE];

    /*! @brief line number */
    uint32_t line;
} MmwDemo_dssAssertInfoMsg;

/**
 * @brief
 *  Message body used in Millimeter Wave Demo for passing configuration from MSS
 * to DSS.
 *
 * @details
 *  The union defines the message body for various configuration messages.
 */
typedef union MmwDemo_message_body_u
{
    /*! @brief   Gui Monitor Selection */
    //MmwDemo_GuiMonSel     guiMonSel;

    /*! @brief   CFAR Range/Doppler configuraiton */
    //MmwDemo_CfarCfg       cfarCfg;

    /*! @brief   Peak grouping configuration */
    //MmwDemo_PeakGroupingCfg peakGroupingCfg;

    /*! @brief   Multi object beam forming configuration */
    //MmwDemo_MultiObjBeamFormingCfg multiObjBeamFormingCfg;

    /*! @brief   Calibrate DC (zero) range signature */
    MmwDemo_CalibDcRangeSigCfg calibDcRangeSigCfg;

    /*! @brief   Extended maximum velocity configuration */
    //MmwDemo_ExtendedMaxVelocityCfg extendedMaxVelocityCfg;

    /*! @brief   Near field correction configuration */
    //MmwDemo_NearFieldCorrectionCfg nearFieldCorrectionCfg;

    /*! @brief   Clutter removal configuration */
    MmwDemo_ClutterRemovalCfg clutterRemovalCfg;

    /*! @brief   Detection Information message */
    MmwDemo_detInfoMsg          detObj;

    /*! @brief   ADCBUF configuration */
    MmwDemo_ADCBufCfg           adcBufCfg;

    /*! @brief   ChirpQuality - RX saturation monitor */
    //rlRxSatMonConf_t            cqSatMonCfg;

    /*! @brief   ChirpQuality - Signal and image band energy monitor */
    //rlSigImgMonConf_t           cqSigImgMonCfg;

    /*! @brief   ChirpQuality - Signal and image band energy monitor */
    //MmwDemo_AnaMonitorCfg       anaMonCfg;

    /*! @brief   Vital Signs configuration */
    VitalSignsDemo_ParamsCfg  vitalSignsParamsCfg;

    VitalSignsDemo_MotionDetection   motionDetectionParamsCfg;

    VitalSignsDemo_GuiMonSel         vitalSigns_GuiMonSel;

    /*! @brief   Datapath output logger setting */
    uint8_t               dataLogger;

    /*! @brief   Configuration for compensation for range bias
                 and rx channel phase offset */
    MmwDemo_compRxChannelBiasCfg_t compRxChanCfg;

    /*! @brief   Configuration for measurement of range bias
                 and rx channel phase offset */
    //MmwDemo_measureRxChannelBiasCfg_t measureRxChanCfg;

    /*! @brief   BPM cfg */
    //MmwDemo_BpmCfg bpmCfg;

    /*! @brief   DSS assertion information */
    MmwDemo_dssAssertInfoMsg  assertInfo;

    /*! @brief Address of DSS to MSS ISR information storage, typically in HSRAM */
    uint32_t  dss2mssISRinfoAddress;

    /*! @brief  LVDS stream configuration */
    MmwDemo_LvdsStreamCfg lvdsStreamCfg;
} MmwDemo_message_body;

/*! @brief For advanced frame config, below define means the configuration given is
 * global at frame level and therefore it is broadcast to all sub-frames.
 */
#define MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG (-1)

/**
 * @brief
 *  DSS/MSS communication messages
 *
 * @details
 *  The structure defines the message structure used to commuincate between MSS
 * and DSS.
 */
typedef struct MmwDemo_message_t
{
    /*! @brief   message type */
    MmwDemo_message_type      type;

    /*! @brief   message length : PROC_TBA does body need to be pointer and not a union structure?*/
    //uint32_t                  len;

    /*! @brief   Subframe number for which this message is applicable.
                 Valid only when advanced frame config is used.
                 When advanced frame config is not used, this field should
                 be set to MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG.
    */
    int8_t                   subFrameNum;

    /*! @brief  message body */
    MmwDemo_message_body      body;

} MmwDemo_message;

#ifdef __cplusplus
}
#endif

#endif /* MMW_MESSAGES_H */
