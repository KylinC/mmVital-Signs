/**
 *   @file  dss_config_edma_util.h
 *
 *   @brief
 *      EDMA Configuration Utility API definitions.
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
#ifndef DSS_CONFIG_EDMA_UTIL_H
#define DSS_CONFIG_EDMA_UTIL_H

#include <ti/drivers/edma/edma.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 *  @b Description
 *  @n
 *     Sets up a EDMA channel for transferring a contiguous set of data
 *     (of length Acnt Bytes) from a source to a destination buffer.
 *     The EDMA can be configured for multiple transfers, so that subsequent
 *     initiations of the EDMA channel, initiate these subsequent transfers.
 *     The number of such multiple transfers is defined by Bcnt.
 *     For subsequent transfers the start pointer of the source buffer increments
 *     by srcBIdx bytes. The destination buffer remains the same across subsequent
 *     transfers. The EDMA is programmed as an "A-Synchronized transfer")
 *
 *     The ParamSet (parameter set) into which the EDMA is programmed is the same
 *     as the EDMA channel number. Additionally, the same parameters are programmed
 *     into a shadow ParamSet which is linked to the original ParamSet. Thus after all
 *     the Bnt transfers are complete, the ParamSet is automatically reinitialized
 *     and ready for another set of Bcnt transfers.
 *
 *     This routine is useful when data from external memory has to be brought
 *     sequentially in chunks into the DSP's internal buffer (L1/L2) for signal
 *     processing. The automatic reinitialization ensures that a one-time
 *     programming suffices (e.g.does not need to be re-programmed every frame)
 *
 *
 *  @param[in]  srcBuff        Pointer to source buffer
 *  @param[in]  dstBuff        Pointer to destination buffer
 *  @param[in]  chId           EDMA channel Id. (The ParamSet allocated to
 *                             this EDMA is also indexed by this EDMA channel number).
 *  @param[in]  isEventTriggered true if event triggered else (manual/chain triggered) false. 
 *  @param[in]  shadowParamId  EDMA channel number for "shadow channels" which
 *                             stores a replica of the orignal EDMA channel.
 *  @param[in]                 This enables reload of the EDMA channel registers
 *                             once an AB-synchronized transfer is complete.
 *  @param[in]  aCount         This specifies the Acnt of the transfer. (i.e number
 *                             of contiguous bytes per transfer).
 *  @param[in]  bCount         Number of EDMA transfers (each of length Acnt bytes).
 *  @param[in]  srcBIdx        Specifies the byte offset between the source pointer
 *                             of subsequent EDMA transfers.
 *  @param[in]  dstBIdx        Specifies the byte offset between the source pointer
 *                             of subsequent EDMA transfers.
 *  @param[in]  eventQueueId   Event Queue Id on which to schedule the transfer.
 *  @param[in]  transferCompletionCallbackFxn Transfer completion call back function.
 *  @param[in]  transferCompletionCallbackFxnArg Transfer completion call back function argument.
 *  @param[in]  isEventTriggered true if event triggered else (manual/chain triggered) false.
 *
 *  @retval
 *      EDMA driver error code, see "EDMA_ERROR_CODES" in EDMA API.
 *
 ******************************************************************************/
int32_t EDMAutil_configType1(EDMA_Handle handle,
    uint8_t *srcBuff,
    uint8_t *dstBuff,
    uint8_t chId,
    bool isEventTriggered,
    uint16_t shadowParamId,
    uint16_t aCount,
    uint16_t bCount,
    int16_t srcBIdx,
    int16_t dstBIdx,
    uint8_t eventQueueId,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
    uintptr_t transferCompletionCallbackFxnArg);

/******************************************************************************
 *  @b Description
 *  @n
 *     Sets up a EDMA channel for transferring a contiguous set of Bcnt samples
 *     from a source to a destination buffer. The data in the destination buffer
 *     is stored in a transposed format, with @ref destBIdx being the offset in bytes
 *     between subsequent complex samples. The EDMA can be configured for multiple
 *     such transfers, so that subsequent initiations of the EDMA channel, initiate
 *     these subsequent transfers. The number of such multiple transfers is
 *     defined by Ccnt. For subsequent transfers the start pointer of the
 *     destination buffer increments by one sample (sampleLenInBytes).
 *     The source buffer remains the same across subsequent transfers.
 *     Note that the EDMA is programmed as an "AB-Synchronized transfer".
 *
 *     The ParamSet (parameter set) into which the EDMA is programmed is the same
 *     as the EDMA channel number. Additionally, the same parameters are
 *     programmed into a shadow ParamSet which is linked to the original ParamSet.
 *     Thus after all the Ccnt transfers are complete, the ParamSet is automatically
 *     reinitialized and ready for another set of Ccnt transfers.
 *
 *     This routine is useful when processed data from the DSP  has to be stored
 *     in external memory in a transpose fashion. The automatic reinitialization
 *     ensures that a one-time programming suffices
 *    (e.g.does not need to be re-programmed every frame)
 *
 *
 *  @param[in]  srcBuff          Pointer to source buffer
 *  @param[in]  dstBuff          Pointer to destination buffer
 *  @param[in]  chId             EDMA channel Id  (The ParamSet allocated to this
 *                               EDMA is also indexed by this EDMA channel number).
 *  @param[in]  isEventTriggered true if event triggered else (manual/chain triggered) false.
 *  @param[in]  shadowParamId    EDMA shadow PaRAM Id which stores a replica of the
 *                               original EDMA channel.
 *  @param[in]                   This enables reload of the EDMA channel registers
 *                               once an AB-synchronized transfer is complete.
 *  @param[in]  sampleLenInBytes Number of bytes corresponding to one complex sample.
 *  @param[in]  numRangeBins     number of range bins.
 *  @param[in]  numTxAnt         number of Tx antenna.
 *  @param[in]  numRxAnt         number of Rx antenna.
 *  @param[in]  numDopplerBins   number of doppler bins.
 *  @param[in]  transferCompletionCallbackFxn Transfer completion call back function.
 *  @param[in]  transferCompletionCallbackFxnArg Transfer completion call back function argument.
 *
 *  @retval
 *      EDMA driver error code, see "EDMA_ERROR_CODES" in EDMA API.
 *
 ******************************************************************************/
int32_t EDMAutil_configType2a(EDMA_Handle handle,
    uint8_t *srcBuff,
    uint8_t *dstBuff,
    uint8_t chId,
    bool isEventTriggered,
    uint16_t shadowParamId,
    uint16_t sampleLenInBytes,
    uint16_t numRangeBins,
    uint8_t numTxAnt,
    uint8_t numRxAnt,
    uint16_t numDopplerBins,
    uint8_t eventQueueId,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
    uintptr_t transferCompletionCallbackFxnArg);

/******************************************************************************
 *  @b Description
 *  @n
 *     Sets up a EDMA channel for transferring a set of Bcnt samples
 *     from a source to a contiguous destination buffer. The data in the source buffer
 *     needs to be transposed when transferring to the destination buffer. The
 *     @ref srcBIdx is the offset in bytes
 *     between subsequent complex samples in the source buffer. 
 *     The EDMA can be configured for multiple
 *     such transfers, so that subsequent initiations of the EDMA channel, initiate
 *     these subsequent transfers. The number of such multiple transfers is
 *     defined by cCount. For subsequent transfers the start pointer of the
 *     source buffer increments by twice (ping-pong) the samples in a range
 *     for non TDM case (numTxAnt == 1) or by the samples in a range for the TDM case
 *     (numTxAnt = 2). 
 *
 *     The destination buffer remains the same across subsequent transfers.
 *     Note that the EDMA is programmed as an "AB-Synchronized transfer".
 *
 *     The ParamSet (parameter set) into which the EDMA is programmed is the same
 *     as the EDMA channel number. Additionally, the same parameters are
 *     programmed into a shadow ParamSet which is linked to the original ParamSet.
 *     Thus after all the Ccnt transfers are complete, the ParamSet is automatically
 *     reinitialized and ready for another set of Ccnt transfers.
 *
 *     This routine is useful in the first stage of inter frame processing 
 *     to read data of 1D output into contiguous doppler samples 
 *     for 2D FFT processing. This routine is applicable when 1D output is stored
 *     in a non-transposed manner so the 2D processing needs to perform a transpose
 *     before doing 2D FFT.
 *     The automatic reinitialization ensures that a one-time programming suffices
 *    (e.g.does not need to be re-programmed every frame)
 *
 *
 *  @param[in]  srcBuff          Pointer to source buffer
 *  @param[in]  dstBuff          Pointer to destination buffer
 *  @param[in]  chId             EDMA channel Id  (The ParamSet allocated to this
 *                               EDMA is also indexed by this EDMA channel number).
 *  @param[in]  isEventTriggered true if event triggered else (manual/chain triggered) false.
 *  @param[in]  shadowParamId    EDMA shadow PaRAM Id which stores a replica of the
 *                               original EDMA channel.
 *  @param[in]                   This enables reload of the EDMA channel registers
 *                               once an AB-synchronized transfer is complete.
 *  @param[in]  sampleLenInBytes Number of bytes corresponding to one complex sample.
 *  @param[in]  numRangeBins     number of range bins.
 *  @param[in]  numTxAnt         number of Tx antenna.
 *  @param[in]  numRxAnt         number of Rx antenna.
 *  @param[in]  numDopplerBins   number of doppler bins.
 *  @param[in]  transferCompletionCallbackFxn Transfer completion call back function.
 *  @param[in]  transferCompletionCallbackFxnArg Transfer completion call back function argument.
 *
 *  @retval
 *      EDMA driver error code, see "EDMA_ERROR_CODES" in EDMA API.
 *
 ******************************************************************************/    
int32_t EDMAutil_configType2b(EDMA_Handle handle,
    uint8_t *srcBuff,
    uint8_t *dstBuff,
    uint8_t chId,
    bool isEventTriggered,
    uint16_t shadowParamId,
    uint16_t sampleLenInBytes,
    uint16_t numRangeBins,
    uint8_t numTxAnt,
    uint8_t numRxAnt,
    uint16_t numDopplerBins,
    uint8_t eventQueueId,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
    uintptr_t transferCompletionCallbackFxnArg);

/******************************************************************************
 *  @b Description
 *  @n
 *    Sets up a EDMA channel for performing a 2-Dimensional EDMA transfer
 *    (with Acnt and Bcnt defined both both source and destination).
 *    The ParamaSet is automatically reinitialized (by linking to a shadow ParamSet)
 *    when the transfer is complete.
 *
 *  @param[in]  srcBuff        Pointer to source buffer.
 *  @param[in]  dstBuff        Pointer to destination buffer.
 *  @param[in]  chId           EDMA channel Id (The ParamSet allocated to this
 *                             EDMA is also indexed by this EDMA channel number).
 *  @param[in]  isEventTriggered true if event triggered else (manual/chain triggered) false.
 *  @param[in]  shadowParamId  EDMA shadow PaRAM Id which stores a replica of
 *                             the original EDMA channel. This enables reload of
 *                             the EDMA channel registers once an AB-synchronized
 *                             transfer is complete.
 *  @param[in]  aCount         This specifies the number of contiguous bytes
 *                             (from the soruce) per transfer. (BCNT of the transfer)
 *  @param[in]  bCount         Number of EDMA transfers (each of length Acnt bytes).
 *  @param[in]  srcBIdx        Specifies the byte offset between the source pointer
 *                             of subsequent EDMA transfers.
 *  @param[in]  dstBIdx        Specifies the byte offset between the source pointer
 *                             of subsequent EDMA transfers.
 *  @param[in]  eventQueueId   Event Queue Id on which to schedule the transfer.
 *  @param[in]  transferCompletionCallbackFxn Transfer completion call back function.
 *  @param[in]  transferCompletionCallbackFxnArg Transfer completion call back function argument.
 *
 *  @retval
 *      EDMA driver error code, see "EDMA_ERROR_CODES" in EDMA API.
 *
 *******************************************************************************/
int32_t EDMAutil_configType3(EDMA_Handle handle,
    uint8_t *srcBuff,
    uint8_t *dstBuff,
    uint8_t chId,    
    bool isEventTriggered,
    uint16_t shadowParamId,
    uint16_t aCount,
    uint16_t bCount,
    int16_t srcBIdx,
    int16_t destBIdx,
    uint8_t eventQueueId,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
    uintptr_t transferCompletionCallbackFxnArg);

/******************************************************************************
 *  @b Description
 *  @n
 *    Reconfigures source and destination addresses of a given channel Id
 *    (whose param Id is assumed to be already configured to be same as channel Id)
 *    and then starts a transfer on that channel.
 *
 *  @param[in]  srcBuff        Pointer to source buffer. If NULL, does not update.
 *  @param[in]  dstBuff        Pointer to destination buffer. If NULL, does not update.
 *  @param[in]  chId           EDMA channel Id.
 *  @param[in]  triggerEnabled =1: trigger EDMA, =0: does not trigger EDMA
 *
 *  @retval
 *      EDMA driver error code, see "EDMA_ERROR_CODES" in EDMA API.
 *
 *******************************************************************************/
int32_t EDMAutil_triggerType3(EDMA_Handle handle,
    uint8_t *srcBuff,
    uint8_t *dstBuff,
    uint8_t chId,
    uint8_t triggerEnabled);

#ifdef __cplusplus
}
#endif
   
#endif
