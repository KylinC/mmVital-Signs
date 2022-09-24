/**
 *   @file  dss_config_edma_util.c
 *
 *   @brief
 *      EDMA Configuration Utility API implementation.
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

#include <xdc/runtime/System.h>
#include <ti/drivers/edma/edma.h>
#include "dss_config_edma_util.h"

#define MAX(x,y) ((x) > (y) ? (x) : (y))

static int32_t EDMA_setup_shadow_link (EDMA_Handle handle, uint8_t chId,
    uint16_t shadowParamId, EDMA_paramSetConfig_t *config,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
    uintptr_t transferCompletionCallbackFxnArg);

static int32_t EDMA_setup_shadow_link (EDMA_Handle handle, uint8_t chId,
    uint16_t shadowParamId, EDMA_paramSetConfig_t *config,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
    uintptr_t transferCompletionCallbackFxnArg)
{
    EDMA_paramConfig_t paramConfig;
    int32_t errorCode = EDMA_NO_ERROR;

    paramConfig.paramSetConfig = *config; //this will copy the entire param set config
    paramConfig.transferCompletionCallbackFxn = transferCompletionCallbackFxn;
    paramConfig.transferCompletionCallbackFxnArg = transferCompletionCallbackFxnArg;
    if ((errorCode = EDMA_configParamSet(handle,
                        shadowParamId, &paramConfig)) != EDMA_NO_ERROR)
    {
        System_printf("Error: EDMA_configParamSet() failed with error code = %d\n", errorCode);
        goto exit;
    }

    if ((errorCode = EDMA_linkParamSets(handle,
                        (uint16_t) chId, shadowParamId)) != EDMA_NO_ERROR)
    {
        System_printf("Error: EDMA_linkParamSets() failed with error code = %d\n", errorCode);
        goto exit;
    }

    if ((errorCode = EDMA_linkParamSets(handle,
                        shadowParamId, shadowParamId)) != EDMA_NO_ERROR)
    {
        System_printf("Error: EDMA_linkParamSets() failed with error code = %d\n", errorCode);
        goto exit;
    }

exit:
    return(errorCode);
}

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
    uintptr_t transferCompletionCallbackFxnArg)
{
    EDMA_channelConfig_t config;
    int32_t errorCode = EDMA_NO_ERROR;

    config.channelId = chId;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = chId;
    config.eventQueueId = eventQueueId;

    config.paramSetConfig.sourceAddress = (uint32_t) srcBuff;
    config.paramSetConfig.destinationAddress = (uint32_t) dstBuff;

    config.paramSetConfig.aCount = aCount;
    config.paramSetConfig.bCount = bCount;
    config.paramSetConfig.cCount = 1U;
    config.paramSetConfig.bCountReload = 0U;

    config.paramSetConfig.sourceBindex = srcBIdx;
    config.paramSetConfig.destinationBindex = dstBIdx;

    config.paramSetConfig.sourceCindex = 0U;
    config.paramSetConfig.destinationCindex = 0U;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;
    config.paramSetConfig.transferCompletionCode = chId;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;

    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled = true;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = true;
    config.paramSetConfig.isFinalChainingEnabled = false;
    config.paramSetConfig.isIntermediateChainingEnabled = false;
    config.transferCompletionCallbackFxn = transferCompletionCallbackFxn;
    config.transferCompletionCallbackFxnArg = transferCompletionCallbackFxnArg;

    if ((errorCode = EDMA_configChannel(handle, &config, isEventTriggered)) != EDMA_NO_ERROR)
    {
        System_printf("Error: EDMA_configChannel() failed with error code = %d\n", errorCode);
        goto exit;
    }

    errorCode = EDMA_setup_shadow_link(handle, chId, shadowParamId,
        &config.paramSetConfig, config.transferCompletionCallbackFxn, transferCompletionCallbackFxnArg);

exit:
    return(errorCode);
}

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
    uintptr_t transferCompletionCallbackFxnArg)
{
    EDMA_channelConfig_t config;
    int32_t errorCode = EDMA_NO_ERROR;

    config.channelId = chId;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = chId;
    config.eventQueueId = eventQueueId;

    config.paramSetConfig.sourceAddress = (uint32_t) srcBuff;
    config.paramSetConfig.destinationAddress = (uint32_t) dstBuff;

    config.paramSetConfig.aCount = sampleLenInBytes;
    config.paramSetConfig.bCount = numRangeBins;
    config.paramSetConfig.cCount = (uint16_t) numRxAnt;
    config.paramSetConfig.bCountReload = 0U;

    config.paramSetConfig.sourceBindex = (int16_t) sampleLenInBytes;
    config.paramSetConfig.destinationBindex = (int16_t)(numDopplerBins * numTxAnt * numRxAnt * sampleLenInBytes);

    config.paramSetConfig.sourceCindex = (int16_t)(numRangeBins * sampleLenInBytes);
    config.paramSetConfig.destinationCindex = (int16_t)(numDopplerBins * sampleLenInBytes);

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_AB;
    config.paramSetConfig.transferCompletionCode = chId;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;
    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled = true;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
    config.paramSetConfig.isFinalChainingEnabled = false;
    config.paramSetConfig.isIntermediateChainingEnabled = true;
    config.transferCompletionCallbackFxn = transferCompletionCallbackFxn;
    config.transferCompletionCallbackFxnArg = transferCompletionCallbackFxnArg;

    if ((errorCode = EDMA_configChannel(handle, &config, isEventTriggered)) != EDMA_NO_ERROR)
    {
        System_printf("Error: EDMA_configChannel() failed with error code = %d\n", errorCode);
        goto exit;
    }

    errorCode = EDMA_setup_shadow_link(handle, chId, shadowParamId,
        &config.paramSetConfig, config.transferCompletionCallbackFxn, transferCompletionCallbackFxnArg);

exit:
    return(errorCode);
}

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
    uintptr_t transferCompletionCallbackFxnArg)
{
    EDMA_channelConfig_t config;
    int32_t errorCode = EDMA_NO_ERROR;

    config.channelId = chId;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = chId;
    config.eventQueueId = eventQueueId;

    config.paramSetConfig.sourceAddress = (uint32_t) srcBuff;
    config.paramSetConfig.destinationAddress = (uint32_t) dstBuff;

    config.paramSetConfig.aCount = sampleLenInBytes;
    config.paramSetConfig.bCount = numDopplerBins;
    config.paramSetConfig.cCount = (uint16_t) MAX((numTxAnt * numRxAnt / 2),1);
    config.paramSetConfig.bCountReload = 0U;

    config.paramSetConfig.sourceBindex = (int16_t) (numRxAnt * numRangeBins * 
                                                    sampleLenInBytes);
    config.paramSetConfig.destinationBindex = (int16_t)sampleLenInBytes;

    if (numTxAnt == 2)
    {
        config.paramSetConfig.sourceCindex = (int16_t)(numRangeBins * sampleLenInBytes);
    }
    else
    {
        config.paramSetConfig.sourceCindex = (int16_t)(numRangeBins * sampleLenInBytes * 2);
    }

    config.paramSetConfig.destinationCindex = 0;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_AB;
    config.paramSetConfig.transferCompletionCode = chId;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;
    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled = true;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = true;
    config.paramSetConfig.isFinalChainingEnabled = false;
    config.paramSetConfig.isIntermediateChainingEnabled = false;
    config.transferCompletionCallbackFxn = transferCompletionCallbackFxn;
    config.transferCompletionCallbackFxnArg = transferCompletionCallbackFxnArg;

    if ((errorCode = EDMA_configChannel(handle, &config, isEventTriggered)) != EDMA_NO_ERROR)
    {
        System_printf("Error: EDMA_configChannel() failed with error code = %d\n", errorCode);
        goto exit;
    }

    errorCode = EDMA_setup_shadow_link(handle, chId, shadowParamId,
        &config.paramSetConfig, config.transferCompletionCallbackFxn, transferCompletionCallbackFxnArg);

exit:
    return(errorCode);
}


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
    uintptr_t transferCompletionCallbackFxnArg)
{
    EDMA_channelConfig_t config;
    int32_t errorCode = EDMA_NO_ERROR;

    config.channelId = chId;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = chId;
    config.eventQueueId = eventQueueId;

    config.paramSetConfig.sourceAddress = (uint32_t) srcBuff;
    config.paramSetConfig.destinationAddress = (uint32_t) dstBuff;

    config.paramSetConfig.aCount = aCount;
    config.paramSetConfig.bCount = bCount;
    config.paramSetConfig.cCount = 1U;
    config.paramSetConfig.bCountReload = 0U;

    config.paramSetConfig.sourceBindex = srcBIdx;
    config.paramSetConfig.destinationBindex = destBIdx;

    config.paramSetConfig.sourceCindex = 0U;
    config.paramSetConfig.destinationCindex = 0U;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_AB;
    config.paramSetConfig.transferCompletionCode = chId;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;

    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled = true;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
    config.paramSetConfig.isFinalChainingEnabled = false;
    config.paramSetConfig.isIntermediateChainingEnabled = false;
    config.transferCompletionCallbackFxn = transferCompletionCallbackFxn;
    config.transferCompletionCallbackFxnArg = transferCompletionCallbackFxnArg;

    if ((errorCode = EDMA_configChannel(handle, &config, isEventTriggered)) != EDMA_NO_ERROR)
    {
        System_printf("Error: EDMA_configChannel() failed with error code = %d\n", errorCode);
        goto exit;
    }

    errorCode = EDMA_setup_shadow_link(handle, chId, shadowParamId,
        &config.paramSetConfig, config.transferCompletionCallbackFxn, transferCompletionCallbackFxnArg);

exit:
    return(errorCode);
}

int32_t EDMAutil_triggerType3(EDMA_Handle handle,
    uint8_t *srcBuff,
    uint8_t *dstBuff,
    uint8_t chId,
    uint8_t triggerEnabled)
{
    int32_t errorCode = EDMA_NO_ERROR;

    if(srcBuff != NULL)
    {
        if ((errorCode = EDMA_setSourceAddress(handle, (uint16_t) chId,
                            (uint32_t) srcBuff)) != EDMA_NO_ERROR)
        {
            System_printf("Error: EDMA_setSourceAddress() failed with error code = %d\n", errorCode);
            goto exit;
        }
    }

    if(dstBuff != NULL)
    {
        if ((errorCode = EDMA_setDestinationAddress(handle, (uint16_t) chId,
                            (uint32_t) dstBuff)) != EDMA_NO_ERROR)
        {
            System_printf("Error: EDMA_setDestinationAddress() failed with error code = %d\n", errorCode);
            goto exit;
        }
    }

    if(triggerEnabled)
    {
        if ((errorCode = EDMA_startDmaTransfer(handle, chId)) != EDMA_NO_ERROR)
        {
            System_printf("Error: EDMA_startTransfer() failed with error code = %d\n", errorCode);
            goto exit;
        }
    }

exit:
    return(errorCode);
}
