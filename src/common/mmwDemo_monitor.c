/**
 *   @file  mmwDemo_monitor.c
 *
 *   @brief
 *      The file implements the functions which are required to support
 *      monitoring functions.
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
#include <ti/common/sys_common.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include "mmwDemo_monitor.h"

/**************************************************************************
 **************************** Local Functions *****************************
 **************************************************************************/


/**************************************************************************
 **************************** Monitor Functions *****************************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      The function is the exported API which is used to configure the mmWave analog monitor.
 *
 *  @param[in]  ptrAnaMonCfg
 *      Pointer to Analog Monitor configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t mmwDemo_cfgAnalogMonitor(MmwDemo_AnaMonitorCfg         *ptrAnaMonCfg)
{
    rlMonAnaEnables_t   analogMonCfg;
    int32_t             retVal = 0;

    if (ptrAnaMonCfg == NULL) 
    {
        /* Error: Invalid argument. */
        retVal = MINUS_ONE;
    }
    else
    {
        /* Set config structure to all zeros */
        memset((void *)&analogMonCfg, 0, sizeof(rlMonAnaEnables_t));

        /* Set/Reset Rx saturation monitor bit */
        if(ptrAnaMonCfg->rxSatMonEn)
        {
            analogMonCfg.enMask  |= 0x1 << MMWDEMO_ANALOG_MONITOR_RX_SATURATION_DETECTOR;
        }

        /* Set/Reset Rx signal image band monitor bit */
        if(ptrAnaMonCfg->sigImgMonEn)
        {
            analogMonCfg.enMask  |= 0x1 << MMWDEMO_ANALOG_MONITOR_RX_SIG_IMG_BAND;
        }
        
        /* Send analog monitor config to sensor */
        retVal = rlRfAnaMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, &analogMonCfg);
    }
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is the exported API which is used to configure the mmWave Rx Signal and
 *      image band energy monitor.
 *
 *  @param[in]  ptrSigImgMonCfg
 *      Pointer to the Signal & image band monitor configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t mmwDemo_cfgRxSigImgMonitor
(
    rlSigImgMonConf_t*  ptrSigImgMonCfg
)
{
    int32_t             retVal = 0;

    /* Get the pointer to the control module */
    if ( (ptrSigImgMonCfg == NULL) || 
       (ptrSigImgMonCfg->numSlices < 1U) ||
       (ptrSigImgMonCfg->numSlices > RL_NUM_MON_SLICES_MAX) ||
       (ptrSigImgMonCfg->timeSliceNumSamples < 4U))
    {
        /* Error: Invalid argument. */
        retVal = MINUS_ONE;
        goto exit;
    }

    retVal = rlRfRxSigImgMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, ptrSigImgMonCfg);
    if(retVal != 0)
    {
        /* Error: . */
        //System_printf ("Error: rlRfRxSigImgMonConfig returns error = %d\n", retVal);
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is the exported API which is used to configure the mmWave Rx
 *      Saturation Monitor.
 *
 *  @param[in]  ptrRxSatMonCfg
 *      Pointer to the Rx Saturation monitor configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t mmwDemo_cfgRxSaturationMonitor
(
    rlRxSatMonConf_t*   ptrRxSatMonCfg
)
{
    int32_t             retVal;

    /* Get the pointer to the control module */
    if ( ptrRxSatMonCfg == NULL )
    {
        /* Error: Invalid argument. */
        retVal = MINUS_ONE;
        goto exit;
    }

    /* Validate parameters */
    if( (ptrRxSatMonCfg->numSlices < 1U) ||
       (ptrRxSatMonCfg->numSlices > RL_NUM_MON_SLICES_MAX) ||
       (ptrRxSatMonCfg->satMonSel < 1U) ||
       (ptrRxSatMonCfg->satMonSel > 3U) ||
       (ptrRxSatMonCfg->primarySliceDuration < 4U) )
    {
        /* Error: Invalid argument. */
        retVal = MINUS_ONE;
        goto exit;
    }

    retVal = rlRfRxIfSatMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, ptrRxSatMonCfg);
    if(retVal != 0)
    {
        //System_printf ("Error: rlRfRxIfSatMonConfig returns error = %d\n", retVal);
    }

exit:
    return retVal;
}
