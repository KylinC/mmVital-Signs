/**
 *   @file  mmwDemo_monitor.h
 *
 *   @brief
 *      This is used to abstract the mmWave demo monitor definitions.
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

#ifndef MMWAVEDEMO_MONITOR_H
#define MMWAVEDEMO_MONITOR_H

/* mmWave SDK Include Files: */
#include "mmw_config.h"
#include <ti/control/mmwavelink/mmwavelink.h>


#ifdef __cplusplus
extern "C" {
#endif


/***********************************************************************************************
 * Analog Monitor Defines
 ***********************************************************************************************/
 
/**
 * @brief
 *  Mmwave link analog monitor definition
 *
 * @details
 *  Enumeration describes the mmwave supported analog monitors.
 */
typedef enum mmwDemo_AnaMonitor_e
{
    MMWDEMO_ANALOG_MONITOR_TEMPERATURE = 0U,
    MMWDEMO_ANALOG_MONITOR_RX_GAIN_PHASE,
    MMWDEMO_ANALOG_MONITOR_RX_NOISE_FIGURE,
    MMWDEMO_ANALOG_MONITOR_IFSTAGE,
    MMWDEMO_ANALOG_MONITOR_TX0_POWER,
    MMWDEMO_ANALOG_MONITOR_TX1_POWER,
    MMWDEMO_ANALOG_MONITOR_TX2_POWER,
    MMWDEMO_ANALOG_MONITOR_TX0_BALLBREAK,
    MMWDEMO_ANALOG_MONITOR_TX1_BALLBREAK,
    MMWDEMO_ANALOG_MONITOR_TX2_BALLBREAK,
    MMWDEMO_ANALOG_MONITOR_TX_GAIN_PHASE_MISMATCH,
    MMWDEMO_ANALOG_MONITOR_TX0_BPM,
    MMWDEMO_ANALOG_MONITOR_TX1_BPM,
    MMWDEMO_ANALOG_MONITOR_TX2_BPM,
    MMWDEMO_ANALOG_MONITOR_SYNTH_FREQ,
    MMWDEMO_ANALOG_MONITOR_EXT_ANALOG_SIGNALS,
    MMWDEMO_ANALOG_MONITOR_INTERNAL_TX0_SIGNALS,
    MMWDEMO_ANALOG_MONITOR_INTERNAL_TX1_SIGNALS,
    MMWDEMO_ANALOG_MONITOR_INTERNAL_TX2_SIGNALS,
    MMWDEMO_ANALOG_MONITOR_INTERNAL_RX_SIGNALS,
    MMWDEMO_ANALOG_MONITOR_INTERNAL_PMCLKLO_SIGNALS,
    MMWDEMO_ANALOG_MONITOR_INTERNAL_GPADC_SIGNALS,
    MMWDEMO_ANALOG_MONITOR_PLL_CONTROL_VOLTAGE,
    MMWDEMO_ANALOG_MONITOR_DCC_CLOCK_FREQ,
    MMWDEMO_ANALOG_MONITOR_RX_SATURATION_DETECTOR,
    MMWDEMO_ANALOG_MONITOR_RX_SIG_IMG_BAND,
    MMWDEMO_ANALOG_MONITOR_RX_MIXER_INPUT_POWER,
    MMWDEMO_ANALOG_MONITOR_MAX
}mmwDemo_AnaMonitor;

/***********************************************************************************************
 * Monitor API: Available in FULL configuration mode
 ***********************************************************************************************/
extern int32_t mmwDemo_cfgAnalogMonitor(MmwDemo_AnaMonitorCfg         *ptrAnaMonCfg);
extern int32_t mmwDemo_cfgRxSigImgMonitor(    rlSigImgMonConf_t*  ptrSigImgMonCfg);
extern int32_t mmwDemo_cfgRxSaturationMonitor(    rlRxSatMonConf_t*   ptrRxSatMonCfg);

#ifdef __cplusplus
}
#endif

#endif /* MMWAVEDEMO_MONITOR_H */


