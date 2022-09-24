/**
 *   @file  vitalSignsDemo_utilsFunc.h
 *
 *   @brief
 *      Routines for several functions used in Vital Signs Demo
 *
 *  \par

 *  NOTE:
 * Copyright (c) 2018 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive license
 * under copyrights and patents it now or hereafter owns or controls to make, have made, use,
 * import, offer to sell and sell ("Utilize") this software subject to the terms herein.
 *
 * With respect to the foregoing patent license, such license is granted  solely to the extent
 * that any such patent is necessary to Utilize the software alone.  The patent license shall
 * not apply to any combinations which include this software, other than combinations with
 * devices manufactured by or for TI (“TI Devices”). No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source code
 * license limitations below) in the documentation and/or other materials provided with the
 * distribution.
 *
 * Redistribution and use in binary form, without modification, are permitted provided that
 * the following conditions are met:
 *
 * No reverse engineering, decompilation, or disassembly of this software is permitted with
 * respect to any software provided in binary form. Any redistribution and use are licensed
 * by TI for use only with TI Devices. Nothing shall obligate TI to provide you with source
 * code for the software licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the source
 * code are permitted provided that the following conditions are met:
 *
 * Any redistribution and use of the source code, including any resulting derivative works,
 * are licensed by TI for use only with TI Devices.
 * Any redistribution and use of any object code compiled from the source code and any
 * resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be
 * used to endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S
 * LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VITALSIGNS_UTIL_FUNCS_H
#define VITALSIGNS_UTIL_FUNCS_H

#include <stdint.h>
#include <ti/common/sys_common.h>


/*! @brief Version of the Vital Signs Measurement Demo. */
#define VITALSIGNS_VERSION        "0.0.0.1"

/*! @brief Number of IIR Filter stages for the Breathing bandpass filter */
#define IIR_FILTER_BREATH_NUM_STAGES        2

/*! @brief Number of IIR Filter stages for the Heart bandpass filter */
#define IIR_FILTER_HEART_NUM_STAGES         4

/*! @brief Second-order IIR-Filter contains 6 coefficeints per stage */
#define IIR_FILTER_COEFS_SECOND_ORDER       6

/*! @brief Number of Delay Elements in Second-order IIR-Filter */
#define NUM_DELAY_ELEMENTS_IIR_FILTER       3

/*! @brief  Size should at least equal to  "Coefficeints per stage * Number of stages + 2"  */
#define BREATHING_WFM_IIR_FILTER_TAPS_LENGTH   (IIR_FILTER_COEFS_SECOND_ORDER * IIR_FILTER_BREATH_NUM_STAGES) + 2

/*! @brief  Size should be equal to Coefficeints per stage * Number of stages + 2 */
#define HEART_WFM_IIR_FILTER_TAPS_LENGTH       (IIR_FILTER_COEFS_SECOND_ORDER * IIR_FILTER_HEART_NUM_STAGES) + 2

/*! @brief  Filter size for the Moving average filter  */
#define FIR_FILTER_SIZE                    10

/*! @brief  Window Size  */
#define DOPPLER_WINDOW_SIZE                16


enum dataType { int32_type = 0,
                uint32_type,
                float_type
              } ;

/*!
 *  @brief     Implementation of a IIR-Biquad Cascade Filter
 *
 *
 *  @param[in] DataIn          Current Input Sample
 *
 *  @param[in] pFilterCoefs    Filter Co-efficients
 *
 *  @param[in] pScaleVals      Scaling Values
 *
 *  @param[in] pDelay          Delay Elements
 *
 *  @param[in] numStages       Number of Stages
 *
 *  @param[out]                Filtered Value
*/

extern float  filter_IIR_BiquadCascade(float DataIn, 
		                               float *pFilterCoefs, 
									   float *pScaleVals, 
									   float *pDelay, 
									   uint16_t numStages);

/*!
 *  @brief     Phase Unwrapping
 *
 *  @param[in] phase          Current Phase Value
 *
 *  @param[in] phasePrev      Previous Phase Value
 *
 *  @param[out]               Unwrapped Phase
*/

float unwrap(float phase, 
		     float phasePrev, 
			 float *diffPhaseCorrectionCum);

/*!
 *  @brief     Computes the maximum Index
 *
 *  @param[in] pDataIn        Pointer to the Input Data
 *
 *  @param[in] startIndex     Start Index
 *
 *  @param[in] endIndex       End Index
 *
 *  @param[out]               Index Number of the max value
*/

uint16_t  computeMaxIndex(float *pDataIn, 
		                  uint16_t startIndex, 
						  uint16_t endIndex);



/*!
 *   @brief     FIR Filter
 *
 *
 *  @param[in] pDataIn         Pointer to the Data
 *
 *  @param[in] filterCoefs     Filter Coeffiecient
 *
 *  @param[in] numCoefs        Number of filter coefficeints
 *
 *  @param[out]                Filtered value
 *
 */
float  filter_FIR(float *pDataIn, 
				 float *filterCoefs, 
				 uint16_t numCoefs);


/*!
 *   @brief   Computes the SNR (confidence Metric) of the peak in the FFT spectrum.
 *
 *
 *  @param[in] pDataSpectrum                     Pointer to the spectrum
 *
 *  @param[in] spectrumIndexStart                Start Index of the spectrum
 *
 *  @param[in] spectrumIndexEnd                  End Index of the spectrum
 *
 *  @param[in] peakIndex                         Index of the peak in the spectrum
 *
 *  @param[in] numIndexAroundPeak                Bandwidth (in indices) around the peak that should be considered as part of the signal
 *
 *  @param[out]                                  Confidence Metric
 *
 */

float  computeConfidenceMetric(float* pDataSpectrum,
                               uint16_t spectrumIndexStart,
                               uint16_t spectrumIndexEnd,
                               uint16_t peakIndex,
                               uint16_t numIndexAroundPeak);

/*!
 *   @brief     Impulse-like noise is removed by computing a
 *              forward a(n)–a(n+1) and backward a(n)-a(n-1) phase difference
 *              for each a(n) and if this exceeds a certain threshold then
 *              a(n) is replaced by an interpolated value.
 *
 *
 *  @param[in] dataPrev3        a [n - 2]
 *
 *  @param[in] dataPrev2        a [n - 1]
 *
 *  @param[in] dataPrev1        a [n ]
 *
 *  @param[in] thresh           Noise Threshold
 *
 *  @param[out]
 *
 */

float  filter_RemoveImpulseNoise(float dataPrev3, 
								 float dataPrev2, 
								 float dataPrev1,
								 float thresh);

/*!
 *   @brief     Sorts the Data
 *
 *  @param[in] pDataIn              Pointer to the input Data
 *
 *  @param[in] dataLength           Length of the input Data
 *
 *  @param[in] pSortOutIndex        Indexes of the sorted array
 *
 *
 */

void  heapsort_index(float *pDataIn, 
		             int dataLength,  
					 uint16_t *pSortOutIndex);


/*!
 *  @brief     Removes peaks that are less than winMin and greater than winMax
 *
 *  @param[in] pPeakLocsIn      Input Peak indices
 *
 *  @param[in] pPeakLocsOut     Peak indices after filtering
 *
 *  @param[in] numPeaksIn       Number of Input peak indcies
 *
 *  @param[in] winMin           Minimum valid peak distance
 *
 *  @param[in] winMax           Maximum valid peak distance
 */


uint16_t  filterPeaksWfm(uint16_t *pPeakLocsIn, 
		                 uint16_t *pPeakLocsOut, 
						 uint16_t numPeaksIn, 
						 uint16_t winMin, 
						 uint16_t winMax);



/*!
 *   @brief    Finds the peaks in the Input Data
 *
 *  @param[in] pDataIn         Pointer to Input Data
 *
 *  @param[in] enum dataType   Date types supported are "int32_type", "uint32_type", "float_type"
 *
 *  @param[in] pPeakIndex      Indices of the peaks
 *
 *  @param[in] pPeakValues     Values at the peaks
 *
 *  @param[in] startIndex      Start Index
 *
 *  @param[in] endIndex        End Index
 *
 *  @param[out]                Number of peaks
 *
 */

uint16_t find_Peaks(void *pDataIn,
                    enum dataType type,
                    uint16_t *pPeakIndex,
                    float *pPeakValues,
                    uint16_t startIndex,
                    uint16_t endIndex);


/*!
 *   @brief     Performs Automatic gain control. If the energy in a segment of lenBlock samples is
 *              above a certain threshold then the values within lenBlock are scaled down
 *
 *
 *  @param[in] pDataIn         Pointer to Input Data
 *
 *  @param[in] dataInLength    Length of input Data
 *
 *  @param[in] lenBlock        Length of block over which the energy is computed
 *
 *  @param[in] thresh          Threshold value
 *
 *  @param[out]                Number of lenBlock segments that were scaled down
 *
 */

float  computeAGC (float* pDataIn,
                   uint16_t dataInLength,
                   uint16_t lenBlock,
                   float thresh);

/*!
 *   @brief     Performs AutoCorrelation.
 *
 *
 *  @param[in] pDataIn         Pointer to Input Data
 *
 *  @param[in] dataInLength    Length of input Data
 *
 *  @param[in] pDataOut        Pointer to Output Data
 *
 *  @param[in] minLag          Minimum Lag for the Autocorrelation
 *
 *  @param[in] maxLag          Maximum Lag for the Autocorrelation
 *
 *  @param[out]
 *
 */
void computeAutoCorrelation (float* pDataIn,
                              uint16_t dataInLength,
                              float* pDataOut,
                              uint16_t minLag,
                              uint16_t maxLag);


/*!
 *   @brief     Computes the energy in the Fundamental Frequency and the 1st Harmonic
 *
 *
 *  @param[in] pDataIn                  Pointer to Input Spectrum Data
 *
 *  @param[in] pDataOut                 Pointer to the Output Data
 *
 *  @param[in] spectrumStartIndex       Index of the starting frequency
 *
 *  @param[in] spectrumEndIndex         Index of the End frequency
 *
 *  @param[in] freqWindowSize           Window Size around the frequency
 *
 *  @param[out]
 *
 */

void computeEnergyHarmonics (float* pSpectrum,
                             float* pDataOut,
                             uint16_t spectrumStartIndex,
                             uint16_t spectrumEndIndex,
                             uint16_t freqWindowSize);

#endif



