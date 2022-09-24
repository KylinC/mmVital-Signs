/**
 *   @file  dss_data_path.c
 *
 *   @brief
 *      Implements Data path processing functionality.
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
#if defined (SUBSYS_DSS)
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#endif
#define DebugP_ASSERT_ENABLED 1
#include <ti/drivers/osal/DebugP.h>
#include <assert.h>
#include <ti/common/sys_common.h>
#include <ti/common/sys_defs.h>
#include <ti/drivers/osal/SemaphoreP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

#include <ti/alg/mmwavelib/mmwavelib.h>

/* C64P dsplib (fixed point part for C674X) */
#include "DSP_fft32x32.h"
#include "DSP_fft16x16.h"

/* C674x mathlib */
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE"
 */
#pragma diag_push
#pragma diag_suppress 48
#include <ti/mathlib/mathlib.h>
#pragma diag_pop


// Occupancy_Detection
#include "mmw_config.h"
#include "dss_mmw.h"
#include "dss_data_path.h"
#include "dss_config_edma_util.h"
#include "dss_resources.h"
#include "dss_vitalSignsDemo_utilsFunc.h"
#include <ti/utils/mathutils/mathutils.h>

extern MmwDemo_DSS_MCB    gMmwDssMCB;

/** @brief Lookup table for Twiddle table generation and DFT single bin DFT calculation.
 * It contains 256 complex exponentials e(k) = cos(2*pi*k/1024)+j*sin(2*pi*k/1024), k=0,...,255,
 * Imaginary values are in even, and real values are in odd locations. Values are in Q31 format,
 * saturated to +/- 2147483647
 *
 * Following Matlab code was used to generate this table:
 *
 * %Generates lookup table for fast twiddle table generation for DSP lib FFT
 * %functions 16x16 and 32x32 for FFT sizes up to 1024. It saturates the
 * %amplitude to +/- 2147483647. The values are saved to file as imaginary
 * %(sine) to even, and real (cosine) to odd index positions.
 * M = 2147483647.5;
 * nMax = 1024;
 * table=round(M*exp(1j*2*pi/1024*[0:1*nMax/4-1]'));
 * table=max(min(real(table),2147483647),-2147483647) + 1j* max(min(imag(table),2147483647),-2147483647);
 * fid = fopen('twiddle_table_all.dat','w');
 *
 * tableRI = [imag(table) real(table)]';
 * tableRI = tableRI(:);
 * tableRI(tableRI<0) = tableRI(tableRI<0) + 4294967296;
 *
 * fprintf(fid, [repmat(' 0x%08x,', 1, 8) '\n'], tableRI);
 * fclose(fid);
 *
**/
#pragma DATA_ALIGN(twiddleTableCommon, 8)
const int32_t twiddleTableCommon[2*256] = {
    0x00000000, 0x7fffffff, 0x00c90f88, 0x7fff6216, 0x01921d20, 0x7ffd885a, 0x025b26d7, 0x7ffa72d1,
    0x03242abf, 0x7ff62182, 0x03ed26e6, 0x7ff09477, 0x04b6195d, 0x7fe9cbbf, 0x057f0035, 0x7fe1c76b,
    0x0647d97c, 0x7fd8878d, 0x0710a345, 0x7fce0c3e, 0x07d95b9e, 0x7fc25596, 0x08a2009a, 0x7fb563b2,
    0x096a9049, 0x7fa736b4, 0x0a3308bd, 0x7f97cebc, 0x0afb6805, 0x7f872bf2, 0x0bc3ac35, 0x7f754e7f,
    0x0c8bd35e, 0x7f62368f, 0x0d53db92, 0x7f4de450, 0x0e1bc2e4, 0x7f3857f5, 0x0ee38766, 0x7f2191b3,
    0x0fab272b, 0x7f0991c3, 0x1072a048, 0x7ef0585f, 0x1139f0cf, 0x7ed5e5c6, 0x120116d5, 0x7eba3a39,
    0x12c8106e, 0x7e9d55fc, 0x138edbb1, 0x7e7f3956, 0x145576b1, 0x7e5fe493, 0x151bdf86, 0x7e3f57fe,
    0x15e21444, 0x7e1d93e9, 0x16a81305, 0x7dfa98a7, 0x176dd9de, 0x7dd6668e, 0x183366e9, 0x7db0fdf7,
    0x18f8b83c, 0x7d8a5f3f, 0x19bdcbf3, 0x7d628ac5, 0x1a82a026, 0x7d3980ec, 0x1b4732ef, 0x7d0f4218,
    0x1c0b826a, 0x7ce3ceb1, 0x1ccf8cb3, 0x7cb72724, 0x1d934fe5, 0x7c894bdd, 0x1e56ca1e, 0x7c5a3d4f,
    0x1f19f97b, 0x7c29fbee, 0x1fdcdc1b, 0x7bf88830, 0x209f701c, 0x7bc5e28f, 0x2161b3a0, 0x7b920b89,
    0x2223a4c5, 0x7b5d039d, 0x22e541af, 0x7b26cb4f, 0x23a6887e, 0x7aef6323, 0x24677757, 0x7ab6cba3,
    0x25280c5e, 0x7a7d055b, 0x25e845b6, 0x7a4210d8, 0x26a82186, 0x7a05eead, 0x27679df4, 0x79c89f6d,
    0x2826b928, 0x798a23b1, 0x28e5714b, 0x794a7c11, 0x29a3c485, 0x7909a92c, 0x2a61b101, 0x78c7aba1,
    0x2b1f34eb, 0x78848413, 0x2bdc4e6f, 0x78403328, 0x2c98fbba, 0x77fab988, 0x2d553afb, 0x77b417df,
    0x2e110a62, 0x776c4edb, 0x2ecc681e, 0x77235f2d, 0x2f875262, 0x76d94988, 0x3041c760, 0x768e0ea5,
    0x30fbc54d, 0x7641af3c, 0x31b54a5d, 0x75f42c0a, 0x326e54c7, 0x75a585cf, 0x3326e2c2, 0x7555bd4b,
    0x33def287, 0x7504d345, 0x34968250, 0x74b2c883, 0x354d9057, 0x745f9dd1, 0x36041ad9, 0x740b53fa,
    0x36ba2014, 0x73b5ebd0, 0x376f9e46, 0x735f6626, 0x382493b0, 0x7307c3d0, 0x38d8fe93, 0x72af05a6,
    0x398cdd32, 0x72552c84, 0x3a402dd2, 0x71fa3948, 0x3af2eeb7, 0x719e2cd2, 0x3ba51e29, 0x71410804,
    0x3c56ba70, 0x70e2cbc6, 0x3d07c1d6, 0x708378fe, 0x3db832a6, 0x70231099, 0x3e680b2c, 0x6fc19385,
    0x3f1749b8, 0x6f5f02b1, 0x3fc5ec98, 0x6efb5f12, 0x4073f21d, 0x6e96a99c, 0x4121589a, 0x6e30e349,
    0x41ce1e64, 0x6dca0d14, 0x427a41d0, 0x6d6227fa, 0x4325c135, 0x6cf934fb, 0x43d09aec, 0x6c8f351c,
    0x447acd50, 0x6c242960, 0x452456bd, 0x6bb812d1, 0x45cd358f, 0x6b4af278, 0x46756828, 0x6adcc964,
    0x471cece6, 0x6a6d98a4, 0x47c3c22f, 0x69fd614a, 0x4869e665, 0x698c246c, 0x490f57ee, 0x6919e320,
    0x49b41533, 0x68a69e81, 0x4a581c9d, 0x683257ab, 0x4afb6c98, 0x67bd0fbc, 0x4b9e038f, 0x6746c7d7,
    0x4c3fdff3, 0x66cf811f, 0x4ce10034, 0x66573cbb, 0x4d8162c4, 0x65ddfbd3, 0x4e210617, 0x6563bf92,
    0x4ebfe8a4, 0x64e88926, 0x4f5e08e3, 0x646c59bf, 0x4ffb654d, 0x63ef328f, 0x5097fc5e, 0x637114cc,
    0x5133cc94, 0x62f201ac, 0x51ced46e, 0x6271fa69, 0x5269126e, 0x61f1003f, 0x53028517, 0x616f146b,
    0x539b2aef, 0x60ec3830, 0x5433027d, 0x60686cce, 0x54ca0a4a, 0x5fe3b38d, 0x556040e2, 0x5f5e0db3,
    0x55f5a4d2, 0x5ed77c89, 0x568a34a9, 0x5e50015d, 0x571deef9, 0x5dc79d7c, 0x57b0d256, 0x5d3e5236,
    0x5842dd54, 0x5cb420df, 0x58d40e8c, 0x5c290acc, 0x59646497, 0x5b9d1153, 0x59f3de12, 0x5b1035cf,
    0x5a82799a, 0x5a82799a, 0x5b1035cf, 0x59f3de12, 0x5b9d1153, 0x59646497, 0x5c290acc, 0x58d40e8c,
    0x5cb420df, 0x5842dd54, 0x5d3e5236, 0x57b0d256, 0x5dc79d7c, 0x571deef9, 0x5e50015d, 0x568a34a9,
    0x5ed77c89, 0x55f5a4d2, 0x5f5e0db3, 0x556040e2, 0x5fe3b38d, 0x54ca0a4a, 0x60686cce, 0x5433027d,
    0x60ec3830, 0x539b2aef, 0x616f146b, 0x53028517, 0x61f1003f, 0x5269126e, 0x6271fa69, 0x51ced46e,
    0x62f201ac, 0x5133cc94, 0x637114cc, 0x5097fc5e, 0x63ef328f, 0x4ffb654d, 0x646c59bf, 0x4f5e08e3,
    0x64e88926, 0x4ebfe8a4, 0x6563bf92, 0x4e210617, 0x65ddfbd3, 0x4d8162c4, 0x66573cbb, 0x4ce10034,
    0x66cf811f, 0x4c3fdff3, 0x6746c7d7, 0x4b9e038f, 0x67bd0fbc, 0x4afb6c98, 0x683257ab, 0x4a581c9d,
    0x68a69e81, 0x49b41533, 0x6919e320, 0x490f57ee, 0x698c246c, 0x4869e665, 0x69fd614a, 0x47c3c22f,
    0x6a6d98a4, 0x471cece6, 0x6adcc964, 0x46756828, 0x6b4af278, 0x45cd358f, 0x6bb812d1, 0x452456bd,
    0x6c242960, 0x447acd50, 0x6c8f351c, 0x43d09aec, 0x6cf934fb, 0x4325c135, 0x6d6227fa, 0x427a41d0,
    0x6dca0d14, 0x41ce1e64, 0x6e30e349, 0x4121589a, 0x6e96a99c, 0x4073f21d, 0x6efb5f12, 0x3fc5ec98,
    0x6f5f02b1, 0x3f1749b8, 0x6fc19385, 0x3e680b2c, 0x70231099, 0x3db832a6, 0x708378fe, 0x3d07c1d6,
    0x70e2cbc6, 0x3c56ba70, 0x71410804, 0x3ba51e29, 0x719e2cd2, 0x3af2eeb7, 0x71fa3948, 0x3a402dd2,
    0x72552c84, 0x398cdd32, 0x72af05a6, 0x38d8fe93, 0x7307c3d0, 0x382493b0, 0x735f6626, 0x376f9e46,
    0x73b5ebd0, 0x36ba2014, 0x740b53fa, 0x36041ad9, 0x745f9dd1, 0x354d9057, 0x74b2c883, 0x34968250,
    0x7504d345, 0x33def287, 0x7555bd4b, 0x3326e2c2, 0x75a585cf, 0x326e54c7, 0x75f42c0a, 0x31b54a5d,
    0x7641af3c, 0x30fbc54d, 0x768e0ea5, 0x3041c760, 0x76d94988, 0x2f875262, 0x77235f2d, 0x2ecc681e,
    0x776c4edb, 0x2e110a62, 0x77b417df, 0x2d553afb, 0x77fab988, 0x2c98fbba, 0x78403328, 0x2bdc4e6f,
    0x78848413, 0x2b1f34eb, 0x78c7aba1, 0x2a61b101, 0x7909a92c, 0x29a3c485, 0x794a7c11, 0x28e5714b,
    0x798a23b1, 0x2826b928, 0x79c89f6d, 0x27679df4, 0x7a05eead, 0x26a82186, 0x7a4210d8, 0x25e845b6,
    0x7a7d055b, 0x25280c5e, 0x7ab6cba3, 0x24677757, 0x7aef6323, 0x23a6887e, 0x7b26cb4f, 0x22e541af,
    0x7b5d039d, 0x2223a4c5, 0x7b920b89, 0x2161b3a0, 0x7bc5e28f, 0x209f701c, 0x7bf88830, 0x1fdcdc1b,
    0x7c29fbee, 0x1f19f97b, 0x7c5a3d4f, 0x1e56ca1e, 0x7c894bdd, 0x1d934fe5, 0x7cb72724, 0x1ccf8cb3,
    0x7ce3ceb1, 0x1c0b826a, 0x7d0f4218, 0x1b4732ef, 0x7d3980ec, 0x1a82a026, 0x7d628ac5, 0x19bdcbf3,
    0x7d8a5f3f, 0x18f8b83c, 0x7db0fdf7, 0x183366e9, 0x7dd6668e, 0x176dd9de, 0x7dfa98a7, 0x16a81305,
    0x7e1d93e9, 0x15e21444, 0x7e3f57fe, 0x151bdf86, 0x7e5fe493, 0x145576b1, 0x7e7f3956, 0x138edbb1,
    0x7e9d55fc, 0x12c8106e, 0x7eba3a39, 0x120116d5, 0x7ed5e5c6, 0x1139f0cf, 0x7ef0585f, 0x1072a048,
    0x7f0991c3, 0x0fab272b, 0x7f2191b3, 0x0ee38766, 0x7f3857f5, 0x0e1bc2e4, 0x7f4de450, 0x0d53db92,
    0x7f62368f, 0x0c8bd35e, 0x7f754e7f, 0x0bc3ac35, 0x7f872bf2, 0x0afb6805, 0x7f97cebc, 0x0a3308bd,
    0x7fa736b4, 0x096a9049, 0x7fb563b2, 0x08a2009a, 0x7fc25596, 0x07d95b9e, 0x7fce0c3e, 0x0710a345,
    0x7fd8878d, 0x0647d97c, 0x7fe1c76b, 0x057f0035, 0x7fe9cbbf, 0x04b6195d, 0x7ff09477, 0x03ed26e6,
    0x7ff62182, 0x03242abf, 0x7ffa72d1, 0x025b26d7, 0x7ffd885a, 0x01921d20, 0x7fff6216, 0x00c90f88
};

 /**
  *  @b Description
  *  @n
  *      Following function is equivalent of the dsplib's gen_twiddle_fft32x32() function
  *      optimized for speed to allow quick reconfiguration when switching sub-frames
  *      in advanced frame mode. The alternative is to store tables for each sub-frame
  *      which is very high in memory consumption. The maximum error with respect to the
  *      dsplib function is in the LSB (+/- 1).
  */
int32_t MmwDemo_gen_twiddle_fft32x32_fast(int32_t *w, int32_t n, double scale)
 {
     int32_t i, j, k;
     int32_t log2n = 30 - _norm(n); //Note n is always power of 2
     int32_t step = 1024 >> log2n;
     int32_t step6 = 3 * step;
     int32_t step4 = 2 * step;
     int32_t step2 = 1 * step;
     int32_t ind, indLsb, indMsb;
     int64_t * restrict table = (int64_t *) twiddleTableCommon;
     int64_t * restrict wd = (int64_t *) w;
     int32_t xRe;
     int32_t xIm;

     for (j = 1, k = 0; j < n >> 2; j = j << 2) {
         for (i =0; i < n >> 2; i += j) {
             ind = step2 * (i);
             indLsb = ind & 0xFF;
             indMsb = (ind >> 8) & 0x3;
             xRe =  _hill(table[indLsb]);
             xIm =  _loll(table[indLsb]);
             if (indMsb == 0)
             {
                 wd[k + 0] =  _itoll(xRe, xIm);
             }
             if (indMsb == 1)
             {
                 wd[k + 0] =  _itoll(-xIm, xRe);
             }
             if (indMsb == 2)
             {
                 wd[k + 0] =  _itoll(-xRe, -xIm);
             }

             ind = step4 * (i);
             indLsb = ind & 0xFF;
             indMsb = (ind >> 8) & 0x3;
             xRe =  _hill(table[indLsb]);
             xIm =  _loll(table[indLsb]);
             if (indMsb == 0)
             {
                 wd[k + 1] =  _itoll(xRe, xIm);
             }
             if (indMsb == 1)
             {
                 wd[k + 1] =  _itoll(-xIm, xRe);
             }
             if (indMsb == 2)
             {
                 wd[k + 1] =  _itoll(-xRe, -xIm);
             }

             ind = step6 * (i);
             indLsb = ind & 0xFF;
             indMsb = (ind >> 8) & 0x3;
             xRe =  _hill(table[indLsb]);
             xIm =  _loll(table[indLsb]);
             if (indMsb == 0)
             {
                 wd[k + 2] =  _itoll(xRe, xIm);
             }
             if (indMsb == 1)
             {
                 wd[k + 2] =  _itoll(-xIm, xRe);
             }
             if (indMsb == 2)
             {
                 wd[k + 2] =  _itoll(-xRe, -xIm);
             }

             k += 3;
         }
     }
     return 2*k;
 }

/**
 *  @b Description
 *  @n
 *      Following function is equivalent of the dsplib's gen_twiddle_fft16x16() function
 *      optimized for speed to allow quick reconfiguration when switching sub-frames
 *      in advanced frame mode. The alternative is to store tables for each sub-frame
 *      which is very high in memory consumption. The maximum error with respect to the
  *     dsplib function is in the LSB (+/- 1).
 */
int32_t MmwDemo_gen_twiddle_fft16x16_fast(short *w, int32_t n)
{
     int32_t i, j, k;
     int32_t log2n = 30 - _norm(n); //Note n is always power of 2
     int32_t step = 1024 >> log2n;
     int32_t step6 = 3 * step;
     int32_t step4 = 2 * step;
     int32_t step2 = 1 * step;
     int32_t ind, indLsb, indMsb;
     int64_t * restrict table = (int64_t *) twiddleTableCommon;
     uint32_t * restrict wd = (uint32_t *) w;
     int32_t xRe;
     int32_t xIm;

     for (j = 1, k = 0; j < n >> 2; j = j << 2) {
         for (i = 0; i < n >> 2; i += j << 1) {
            ind = step2 * (i + j);
            indLsb = ind & 0xFF;
            indMsb = (ind >> 8) & 0x3;
            xRe =  ((int32_t)_sadd(_hill(table[indLsb]), 0x00008000)) >> 16;
            xIm =  ((int32_t)_sadd(_loll(table[indLsb]), 0x00008000)) >> 16;
            if (indMsb == 0)
            {
             wd[k + 1] =  _pack2(xRe, xIm);
            }
            if (indMsb == 1)
            {
             wd[k + 1] =  _pack2(-xIm, xRe);
            }
            if (indMsb == 2)
            {
             wd[k + 1] =  _pack2(-xRe, -xIm);
            }

            ind = step2 * (i);
            indLsb = ind & 0xFF;
            indMsb = (ind >> 8) & 0x3;
            xRe =  ((int32_t)_sadd(_hill(table[indLsb]), 0x00008000)) >> 16;
            xIm =  ((int32_t)_sadd(_loll(table[indLsb]), 0x00008000)) >> 16;
            if (indMsb == 0)
            {
             wd[k + 0] =  _pack2(xRe, xIm);
            }
            if (indMsb == 1)
            {
             wd[k + 0] =  _pack2(-xIm, xRe);
            }
            if (indMsb == 2)
            {
             wd[k + 0] =  _pack2(-xRe, -xIm);
            }

            ind = step4 * (i + j);
            indLsb = ind & 0xFF;
            indMsb = (ind >> 8) & 0x3;
            xRe =  ((int32_t)_sadd(_hill(table[indLsb]), 0x00008000)) >> 16;
            xIm =  ((int32_t)_sadd(_loll(table[indLsb]), 0x00008000)) >> 16;
            if (indMsb == 0)
            {
              wd[k + 3] =  _pack2(-xRe, -xIm);
            }
            if (indMsb == 1)
            {
              wd[k + 3] =  _pack2(xIm, -xRe);
            }
            if (indMsb == 2)
            {
              wd[k + 3] =  _pack2(xRe, xIm);
            }

            ind = step4 * (i);
            indLsb = ind & 0xFF;
            indMsb = (ind >> 8) & 0x3;
            xRe =  ((int32_t)_sadd(_hill(table[indLsb]), 0x00008000)) >> 16;
            xIm =  ((int32_t)_sadd(_loll(table[indLsb]), 0x00008000)) >> 16;
            if (indMsb == 0)
            {
              wd[k + 2] =  _pack2(-xRe, -xIm);
            }
            if (indMsb == 1)
            {
              wd[k + 2] =  _pack2(xIm, -xRe);
            }
            if (indMsb == 2)
            {
              wd[k + 2] =  _pack2(xRe, xIm);
            }

            ind = step6 * (i + j);
            indLsb = ind & 0xFF;
            indMsb = (ind >> 8) & 0x3;
            xRe =  ((int32_t)_sadd(_hill(table[indLsb]), 0x00008000)) >> 16;
            xIm =  ((int32_t)_sadd(_loll(table[indLsb]), 0x00008000)) >> 16;
            if (indMsb == 0)
            {
               wd[k + 5] =  _pack2(xRe, xIm);
            }
            if (indMsb == 1)
            {
               wd[k + 5] =  _pack2(-xIm, xRe);
            }
            if (indMsb == 2)
            {
               wd[k + 5] =  _pack2(-xRe, -xIm);
            }

            ind = step6 * (i);
            indLsb = ind & 0xFF;
            indMsb = (ind >> 8) & 0x3;
            xRe =  ((int32_t)_sadd(_hill(table[indLsb]), 0x00008000)) >> 16;
            xIm =  ((int32_t)_sadd(_loll(table[indLsb]), 0x00008000)) >> 16;
            if (indMsb == 0)
            {
               wd[k + 4] =  _pack2(xRe, xIm);
            }
            if (indMsb == 1)
            {
               wd[k + 4] =  _pack2(-xIm, xRe);
            }
            if (indMsb == 2)
            {
               wd[k + 4] =  _pack2(-xRe, -xIm);
            }

            k += 6;
         }
     }
     return 2*k;
}

/*! @brief L2 heap used for allocating buffers in L2 SRAM,
    mostly scratch buffers */
#define MMW_L2_HEAP_SIZE    0xC000U

/*! @brief L1 heap used for allocating buffers in L1D SRAM,
    mostly scratch buffers */
#define MMW_L1_HEAP_SIZE    0x4000U

/*! L3 RAM buffer */
#pragma DATA_SECTION(gMmwL3, ".l3data");
#pragma DATA_ALIGN(gMmwL3, 8);
uint8_t gMmwL3[SOC_L3RAM_SIZE];

/*! L2 Heap */
#pragma DATA_SECTION(gMmwL2, ".l2data");
#pragma DATA_ALIGN(gMmwL2, 8);
uint8_t gMmwL2[MMW_L2_HEAP_SIZE];

/*! L1 Heap */
#pragma DATA_SECTION(gMmwL1, ".l1data");
#pragma DATA_ALIGN(gMmwL1, 8);
uint8_t gMmwL1[MMW_L1_HEAP_SIZE];

/*! Types of FFT window */
/*! FFT window 16 - samples format is int16_t */
#define FFT_WINDOW_INT16 0
/*! FFT window 32 - samples format is int32_t */
#define FFT_WINDOW_INT32 1

/* FFT Window */
/*! Hanning window */
#define MMW_WIN_HANNING  0
/*! Blackman window */
#define MMW_WIN_BLACKMAN 1
/*! Rectangular window */
#define MMW_WIN_RECT     2

/*! If MMW_USE_SINGLE_POINT_DFT is defined azimuth calculation uses single
 * point DFT, otherwise FFT function from DSP lib*/
#define MMW_USE_SINGLE_POINT_DFT

/* Vital Signs Prams */

static uint32_t gFrameCount;

void MmwDemo_genWindow(void *win,
                        uint32_t windowDatumType,
                        uint32_t winLen,
                        uint32_t winGenLen,
                        int32_t oneQformat,
                        uint32_t winType);


#define MMW_EDMA_TRIGGER_ENABLE  1
#define MMW_EDMA_TRIGGER_DISABLE 0


extern volatile cycleLog_t gCycleLog;


/**
 *  @b Description
 *  @n
 *      Resets the Doppler line bit mask. Doppler line bit mask indicates Doppler
 *      lines (bins) on wich the CFAR in Doppler direction detected objects.
 *      After the CFAR in Doppler direction is completed for all range bins, the
 *      CFAR in range direction is performed on indicated Doppler lines.
 *      The array dopplerLineMask is uint32_t array. The LSB bit of the first word
 *      corresponds to Doppler line (bin) zero.
 *
 */
void MmwDemo_resetDopplerLines(MmwDemo_1D_DopplerLines_t * ths)
{
    memset((void *) ths->dopplerLineMask, 0 , ths->dopplerLineMaskLen * sizeof(uint32_t));
    ths->currentIndex = 0;
}

/**
 *  @b Description
 *  @n
 *      Sets the bit in the Doppler line bit mask dopplerLineMask corresponding to Doppler
 *      line on which CFAR in Doppler direction detected object. Indicating the Doppler
 *      line being active in observed frame. @sa MmwDemo_resetDopplerLines
 */
void MmwDemo_setDopplerLine(MmwDemo_1D_DopplerLines_t * ths, uint16_t dopplerIndex)
{
    uint32_t word = dopplerIndex >> 5;
    uint32_t bit = dopplerIndex & 31;

    ths->dopplerLineMask[word] |= (0x1 << bit);
}

/**
 *  @b Description
 *  @n
 *      Checks whetehr Doppler line is active in the observed frame. It checks whether the bit
 *      is set in the Doppler line bit mask corresponding to Doppler
 *      line on which CFAR in Doppler direction detected object.
 *      @sa MmwDemo_resetDopplerLines
 */
uint32_t MmwDemo_isSetDopplerLine(MmwDemo_1D_DopplerLines_t * ths, uint16_t index)
{
    uint32_t dopplerLineStat;
    uint32_t word = index >> 5;
    uint32_t bit = index & 31;

    if(ths->dopplerLineMask[word] & (0x1 << bit))
    {
        dopplerLineStat = 1;
    }
    else
    {
        dopplerLineStat = 0;
    }
    return dopplerLineStat;
}

/**
 *  @b Description
 *  @n
 *      Gets the Doppler index from the Doppler line bit mask, starting from the
 *      smallest active Doppler lin (bin). Subsequent calls return the next
 *      active Doppler line. @sa MmwDemo_resetDopplerLines
 *
 */
int32_t MmwDemo_getDopplerLine(MmwDemo_1D_DopplerLines_t * ths)
{
    uint32_t index = ths->currentIndex;
    uint32_t word = index >> 5;
    uint32_t bit = index & 31;

    while (((ths->dopplerLineMask[word] >> bit) & 0x1) == 0)
    {
        index++;
        bit++;
        if (bit == 32)
        {
            word++;
            bit = 0;
            if (word >= ths->dopplerLineMaskLen)
            {
                MmwDemo_dssAssert(0);
            }
        }
    }
    ths->currentIndex = index + 1;
    return index;
}


/**
 *  @b Description
 *  @n
 *      Power of 2 round up function.
 */
uint32_t MmwDemo_pow2roundup (uint32_t x)
{
    uint32_t result = 1;
    while(x > result)
    {
        result <<= 1;
    }
    return result;
}

#if 0
/**
 *  @b Description
 *  @n
 *      Calculates X/Y coordinates in meters based on the maximum position in
 *      the magnitude square of the azimuth FFT output. The function is called
 *      per detected object. The detected object structure already has populated
 *      range and Doppler indices. It calculates X and Y and coordinates and
 *      stores them into object fields along with the peak height. Also it
 *      populates the azimuth index in azimuthMagSqr array.
 *
 *  @param[in] obj                Pointer to data path object
 *
 *  @param[in] objIndex           Detected object index
 *
 *  @param[in] azimIdx            Index of the peak position in Azimuth FFT output
 *
 *  @param[in] maxVal             Peak value
 *
 *  @retval
 *      NONE
 */
void MmwDemo_XYcalc (MmwDemo_DSS_DataPathObj *obj,
                     uint32_t objIndex,
                     uint16_t azimIdx,
                     float maxVal)
{
    int32_t sMaxIdx;
    float temp;
    float Wx;
    float range;
    float x, y;
    uint32_t xyzOutputQFormat = obj->xyzOutputQFormat;
#define ONE_QFORMAT (1 << xyzOutputQFormat)
    float rangeResolution = obj->rangeResolution;


    uint32_t numAngleBins = obj->numAngleBins;
    uint32_t numDopplerBins = obj->numDopplerBins;
    uint32_t numRangeBins = obj->numRangeBins;


    obj->detObj2dAzimIdx[objIndex] = azimIdx;

    /* Save square root of maximum peak to object peak value scaling */
    temp = divsp(maxVal, (numRangeBins * numAngleBins * numDopplerBins));
    obj->detObj2D[objIndex].peakVal = (uint16_t)sqrtsp(temp);

    /* Calculate X and Y coordinates in meters in Q format */
#ifdef MMW_ENABLE_NEGATIVE_FREQ_SLOPE
    if (rangeResolution > 0)
    {
        range = obj->detObj2D[objIndex].rangeIdx * rangeResolution;
    }
    else
    {
        obj->detObj2D[objIndex].rangeIdx = (uint16_t) ((int32_t) obj->numRangeBins - (int32_t) obj->detObj2D[objIndex].rangeIdx);
        range = obj->detObj2D[objIndex].rangeIdx * -rangeResolution;
    }
#else
    range = obj->detObj2D[objIndex].rangeIdx * rangeResolution;
#endif

    /* Compensate for range bias */
    range -= obj->cliCommonCfg->compRxChanCfg.rangeBias;
    if (range < 0)
    {
        range = 0;
    }

    if(azimIdx > (numAngleBins/2 -1))
    {
        sMaxIdx = azimIdx - numAngleBins;
    }
    else
    {
        sMaxIdx = azimIdx;
    }

    Wx = 2 * (float) sMaxIdx / numAngleBins;
    x = range * Wx;

    /* y = sqrt(range^2 - x^2) */
    temp = range*range -x*x ;
    if (temp > 0)
    {
        y = sqrtsp(temp);
    }
    else
    {
        y = 0;
    }

    /* Convert to Q format */
    obj->detObj2D[objIndex].x = (int16_t) ROUND(x * ONE_QFORMAT);
    obj->detObj2D[objIndex].y = (int16_t) ROUND(y * ONE_QFORMAT);
    obj->detObj2D[objIndex].z = 0;
}

/**
 *  @n
 *      Calculates X/Y coordinates in meters based on the maximum position in
 *      the magnitude square of the azimuth FFT output. The function is called
 *      per detected object. The detected object structure already has populated
 *      range and Doppler indices. This function finds maximum index in the
 *      azimuth FFT, calculates X and Y and coordinates and stores them into
 *      object fields along with the peak height. Also it populates the azimuth
 *      index in azimuthMagSqr array.
 *
 *  @param[in] obj                Pointer to data path object
 *
 *  @param[in] objIndex           Detected object index
 *
 *  @retval
 *      NONE
 */
void MmwDemo_XYestimation(MmwDemo_DSS_DataPathObj *obj,
                          uint32_t objIndex)
{
    uint32_t i;
    float *azimuthMagSqr = obj->azimuthMagSqr;
    uint32_t numAngleBins = obj->numAngleBins;
    uint32_t numSearchBins;

    uint32_t azimIdx = 0;
    float maxVal = 0;

    if(obj->cliCfg->extendedMaxVelocityCfg.enabled && (obj->numVirtualAntAzim > obj->numRxAntennas))
    {
        numSearchBins = numAngleBins * 2;
    }
    else
    {
        numSearchBins = numAngleBins;
    }
    /* Find peak position - search in original and flipped output */
    for (i = 0; i < numSearchBins ; i++)
    {
        if (azimuthMagSqr[i] > maxVal)
        {
            azimIdx = i;
            maxVal = azimuthMagSqr[i];
        }
    }
    if(obj->cliCfg->extendedMaxVelocityCfg.enabled && (obj->numVirtualAntAzim > obj->numRxAntennas))
    {
        if(azimIdx >= numAngleBins)
        {
            /* Velocity aliased: |velocity| > Vmax */
            /* Correct peak index: */
            azimIdx -= numAngleBins;
            /* Correct velocity */
            if (obj->detObj2D[objIndex].dopplerIdx < 0)
            {
                obj->detObj2D[objIndex].dopplerIdx += (int16_t) obj->numDopplerBins;
            }
            else
            {
                obj->detObj2D[objIndex].dopplerIdx -= (int16_t) obj->numDopplerBins;
            }
        }
    }

    MmwDemo_XYcalc (obj,
                    objIndex,
                    azimIdx,
                    maxVal);

    /* Check for second peak */
    if (obj->cliCfg->multiObjBeamFormingCfg.enabled && (!obj->cliCfg->extendedMaxVelocityCfg.enabled))
    {
        uint32_t leftSearchIdx;
        uint32_t rightSearchIdx;
        uint32_t secondSearchLen;
        uint32_t iModAzimLen;
        float maxVal2;
        int32_t k;

        /* Find right edge of the first peak */
        i = azimIdx;
        leftSearchIdx = (i + 1) & (numAngleBins-1);
        k = numAngleBins;
        while ((azimuthMagSqr[i] >= azimuthMagSqr[leftSearchIdx]) && (k > 0))
        {
            i = (i + 1) & (numAngleBins-1);
            leftSearchIdx = (leftSearchIdx + 1) & (numAngleBins-1);
            k--;
        }

        /* Find left edge of the first peak */
        i = azimIdx;
        rightSearchIdx = (i - 1) & (numAngleBins-1);
        k = numAngleBins;
        while ((azimuthMagSqr[i] >= azimuthMagSqr[rightSearchIdx]) && (k > 0))
        {
            i = (i - 1) & (numAngleBins-1);
            rightSearchIdx = (rightSearchIdx - 1) & (numAngleBins-1);
            k--;
        }

        secondSearchLen = ((rightSearchIdx - leftSearchIdx) & (numAngleBins-1)) + 1;
        /* Find second peak */
        maxVal2 = azimuthMagSqr[leftSearchIdx];
        azimIdx = leftSearchIdx;
        for (i = leftSearchIdx; i < (leftSearchIdx + secondSearchLen); i++)
        {
            iModAzimLen = i & (numAngleBins-1);
            if (azimuthMagSqr[iModAzimLen] > maxVal2)
            {
                azimIdx = iModAzimLen;
                maxVal2 = azimuthMagSqr[iModAzimLen];
            }
        }

        /* Is second peak greater than threshold? */
        if (maxVal2 > (maxVal * obj->cliCfg->multiObjBeamFormingCfg.multiPeakThrsScal) && (obj->numDetObj < MMW_MAX_OBJ_OUT))
        {
            /* Second peak detected! Add it to the end of the list */
            obj->detObj2D[obj->numDetObj].dopplerIdx = obj->detObj2D[objIndex].dopplerIdx;
            obj->detObj2D[obj->numDetObj].rangeIdx = obj->detObj2D[objIndex].rangeIdx;
            objIndex = obj->numDetObj;
            obj->numDetObj++;

            MmwDemo_XYcalc (obj,
                            objIndex,
                            azimIdx,
                            maxVal2);
        }
    }
}


/**
 *  @b Description
 *  @n
 *      Calculates Y coordinates in meters based on the maximum position in
 *      the magnitude square of the azimuth FFT output. This is called
 *      when the number of rx antennas is 1.
 *
 *  @param[in] obj                Pointer to data path object
 *
 *  @param[in] objIndex           Detected object index
 */
void MmwDemo_Yestimation(MmwDemo_DSS_DataPathObj *obj,
                          uint32_t objIndex)
{
    float range;
    float rangeResolution = obj->rangeResolution;
    uint32_t xyzOutputQFormat = obj->xyzOutputQFormat;
#define ONE_QFORMAT (1 << xyzOutputQFormat)

    /* Calculate Y coordiante in meters in Q8 format */
#ifdef MMW_ENABLE_NEGATIVE_FREQ_SLOPE
    if (rangeResolution > 0)
    {
        range = obj->detObj2D[objIndex].rangeIdx * rangeResolution;
    }
    else
    {
        obj->detObj2D[objIndex].rangeIdx = (uint16_t) ((int32_t) obj->numRangeBins - (int32_t) obj->detObj2D[objIndex].rangeIdx);
        range = obj->detObj2D[objIndex].rangeIdx * -rangeResolution;
    }
#else
    range = obj->detObj2D[objIndex].rangeIdx * rangeResolution;
#endif

    /* Compensate for range bias */
    range -= obj->cliCommonCfg->compRxChanCfg.rangeBias;;
    if (range < 0)
    {
        range = 0;
    }

    /* Convert to Q8 format */
    obj->detObj2D[objIndex].x = 0;
    obj->detObj2D[objIndex].y = (int16_t) ROUND(range * ONE_QFORMAT);
    obj->detObj2D[objIndex].z = 0;
}
#endif

/**
 *  @b Description
 *  @n
 *      Waits for 1D FFT data to be transferred to input buffer.
 *      This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *  @param[in] pingPongId ping-pong id (ping is 0 and pong is 1)
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWait1DInputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId)
{
    MmwDemo_DSS_dataPathContext_t *context = obj->context;

#ifdef EDMA_1D_INPUT_BLOCKING
    Bool       status;

    status = Semaphore_pend(context->EDMA_1D_InputDone_semHandle[pingPongId], BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        System_printf("Error: Semaphore_pend returned %d\n",status);
    }
#else
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t chId;
    if(pingPongId == 0)
    {
        chId = MMW_EDMA_CH_1D_IN_PING;
    }
    else
    {
        chId = MMW_EDMA_CH_1D_IN_PONG;
    }
    do {
        if (EDMA_isTransferComplete(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
                                    chId,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
#endif
}

/**
 *  @b Description
 *  @n
 *      Waits for 1D FFT data to be transferred to output buffer.
 *      This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *  @param[in] pingPongId ping-pong id (ping is 0 and pong is 1)
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWait1DOutputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId)
{
    MmwDemo_DSS_dataPathContext_t *context = obj->context;

#ifdef EDMA_1D_OUTPUT_BLOCKING
    Bool       status;

    status = Semaphore_pend(context->EDMA_1D_OutputDone_semHandle[pingPongId], BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        System_printf("Error: Semaphore_pend returned %d\n",status);
    }
#else
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t chId;
    if(pingPongId == 0)
    {
        chId = MMW_EDMA_CH_1D_OUT_PING;
    }
    else
    {
        chId = MMW_EDMA_CH_1D_OUT_PONG;
    }
    do {
        if (EDMA_isTransferComplete(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
                                    chId,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
#endif
}

#if 0
/**
 *  @b Description
 *  @n
 *      Waits for 1D FFT data to be transferred to input buffer for 2D-FFT caclulation.
 *      This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *  @param[in] pingPongId ping-pong id (ping is 0 and pong is 1)
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWait2DInputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId)
{
    MmwDemo_DSS_dataPathContext_t *context = obj->context;

#ifdef EDMA_2D_INPUT_BLOCKING
    Bool       status;

    status = Semaphore_pend(context->EDMA_2D_InputDone_semHandle[pingPongId], BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        System_printf("Error: Semaphore_pend returned %d\n",status);
    }
#else
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t chId;
    if(pingPongId == 0)
    {
        chId = MMW_EDMA_CH_2D_IN_PING;
    }
    else
    {
        chId = MMW_EDMA_CH_2D_IN_PONG;
    }
    do {
        if (EDMA_isTransferComplete(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
                                    chId,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
#endif
}
/**
 *  @b Description
 *  @n
 *      Waits for 1D FFT data to be transferred to input buffer for 3D-FFT calculation.
 *      This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *  @param[in] pingPongId ping-pong id (ping is 0 and pong is 1)
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWait3DInputData(MmwDemo_DSS_DataPathObj *obj, uint32_t pingPongId)
{
    MmwDemo_DSS_dataPathContext_t *context = obj->context;

#ifdef EDMA_3D_INPUT_BLOCKING
    Bool       status;

    status = Semaphore_pend(context->EDMA_3D_InputDone_semHandle[pingPongId], BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        System_printf("Error: Semaphore_pend returned %d\n",status);
    }
#else
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t chId;
    if(pingPongId == 0)
    {
        chId = MMW_EDMA_CH_3D_IN_PING;
    }
    else
    {
        chId = MMW_EDMA_CH_3D_IN_PONG;
    }
    do {
        if (EDMA_isTransferComplete(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
                                    chId,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
#endif
}

/**
 *  @b Description
 *  @n
 *      Waits for 2D FFT calculated data to be transferred out from L2 memory
 *      to detection matrix located in L3 memory.
 *      This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWaitTransDetMatrix(MmwDemo_DSS_DataPathObj *obj)
{
    MmwDemo_DSS_dataPathContext_t *context = obj->context;

#ifdef EDMA_2D_OUTPUT_BLOCKING
    Bool       status;

    status = Semaphore_pend(context->EDMA_DetMatrix_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        System_printf("Error: Semaphore_pend returned %d\n",status);
    }
#else
    volatile bool isTransferDone;
    do {
        if (EDMA_isTransferComplete(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
                                    (uint8_t) MMW_EDMA_CH_DET_MATRIX,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
#endif
}

/**
 *  @b Description
 *  @n
 *      Waits for 2D FFT data to be transferred from detection matrix in L3
 *      memory to L2 memory for CFAR detection in range direction.
 *      This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWaitTransDetMatrix2(MmwDemo_DSS_DataPathObj *obj)
{
    MmwDemo_DSS_dataPathContext_t *context = obj->context;

#ifdef EDMA_MATRIX2_INPUT_BLOCKING
    Bool       status;

    status = Semaphore_pend(context->EDMA_DetMatrix2_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        System_printf("Error: Semaphore_pend returned %d\n",status);
    }
#else
    /* wait until transfer done */
    volatile bool isTransferDone;
    do {
        if (EDMA_isTransferComplete(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
                                    (uint8_t) MMW_EDMA_CH_DET_MATRIX2,
                                    (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            MmwDemo_dssAssert(0);
        }
    } while (isTransferDone == false);
#endif
}
#endif

/**
 *  @b Description
 *  @n
 *      EDMA transfer completion call back function as per EDMA API.
 *      Depending on the programmed transfer completion codes,
 *      posts the corresponding done/completion semaphore.
 *      Per current design, a single semaphore could have been used as the
 *      1D, 2D and CFAR stages are sequential, this code gives some flexibility
 *      if some design change in future.
 */
#if (defined(EDMA_1D_INPUT_BLOCKING) || defined(EDMA_1D_OUTPUT_BLOCKING) || \
     defined(EDMA_2D_INPUT_BLOCKING) || defined(EDMA_2D_OUTPUT_BLOCKING) || \
     defined(EDMA_MATRIX2_INPUT_BLOCKING) || defined(EDMA_3D_INPUT_BLOCKING))
void MmwDemo_EDMA_transferCompletionCallbackFxn(uintptr_t arg,
    uint8_t transferCompletionCode)
{
    MmwDemo_DSS_DataPathObj *obj = (MmwDemo_DSS_DataPathObj *)arg;
    MmwDemo_DSS_dataPathContext_t *context = obj->context;

    switch (transferCompletionCode)
    {
#ifdef EDMA_1D_INPUT_BLOCKING
        case MMW_EDMA_CH_1D_IN_PING:
            Semaphore_post(context->EDMA_1D_InputDone_semHandle[0]);
        break;

        case MMW_EDMA_CH_1D_IN_PONG:
            Semaphore_post(context->EDMA_1D_InputDone_semHandle[1]);
        break;
#endif

#ifdef EDMA_1D_OUTPUT_BLOCKING
        case MMW_EDMA_CH_1D_OUT_PING:
            Semaphore_post(context->EDMA_1D_OutputDone_semHandle[0]);
        break;

        case MMW_EDMA_CH_1D_OUT_PONG:
            Semaphore_post(context->EDMA_1D_OutputDone_semHandle[1]);
        break;
#endif

#ifdef EDMA_2D_INPUT_BLOCKING
        case MMW_EDMA_CH_2D_IN_PING:
            Semaphore_post(context->EDMA_2D_InputDone_semHandle[0]);
        break;

        case MMW_EDMA_CH_2D_IN_PONG:
            Semaphore_post(context->EDMA_2D_InputDone_semHandle[1]);
        break;
#endif

#ifdef EDMA_2D_OUTPUT_BLOCKING
        case MMW_EDMA_CH_DET_MATRIX:
            Semaphore_post(context->EDMA_DetMatrix_semHandle);
        break;
#endif

#ifdef EDMA_MATRIX2_INPUT_BLOCKING
        case MMW_EDMA_CH_DET_MATRIX2:
            Semaphore_post(context->EDMA_DetMatrix2_semHandle);
        break;
#endif

#ifdef EDMA_3D_INPUT_BLOCKING
        case MMW_EDMA_CH_3D_IN_PING:
            Semaphore_post(context->EDMA_3D_InputDone_semHandle[0]);
        break;

        case MMW_EDMA_CH_3D_IN_PONG:
            Semaphore_post(context->EDMA_3D_InputDone_semHandle[1]);
        break;
#endif
        default:
            MmwDemo_dssAssert(0);
        break;
    }
}
#endif


void MmwDemo_EDMA_CQTransferCompletionCallbackFxn(uintptr_t arg,
    uint8_t transferCompletionCode)
{
    MmwDemo_DSS_DataPathObj *obj = (MmwDemo_DSS_DataPathObj *)arg;
    uint8_t     chirpIdx;

    switch (transferCompletionCode)
    {
        /* Validate numSlices from data buffer for every chirp.
           Note1: If the configured number of slices covers more time period than the sampling time,
           the actual reported number of slices maybe smaller.

           Note 2: If there are extensive CQ data processing,
           then the processing code should be moved to a task
         */

        case MMW_EDMA_CH_SIGIMG_MON:
            obj->datapathCQ.sigImgEdmaCnt++;
            for(chirpIdx = 0; chirpIdx < obj->numChirpsPerChirpEvent; chirpIdx++)
            {
                uint8_t *sigImgData = obj->datapathCQ.sigImgData;

                if(sigImgData[chirpIdx * obj->datapathCQ.sigImgMonDataSizePerChirp] > obj->datapathCQ.sigImgMonCfg->numSlices)
                {
                    obj->datapathCQ.sigImgErrCnt++;
                    MmwDemo_dssAssert(0);
                }
            }
        break;

        case MMW_EDMA_CH_RX_SATURATION_MON:
            obj->datapathCQ.rxSatEdmaCnt++;
            for(chirpIdx = 0; chirpIdx < obj->numChirpsPerChirpEvent; chirpIdx++)
            {
                uint8_t *rxSatData = obj->datapathCQ.rxSatData;
                if(rxSatData[chirpIdx * obj->datapathCQ.satMonDataSizePerChirp] > obj->datapathCQ.rxSatMonCfg->numSlices)
                {
                    obj->datapathCQ.rxSatErrCnt++;
                    MmwDemo_dssAssert(0);
                }
            }
        break;

        default:
            MmwDemo_dssAssert(0);
        break;
    }
}

/**
 *  @b Description
 *  @n
 *      Configures all EDMA channels and param sets used in data path processing
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      -1 if error, 0 for no error
 */
int32_t MmwDemo_dataPathConfigCQEdma(MmwDemo_DSS_DataPathObj *obj)
{
    uint32_t eventQueue;
    int32_t retVal = 0;
    MmwDemo_DSS_dataPathContext_t *context = obj->context;

    /*****************************************************
     * EDMA configuration for getting CQ data from CQ buffer
     * to Datapath CQ storage
     *****************************************************/
    eventQueue = 0U;
    if (obj->datapathCQ.anaMonCfg->sigImgMonEn)
    {
        retVal = EDMAutil_configType1(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
            (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.sigImgMonAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
            (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.sigImgData,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
            MMW_EDMA_CH_SIGIMG_MON,
            false,
            MMW_EDMA_CH_SIGIMG_MON,
            obj->datapathCQ.sigImgMonTotalSize,
            1,
            0,
            0,
            eventQueue,
            MmwDemo_EDMA_CQTransferCompletionCallbackFxn,
            (uintptr_t) obj);
        if (retVal < 0)
        {
            return -1;
        }
    }

    if (obj->datapathCQ.anaMonCfg->rxSatMonEn)
    {
        retVal = EDMAutil_configType1(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
            (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.satMonAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
            (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.rxSatData,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
            MMW_EDMA_CH_RX_SATURATION_MON,
            false,
            MMW_EDMA_CH_RX_SATURATION_MON,
            obj->datapathCQ.satMonTotalSize,
            1,
            0,
            0,
            eventQueue,
            MmwDemo_EDMA_CQTransferCompletionCallbackFxn,
            (uintptr_t) obj);
        if (retVal < 0)
        {
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to configure CQ.
 *  @param[in] ptrDataPathObj Pointer to data path object.
 *
 *  @retval
 *      0 if no error, else error (there will be system prints for these).
 */
void MmwDemo_dssDataPathStartCQEdma(MmwDemo_DSS_DataPathObj *ptrDataPathObj)
{
    MmwDemo_DSS_dataPathContext_t *context = ptrDataPathObj->context;

    /* Manually start EDMA transfer */
    if (ptrDataPathObj->datapathCQ.anaMonCfg->sigImgMonEn)
    {
        /* Kick off DMA to fetch data from CQ1 buffer */
        EDMA_startDmaTransfer(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
                           MMW_EDMA_CH_SIGIMG_MON);
    }

    if (ptrDataPathObj->datapathCQ.anaMonCfg->rxSatMonEn)
    {
        /* Kick off DMA to fetch data from CQ2 buffer */
        EDMA_startDmaTransfer(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
                           MMW_EDMA_CH_RX_SATURATION_MON);
    }
}

/**
 *  @b Description
 *  @n
 *      Configures all EDMA channels and param sets used in data path processing
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      -1 if error, 0 for no error
 */
int32_t MmwDemo_dataPathConfigEdma(MmwDemo_DSS_DataPathObj *obj)
{
    uint32_t eventQueue;
    uint16_t shadowParam = EDMA_NUM_DMA_CHANNELS;
    int32_t  retVal = 0;
    uint16_t numPingOrPongSamples, aCount;
    int16_t oneD_destinationBindex;
    MmwDemo_DSS_dataPathContext_t *context = obj->context;
    uint8_t *oneD_destinationPongAddress;
    //MmwDemo_AnaMonitorCfg*      ptrAnaMonitorCfg;

    /*****************************************************
     * EDMA configuration for getting ADC data from ADC buffer
     * to L2 (prior to 1D FFT)
     * For ADC Buffer to L2 use EDMA-A TPTC =1
     *****************************************************/
    eventQueue = 0U;
     /* Ping - copies chirp samples from even antenna numbers (e.g. RxAnt0 and RxAnt2) */
    retVal =
    EDMAutil_configType1(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
        (uint8_t *)(&obj->ADCdataBuf[0]),
        (uint8_t *)(SOC_translateAddress((uint32_t)&obj->adcDataIn[0],SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        MMW_EDMA_CH_1D_IN_PING,
        false,
        shadowParam++,
        obj->numAdcSamples * BYTES_PER_SAMP_1D,
        MAX(obj->numRxAntennas / 2, 1) * obj->numChirpsPerChirpEvent,
        (obj->numAdcSamples * BYTES_PER_SAMP_1D * 2) * obj->numChirpsPerChirpEvent,
        0,
        eventQueue,
#ifdef EDMA_1D_INPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }

    /* Pong - copies chirp samples from odd antenna numbers (e.g. RxAnt1 and RxAnt3) */
    retVal =
    EDMAutil_configType1(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
        (uint8_t *)(&obj->ADCdataBuf[obj->numAdcSamples * obj->numChirpsPerChirpEvent]),
        (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->adcDataIn[obj->numRangeBins]),SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        MMW_EDMA_CH_1D_IN_PONG,
        false,
        shadowParam++,
        obj->numAdcSamples * BYTES_PER_SAMP_1D,
        MAX(obj->numRxAntennas / 2, 1) * obj->numChirpsPerChirpEvent,
        (obj->numAdcSamples * BYTES_PER_SAMP_1D * 2) * obj->numChirpsPerChirpEvent,
        0,
        eventQueue,
#ifdef EDMA_1D_INPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }


    //Vital Signs Demo: Uses EMDA format implemented in SDK 2.0
    eventQueue = 1U;
    /*****************************************************
     * EDMA configuration for storing 1d fft output to L3.
     * It copies all Rx antennas of the chirp per trigger event.
     *****************************************************/
    numPingOrPongSamples = obj->numRangeBins * obj->numRxAntennas;
    aCount = numPingOrPongSamples * BYTES_PER_SAMP_1D;

    /* If TDM-MIMO (BPM or otherwise), store odd chirps consecutively and even
       chirps consecutively. This is done because for the case of 1024 range bins
       and 4 rx antennas, the source jump required for 2D processing will be 32768
       which is negative jump for the EDMA (16-bit signed jump). Storing in this way
       reduces the jump to be positive which makes 2D processing feasible */
    if (obj->numTxAntennas == 2)
    {
        oneD_destinationBindex = (int16_t)aCount;
        oneD_destinationPongAddress = (uint8_t *)(&obj->radarCube[numPingOrPongSamples * obj->numDopplerBins]);
    }
    else
    {
        oneD_destinationBindex = (int16_t)(aCount * 2);
        oneD_destinationPongAddress = (uint8_t *)(&obj->radarCube[numPingOrPongSamples]);
    }

    /* Ping - Copies from ping FFT output (even chirp indices)  to L3 */
    retVal =
    EDMAutil_configType1(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
        (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->fftOut1D[0]),SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        (uint8_t *)(&obj->radarCube[0]),
        MMW_EDMA_CH_1D_OUT_PING,
        false,
        shadowParam++,
        aCount,
        obj->numChirpsPerFrame / 2, //bCount
        0, //srcBidx
        oneD_destinationBindex, //dstBidx
        eventQueue,
#ifdef EDMA_1D_OUTPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);

    if (retVal < 0)
    {
        return -1;
    }

    /* Pong - Copies from pong FFT output (odd chirp indices)  to L3 */
    retVal =
    EDMAutil_configType1(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
        (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->fftOut1D[numPingOrPongSamples]),
                                         SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        oneD_destinationPongAddress,
        MMW_EDMA_CH_1D_OUT_PONG,
        false,
        shadowParam++,
        aCount,
        obj->numChirpsPerFrame / 2, //bCount
        0, //srcBidx
        oneD_destinationBindex, //dstBidx
        eventQueue,
#ifdef EDMA_1D_OUTPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);

    if (retVal < 0)
    {
        return -1;
    }




#if 0 //Vital Signs Demo: Remove EDMA transfers after 1D process

    /*****************************************
     * Interframe processing related EDMA configuration
     *****************************************/
    eventQueue = 0U;
    if (obj->numTxAntennas == 2)
    {
        twoD_sourcePongAddress = oneD_destinationPongAddress;
    }
    else
    {
        twoD_sourcePongAddress = (uint8_t *)(&obj->radarCube[obj->numRangeBins]);
    }
    /* Ping: This DMA channel is programmed to fetch the 1D FFT data from radarCube
     * matrix in L3 mem of even antenna rows into the Ping Buffer in L2 mem*/
    retVal =
        EDMAutil_configType2b(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
        (uint8_t *)(&obj->radarCube[0]),
        (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->dstPingPong[0]),
                        SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
        MMW_EDMA_CH_2D_IN_PING,
        false,
        MMW_EDMA_CH_2D_IN_PING_SHADOW,
        BYTES_PER_SAMP_1D,
        obj->numRangeBins,
        obj->numTxAntennas,
        obj->numRxAntennas,
        obj->numDopplerBins,
        eventQueue,
#ifdef EDMA_2D_INPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }

    /* Pong: This DMA channel is programmed to fetch the 1D FFT data from radarCube
     * matrix in L3 mem of odd antenna rows into thePong Buffer in L2 mem*/
    retVal =
        EDMAutil_configType2b(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
        twoD_sourcePongAddress,
        (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->dstPingPong[obj->numDopplerBins]),
                        SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
        MMW_EDMA_CH_2D_IN_PONG,
        false,
        MMW_EDMA_CH_2D_IN_PONG_SHADOW,
        BYTES_PER_SAMP_1D,
        obj->numRangeBins,
        obj->numTxAntennas,
        obj->numRxAntennas,
        obj->numDopplerBins,
        eventQueue,
#ifdef EDMA_2D_INPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }

    eventQueue = 1U;
    /* This EDMA channel copes the sum (across virtual antennas) of log2
     * magnitude squared of Doppler FFT bins from L2 mem to detection
     * matrix in L3 mem */
    retVal =
    EDMAutil_configType1(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
        (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->sumAbs[0U]),SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        (uint8_t *)(obj->detMatrix),
        MMW_EDMA_CH_DET_MATRIX,
        false,
        MMW_EDMA_CH_DET_MATRIX_SHADOW,
        obj->numDopplerBins * BYTES_PER_SAMP_DET,
        obj->numRangeBins,
        0,
        obj->numDopplerBins * BYTES_PER_SAMP_DET,
        eventQueue,
#ifdef EDMA_2D_OUTPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }

    /* This EDMA Channel brings selected range bins  from detection matrix in
     * L3 mem (reading in transposed manner) into L2 mem for CFAR detection (in
     * range direction) */
    retVal =
    EDMAutil_configType3(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
        (uint8_t *)0,
        (uint8_t *)0,
        MMW_EDMA_CH_DET_MATRIX2,
        false,
        MMW_EDMA_CH_DET_MATRIX2_SHADOW,
        BYTES_PER_SAMP_DET,\
        obj->numRangeBins,
        obj->numDopplerBins * BYTES_PER_SAMP_DET,
        BYTES_PER_SAMP_DET,
        eventQueue,
#ifdef EDMA_MATRIX2_INPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }

    /*********************************************************
     * These EDMA Channels are for Azimuth calculation. They bring
     * 1D FFT data for 2D DFT and Azimuth FFT calculation.
     ********************************************************/
    /* Ping: This DMA channel is programmed to fetch the 1D FFT data from radarCube
     * matrix in L3 mem of even antenna rows into the Ping Buffer in L2 mem.
     * */
    retVal =
        EDMAutil_configType2b(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
        (uint8_t *) NULL,
        (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->dstPingPong[0]),
                        SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
        MMW_EDMA_CH_3D_IN_PING,
        false,
        MMW_EDMA_CH_3D_IN_PING_SHADOW,
        BYTES_PER_SAMP_1D,
        obj->numRangeBins,
        obj->numTxAntennas,
        obj->numRxAntennas,
        obj->numDopplerBins,
        eventQueue,
#ifdef EDMA_3D_INPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }

    /* Pong: This DMA channel is programmed to fetch the 1D FFT data from radarCube
     * matrix in L3 mem of odd antenna rows into the Pong Buffer in L2 mem*/
    retVal =
        EDMAutil_configType2b(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
        (uint8_t *) NULL,
        (uint8_t *)(SOC_translateAddress((uint32_t)(&obj->dstPingPong[obj->numDopplerBins]),
                        SOC_TranslateAddr_Dir_TO_EDMA, NULL)),
        MMW_EDMA_CH_3D_IN_PONG,
        false,
        MMW_EDMA_CH_3D_IN_PONG_SHADOW,
        BYTES_PER_SAMP_1D,
        obj->numRangeBins,
        obj->numTxAntennas,
        obj->numRxAntennas,
        obj->numDopplerBins,
        eventQueue,
#ifdef EDMA_3D_INPUT_BLOCKING
        MmwDemo_EDMA_transferCompletionCallbackFxn,
#else
        NULL,
#endif
        (uintptr_t) obj);
    if (retVal < 0)
    {
        return -1;
    }

    /* Get analog monitor configuration */
    ptrAnaMonitorCfg = &obj->cliCommonCfg->anaMonCfg;

    if ((ptrAnaMonitorCfg->rxSatMonEn) || (ptrAnaMonitorCfg->sigImgMonEn))
    {
        MmwDemo_dataPathConfigCQEdma(obj);
    }
#endif

    return(0);
}


#if 0 //Vital Signs Demo: Remove functions that are not needed
/**
 *  @b Description
 *  @n
 *    The function groups neighboring peaks into one. The grouping is done
 *    according to two input flags: groupInDopplerDirection and
 *    groupInDopplerDirection. For each detected peak the function checks
 *    if the peak is greater than its neighbors. If this is true, the peak is
 *    copied to the output list of detected objects. The neighboring peaks that are used
 *    for checking are taken from the detection matrix and copied into 3x3 kernel
 *    regardless of whether they are CFAR detected or not.
 *    Note: Function always reads 9 samples per detected object
 *    from L3 memory into local array tempBuff, but it only needs to read according to input flags.
 *    For example if only the groupInDopplerDirection flag is set, it only needs
 *    to read middle row of the kernel, i.e. 3 samples per target from detection matrix.
 *  @param[out]   objOut             Output array of  detected objects after peak grouping
 *  @param[in]    objRaw             Array of detected objects after CFAR detection
 *  @param[in]    numDetectedObjects Number of detected objects by CFAR
 *  @param[in]    detMatrix          Detection Range/Doppler matrix
 *  @param[in]    numDopplerBins     Number of Doppler bins
 *  @param[in]    maxRangeIdx        Maximum range limit
 *  @param[in]    minRangeIdx        Minimum range limit
 *  @param[in]    groupInDopplerDirection Flag enables grouping in Doppler directiuon
 *  @param[in]    groupInRangeDirection   Flag enables grouping in Range directiuon
 *
 *  @retval
 *      Number of detected objects after grouping
 */
uint32_t MmwDemo_cfarPeakGrouping(
                                MmwDemo_detectedObj*  objOut,
                                MmwDemo_objRaw_t * objRaw,
                                uint32_t numDetectedObjects,
                                uint16_t* detMatrix,
                                uint32_t numDopplerBins,
                                uint32_t maxRangeIdx,
                                uint32_t minRangeIdx,
                                uint32_t groupInDopplerDirection,
                                uint32_t groupInRangeDirection)
{
    int32_t i, j;
    int32_t rowStart, rowEnd;
    uint32_t numObjOut = 0;
    uint32_t rangeIdx, dopplerIdx, peakVal;
    uint16_t *tempPtr;
    uint16_t kernel[9], detectedObjFlag;
    int32_t k, l;
    uint32_t startInd, stepInd, endInd;

    if ((groupInDopplerDirection == 1) && (groupInRangeDirection == 1))
    {
        /* Grouping both in Range and Doppler direction */
        startInd = 0;
        stepInd = 1;
        endInd = 8;
    }
    else if ((groupInDopplerDirection == 0) && (groupInRangeDirection == 1))
    {
        /* Grouping only in Range direction */
        startInd = 1;
        stepInd = 3;
        endInd = 7;
    }
    else if ((groupInDopplerDirection == 1) && (groupInRangeDirection == 0))
    {
        /* Grouping only in Doppler direction */
        startInd = 3;
        stepInd = 1;
        endInd = 5;
    }
    else
    {
        /* No grouping, copy all detected objects to the output matrix within specified min max range*/
        for(i = 0; i < MIN(numDetectedObjects, MMW_MAX_OBJ_OUT) ; i++)
        {
            if ((objRaw[i].rangeIdx <= maxRangeIdx) && ((objRaw[i].rangeIdx >= minRangeIdx)))
            {
                objOut[numObjOut].rangeIdx = objRaw[i].rangeIdx;
                objOut[numObjOut].dopplerIdx = DOPPLER_IDX_TO_SIGNED(objRaw[i].dopplerIdx, numDopplerBins);
                objOut[numObjOut].peakVal = objRaw[i].peakVal;
                numObjOut++;
            }
        }
        return numObjOut;
    }

    /* Start checking */
    for(i = 0; i < numDetectedObjects; i++)
    {
        detectedObjFlag = 0;
        rangeIdx = objRaw[i].rangeIdx;
        dopplerIdx = objRaw[i].dopplerIdx;
        peakVal = objRaw[i].peakVal;
        if((rangeIdx <= maxRangeIdx) && (rangeIdx >= minRangeIdx))
        {
            detectedObjFlag = 1;

            /* Fill local 3x3 kernel from detection matrix in L3*/
            tempPtr = detMatrix + (rangeIdx-1)*numDopplerBins;
            rowStart = 0;
            rowEnd = 2;
            if (rangeIdx == minRangeIdx)
            {
                tempPtr = detMatrix + (rangeIdx)*numDopplerBins;
                rowStart = 1;
                memset((void *) kernel, 0, 3 * sizeof(uint16_t));
            }
            else if (rangeIdx == maxRangeIdx)
            {
                rowEnd = 1;
                memset((void *) &kernel[6], 0, 3 * sizeof(uint16_t));
            }

            for (j = rowStart; j <= rowEnd; j++)
            {
                for (k = 0; k < 3; k++)
                {
                    l = dopplerIdx + (k - 1);
                    if(l < 0)
                    {
                        l += numDopplerBins;
                    }
                    else if(l >= numDopplerBins)
                    {
                        l -= numDopplerBins;
                    }
                    kernel[j*3+k] = tempPtr[l];
                }
                tempPtr += numDopplerBins;
            }
            /* Compare the detected object to its neighbors.
             * Detected object is at index 4*/
            for (k = startInd; k <= endInd; k += stepInd)
            {
                if(kernel[k] > kernel[4])
                {
                    detectedObjFlag = 0;
                }
            }
        }
        if (detectedObjFlag == 1)
        {
            objOut[numObjOut].rangeIdx = rangeIdx;
            objOut[numObjOut].dopplerIdx = DOPPLER_IDX_TO_SIGNED(dopplerIdx, numDopplerBins);
            objOut[numObjOut].peakVal = peakVal;
            numObjOut++;
        }
        if (numObjOut >= MMW_MAX_OBJ_OUT)
        {
            break;
        }

    }

    return(numObjOut);
}


/**
 *  @b Description
 *  @n
 *    The function groups neighboring peaks into one. The grouping is done
 *    according to two input flags: groupInDopplerDirection and
 *    groupInDopplerDirection. For each detected peak the function checks
 *    if the peak is greater than its neighbors. If this is true, the peak is
 *    copied to the output list of detected objects. The neighboring peaks that
 *    are used for checking are taken from the list of CFAR detected objects,
 *    (not from the detection matrix), and copied into 3x3 kernel that has been
 *    initialized to zero for each peak under test. If the neighboring cell has
 *    not been detected by CFAR, its peak value is not copied into the kernel.
 *    Note: Function always search for 8 peaks in the list, but it only needs to search
 *    according to input flags.
 *  @param[out]   objOut             Output array of  detected objects after peak grouping
 *  @param[in]    objRaw             Array of detected objects after CFAR detection
 *  @param[in]    numDetectedObjects Number of detected objects by CFAR
 *  @param[in]    numDopplerBins     Number of Doppler bins
 *  @param[in]    maxRangeIdx        Maximum range limit
 *  @param[in]    minRangeIdx        Minimum range limit
 *  @param[in]    groupInDopplerDirection Flag enables grouping in Doppler directiuon
 *  @param[in]    groupInRangeDirection   Flag enables grouping in Range directiuon
 *
 *  @retval
 *      Number of detected objects after grouping
 */
uint32_t MmwDemo_cfarPeakGroupingCfarQualified(
                                MmwDemo_detectedObj*  objOut,
                                MmwDemo_objRaw_t * objRaw,
                                uint32_t numDetectedObjects,
                                uint32_t numDopplerBins,
                                uint32_t maxRangeIdx,
                                uint32_t minRangeIdx,
                                uint32_t groupInDopplerDirection,
                                uint32_t groupInRangeDirection)
{
    int32_t i;
    uint32_t numObjOut = 0;
    uint32_t rangeIdx, dopplerIdx, peakVal;
    uint16_t kernel[9], detectedObjFlag;
    int32_t k;
    int32_t l;
    uint32_t startInd, stepInd, endInd;

#define WRAP_DOPPLER_IDX(_x) ((_x) & (numDopplerBins-1))
#define WRAP_DWN_LIST_IDX(_x) ((_x) >= numDetectedObjects ? ((_x) - numDetectedObjects) : (_x))
#define WRAP_UP_LIST_IDX(_x) ((_x) < 0 ? ((_x) + numDetectedObjects) : (_x))

    if ((groupInDopplerDirection == 1) && (groupInRangeDirection == 1))
    {
        /* Grouping both in Range and Doppler direction */
        startInd = 0;
        stepInd = 1;
        endInd = 8;
    }
    else if ((groupInDopplerDirection == 0) && (groupInRangeDirection == 1))
    {
        /* Grouping only in Range direction */
        startInd = 1;
        stepInd = 3;
        endInd = 7;
    }
    else if ((groupInDopplerDirection == 1) && (groupInRangeDirection == 0))
    {
        /* Grouping only in Doppler direction */
        startInd = 3;
        stepInd = 1;
        endInd = 5;
    }
    else
    {
        /* No grouping, copy all detected objects to the output matrix */
        for(i = 0; i < MIN(numDetectedObjects, MMW_MAX_OBJ_OUT) ; i++)
        {
            if ((objRaw[i].rangeIdx <= maxRangeIdx) && ((objRaw[i].rangeIdx >= minRangeIdx)))
            {
                objOut[numObjOut].rangeIdx = objRaw[i].rangeIdx;;
                objOut[numObjOut].dopplerIdx = DOPPLER_IDX_TO_SIGNED(objRaw[i].dopplerIdx, numDopplerBins);
                objOut[numObjOut].peakVal = objRaw[i].peakVal;
                numObjOut++;
            }
        }
        return numObjOut;
    }

    /* Start checking  */
    for(i = 0; i < numDetectedObjects ; i++)
    {
        detectedObjFlag = 0;
        rangeIdx = objRaw[i].rangeIdx;
        dopplerIdx = objRaw[i].dopplerIdx;
        peakVal = objRaw[i].peakVal;

        if((rangeIdx <= maxRangeIdx) && (rangeIdx >= minRangeIdx))
        {
            detectedObjFlag = 1;
            memset((void *) kernel, 0, 9*sizeof(uint16_t));

            /* Fill the middle column of the kernel */
            kernel[4] = peakVal;

            if (i > 0)
            {
                if ((objRaw[i-1].rangeIdx == (rangeIdx-1)) &&
                    (objRaw[i-1].dopplerIdx == dopplerIdx))
                {
                    kernel[1] = objRaw[i-1].peakVal;
                }
            }

            if (i < (numDetectedObjects - 1))
            {
                if ((objRaw[i+1].rangeIdx == (rangeIdx+1)) &&
                    (objRaw[i+1].dopplerIdx == dopplerIdx))
                {
                    kernel[7] = objRaw[i+1].peakVal;
                }
            }

            /* Fill the left column of the kernel */
            k = WRAP_UP_LIST_IDX(i-1);
            for (l = 0; l < numDetectedObjects; l++)
            {
                if (objRaw[k].dopplerIdx == WRAP_DOPPLER_IDX(dopplerIdx - 2))
                {
                    break;
                }
                if ((objRaw[k].rangeIdx == (rangeIdx + 1)) &&
                    (objRaw[k].dopplerIdx == WRAP_DOPPLER_IDX(dopplerIdx - 1)))
                {
                    kernel[6] = objRaw[k].peakVal;
                }
                else if ((objRaw[k].rangeIdx == (rangeIdx)) &&
                    (objRaw[k].dopplerIdx == WRAP_DOPPLER_IDX(dopplerIdx - 1)))
                {
                    kernel[3] = objRaw[k].peakVal;
                }
                else if ((objRaw[k].rangeIdx == (rangeIdx - 1)) &&
                    (objRaw[k].dopplerIdx == WRAP_DOPPLER_IDX(dopplerIdx - 1)))
                {
                    kernel[0] = objRaw[k].peakVal;
                }
                k = WRAP_UP_LIST_IDX(k-1);
            }

            /* Fill the right column of the kernel */
            k = WRAP_DWN_LIST_IDX(i+1);
            for (l = 0; l < numDetectedObjects; l++)
            {
                if (objRaw[k].dopplerIdx == WRAP_DOPPLER_IDX(dopplerIdx + 2))
                {
                    break;
                }
                if ((objRaw[k].rangeIdx == (rangeIdx - 1)) &&
                    (objRaw[k].dopplerIdx == WRAP_DOPPLER_IDX(dopplerIdx + 1)))
                {
                    kernel[2] = objRaw[k].peakVal;
                }
                else if ((objRaw[k].rangeIdx == (rangeIdx)) &&
                    (objRaw[k].dopplerIdx == WRAP_DOPPLER_IDX(dopplerIdx + 1)))
                {
                    kernel[5] = objRaw[k].peakVal;
                }
                else if ((objRaw[k].rangeIdx == (rangeIdx + 1)) &&
                    (objRaw[k].dopplerIdx == WRAP_DOPPLER_IDX(dopplerIdx + 1)))
                {
                    kernel[8] = objRaw[k].peakVal;
                }
                k = WRAP_DWN_LIST_IDX(k+1);
            }

            /* Compare the detected object to its neighbors.
             * Detected object is at index 4*/
            for (k = startInd; k <= endInd; k += stepInd)
            {
                if(kernel[k] > kernel[4])
                {
                    detectedObjFlag = 0;
                }
            }
        }
        if(detectedObjFlag == 1)
        {
            objOut[numObjOut].rangeIdx = rangeIdx;
            objOut[numObjOut].dopplerIdx = DOPPLER_IDX_TO_SIGNED(dopplerIdx, numDopplerBins);
            objOut[numObjOut].peakVal = peakVal;
            numObjOut++;
        }
        if (numObjOut >= MMW_MAX_OBJ_OUT)
        {
            break;
        }
    }

    return(numObjOut);
}
#endif

/**
 *  @b Description
 *  @n
 *    Outputs magnitude squared float array of input complex32 array
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_magnitudeSquared(cmplx32ReIm_t * restrict inpBuff, float * restrict magSqrdBuff, uint32_t numSamples)
{
    uint32_t i;
    for (i = 0; i < numSamples; i++)
    {
        magSqrdBuff[i] = (float) inpBuff[i].real * (float) inpBuff[i].real +
                (float) inpBuff[i].imag * (float) inpBuff[i].imag;
    }
}


#define pingPongId(x) ((x) & 0x1U)
#define isPong(x) (pingPongId(x))

/**
 *  @b Description
 *  @n
 *    Compensation of DC range antenna signature
 *
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dcRangeSignatureCompensation(MmwDemo_DSS_DataPathObj *obj,  uint8_t chirpPingPongId)
{
    uint32_t rxAntIdx, binIdx;
    uint32_t ind;
    int32_t chirpPingPongOffs;
    int32_t chirpPingPongSize;
    MmwDemo_CalibDcRangeSigCfg *calibDcCfg = &obj->cliCfg->calibDcRangeSigCfg;

    chirpPingPongSize = obj->numRxAntennas * (calibDcCfg->positiveBinIdx - calibDcCfg->negativeBinIdx + 1);
    if (obj->dcRangeSigCalibCntr == 0)
    {
        memset(obj->dcRangeSigMean, 0, obj->numTxAntennas * chirpPingPongSize * sizeof(cmplx32ImRe_t));
    }

    chirpPingPongOffs = chirpPingPongId * chirpPingPongSize;

    /* Calibration */
    if (obj->dcRangeSigCalibCntr < (calibDcCfg->numAvgChirps * obj->numTxAntennas))
    {
        /* Accumulate */
        ind = 0;
        for (rxAntIdx  = 0; rxAntIdx < obj->numRxAntennas; rxAntIdx++)
        {
            uint32_t chirpInOffs = chirpPingPongId * (obj->numRxAntennas * obj->numRangeBins) +
                                  (obj->numRangeBins * rxAntIdx);
            int64_t *meanPtr = (int64_t *) &obj->dcRangeSigMean[chirpPingPongOffs];
            uint32_t *fftPtr =  (uint32_t *) &obj->fftOut1D[chirpInOffs];
            int64_t meanBin;
            uint32_t fftBin;
            int32_t Re, Im;
            for (binIdx = 0; binIdx <= calibDcCfg->positiveBinIdx; binIdx++)
            {
                meanBin = _amem8(&meanPtr[ind]);
                fftBin = _amem4(&fftPtr[binIdx]);
                Im = _loll(meanBin) + _ext(fftBin, 0, 16);
                Re = _hill(meanBin) + _ext(fftBin, 16, 16);
                _amem8(&meanPtr[ind]) = _itoll(Re, Im);
                ind++;
            }

            chirpInOffs = chirpPingPongId * (obj->numRxAntennas * obj->numRangeBins) +
                (obj->numRangeBins * rxAntIdx) + obj->numRangeBins + calibDcCfg->negativeBinIdx;
            fftPtr =  (uint32_t *) &obj->fftOut1D[chirpInOffs];
            for (binIdx = 0; binIdx < -calibDcCfg->negativeBinIdx; binIdx++)
            {
                meanBin = _amem8(&meanPtr[ind]);
                fftBin = _amem4(&fftPtr[binIdx]);
                Im = _loll(meanBin) + _ext(fftBin, 0, 16);
                Re = _hill(meanBin) + _ext(fftBin, 16, 16);
                _amem8(&meanPtr[ind]) = _itoll(Re, Im);
                ind++;
            }
        }
        obj->dcRangeSigCalibCntr++;

        if (obj->dcRangeSigCalibCntr == (calibDcCfg->numAvgChirps * obj->numTxAntennas))
        {
            /* Divide */
            int64_t *meanPtr = (int64_t *) obj->dcRangeSigMean;
            int32_t Re, Im;
            int64_t meanBin;
            int32_t divShift = obj->log2NumAvgChirps;
            for (ind  = 0; ind < (obj->numTxAntennas * chirpPingPongSize); ind++)
            {
                meanBin = _amem8(&meanPtr[ind]);
                Im = _sshvr(_loll(meanBin), divShift);
                Re = _sshvr(_hill(meanBin), divShift);
                _amem8(&meanPtr[ind]) = _itoll(Re, Im);
            }
        }
    }
    else
    {
       /* fftOut1D -= dcRangeSigMean */
        ind = 0;
        for (rxAntIdx  = 0; rxAntIdx < obj->numRxAntennas; rxAntIdx++)
        {
            uint32_t chirpInOffs = chirpPingPongId * (obj->numRxAntennas * obj->numRangeBins) +
                                   (obj->numRangeBins * rxAntIdx);
            int64_t *meanPtr = (int64_t *) &obj->dcRangeSigMean[chirpPingPongOffs];
            uint32_t *fftPtr =  (uint32_t *) &obj->fftOut1D[chirpInOffs];
            int64_t meanBin;
            uint32_t fftBin;
            int32_t Re, Im;
            for (binIdx = 0; binIdx < calibDcCfg->positiveBinIdx; binIdx++)
            {
                meanBin = _amem8(&meanPtr[ind]);
                fftBin = _amem4(&fftPtr[binIdx]);
                Im = _ext(fftBin, 0, 16) - _loll(meanBin);
                Re = _ext(fftBin, 16, 16) - _hill(meanBin);
                _amem4(&fftPtr[binIdx]) = _pack2(Im, Re);
                ind++;
            }

            chirpInOffs = chirpPingPongId * (obj->numRxAntennas * obj->numRangeBins) +
                (obj->numRangeBins * rxAntIdx) + obj->numRangeBins + calibDcCfg->negativeBinIdx;
            fftPtr =  (uint32_t *) &obj->fftOut1D[chirpInOffs];
            for (binIdx = 0; binIdx < -calibDcCfg->negativeBinIdx; binIdx++)
            {
                meanBin = _amem8(&meanPtr[ind]);
                fftBin = _amem4(&fftPtr[binIdx]);
                Im = _ext(fftBin, 0, 16) - _loll(meanBin);
                Re = _ext(fftBin, 16, 16) - _hill(meanBin);
                _amem4(&fftPtr[binIdx]) = _pack2(Im, Re);
                //_amem4(&fftPtr[binIdx]) = _packh2(_sshvl(Im,16) , _sshvl(Re, 16));
                ind++;
            }
        }
    }
}


/**
 *  @b Description
 *  @n
 *    Interchirp processing. It is executed per chirp event, after ADC
 *    buffer is filled with chirp samples.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_interChirpProcessing(MmwDemo_DSS_DataPathObj *obj, uint8_t chirpPingPongId)
{
    uint32_t antIndx, waitingTime;
    volatile uint32_t startTime;
    volatile uint32_t startTime1;
    MmwDemo_DSS_dataPathContext_t *context = obj->context;

    waitingTime = 0;
    startTime = Cycleprofiler_getTimeStamp();

    /* Kick off DMA to fetch data from ADC buffer for first channel */
    EDMA_startDmaTransfer(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
                       MMW_EDMA_CH_1D_IN_PING);

    /* 1d fft for first antenna, followed by kicking off the DMA of fft output */
    for (antIndx = 0; antIndx < obj->numRxAntennas; antIndx++)
    {
        /* kick off DMA to fetch data for next antenna */
        if (antIndx < (obj->numRxAntennas - 1))
        {
            if (isPong(antIndx))
            {
                EDMA_startDmaTransfer(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
                        MMW_EDMA_CH_1D_IN_PING);
            }
            else
            {
                EDMA_startDmaTransfer(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE],
                        MMW_EDMA_CH_1D_IN_PONG);
            }
        }

        /* verify if DMA has completed for current antenna */
        startTime1 = Cycleprofiler_getTimeStamp();
        MmwDemo_dataPathWait1DInputData (obj, pingPongId(antIndx));
        waitingTime += Cycleprofiler_getTimeStamp() - startTime1;

        mmwavelib_windowing16x16(
                (int16_t *) &obj->adcDataIn[pingPongId(antIndx) * obj->numRangeBins],
                (int16_t *) obj->window1D,
                obj->numAdcSamples);
        memset((void *)&obj->adcDataIn[pingPongId(antIndx) * obj->numRangeBins + obj->numAdcSamples],
            0 , (obj->numRangeBins - obj->numAdcSamples) * sizeof(cmplx16ReIm_t));

        DSP_fft16x16(
                (int16_t *) obj->twiddle16x16_1D,
                obj->numRangeBins,
                (int16_t *) &obj->adcDataIn[pingPongId(antIndx) * obj->numRangeBins],
                (int16_t *) &obj->fftOut1D[chirpPingPongId * (obj->numRxAntennas * obj->numRangeBins) +
                    (obj->numRangeBins * antIndx)]);

    }

    if(obj->cliCfg->calibDcRangeSigCfg.enabled)
    {
        MmwDemo_dcRangeSignatureCompensation(obj, chirpPingPongId);
    }

    gCycleLog.interChirpProcessingTime += Cycleprofiler_getTimeStamp() - startTime - waitingTime;
    gCycleLog.interChirpWaitTime += waitingTime;
}

#if 0
/*!*****************************************************************************************************************
 * \brief
 * Function Name       :    MmwDemo_DopplerCompensation
 *
 * \par
 * <b>Description</b>  : Compensation of Doppler phase shift in the virtual antennas,
 *                       (corresponding to second Tx antenna chirps). Symbols
 *                       corresponding to virtual antennas, are rotated by half
 *                       of the Doppler phase shift measured by Doppler FFT.
 *                       The phase shift read from the table using half of the
 *                       object Doppler index  value. If the Doppler index is
 *                       odd, an extra half of the bin phase shift is added.
 *
 * @param[in]               dopplerIdx     : Doppler index of the object
 *
 * @param[in]               numDopplerBins : Number of Doppler bins
 *
 * @param[in]               azimuthModCoefs: Table with cos/sin values SIN in even position, COS in odd position
 *                                           exp(1j*2*pi*k/N) for k=0,...,N-1 where N is number of Doppler bins.
 *
 * @param[out]              azimuthModCoefsHalfBin :  exp(1j*2*pi* 0.5 /N)
 *
 * @param[in,out]           azimuthIn        :Pointer to antenna symbols to be Doppler compensated
 *
 * @param[in]              numAnt       : Number of antenna symbols to be Doppler compensated
 *
 * @return                  void
 *
 *******************************************************************************************************************
 */
void MmwDemo_addDopplerCompensation(int32_t dopplerIdx,
                                    int32_t numDopplerBins,
                                    uint32_t *azimuthModCoefs,
                                    uint32_t *azimuthModCoefsHalfBin,
                                    int64_t *azimuthIn,
                                    uint32_t numAnt)
{
    uint32_t expDoppComp;
    int32_t dopplerCompensationIdx = dopplerIdx;
    int64_t azimuthVal;
    int32_t Re, Im;
    uint32_t antIndx;

    if (numAnt == 0)
    {
        return;
    }

    /*Divide Doppler index by 2*/
    if (dopplerCompensationIdx >= numDopplerBins/2)
    {
        dopplerCompensationIdx -=  (int32_t)numDopplerBins;
    }
    dopplerCompensationIdx = dopplerCompensationIdx / 2;
    if (dopplerCompensationIdx < 0)
    {
        dopplerCompensationIdx +=  (int32_t) numDopplerBins;
    }
    expDoppComp = azimuthModCoefs[dopplerCompensationIdx];
    /* Add half bin rotation if Doppler index was odd */
    if (dopplerIdx & 0x1)
    {
        expDoppComp = _cmpyr1(expDoppComp, *azimuthModCoefsHalfBin);
    }

    /* Rotate symbols */
    for (antIndx = 0; antIndx < numAnt; antIndx++)
    {
        azimuthVal = _amem8(&azimuthIn[antIndx]);
        Re = _ssub(_mpyhir(expDoppComp, _loll(azimuthVal) ),
                    _mpylir(expDoppComp, _hill(azimuthVal)));
        Im = _sadd(_mpylir(expDoppComp, _loll(azimuthVal)),
                    _mpyhir(expDoppComp, _hill(azimuthVal)));
        _amem8(&azimuthIn[antIndx]) =  _itoll(Im, Re);
    }
}

# if 0
/*!*****************************************************************************************************************
 * \brief
 * Function Name       :    MmwDemo_rxChanPhaseBiasCompensation
 *
 * \par
 * <b>Description</b>  : Compensation of rx channel phase bias
 *
 * @param[in]               rxChComp : rx channel compensation coefficient
 *
 * @param[in]               input : 32-bit complex input symbols must be 64 bit aligned
 *
 * @param[in]               numAnt : number of symbols
 *
 *
 * @return                  void
 *
 *******************************************************************************************************************
 */
static inline void MmwDemo_rxChanPhaseBiasCompensation(uint32_t *rxChComp,
                                         int64_t *input,
                                         uint32_t numAnt)
{
    int64_t azimuthVal;
    int32_t Re, Im;
    uint32_t antIndx;
    uint32_t rxChCompVal;


    /* Compensation */
    for (antIndx = 0; antIndx < numAnt; antIndx++)
    {
        azimuthVal = _amem8(&input[antIndx]);

        rxChCompVal = _amem4(&rxChComp[antIndx]);

        Re = _ssub(_mpyhir(rxChCompVal, _loll(azimuthVal) ),
                    _mpylir(rxChCompVal, _hill(azimuthVal)));
        Im = _sadd(_mpylir(rxChCompVal, _loll(azimuthVal)),
                    _mpyhir(rxChCompVal, _hill(azimuthVal)));
        _amem8(&input[antIndx]) =  _itoll(Im, Re);
    }
}

#endif

/**
 *  @b Description
 *  @n
 *    Decodes the BPM antenna data.
 *
 * @param[in]  azimuthIn : Pointer to antenna symbols
 *
 * @param[in]  numRxAnt  : Number of RX antennas

 *  @retval
 *      Not Applicable.
 */
void MmwDemo_decodeBPM(cmplx32ReIm_t *azimuthIn, uint32_t numRxAnt)
{
    cmplx32ReIm_t *bpmAPtr;
    cmplx32ReIm_t *bpmBPtr;
    int32_t real, imag, idx;

    bpmAPtr  = &azimuthIn[0];
    bpmBPtr  = &azimuthIn[numRxAnt];

    for(idx = 0; idx < numRxAnt; idx++)
    {
        /*BPM decoding*/
        /*store A*/
        real = bpmAPtr[idx].real;
        imag = bpmAPtr[idx].imag;
        /*compute S1 and store in place*/
        bpmAPtr[idx].real = (bpmAPtr[idx].real + bpmBPtr[idx].real)/2;
        bpmAPtr[idx].imag = (bpmAPtr[idx].imag + bpmBPtr[idx].imag)/2;
        /*compute S2 and store in place*/
        bpmBPtr[idx].real = (real - bpmBPtr[idx].real)/2;
        bpmBPtr[idx].imag = (imag - bpmBPtr[idx].imag)/2;
    }
}


/**
 *  @b Description
 *  @n
 *    Computes log2Abs and accumulates for each dopple bin.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_log2AbsAccum(MmwDemo_DSS_DataPathObj *obj, int32_t rxAntIdx, uint32_t rangeIdx, uint32_t* waitingTime, uint32_t fftOutIndx)
{
    volatile uint32_t startTimeWait;
    uint32_t idx;

    mmwavelib_log2Abs32(
                (int32_t *) &obj->fftOut2D[fftOutIndx],
                obj->log2Abs,
                obj->numDopplerBins);

    if (rxAntIdx == 0)
    {
        /* check if previous  sumAbs has been transferred */
        if (rangeIdx > 0)
        {
            startTimeWait = Cycleprofiler_getTimeStamp();
            MmwDemo_dataPathWaitTransDetMatrix (obj);
            waitingTime += Cycleprofiler_getTimeStamp() - startTimeWait;
        }

        for (idx = 0; idx < obj->numDopplerBins; idx++)
        {
            obj->sumAbs[idx] = obj->log2Abs[idx];
        }
    }
    else
    {
        mmwavelib_accum16(obj->log2Abs, obj->sumAbs, obj->numDopplerBins);
    }
}


/**
 *  @b Description
 *  @n
 *    Calculates near field correction for input detected index (corresponding
 *    to a range position). Referring to top level doxygen @ref nearFieldImplementation,
 *    this function performs the Set 1 rotation with the correction and adds to Set 0
 *    in place to produce result in Set 0 of the azimuthOut.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_nearFieldCorrection(MmwDemo_DSS_DataPathObj *obj,
                                 uint32_t detIdx2)
{
/* All length units are in mm. The LAMBDA (wavelength) below is based on 77 GHz
 * and corresponds to the actual spacing on the EVM, it is not
 *  tied to the actual start frequency set in profile config, hence does not
 *  need to be computed on the fly from that configuration.
 */
#define MMWDEMO_XWR16XX_EVM_77GHZ_LAMBDA       (3.8961)
#define MMWDEMO_NEAR_FIELD_A (0)

/* B can be changed to position the desired reference (boresight) in the geometry */
#define MMWDEMO_NEAR_FIELD_B (MMWDEMO_XWR16XX_EVM_77GHZ_LAMBDA) //((((2 + 0.75) * MMWDEMO_XWR16XX_EVM_77GHZ_LAMBDA)) + 8.7)

#define MMWDEMO_NEAR_FIELD_C (2 * MMWDEMO_XWR16XX_EVM_77GHZ_LAMBDA)

/* 8.7 mm is the actual (approximate) measurement of distance between tx1 and rx4,
 * measured using a wooden scale that has a resolution of 1 mm */
#define MMWDEMO_NEAR_FIELD_D (MMWDEMO_NEAR_FIELD_C + 8.7)

#define MMWDEMO_NEAR_FIELD_E (MMWDEMO_NEAR_FIELD_D + 1.5 * MMWDEMO_XWR16XX_EVM_77GHZ_LAMBDA)

#define MMWDEMO_NEAR_FIELD_AB2 (2.0 * (MMWDEMO_NEAR_FIELD_A - MMWDEMO_NEAR_FIELD_B))
#define MMWDEMO_NEAR_FIELD_CB2 (2.0 * (MMWDEMO_NEAR_FIELD_C - MMWDEMO_NEAR_FIELD_B))
#define MMWDEMO_NEAR_FIELD_DB2 (2.0 * (MMWDEMO_NEAR_FIELD_D - MMWDEMO_NEAR_FIELD_B))
#define MMWDEMO_NEAR_FIELD_EB2 (2.0 * (MMWDEMO_NEAR_FIELD_E - MMWDEMO_NEAR_FIELD_B))

#define MMWDEMO_NEAR_FIELD_AB_SQ ((MMWDEMO_NEAR_FIELD_A - MMWDEMO_NEAR_FIELD_B) * (MMWDEMO_NEAR_FIELD_A - MMWDEMO_NEAR_FIELD_B))
#define MMWDEMO_NEAR_FIELD_CB_SQ ((MMWDEMO_NEAR_FIELD_C - MMWDEMO_NEAR_FIELD_B) * (MMWDEMO_NEAR_FIELD_C - MMWDEMO_NEAR_FIELD_B))
#define MMWDEMO_NEAR_FIELD_DB_SQ ((MMWDEMO_NEAR_FIELD_D - MMWDEMO_NEAR_FIELD_B) * (MMWDEMO_NEAR_FIELD_D - MMWDEMO_NEAR_FIELD_B))
#define MMWDEMO_NEAR_FIELD_EB_SQ ((MMWDEMO_NEAR_FIELD_E - MMWDEMO_NEAR_FIELD_B) * (MMWDEMO_NEAR_FIELD_E - MMWDEMO_NEAR_FIELD_B))

#define MMWDEMO_TWO_PI_OVER_LAMBDA  (2. * PI_ / MMWDEMO_XWR16XX_EVM_77GHZ_LAMBDA)

    int32_t i;
    float range;
    float rangeSq;
    cmplx32ReIm_t * azimuthOut0 = obj->azimuthOut;
    cmplx32ReIm_t * azimuthOut1 = &obj->azimuthOut[obj->numAngleBins];
    cmplx32ReIm_t azimuthOut1Corr;
    float corrReal;
    float corrImag;
    int32_t numAngleBins = obj->numAngleBins;
    float theta;
    float thetaInc = 2.0 / (float) obj->numAngleBins;
    float tx1, tx2, rx4, rx1;
    float psi;

    range = (obj->detObj2D[detIdx2].rangeIdx * obj->rangeResolution -
             obj->cliCommonCfg->compRxChanCfg.rangeBias) * 1000;
    rangeSq = range * range;

    corrReal = 1.0;
    corrImag = 0.0;

    for (i = 0; i < numAngleBins; i++)
    {
        if (i < numAngleBins/2)
        {
            theta =  i * thetaInc;
        }
        else
        {
            theta =  (i-numAngleBins) * thetaInc;
        }
        tx1 = sqrtsp(rangeSq + MMWDEMO_NEAR_FIELD_CB_SQ - range * theta * MMWDEMO_NEAR_FIELD_CB2);
        rx4 = sqrtsp(rangeSq + MMWDEMO_NEAR_FIELD_DB_SQ - range * theta * MMWDEMO_NEAR_FIELD_DB2);
        tx2 = sqrtsp(rangeSq + MMWDEMO_NEAR_FIELD_AB_SQ - range * theta * MMWDEMO_NEAR_FIELD_AB2);
        rx1 = sqrtsp(rangeSq + MMWDEMO_NEAR_FIELD_EB_SQ - range * theta * MMWDEMO_NEAR_FIELD_EB2);

        if (range > 0)
        {
            psi = MMWDEMO_TWO_PI_OVER_LAMBDA * ((tx2+rx1)-(rx4+tx1)) - PI_*theta;
            corrReal = cossp(psi);
            corrImag = sinsp(-psi);
        }

        azimuthOut1Corr.real = (int32_t) (azimuthOut1[i].real * corrReal - azimuthOut1[i].imag * corrImag);
        azimuthOut1Corr.imag = (int32_t) (azimuthOut1[i].imag * corrReal + azimuthOut1[i].real * corrImag);
        azimuthOut0[i].real = azimuthOut0[i].real + azimuthOut1Corr.real;
        azimuthOut0[i].imag = azimuthOut0[i].imag + azimuthOut1Corr.imag;
    }
}
#endif


void Re16bitIm16bit_swap(cmplx16ReIm_t *input,
                         cmplx16ImRe_t *output,
                         int n)
{

    int i;

    for (i = 0; i < n; i++)
    {
        output[i].real = input[i].real;
        output[i].imag = input[i].imag;
    }
}


/**
 *  @b Description
 *  @n
 *    Interframe processing. It is called from MmwDemo_dssDataPathProcessEvents
 *    after all chirps of the frame have been received and 1D FFT processing on them
 *    has been completed.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_interFrameProcessing(MmwDemo_DSS_DataPathObj *obj)
{
        //Vital Signs Demo Processing Chain

        gFrameCount++;                         // Increment the Global Frame Count
        static uint16_t frameCountLocal;       // Local circular count
        uint16_t loopIndexBuffer;              // Index that loops over the buffers

        /* Obtain GUI-related Flags sent through the CLI */
        VitalSignsDemo_GuiMonSel *pGuiMonSel;
        pGuiMonSel = (VitalSignsDemo_GuiMonSel *) &(obj->cliCfg->vitalSigns_GuiMonSel);

        /* Variables for Impulse Noise Removal */
        static float dataCurr = 0;
        static float dataPrev2 = 0;
        static float dataPrev1 = 0;

        /* Variables for Clutter Removal */
        float maxValClutter;
        uint16_t guiFlag_ClutterRemoval  = pGuiMonSel->guiFlag_ClutterRemoval;
        uint16_t rangeBinMaxClutter;

        /* Variables for Phase Unwrapping */
        static float phasePrevFrame = 0;              // Phase value of Previous frame (For phase unwrapping)
        static float diffPhaseCorrectionCum = 0;      // Phase correction cumulative (For phase unwrapping)
        static float phaseUsedComputationPrev = 0;    // Phase values used for the Previous frame
        float phaseUsedComputation = 0;               // Unwrapped Phase value used for computation

        /* Variables for Detecting Motion Corrupted Segments */
        uint16_t guiFlag_MotionDetection = obj->cliCfg->motionDetectionParamsCfg.enabled;       // GUI Flag. Set from the configuration File
        uint16_t guiFlag_GainControl     = obj->cliCfg->motionDetectionParamsCfg.gainControl;   // GUI Flag. Set from the configuration File
        uint16_t indexMotionDetection;      // Temporary Index
        float sumEnergy;                    // Energy in the data-segment checked for presence of large-scale motion

        /* Vital Signs Waveform */
        float outputFilterBreathOut;              // Breathing waveform after the IIR-Filter
        float outputFilterHeartOut;               // Cardiac waveform after the IIR-Filter

        /* Variables for FFT-based Spectral Estimation */
        uint16_t pPeakSortOutIndex[MAX_NUM_PEAKS_SPECTRUM];   // Sorted Peaks in the Spectrum
        uint16_t numPeaks_BreathSpectrum;                     // Number of Peaks in the Breathing Spectrum
        uint16_t numPeaks_heartSpectrum;                      // Number of Peaks in the Cardiac Spectrum
        uint16_t maxIndexHeartBeatSpect, maxIndexBreathSpect; // Indices corresponding to the max peak in the Breathing and Cardiac Spectrum
        uint16_t maxIndexHeartBeatSpect_4Hz;                  // Indices corresponding to the max peak from [1.6 - 4.0] Hz
        float breathingRateEst_FFT, heartRateEst_FFT;         // Vital Signs Estimate based on the FFT
        float heartRateEst_FFT_4Hz;                           // Vital Signs Estimate based on the FFT

        /* Confidence Metric associated with the estimates */
        float confidenceMetricBreath[MAX_NUM_PEAKS_SPECTRUM];         // Confidence Metric associated with each Breathing Spectrum Peak
        float confidenceMetricHeart[MAX_NUM_PEAKS_SPECTRUM];          // Confidence Metric associated with each Cardiac Spectrum Peak
        float confidenceMetricBreathOut, confidenceMetricHeartOut;    // Confidence Metric associated with the estimates
        float confidenceMetricHeartOut_4Hz;                           // Confidence Metric for the 1st Heart beat Harmonic
        float confidenceMetricHeartOut_xCorr;                         // Confidence Metric for the Autocorrelation
        float confidenceMetricBreathOut_xCorr;                        // Confidence Metric for the Autocorrelation based breathing-rate estimate
        float peakValueBreathSpect;

        /* Variables for peak-counting */
        uint16_t pPeakLocsHeart[MAX_PEAKS_ALLOWED_WFM];    // Peak locations (indices) of the Cardiac Waveform
        uint16_t pPeakLocsBreath[MAX_PEAKS_ALLOWED_WFM];   // Peak locations (indices) of the Breathing Waveform
        uint16_t pPeakLocsValid[MAX_PEAKS_ALLOWED_WFM];    // Peak locations after only retaining the valid peaks
        uint16_t numPeaksBreath, numPeaksHeart;                       // Number of peaks in the time-domain filtered waveform
        float breathingRateEst_peakCount, heartRateEst_peakCount;     // Vital Signs Estimate based on peak-Interval
        float heartRateEst_peakCount_filtered;                        // Heart-rate peak-interval based estimate after filtering

        /* Exponential smoothing filter */
        static float breathWfmOutUpdated, heartWfmOutUpdated;    // Updated values after exponential smoothing
        float breathWfmOutPrev, heartWfmOutPrev;                 // Exponential smoothing values at time instance (t-1)
        float sumEnergyBreathWfm, sumEnergyHeartWfm;             // These values are used to make a decision if the energy in the waveform is sufficient for it to be classfied as a valid waveform

        /* Variables for Auto-Correlation */
        float heartRateEst_xCorr;         // Heart-rate estimate from the Autocorrelation Method
        float breathRateEst_xCorr;        // Breathing-rate estimate from the Autocorrelation Method

        /* For FIR Filtering */
        static float pDataIn[FIR_FILTER_SIZE];

        /* Variables for Extracting the Range-FFT output  */
        uint16_t rangeBinIndex;             // Range-bin Index
        float rangeBinPhase;                // Phase of the Range-bin selected for processing
        static uint16_t rangeBinIndexPhase; // Index of the Range Bin for which the phase is computed
        uint16_t rangeBinMax;               // Index of the Strongest Range-Bin

        uint16_t indexTemp, indexNumPeaks;  // Temporary Indices
        int16_t temp_real, temp_imag;       // Temporary variables storing the Real and Imaginary part of a range-bin in the Range-FFT Output
        float absVal;                       // Absolute value based on the Real and Imaginary values
        float maxVal;                       // Maximum Value of the Range-Bin in the current range-profile

        if (pGuiMonSel->guiFlag_Reset  == 1)   // Refresh Pushbutton on the GUI Pressed
        {
            gFrameCount = 1;
            pGuiMonSel->guiFlag_Reset = 0;
            breathWfmOutUpdated = 0;
            heartWfmOutUpdated = 0;
        }

        if (RANGE_BIN_TRACKING)
        {
            frameCountLocal = gFrameCount % RESET_LOCAL_COUNT_VAL;
        }
        else
        {
            frameCountLocal = gFrameCount;
        }

        rangeBinMax = 0;
        maxVal = 0;
        rangeBinMaxClutter = 0;
        maxValClutter = 0;

        // tempPtr points towards the RX-Channel to process
        cmplx16ReIm_t *tempPtr = obj->fftOut1D + (obj->rxAntennaProcess - 1)*obj->numRangeBins;
        // tempPtr points towards rangeBinStartIndex
        tempPtr += (obj->rangeBinStartIndex) ;

        for (rangeBinIndex = obj->rangeBinStartIndex; rangeBinIndex <= obj->rangeBinEndIndex; rangeBinIndex++)
          {
            // Points towards the real part of the current range-bin i.e. rangeBinIndex
            temp_real = (int16_t) tempPtr->real;
            obj->pRangeProfileCplx[rangeBinIndex - obj->rangeBinStartIndex].real = temp_real;

            // Points towards the imaginary part of the current range-bin i.e. rangeBinIndex
            temp_imag = (int16_t) tempPtr->imag;
            obj->pRangeProfileCplx[rangeBinIndex - obj->rangeBinStartIndex].imag = temp_imag;

            // Move the pointer towards the next range-bin
            tempPtr ++;


         if (guiFlag_ClutterRemoval == 1)
         {
             // Clutter Removed Range Profile
             float tempReal_Curr,tempImag_Curr;
             float alphaClutter = 0.01;
             float currVal;

             tempReal_Curr = (float) temp_real;
             tempImag_Curr = (float) temp_imag;
             uint16_t currRangeIndex;
             currRangeIndex = rangeBinIndex - obj->rangeBinStartIndex;

             obj->pTempReal_Prev[currRangeIndex]  = alphaClutter*tempReal_Curr + (1-alphaClutter)*obj->pTempReal_Prev[currRangeIndex];
             obj->pTempImag_Prev[currRangeIndex]  = alphaClutter*tempImag_Curr + (1-alphaClutter)*obj->pTempImag_Prev[currRangeIndex];

             currVal = sqrt((tempReal_Curr - obj->pTempReal_Prev[currRangeIndex])*(tempReal_Curr - obj->pTempReal_Prev[currRangeIndex]) + (tempImag_Curr - obj->pTempImag_Prev[currRangeIndex])*(tempImag_Curr - obj->pTempImag_Prev[currRangeIndex]));
             obj->pRangeProfileClutterRemoved[rangeBinIndex - obj->rangeBinStartIndex] = currVal;

         // Based on the Max value Range-bin
             if (currVal > maxValClutter)
              {
                 maxValClutter = currVal;
                 rangeBinMaxClutter = rangeBinIndex;
              }
         }
            else
         {
           // Magnitude of the current range-bin
           absVal = (float) temp_real * (float) temp_real + (float) temp_imag * (float) temp_imag;
           // Maximum value range-bin of the current range-profile
           if (absVal > maxVal)
           {
              maxVal = absVal;
              rangeBinMax = rangeBinIndex;
           }
        }
           // If the Refresh button in the GUI is pressed
           if (frameCountLocal == 1)
              {
               if (guiFlag_ClutterRemoval == 1)
               {
               rangeBinIndexPhase = rangeBinMaxClutter;
               }
               else
               {
               rangeBinIndexPhase = rangeBinMax;
               }
              }
           if (rangeBinIndex == (rangeBinIndexPhase))
              {
               rangeBinPhase = atan2(temp_imag, temp_real);
              }
          } // For Loop ends

        // Phase-Unwrapping
        obj->unwrapPhasePeak = unwrap(rangeBinPhase, phasePrevFrame, &diffPhaseCorrectionCum);
        phasePrevFrame = rangeBinPhase;

#ifdef TEST_TONE  // Generates a Test-Tone

        float testTone_ampBreath_mm  = 1.0;
        float testTone_freqBreath_Hz = 0.45;
        float testTone_ampHeart_mm   = 0.01;
        float testTone_freqHeart_Hz  = 1.8;

        obj->unwrapPhasePeak = (4 * PI_/WAVELENGTH_MM) *(
                               (testTone_ampBreath_mm * sin(2*PI_*testTone_freqBreath_Hz* gFrameCount/obj->samplingFreq_Hz))+
                               (testTone_ampHeart_mm  * sin(2*PI_*testTone_freqHeart_Hz*gFrameCount/obj->samplingFreq_Hz))
                               );
         //                     +0.5*(testTone_ampBreath_mm *sin(2*PI_*(2*testTone_freqBreath_Hz)* gFrameCount/obj->samplingFreq_Hz)); // Harmonic signal
#endif

        // Computes the phase differences between successive phase samples
        if(FLAG_COMPUTE_PHASE_DIFFERENCE)
          {
            phaseUsedComputation = obj->unwrapPhasePeak - phaseUsedComputationPrev;
            phaseUsedComputationPrev = obj->unwrapPhasePeak;
          }
        else
          {
            phaseUsedComputation = obj->unwrapPhasePeak;
          }

        // Removes impulse like noise from the waveforms
        if (FLAG_REMOVE_IMPULSE_NOISE)
        {
            dataPrev2 = dataPrev1;
            dataPrev1 = dataCurr;
            dataCurr = phaseUsedComputation;
            phaseUsedComputation = filter_RemoveImpulseNoise(dataPrev2, dataPrev1, dataCurr, obj->noiseImpulse_Thresh);
        }

        // IIR Filtering
        outputFilterBreathOut = filter_IIR_BiquadCascade(phaseUsedComputation, obj->pFilterCoefsBreath, obj->pScaleValsBreath, obj->pDelayBreath, IIR_FILTER_BREATH_NUM_STAGES);
        outputFilterHeartOut  = filter_IIR_BiquadCascade(phaseUsedComputation, obj->pFilterCoefsHeart_4Hz, obj->pScaleValsHeart_4Hz, obj->pDelayHeart, IIR_FILTER_HEART_NUM_STAGES);

        // Copies the "Breathing Waveform" in a circular Buffer
        for (loopIndexBuffer = 1; loopIndexBuffer < obj->circularBufferSizeBreath; loopIndexBuffer++)
            {
              obj->pVitalSigns_Breath_CircularBuffer[loopIndexBuffer - 1] = obj->pVitalSigns_Breath_CircularBuffer[loopIndexBuffer];
            }
        obj->pVitalSigns_Breath_CircularBuffer[obj->circularBufferSizeBreath - 1] = outputFilterBreathOut;

       // Detection of Motion corrupted Segments
       if (guiFlag_MotionDetection == 1)
          {
       // Update the Motion Removal Circular Buffer
           for (loopIndexBuffer = 1; loopIndexBuffer < obj->motionDetection_BlockSize; loopIndexBuffer++)
                {
                  obj->pMotionCircularBuffer[loopIndexBuffer - 1] = obj->pMotionCircularBuffer[loopIndexBuffer] ;
                }
            obj->pMotionCircularBuffer[obj->motionDetection_BlockSize-1] = outputFilterHeartOut;
            indexMotionDetection = gFrameCount % obj->motionDetection_BlockSize;

       // Only perform these steps for every obj->motionDetection_BlockSize sample
            if (indexMotionDetection == 0)
            {
       // Check if the current segment is "Noisy"
                sumEnergy = 0;
                for (loopIndexBuffer = 0; loopIndexBuffer < obj->motionDetection_BlockSize; loopIndexBuffer++)
                {
                    sumEnergy +=  (obj->pMotionCircularBuffer[loopIndexBuffer]*obj->pMotionCircularBuffer[loopIndexBuffer]);
                }

                if (sumEnergy > obj->motionDetection_Thresh)
                   {
                     obj->motionDetected = 1;   // Temporary variable to send to the GUI
                   }
                else
                   {
                     obj->motionDetected = 0;   // Temporary variable to send to the GUI
                   }

        // If NO motion detected in the current segment
                if ( obj->motionDetected == 0)
                {
                    uint16_t tempEndIndex;
        //  Shift the current contents of the circular Buffer
                    for (loopIndexBuffer = obj->motionDetection_BlockSize; loopIndexBuffer < obj->circularBufferSizeHeart; loopIndexBuffer++)
                        {
                        obj->pVitalSigns_Heart_CircularBuffer[loopIndexBuffer - obj->motionDetection_BlockSize] = obj->pVitalSigns_Heart_CircularBuffer[loopIndexBuffer] ;
                        }
        // Copy the current data segment to the end of the Circular Buffer
                    for (loopIndexBuffer = 0; loopIndexBuffer < obj->motionDetection_BlockSize; loopIndexBuffer++)
                        {
                        tempEndIndex = obj->circularBufferSizeHeart - obj->motionDetection_BlockSize;
                        obj->pVitalSigns_Heart_CircularBuffer[ tempEndIndex + loopIndexBuffer] = obj->pMotionCircularBuffer[loopIndexBuffer] ;
                        }
                }
            }
          }
        // If Motion DETECTED then don't UPDATE or SHIFT the values in the buffer
        else // Regular processing
           {
        // Copies the "Cardiac Waveform" in a circular Buffer
        for (loopIndexBuffer = 1; loopIndexBuffer < obj->circularBufferSizeHeart; loopIndexBuffer++)
            {
               obj->pVitalSigns_Heart_CircularBuffer[loopIndexBuffer - 1] = obj->pVitalSigns_Heart_CircularBuffer[loopIndexBuffer];
            }
               obj->pVitalSigns_Heart_CircularBuffer[obj->circularBufferSizeHeart - 1] = outputFilterHeartOut;
         }

        /* Spectral Estimation based on the Inter-Peaks Distance */
        numPeaksHeart  = find_Peaks(obj->pVitalSigns_Heart_CircularBuffer, float_type, pPeakLocsHeart,obj->pPeakValues, 0, obj->circularBufferSizeHeart  - 1);
        if (numPeaksHeart != 0)
        {
            numPeaksHeart  =  filterPeaksWfm(pPeakLocsHeart, pPeakLocsValid, numPeaksHeart, obj->peakDistanceHeart_Min, obj->peakDistanceHeart_Max);
        }
        heartRateEst_peakCount = CONVERT_HZ_BPM * ((numPeaksHeart * obj->samplingFreq_Hz) / obj->circularBufferSizeHeart);

            for (loopIndexBuffer = 1; loopIndexBuffer < FIR_FILTER_SIZE; loopIndexBuffer++)
            {
                pDataIn[loopIndexBuffer - 1] = pDataIn[loopIndexBuffer];
            }
            pDataIn[FIR_FILTER_SIZE - 1] = heartRateEst_peakCount;
            heartRateEst_peakCount_filtered = filter_FIR(pDataIn, obj->pFilterCoefs, FIR_FILTER_SIZE);

        numPeaksBreath = find_Peaks(obj->pVitalSigns_Breath_CircularBuffer, int32_type, pPeakLocsBreath, obj->pPeakValues, 0, obj->circularBufferSizeBreath - 1);
        if (numPeaksBreath != 0)
        {
            numPeaksBreath  =  filterPeaksWfm(pPeakLocsBreath, pPeakLocsValid, numPeaksBreath, obj->peakDistanceBreath_Min, obj->peakDistanceBreath_Max);
        }

        breathingRateEst_peakCount = CONVERT_HZ_BPM  * ((numPeaksBreath * obj->samplingFreq_Hz) / obj->circularBufferSizeHeart);
        heartRateEst_peakCount     = CONVERT_HZ_BPM  * ((numPeaksHeart * obj->samplingFreq_Hz) /  obj->circularBufferSizeBreath);

        // Input to the FFT needs to be complex
        memset((uint8_t *)obj->pVitalSignsBuffer_Cplx, 0, obj->breathingWfm_Spectrum_FftSize * sizeof(cmplx32ReIm_t));
        for (loopIndexBuffer = 0; loopIndexBuffer < obj->circularBufferSizeBreath; loopIndexBuffer++)
            {
                obj->pVitalSignsBuffer_Cplx[loopIndexBuffer].real = (int32_t) obj->scale_breathingWfm*obj->pVitalSigns_Breath_CircularBuffer[loopIndexBuffer];
                obj->pVitalSignsBuffer_Cplx[loopIndexBuffer].imag=0;
            }

        // Input is overwritten by the DSP_fft32x32 function
        DSP_fft32x32(
                    (int32_t *)obj->pVitalSignsSpectrumTwiddle32x32,
                    obj->breathingWfm_Spectrum_FftSize,
                    (int32_t *) obj->pVitalSignsBuffer_Cplx,
                    (int32_t *) obj->pVitalSigns_SpectrumCplx);

        MmwDemo_magnitudeSquared(
                       obj->pVitalSigns_SpectrumCplx,
                       obj->pVitalSigns_Breath_AbsSpectrum,
                       obj->breathingWfm_Spectrum_FftSize);

        memset((uint8_t *)obj->pVitalSignsBuffer_Cplx, 0, obj->heartWfm_Spectrum_FftSize * sizeof(cmplx32ReIm_t));

        // Pre-Processing Steps for the Cardiac Waveform
        // Perform Automatic Gain Control if enabled from the GUI
       if (guiFlag_GainControl == 1)
       {
        computeAGC ( obj->pVitalSigns_Heart_CircularBuffer, obj->circularBufferSizeHeart, obj->motionDetection_BlockSize, obj->motionDetection_Thresh);
       }

       if (guiFlag_MotionDetection == 1)
       {
         outputFilterHeartOut =  obj->pMotionCircularBuffer[obj->motionDetection_BlockSize-1];
       }
       else
       {
         outputFilterHeartOut = obj->pVitalSigns_Heart_CircularBuffer[obj->circularBufferSizeHeart-1];
       }

       // Perform Autocorrelation on the Waveform
       if (PERFORM_XCORR)
       {
           float temp;
           // Perform Autocorrelation on the Cardiac-Waveform
           uint16_t xCorr_numPeaks;
           uint16_t maxIndex_lag;
           computeAutoCorrelation ( obj->pVitalSigns_Heart_CircularBuffer,  obj->circularBufferSizeHeart , obj->pXcorr, obj->xCorr_minLag, obj->xCorr_maxLag);
           xCorr_numPeaks  = find_Peaks(obj->pXcorr, float_type, obj->pPeakIndex, obj->pPeakValues, obj->xCorr_minLag, obj->xCorr_maxLag);
           maxIndex_lag    = computeMaxIndex((float*) obj->pXcorr, obj->xCorr_minLag, obj->xCorr_maxLag);
           temp = (float) (1.0)/(maxIndex_lag/obj->samplingFreq_Hz);
           heartRateEst_xCorr = (float) CONVERT_HZ_BPM *temp;

           if (xCorr_numPeaks == 0 )
           {
               confidenceMetricHeartOut_xCorr = 0;
           }
           else
           {
               confidenceMetricHeartOut_xCorr = obj->pXcorr[maxIndex_lag];
           }

          // Auto-correlation on the Breathing Waveform
          computeAutoCorrelation ( obj->pVitalSigns_Breath_CircularBuffer,  obj->circularBufferSizeBreath,
                                   obj->pXcorr, obj->xCorr_Breath_minLag, obj->xCorr_Breath_maxLag);
          xCorr_numPeaks  = find_Peaks(obj->pXcorr, float_type, obj->pPeakIndex, obj->pPeakValues, obj->xCorr_Breath_minLag, obj->xCorr_Breath_maxLag);
          maxIndex_lag    = computeMaxIndex((float*) obj->pXcorr, obj->xCorr_Breath_minLag, obj->xCorr_Breath_maxLag);
          temp = (float) (1.0)/(maxIndex_lag/obj->samplingFreq_Hz);
          breathRateEst_xCorr = (float) CONVERT_HZ_BPM *temp;

          if (xCorr_numPeaks == 0 )
          {
              confidenceMetricBreathOut_xCorr = 0;
          }
          else
          {
              confidenceMetricBreathOut_xCorr = obj->pXcorr[maxIndex_lag];
          }
       }

        // Apply Window on the Cardiac Waveform prior to FFT-based spectral estimation
        // and copies the Pre-processed data to pCircularBufferHeart
        if (FLAG_APPLY_WINDOW)
        {
            uint16_t index_win;
            uint16_t index_WinEnd;

            float tempFloat;
            index_WinEnd =  obj->circularBufferSizeHeart - 1;
            for (index_win = 0; index_win < DOPPLER_WINDOW_SIZE; index_win++)
            {
              tempFloat = obj->pDopplerWindow[index_win];
              obj->pVitalSignsBuffer_Cplx[index_win].real    = (int32_t) obj->scale_heartWfm * tempFloat * obj->pVitalSigns_Heart_CircularBuffer[index_win];
              obj->pVitalSignsBuffer_Cplx[index_WinEnd].real = (int32_t) obj->scale_heartWfm * tempFloat * obj->pVitalSigns_Heart_CircularBuffer[index_WinEnd];
              obj->pVitalSignsBuffer_Cplx[index_win].imag    = 0;
              obj->pVitalSignsBuffer_Cplx[index_WinEnd].imag = 0;
              index_WinEnd --;
            }
            for (loopIndexBuffer = DOPPLER_WINDOW_SIZE; loopIndexBuffer < obj->circularBufferSizeHeart - DOPPLER_WINDOW_SIZE ; loopIndexBuffer++)
            {
              obj->pVitalSignsBuffer_Cplx[loopIndexBuffer].real = (int32_t)  (obj->scale_heartWfm)*obj->pVitalSigns_Heart_CircularBuffer[loopIndexBuffer];
              obj->pVitalSignsBuffer_Cplx[loopIndexBuffer].imag = 0;
            }
        }
        else
        {
           for (loopIndexBuffer = 0; loopIndexBuffer < obj->circularBufferSizeHeart; loopIndexBuffer++)
               {
                obj->pVitalSignsBuffer_Cplx[loopIndexBuffer].real = (int32_t) (obj->scale_heartWfm)*obj->pVitalSigns_Heart_CircularBuffer[loopIndexBuffer];
                obj->pVitalSignsBuffer_Cplx[loopIndexBuffer].imag=0;
               }
        }

        // FFT of the Cardiac Waveform
        DSP_fft32x32(
                    (int32_t *)obj->pVitalSignsSpectrumTwiddle32x32,
                    obj->heartWfm_Spectrum_FftSize,
                    (int32_t *) obj->pVitalSignsBuffer_Cplx,
                    (int32_t *) obj->pVitalSigns_SpectrumCplx);

        MmwDemo_magnitudeSquared(
                       obj->pVitalSigns_SpectrumCplx,
                       obj->pVitalSigns_Heart_AbsSpectrum,
                       obj->heartWfm_Spectrum_FftSize);

        // Pick the Peaks in the Breathing Spectrum
        numPeaks_BreathSpectrum = find_Peaks(obj->pVitalSigns_Breath_AbsSpectrum, float_type, obj->pPeakIndex, obj->pPeakValues,
                                             obj->breath_startFreq_Index, obj->breath_endFreq_Index);
        indexNumPeaks = (numPeaks_BreathSpectrum < MAX_NUM_PEAKS_SPECTRUM) ? numPeaks_BreathSpectrum : MAX_NUM_PEAKS_SPECTRUM;

        if (indexNumPeaks != 0)
           {
           heapsort_index(obj->pPeakValues, numPeaks_BreathSpectrum, obj->pPeakIndexSorted);
           for(indexTemp = 0; indexTemp < indexNumPeaks; indexTemp++ )
              {
              pPeakSortOutIndex[indexTemp] = obj->pPeakIndex [obj->pPeakIndexSorted[numPeaks_BreathSpectrum-indexTemp-1] ];
              confidenceMetricBreath[indexTemp]  = computeConfidenceMetric(obj->pVitalSigns_Breath_AbsSpectrum,
                                                                           obj->confMetric_spectrumBreath_IndexStart,
                                                                           obj->confMetric_spectrumBreath_IndexEnd,
                                                                           pPeakSortOutIndex[indexTemp],
                                                                           obj->confMetric_numIndexAroundPeak_breath);
              }
           maxIndexBreathSpect       = pPeakSortOutIndex[0]; // The maximum peak
           confidenceMetricBreathOut = confidenceMetricBreath[0];
           }
           else
        {
           maxIndexBreathSpect    = computeMaxIndex((float*) obj->pVitalSigns_Breath_AbsSpectrum, obj->breath_startFreq_Index, obj->breath_endFreq_Index);
           confidenceMetricBreathOut  = computeConfidenceMetric(obj->pVitalSigns_Breath_AbsSpectrum,
                                                                0,
                                                                PHASE_FFT_SIZE/4,
                                                                maxIndexBreathSpect,
                                                                obj->confMetric_numIndexAroundPeak_breath);
        }
        peakValueBreathSpect = obj->pVitalSigns_Breath_AbsSpectrum[maxIndexBreathSpect]/(10*obj->scale_breathingWfm);

       // Pick the Peaks in the Heart Spectrum [1.6 - 4.0 Hz]
       numPeaks_heartSpectrum = find_Peaks(obj->pVitalSigns_Heart_AbsSpectrum, uint32_type, obj->pPeakIndex, obj->pPeakValues,
                                           obj->heart_startFreq_Index_1p6Hz, obj->heart_endFreq_Index_4Hz);
       indexNumPeaks = (numPeaks_heartSpectrum < MAX_NUM_PEAKS_SPECTRUM) ? numPeaks_heartSpectrum : MAX_NUM_PEAKS_SPECTRUM;
       if (indexNumPeaks != 0)
            {
            heapsort_index(obj->pPeakValues, numPeaks_heartSpectrum, obj->pPeakIndexSorted);
            for(indexTemp = 0; indexTemp < indexNumPeaks; indexTemp++ )
               {
                pPeakSortOutIndex[indexTemp] = obj->pPeakIndex [obj->pPeakIndexSorted[numPeaks_heartSpectrum-indexTemp-1] ];
               }
       maxIndexHeartBeatSpect_4Hz    = pPeakSortOutIndex[0]; // The maximum peak
       confidenceMetricHeartOut_4Hz  = computeConfidenceMetric(obj->pVitalSigns_Heart_AbsSpectrum,
                                                               obj->confMetric_spectrumHeart_IndexStart_1p6Hz,
                                                               obj->confMetric_spectrumHeart_IndexStart_4Hz,
                                                               maxIndexHeartBeatSpect_4Hz,
                                                               obj->confMetric_numIndexAroundPeak_heart);
           }
       else
           {
              maxIndexHeartBeatSpect_4Hz = computeMaxIndex((float*) obj->pVitalSigns_Heart_AbsSpectrum, obj->heart_startFreq_Index_1p6Hz, obj->heart_endFreq_Index_4Hz);
              confidenceMetricHeartOut_4Hz = computeConfidenceMetric(obj->pVitalSigns_Heart_AbsSpectrum,
                                                                     0,
                                                                     PHASE_FFT_SIZE/4,
                                                                     maxIndexHeartBeatSpect_4Hz,
                                                                     obj->confMetric_numIndexAroundPeak_heart);
           }
          heartRateEst_FFT_4Hz  = (float) CONVERT_HZ_BPM * maxIndexHeartBeatSpect_4Hz * (obj->freqIncrement_Hz);

          // If a peak is within [1.6 2.0] Hz then check if a harmonic is present is the cardiac spectrum region [0.8 - 2.0] Hz
          if (heartRateEst_FFT_4Hz < MAX_HEART_RATE_BPM)
          {
             for (indexTemp =1; indexTemp<numPeaks_heartSpectrum;indexTemp++)

                 if(abs(heartRateEst_FFT_4Hz - CONVERT_HZ_BPM *(obj->freqIncrement_Hz)*pPeakSortOutIndex[indexTemp]) < HEART_HAMRONIC_THRESH_BPM)
                 {
                     heartRateEst_FFT_4Hz = CONVERT_HZ_BPM *(obj->freqIncrement_Hz)*pPeakSortOutIndex[indexTemp];
                     break;
                 }
          }

        // Pick the Peaks in the Cardiac Spectrum
        numPeaks_heartSpectrum = find_Peaks(obj->pVitalSigns_Heart_AbsSpectrum, float_type, obj->pPeakIndex, obj->pPeakValues, obj->heart_startFreq_Index, obj->heart_endFreq_Index);
        indexNumPeaks = (numPeaks_heartSpectrum < MAX_NUM_PEAKS_SPECTRUM) ? numPeaks_heartSpectrum : MAX_NUM_PEAKS_SPECTRUM;

        if (indexNumPeaks != 0)
        {
        heapsort_index(obj->pPeakValues, numPeaks_heartSpectrum, obj->pPeakIndexSorted);
        for(indexTemp = 0; indexTemp < indexNumPeaks; indexTemp++ )
                  {
                   pPeakSortOutIndex[indexTemp] = obj->pPeakIndex [obj->pPeakIndexSorted[numPeaks_heartSpectrum-indexTemp-1] ];
                   confidenceMetricHeart[indexTemp]  = computeConfidenceMetric(obj->pVitalSigns_Heart_AbsSpectrum,
                                                                                          obj->confMetric_spectrumHeart_IndexStart,
                                                                                          obj->confMetric_spectrumHeart_IndexEnd,
                                                                                          pPeakSortOutIndex[indexTemp],
                                                                                          obj->confMetric_numIndexAroundPeak_heart);
                  }
            maxIndexHeartBeatSpect   = pPeakSortOutIndex[0]; // The maximum peak
            confidenceMetricHeartOut = confidenceMetricHeart[0];
        }
        else
        {
         maxIndexHeartBeatSpect    = computeMaxIndex((float*) obj->pVitalSigns_Heart_AbsSpectrum, obj->heart_startFreq_Index, obj->heart_endFreq_Index);
         confidenceMetricHeartOut  = computeConfidenceMetric(obj->pVitalSigns_Heart_AbsSpectrum,
                                                                        0,
                                                                        PHASE_FFT_SIZE/4,
                                                                        maxIndexHeartBeatSpect,
                                                                        obj->confMetric_numIndexAroundPeak_heart);
        }

        // Remove the First Breathing Harmonic (if present in the cardiac Spectrum)
        if(FLAG_HARMONIC_CANCELLATION)
        {
            float diffIndex = abs(maxIndexHeartBeatSpect - BREATHING_HARMONIC_NUM*maxIndexBreathSpect);
            if ( diffIndex*(obj->freqIncrement_Hz)*CONVERT_HZ_BPM < BREATHING_HAMRONIC_THRESH_BPM ) // Only cancel the 2nd Breathing Harmonic
            {
                maxIndexHeartBeatSpect = pPeakSortOutIndex[1]; // Pick the 2nd Largest peak in the cardiac-spectrum
                confidenceMetricHeartOut = confidenceMetricHeart[1];
            }
        }

       heartRateEst_FFT     = (float) CONVERT_HZ_BPM * maxIndexHeartBeatSpect * (obj->freqIncrement_Hz);
       breathingRateEst_FFT = (float) CONVERT_HZ_BPM * maxIndexBreathSpect  * (obj->freqIncrement_Hz);

#ifdef HARMONICS_ENERGY
       float heartRateEst_HarmonicEnergy;
       float breathRateEst_HarmonicEnergy;
       uint16_t maxIndexHeartBeatSpect_temp;
       uint16_t maxIndexBreathSpect_temp;

       memset((void *)obj->pDataOutTemp, 0, obj->heartWfm_Spectrum_FftSize*sizeof(float));
       computeEnergyHarmonics (obj->pVitalSigns_Heart_AbsSpectrum,
                               obj->pDataOutTemp,
                               obj->confMetric_spectrumHeart_IndexStart,
                               obj->confMetric_spectrumHeart_IndexEnd,
                               obj->confMetric_numIndexAroundPeak_heart);
       maxIndexHeartBeatSpect_temp = computeMaxIndex(obj->pDataOutTemp,
                       obj->confMetric_spectrumHeart_IndexStart,
                       obj->confMetric_spectrumHeart_IndexEnd);
       heartRateEst_HarmonicEnergy  = (float) CONVERT_HZ_BPM * maxIndexHeartBeatSpect_temp * (obj->freqIncrement_Hz);


       memset((void *)obj->pDataOutTemp, 0, obj->heartWfm_Spectrum_FftSize*sizeof(float));
       computeEnergyHarmonics (obj->pVitalSigns_Breath_AbsSpectrum,
                               obj->pDataOutTemp,
                               obj->confMetric_spectrumBreath_IndexStart,
                               obj->confMetric_spectrumBreath_IndexEnd,
                               obj->confMetric_numIndexAroundPeak_breath);
       maxIndexBreathSpect_temp = computeMaxIndex(obj->pDataOutTemp,
                       obj->confMetric_spectrumBreath_IndexStart,
                       obj->confMetric_spectrumBreath_IndexEnd);
       breathRateEst_HarmonicEnergy  = (float) CONVERT_HZ_BPM * maxIndexBreathSpect_temp * (obj->freqIncrement_Hz);

#endif

      //  Median Value for Heart Rate and Breathing Rate based on 'MEDIAN_WINDOW_LENGTH' previous estimates
       if(FLAG_MEDIAN_FILTER)
       {
          for (loopIndexBuffer = 1; loopIndexBuffer < MEDIAN_WINDOW_LENGTH; loopIndexBuffer++)
          {
           obj->pBufferHeartRate[loopIndexBuffer - 1]     = obj->pBufferHeartRate[loopIndexBuffer];
           obj->pBufferBreathingRate[loopIndexBuffer - 1] = obj->pBufferBreathingRate[loopIndexBuffer];
           obj->pBufferHeartRate_4Hz[loopIndexBuffer - 1]     = obj->pBufferHeartRate_4Hz[loopIndexBuffer];
          }
       obj->pBufferHeartRate[MEDIAN_WINDOW_LENGTH - 1]     = heartRateEst_FFT;
       obj->pBufferBreathingRate[MEDIAN_WINDOW_LENGTH - 1] = breathingRateEst_FFT;
       obj->pBufferHeartRate_4Hz[MEDIAN_WINDOW_LENGTH - 1] = heartRateEst_FFT_4Hz;

       heapsort_index(obj->pBufferHeartRate, MEDIAN_WINDOW_LENGTH,  obj->pPeakSortTempIndex);
       heartRateEst_FFT = obj->pBufferHeartRate[ obj->pPeakSortTempIndex[MEDIAN_WINDOW_LENGTH/2+1]];

       heapsort_index(obj->pBufferHeartRate_4Hz, MEDIAN_WINDOW_LENGTH,  obj->pPeakSortTempIndex);
       heartRateEst_FFT_4Hz = obj->pBufferHeartRate_4Hz[ obj->pPeakSortTempIndex[MEDIAN_WINDOW_LENGTH/2+1]];

       heapsort_index(obj->pBufferBreathingRate, MEDIAN_WINDOW_LENGTH,  obj->pPeakSortTempIndex);
       breathingRateEst_FFT = obj->pBufferBreathingRate[ obj->pPeakSortTempIndex[MEDIAN_WINDOW_LENGTH/2+1]];
       }

       // Exponential Smoothing
       breathWfmOutPrev = breathWfmOutUpdated;
       breathWfmOutUpdated = (obj->alpha_breathing)*(outputFilterBreathOut*outputFilterBreathOut) + (1 - (obj->alpha_breathing))*breathWfmOutPrev; // Exponential Smoothing
       sumEnergyBreathWfm = breathWfmOutUpdated*10000;

       heartWfmOutPrev = heartWfmOutUpdated;
       heartWfmOutUpdated = (obj->alpha_heart)*(outputFilterHeartOut*outputFilterHeartOut) + (1 - (obj->alpha_heart))*heartWfmOutPrev;  // Exponential Smoothing
       sumEnergyHeartWfm = heartWfmOutUpdated*10000;

       // Output Values
       obj->VitalSigns_Output.unwrapPhasePeak_mm    = obj->unwrapPhasePeak;
       obj->VitalSigns_Output.outputFilterBreathOut = outputFilterBreathOut;
       obj->VitalSigns_Output.outputFilterHeartOut  = outputFilterHeartOut;
       obj->VitalSigns_Output.rangeBinIndexPhase = rangeBinIndexPhase;						//frameCountLocal;
       obj->VitalSigns_Output.maxVal   = maxVal;
       obj->VitalSigns_Output.sumEnergyHeartWfm  = sumEnergyHeartWfm;
       obj->VitalSigns_Output.sumEnergyBreathWfm = sumEnergyBreathWfm*peakValueBreathSpect;

       obj->VitalSigns_Output.confidenceMetricBreathOut = confidenceMetricBreathOut;
       obj->VitalSigns_Output.confidenceMetricHeartOut = confidenceMetricHeartOut;    		// Confidence Metric associated with the estimates
       obj->VitalSigns_Output.confidenceMetricHeartOut_4Hz = confidenceMetricHeartOut_4Hz;
       obj->VitalSigns_Output.confidenceMetricHeartOut_xCorr = confidenceMetricHeartOut_xCorr;

       obj->VitalSigns_Output.breathingRateEst_FFT = breathingRateEst_FFT;
       obj->VitalSigns_Output.breathingRateEst_peakCount = breathingRateEst_peakCount;
       obj->VitalSigns_Output.heartRateEst_peakCount_filtered = heartRateEst_peakCount_filtered;
       obj->VitalSigns_Output.heartRateEst_xCorr = heartRateEst_xCorr;
       obj->VitalSigns_Output.heartRateEst_FFT_4Hz = heartRateEst_FFT_4Hz;
       obj->VitalSigns_Output.heartRateEst_FFT = heartRateEst_FFT;

       obj->VitalSigns_Output.processingCyclesOut = obj->timingInfo.interFrameProcCycles/DSP_CLOCK_MHZ;
       obj->VitalSigns_Output.rangeBinStartIndex = obj->rangeBinStartIndex;
       obj->VitalSigns_Output.rangeBinEndIndex  = obj->rangeBinEndIndex;
       obj->VitalSigns_Output.motionDetectedFlag         = obj->motionDetected;
       obj->VitalSigns_Output.breathingRateEst_xCorr  = breathRateEst_xCorr;							//breathRateEst_HarmonicEnergy;
       obj->VitalSigns_Output.confidenceMetricBreathOut_xCorr    = confidenceMetricBreathOut_xCorr;		//breathRateEst_HarmonicEnergy;
       obj->VitalSigns_Output.breathingRateEst_harmonicEnergy = breathRateEst_HarmonicEnergy;			//heartRateEst_HarmonicEnergy;
       obj->VitalSigns_Output.heartRateEst_harmonicEnergy = heartRateEst_HarmonicEnergy;

}


/**
 *  @b Description
 *  @n
 *    Chirp processing. It is called from MmwDemo_dssDataPathProcessEvents. It
 *    is executed per chirp
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_processChirp(MmwDemo_DSS_DataPathObj *obj, uint16_t chirpIndxInMultiChirp)
{
    volatile uint32_t startTime;
    uint32_t chirpBytes, channelId;
    MmwDemo_DSS_dataPathContext_t *context = obj->context;

    startTime = Cycleprofiler_getTimeStamp();

    EDMA_setSourceAddress(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE], MMW_EDMA_CH_1D_IN_PING,
        (uint32_t) &obj->ADCdataBuf[chirpIndxInMultiChirp * obj->numAdcSamples]);

    EDMA_setSourceAddress(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE], MMW_EDMA_CH_1D_IN_PONG,
        (uint32_t) &obj->ADCdataBuf[(chirpIndxInMultiChirp +
                                     obj->numChirpsPerChirpEvent) * obj->numAdcSamples]);

    if(obj->chirpCount > 1) //verify if ping(or pong) buffer is free for odd(or even) chirps
    {
        MmwDemo_dataPathWait1DOutputData (obj, pingPongId(obj->chirpCount));
    }
    gCycleLog.interChirpWaitTime += Cycleprofiler_getTimeStamp() - startTime;

    MmwDemo_interChirpProcessing (obj, pingPongId(obj->chirpCount));

    if(isPong(obj->chirpCount))
    {
        channelId = MMW_EDMA_CH_1D_OUT_PONG;
    }
    else
    {
        channelId = MMW_EDMA_CH_1D_OUT_PING;
    }

    /* for non TDM case, when chirpBytes is >= 16384, the destinationBindex in
       EDMA will be twice of this which is negative jump, so need to set the
       destination address in this situation.
       e.g if numRangeBins = 1024, numRxAntennas = 4 then destinationBindex becomes -32768 */
    chirpBytes = obj->numRangeBins * obj->numRxAntennas * sizeof(cmplx16ReIm_t);
    if ((obj->numTxAntennas == 1) && (chirpBytes >= (uint32_t)16384))
    {
        EDMA_setDestinationAddress(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE], channelId,
            (uint32_t)obj->radarCube + (uint32_t)obj->chirpCount * chirpBytes);
    }

    EDMA_startDmaTransfer(context->edmaHandle[MMW_DATA_PATH_EDMA_INSTANCE], channelId);

    obj->chirpCount++;
    obj->txAntennaCount++;
    if(obj->txAntennaCount == obj->numTxAntennas)
    {
        obj->txAntennaCount = 0;
        obj->dopplerBinCount++;
        if (obj->dopplerBinCount == obj->numDopplerBins)
        {
            obj->dopplerBinCount = 0;
            obj->chirpCount = 0;
        }
    }
}

 /**
  *  @b Description
  *  @n
  *  Wait for transfer of data corresponding to the last 2 chirps (ping/pong)
  *  to the radarCube matrix before starting interframe processing.
  *  @retval
  *      Not Applicable.
  */
void MmwDemo_waitEndOfChirps(MmwDemo_DSS_DataPathObj *obj)
{
    volatile uint32_t startTime;

    startTime = Cycleprofiler_getTimeStamp();
    /* Wait for transfer of data corresponding to last 2 chirps (ping/pong) */
    MmwDemo_dataPathWait1DOutputData (obj, 0);
    MmwDemo_dataPathWait1DOutputData (obj, 1);

    gCycleLog.interChirpWaitTime += Cycleprofiler_getTimeStamp() - startTime;
}

#if 0
/**
 *  @b Description
 *  @n
 *  Generate SIN/COS table in Q15 (SIN to even int16 location, COS to
 *  odd int16 location. Also generates Sine/Cosine at half the bin value
 *  The table is generated using a lookup table @ref twiddleTableCommon
 *
 *  @param[out]    dftSinCosTable Array with generated Sin Cos table
 *  @param[out]    dftHalfBinVal  Sin/Cos value at half the bin
 *  @param[in]     dftLen Length of the DFT
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_genDftSinCosTable(cmplx16ImRe_t *dftSinCosTable,
                            cmplx16ImRe_t *dftHalfBinVal,
                            uint32_t dftLen)
{
    int32_t i, ind;
    int32_t itemp;
    float temp;
    int32_t indLsb, indMsb;
    int32_t step = 1024 >> (30 - _norm(dftLen)); //1024/dftLen, dftLen power of 2

    int64_t * restrict table = (int64_t *) twiddleTableCommon;
    uint32_t * restrict wd = (uint32_t *) dftSinCosTable;
    int32_t xRe;
    int32_t xIm;

    #pragma MUST_ITERATE(4,,4)
    for (i = 0; i < dftLen; i++)
    {
        ind = step * i;
        indLsb = ind & 0xFF;
        indMsb = (ind >> 8) & 0x3;
        xRe =  ((int32_t)_sadd(_hill(table[indLsb]), 0x00008000)) >> 16;
        xIm =  ((int32_t)_sadd(_loll(table[indLsb]), 0x00008000)) >> 16;
        if (indMsb == 0)
        {
            wd[i] =  _pack2(xRe, -xIm);
        }
        if (indMsb == 1)
        {
            wd[i] =  _pack2(-xIm, -xRe);
        }
        if (indMsb == 2)
        {
            wd[i] =  _pack2(-xRe, xIm);
        }
        if (indMsb == 3)
        {
            wd[i] =  _pack2(xIm, xRe);
        }
    }

    /*Calculate half bin value*/
    temp = ONE_Q15 * - sinsp(PI_/dftLen);
    itemp = (int32_t) ROUND(temp);

    if(itemp >= ONE_Q15)
    {
        itemp = ONE_Q15 - 1;
    }
    dftHalfBinVal[0].imag = itemp;

    temp = ONE_Q15 * cossp(PI_/dftLen);
    itemp = (int32_t) ROUND(temp);

    if(itemp >= ONE_Q15)
    {
        itemp = ONE_Q15 - 1;
    }
    dftHalfBinVal[0].real = itemp;
}
#endif

void MmwDemo_edmaErrorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo)
{
    MmwDemo_dssAssert(0);
}

void MmwDemo_edmaTransferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo)
{
    MmwDemo_dssAssert(0);
}

void MmwDemo_dataPathObjInit(MmwDemo_DSS_DataPathObj *obj,
                             MmwDemo_DSS_dataPathContext_t *context,
                             MmwDemo_CliCfg_t *cliCfg,
                             MmwDemo_CliCommonCfg_t *cliCommonCfg,
                             MmwDemo_Cfg *cfg)
{
    memset((void *)obj, 0, sizeof(MmwDemo_DSS_DataPathObj));
    obj->context = context;
    obj->cliCfg = cliCfg;
    obj->cliCommonCfg = cliCommonCfg;
    obj->cfg = cfg;
}

void MmwDemo_dataPathInit1Dstate(MmwDemo_DSS_DataPathObj *obj)
{
    obj->chirpCount = 0;
    obj->dopplerBinCount = 0;
    obj->txAntennaCount = 0;

    /* reset profiling logs before start of frame */
    memset((void *) &gCycleLog, 0, sizeof(cycleLog_t));
}

void MmwDemo_dataPathInitVitalSigns(MmwDemo_DSS_DataPathObj *obj)
{

    /* Obtain Parameters from the Command Line Interface */
    VitalSignsDemo_ParamsCfg  vitalSignsParamCLI;
    vitalSignsParamCLI = obj->cliCfg->vitalSignsParamsCfg;

    obj->circularBufferSizeHeart    = vitalSignsParamCLI.winLen_heartRate;
    obj->circularBufferSizeBreath   = vitalSignsParamCLI.winLen_breathing;
    obj->rxAntennaProcess           = vitalSignsParamCLI.rxAntennaProcess;
    obj->alpha_breathing            = vitalSignsParamCLI.alpha_breathingWfm;
    obj->alpha_heart                = vitalSignsParamCLI.alpha_heartWfm;
    obj->scale_breathingWfm         = vitalSignsParamCLI.scale_breathingWfm;
    obj->scale_heartWfm             = vitalSignsParamCLI.scale_heartWfm;

    // Make sure that the RX-channel to process is valid
    obj->rxAntennaProcess = (obj->rxAntennaProcess < 1) ? 1 : obj->rxAntennaProcess;
    obj->rxAntennaProcess = (obj->rxAntennaProcess > obj->numRxAntennas) ? obj->numRxAntennas : obj->rxAntennaProcess;

    obj->samplingFreq_Hz = (float) 1000 / (obj->framePeriodicity_ms); // 1000 to convert from ms to seconds
    obj->freqIncrement_Hz = obj->samplingFreq_Hz  / PHASE_FFT_SIZE;

    obj->noiseImpulse_Thresh   = 1.5;
    obj->motionDetection_Thresh = obj->cliCfg->motionDetectionParamsCfg.threshold;      //1.0;   // Threshold for Large Motions
    obj->motionDetection_BlockSize = obj->cliCfg->motionDetectionParamsCfg.blockSize;   //20;    // Block Size for Large Motion

    float rangeStart_meter = vitalSignsParamCLI.startRange_m;    // Get the Start Range. Read from the configuration file
    float rangeEnd_meter   = vitalSignsParamCLI.endRange_m;      // Get the End Range. Read from the configuration file

    obj->rangeBinStartIndex = floor(rangeStart_meter/(obj->rangeBinSize_meter));    // Range-bin index corresponding to the Starting range in meters
    obj->rangeBinEndIndex   = floor(rangeEnd_meter/(obj->rangeBinSize_meter));      // Range-bin index corresponding to the Ending range in meters
    obj->numRangeBinProcessed = obj->rangeBinEndIndex - obj->rangeBinStartIndex + 1;          // Number of range bins that can be processed

    obj->breathingWfm_Spectrum_FftSize = PHASE_FFT_SIZE;
    obj->heartWfm_Spectrum_FftSize = PHASE_FFT_SIZE;

    obj->scale_breathingWfm = (obj->scale_breathingWfm == 0) ? 10000 : obj->scale_breathingWfm;
    obj->scale_heartWfm     = (obj->scale_heartWfm == 0) ? 10000 : obj->scale_heartWfm;

    // Vital-signs peak search frequencies may be different from the Band-pass filter cut-off frequencies
    obj->breath_startFreq_Hz = 0.1;   // Breathing-Rate peak search Start-Frequency
    obj->breath_endFreq_Hz   = 0.6;   // Breathing-Rate peak search End-Frequency

    obj->heart_startFreq_Hz = 0.8;    // Heart-Rate peak search Start-Frequency
    obj->heart_endFreq_Hz   = 2.0;    // Heart-Rate peak search End-Frequency

    obj->heart_startFreq_Index = floor(obj->heart_startFreq_Hz / obj->freqIncrement_Hz);
    obj->heart_endFreq_Index   = ceil(obj->heart_endFreq_Hz / obj->freqIncrement_Hz);

    obj->breath_startFreq_Index = floor(obj->breath_startFreq_Hz / obj->freqIncrement_Hz);
    obj->breath_endFreq_Index   = ceil(obj->breath_endFreq_Hz / obj->freqIncrement_Hz);
    obj->heart_startFreq_Index_1p6Hz = floor(1.6/obj->freqIncrement_Hz);
    obj->heart_endFreq_Index_4Hz   = ceil(4.0/obj->freqIncrement_Hz);

    obj->confMetric_spectrumHeart_IndexStart  = floor(obj->heart_startFreq_Hz/obj->freqIncrement_Hz);
    obj->confMetric_spectrumHeart_IndexEnd    = ceil( obj->heart_endFreq_Hz/obj->freqIncrement_Hz);
    obj->confMetric_spectrumBreath_IndexStart = floor(obj->breath_startFreq_Hz/obj->freqIncrement_Hz);
    obj->confMetric_spectrumBreath_IndexEnd   = ceil( obj->breath_endFreq_Hz/obj->freqIncrement_Hz);
    obj->confMetric_spectrumHeart_IndexStart_1p6Hz  = obj->heart_startFreq_Index_1p6Hz;
    obj->confMetric_spectrumHeart_IndexStart_4Hz  = obj->heart_endFreq_Index_4Hz;
    obj->confMetric_numIndexAroundPeak_heart  = floor(CONF_METRIC_BANDWIDTH_PEAK_HEART_HZ/obj->freqIncrement_Hz);
    obj->confMetric_numIndexAroundPeak_breath = floor(CONF_METRIC_BANDWIDTH_PEAK_BREATH_HZ/obj->freqIncrement_Hz);

    obj->scaleFactor_PhaseToDisp = WAVELENGTH_MM/(4*PI_);

    // Auto-Correlation
    obj->xCorr_minLag = (uint16_t) obj->samplingFreq_Hz/2.1;    // (Fs/Freq)  Corresponding to f = 2.1 Hz at a sampling Frequency of 20 Hz
    obj->xCorr_maxLag = (uint16_t) obj->samplingFreq_Hz/0.8;    // (Fs/Freq)  Corresponding to f = 0.8 Hz at a sampling Frequency of 20 Hz
    obj->xCorr_Breath_minLag = (uint16_t) obj->samplingFreq_Hz/obj->breath_endFreq_Hz;    // (Fs/Freq)  Corresponding to f = 0.6 Hz at a sampling Frequency of 20 Hz
    obj->xCorr_Breath_maxLag = (uint16_t) obj->samplingFreq_Hz/obj->breath_startFreq_Hz;    // (Fs/Freq)  Corresponding to f = 0.1 Hz at a sampling Frequency of 20 Hz

    obj->peakDistanceBreath_Max = (uint16_t) obj->samplingFreq_Hz/(obj->breath_startFreq_Hz);
    obj->peakDistanceBreath_Min = (uint16_t) obj->samplingFreq_Hz/(obj->breath_endFreq_Hz);
    obj->peakDistanceHeart_Max  = (uint16_t) obj->samplingFreq_Hz/(obj->heart_startFreq_Hz);
    obj->peakDistanceHeart_Min  = (uint16_t) obj->samplingFreq_Hz/(obj->heart_endFreq_Hz);
}

void MmwDemo_dataPathDeleteSemaphore(MmwDemo_DSS_dataPathContext_t *context)
{
#ifdef EDMA_1D_INPUT_BLOCKING
    Semaphore_delete(&context->EDMA_1D_InputDone_semHandle[0]);
    Semaphore_delete(&context->EDMA_1D_InputDone_semHandle[1]);
#endif
#ifdef EDMA_1D_OUTPUT_BLOCKING
    Semaphore_delete(&context->EDMA_1D_OutputDone_semHandle[0]);
    Semaphore_delete(&context->EDMA_1D_OutputDone_semHandle[1]);
#endif

#if 0 //OD DEMO
#ifdef EDMA_2D_INPUT_BLOCKING
    Semaphore_delete(&context->EDMA_2D_InputDone_semHandle[0]);
    Semaphore_delete(&context->EDMA_2D_InputDone_semHandle[1]);
#endif
#ifdef EDMA_2D_OUTPUT_BLOCKING
    Semaphore_delete(&context->EDMA_DetMatrix_semHandle);
#endif
#ifdef EDMA_MATRIX2_INPUT_BLOCKING
    Semaphore_delete(&context->EDMA_DetMatrix2_semHandle);
#endif
#ifdef EDMA_3D_INPUT_BLOCKING
    Semaphore_delete(&context->EDMA_3D_InputDone_semHandle[0]);
    Semaphore_delete(&context->EDMA_3D_InputDone_semHandle[1]);
#endif
#endif
}

int32_t MmwDemo_dataPathInitEdma(MmwDemo_DSS_dataPathContext_t *context)
{
    Semaphore_Params       semParams;
    uint8_t numInstances;
    int32_t errorCode;
    EDMA_Handle handle;
    EDMA_errorConfig_t errorConfig;
    uint32_t instanceId;
    EDMA_instanceInfo_t instanceInfo;

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
#ifdef EDMA_1D_INPUT_BLOCKING
    context->EDMA_1D_InputDone_semHandle[0] = Semaphore_create(0, &semParams, NULL);
    context->EDMA_1D_InputDone_semHandle[1] = Semaphore_create(0, &semParams, NULL);
#endif
#ifdef EDMA_1D_OUTPUT_BLOCKING
    context->EDMA_1D_OutputDone_semHandle[0] = Semaphore_create(0, &semParams, NULL);
    context->EDMA_1D_OutputDone_semHandle[1] = Semaphore_create(0, &semParams, NULL);
#endif

#if 0 //OD DEMO
#ifdef EDMA_2D_INPUT_BLOCKING
    context->EDMA_2D_InputDone_semHandle[0] = Semaphore_create(0, &semParams, NULL);
    context->EDMA_2D_InputDone_semHandle[1] = Semaphore_create(0, &semParams, NULL);
#endif
#ifdef EDMA_2D_OUTPUT_BLOCKING
    context->EDMA_DetMatrix_semHandle = Semaphore_create(0, &semParams, NULL);
#endif
#ifdef EDMA_MATRIX2_INPUT_BLOCKING
    context->EDMA_DetMatrix2_semHandle = Semaphore_create(0, &semParams, NULL);
#endif
#ifdef EDMA_3D_INPUT_BLOCKING
    context->EDMA_3D_InputDone_semHandle[0] = Semaphore_create(0, &semParams, NULL);
    context->EDMA_3D_InputDone_semHandle[1] = Semaphore_create(0, &semParams, NULL);
#endif
#endif

    numInstances = EDMA_getNumInstances();

    /* Initialize the edma instance to be tested */
    for(instanceId = 0; instanceId < numInstances; instanceId++) {
        EDMA_init(instanceId);

        handle = EDMA_open(instanceId, &errorCode, &instanceInfo);
        if (handle == NULL)
        {
            System_printf("Error: Unable to open the edma Instance, erorCode = %d\n", errorCode);
            return -1;
        }
        context->edmaHandle[instanceId] = handle;

        errorConfig.isConfigAllEventQueues = true;
        errorConfig.isConfigAllTransferControllers = true;
        errorConfig.isEventQueueThresholdingEnabled = true;
        errorConfig.eventQueueThreshold = EDMA_EVENT_QUEUE_THRESHOLD_MAX;
        errorConfig.isEnableAllTransferControllerErrors = true;
        errorConfig.callbackFxn = MmwDemo_edmaErrorCallbackFxn;
        errorConfig.transferControllerCallbackFxn = MmwDemo_edmaTransferControllerErrorCallbackFxn;
        if ((errorCode = EDMA_configErrorMonitoring(handle, &errorConfig)) != EDMA_NO_ERROR)
        {
            System_printf("Debug: EDMA_configErrorMonitoring() failed with errorCode = %d\n", errorCode);
            return -1;
        }
    }
    return 0;
}

void MmwDemo_printHeapStats(char *name, uint32_t heapUsed, uint32_t heapSize)
{
    System_printf("Heap %s : size %d (0x%x), free %d (0x%x)\n", name, heapSize, heapSize,
        heapSize - heapUsed, heapSize - heapUsed);
}


#define SOC_MAX_NUM_RX_ANTENNAS SYS_COMMON_NUM_RX_CHANNEL
#define SOC_MAX_NUM_TX_ANTENNAS SYS_COMMON_NUM_TX_ANTENNAS

void MmwDemo_dataPathComputeDerivedConfig(MmwDemo_DSS_DataPathObj *obj)
{
    obj->log2NumDopplerBins = MmwDemo_floorLog2(obj->numDopplerBins);

    /* check for numDopplerBins to be exact power of 2 */
    if ((1U << obj->log2NumDopplerBins) != obj->numDopplerBins)
    {
        System_printf("Number of doppler bins must be a power of 2\n");
        MmwDemo_dssAssert(0);
    }
}

void MmwDemo_dataPathConfigBuffers(MmwDemo_DSS_DataPathObj *obj, uint32_t adcBufAddress)
{
/* below defines for debugging purposes, do not remove as overlays can be hard to debug */
//#define NO_L1_ALLOC /* don't allocate from L1D, use L2 instead */
#define NO_OVERLAY  /* do not overlay */

#define ALIGN(x,a)  (((x)+((a)-1))&~((a)-1))

#ifdef NO_OVERLAY
#define MMW_ALLOC_BUF(name, nameType, startAddr, alignment, size) \
        obj->name = (nameType *) ALIGN(prev_end, alignment); \
        prev_end = (uint32_t)obj->name + (size) * sizeof(nameType);
#else
#define MMW_ALLOC_BUF(name, nameType, startAddr, alignment, size) \
        obj->name = (nameType *) ALIGN(startAddr, alignment); \
        uint32_t name##_end = (uint32_t)obj->name + (size) * sizeof(nameType);

#define MMW_ALLOC_BUF_NO_END(name, nameType, startAddr, alignment) \
        obj->name = (nameType *) ALIGN(startAddr, alignment);
#endif

    uint32_t heapUsed;
    uint32_t heapL1start = (uint32_t) &gMmwL1[0];
    uint32_t heapL2start = (uint32_t) &gMmwL2[0];
    uint32_t heapL3start = (uint32_t) &gMmwL3[0];

    /* L3 is overlaid with one-time only accessed code. Although heap is not
       required to be initialized to 0, it may help during debugging when viewing memory
       in CCS */
    memset((void *)heapL3start, 0, sizeof(gMmwL3));

#ifdef NO_L1_ALLOC
    heapL1start = heapL2start;
#endif
#ifdef NO_OVERLAY
    uint32_t prev_end = heapL1start;
#endif

    MMW_ALLOC_BUF(adcDataIn, cmplx16ReIm_t,
		heapL1start, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
        2 * obj->numRangeBins);
    memset((void *)obj->adcDataIn, 0, 2 * obj->numRangeBins * sizeof(cmplx16ReIm_t));

    MMW_ALLOC_BUF(dstPingPong, cmplx16ReIm_t,
		heapL1start, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
        2 * obj->numDopplerBins);

#ifndef NO_L1_ALLOC
#ifdef NO_OVERLAY
    heapUsed = prev_end  - heapL1start;
#else
    heapUsed = MAX(MAX(MAX(sumAbs_end, adcDataIn_end),
                     azimuthMagSqr_end), detObj2DRaw_end) - heapL1start;
#endif
    MmwDemo_dssAssert(heapUsed <= MMW_L1_HEAP_SIZE);
    MmwDemo_printHeapStats("L1", heapUsed, MMW_L1_HEAP_SIZE);
#endif
    /* L2 allocation
            {
                { 1D
                    (fftOut1D)
                } |
                { 2D + 3D
                   (cfarDetObjIndexBuf + detDopplerLines.dopplerLineMask) + sumAbsRange
                }
            } +
            {
                twiddle16x16_1D +
                window1D +
                twiddle32x32_2D +
                window2D +
                detObj2D +
                detObj2dAzimIdx +
                azimuthTwiddle32x32 +
                azimuthModCoefs
            }
        */
#ifdef NO_L1_ALLOC
#ifdef NO_OVERLAY
    heapL2start = prev_end;
#else
    heapL2start = MAX(MAX(sumAbs_end, adcDataIn_end), azimuthMagSqr_end);
#endif
#else
#ifdef NO_OVERLAY
    prev_end = heapL2start;
#endif
#endif

    MMW_ALLOC_BUF(fftOut1D, cmplx16ReIm_t,
		heapL2start, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
        2 * obj->numRxAntennas * obj->numRangeBins);

    MMW_ALLOC_BUF(cfarDetObjIndexBuf, uint16_t,
        heapL2start, sizeof(uint16_t),
        MAX(obj->numRangeBins, obj->numDopplerBins));

    /* Expansion of below macro (macro cannot be used due to x.y type reference)
            MMW_ALLOC_BUF(detDopplerLines.dopplerLineMask, uint32_t,
                cfarDetObjIndexBuf_end, MMWDEMO_MEMORY_ALLOC_MAX_STRUCT_ALIGN,
                MAX((obj->numDopplerBins>>5),1));
    */

#ifdef NO_OVERLAY
    obj->detDopplerLines.dopplerLineMask = (uint32_t *) ALIGN(prev_end,
		SYS_MEMORY_ALLOC_MAX_STRUCT_BYTE_ALIGNMENT_DSP);
    prev_end = (uint32_t)obj->detDopplerLines.dopplerLineMask  +
                MAX((obj->numDopplerBins>>5),1) * sizeof(uint32_t);
#else
    obj->detDopplerLines.dopplerLineMask = (uint32_t *) ALIGN(cfarDetObjIndexBuf_end,
		SYS_MEMORY_ALLOC_MAX_STRUCT_BYTE_ALIGNMENT_DSP);
    uint32_t detDopplerLines_dopplerLineMask_end = (uint32_t)obj->detDopplerLines.dopplerLineMask  +
                MAX((obj->numDopplerBins>>5),1) * sizeof(uint32_t);
#endif

    obj->detDopplerLines.dopplerLineMaskLen = MAX((obj->numDopplerBins>>5),1);

    MMW_ALLOC_BUF(sumAbsRange, uint16_t,
        detDopplerLines_dopplerLineMask_end, sizeof(uint16_t),
        2 * obj->numRangeBins);

    MMW_ALLOC_BUF(twiddle16x16_1D, cmplx16ReIm_t,
        MAX(fftOut1D_end, sumAbsRange_end),
		SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
        obj->numRangeBins);

    MMW_ALLOC_BUF(window1D, int16_t,
		twiddle16x16_1D_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
        obj->numAdcSamples / 2);

	MMW_ALLOC_BUF(twiddle32x32_2D, cmplx32ReIm_t,
		window1D_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
		obj->numDopplerBins);

    MMW_ALLOC_BUF(window2D, int32_t,
        window1D_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
        obj->numDopplerBins / 2);

    MMW_ALLOC_BUF(dcRangeSigMean, cmplx32ImRe_t,
                  window2D_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
        SOC_MAX_NUM_TX_ANTENNAS * SOC_MAX_NUM_RX_ANTENNAS * DC_RANGE_SIGNATURE_COMP_MAX_BIN_SIZE);

#ifdef NO_OVERLAY
    heapUsed = prev_end - heapL2start;
#else
    heapUsed = dcRangeSigMean_end - heapL2start;
#endif
    MmwDemo_dssAssert(heapUsed <= MMW_L2_HEAP_SIZE);
    MmwDemo_printHeapStats("L2", heapUsed, MMW_L2_HEAP_SIZE);

    /* L3 allocation:
        ADCdataBuf (for unit test) +
        radarCube +
        azimuthStaticHeatMap +
        detMatrix
    */
#ifdef NO_OVERLAY
    prev_end = heapL3start;
#endif
    MMW_ALLOC_BUF(ADCdataBuf, cmplx16ReIm_t,
        heapL3start, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP, obj->numRangeBins * obj->numRxAntennas * obj->numTxAntennas);

    if (adcBufAddress != NULL)
    {
        obj->ADCdataBuf = (cmplx16ReIm_t *)adcBufAddress;
#ifndef NO_OVERLAY
        ADCdataBuf_end = heapL3start;
#endif
    }


    MMW_ALLOC_BUF(radarCube, cmplx16ReIm_t,
        ADCdataBuf_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
        obj->numRangeBins * obj->numDopplerBins * obj->numRxAntennas *
        obj->numTxAntennas);

    MMW_ALLOC_BUF(pRangeProfileCplx, cmplx16ReIm_t,
                  radarCube_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->numRangeBins);

    MMW_ALLOC_BUF(pVitalSigns_Breath_CircularBuffer, float,
                  pRangeProfileCplx_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->circularBufferSizeBreath);

    MMW_ALLOC_BUF(pVitalSigns_Heart_CircularBuffer, float,
                  pVitalSigns_Breath_CircularBuffer_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->circularBufferSizeHeart);

    MMW_ALLOC_BUF(pMotionCircularBuffer, float,
                  pVitalSigns_Heart_CircularBuffer_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->motionDetection_BlockSize);

    MMW_ALLOC_BUF(pDopplerWindow, float,
                  pMotionCircularBuffer_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  DOPPLER_WINDOW_SIZE);

    MMW_ALLOC_BUF(pVitalSigns_SpectrumCplx, cmplx32ReIm_t,
                  pDopplerWindow_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->breathingWfm_Spectrum_FftSize);

    MMW_ALLOC_BUF(pVitalSignsBuffer_Cplx, cmplx32ReIm_t,
                  pVitalSigns_SpectrumCplx_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->breathingWfm_Spectrum_FftSize);

    MMW_ALLOC_BUF(pVitalSignsSpectrumTwiddle32x32, cmplx32ReIm_t,
                  pVitalSignsBuffer_Cplx_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->breathingWfm_Spectrum_FftSize);

    MMW_ALLOC_BUF(pVitalSigns_Breath_AbsSpectrum, float,
                  pVitalSignsSpectrumTwiddle32x32_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->breathingWfm_Spectrum_FftSize);

    MMW_ALLOC_BUF(pVitalSigns_Heart_AbsSpectrum, float,
                  pVitalSigns_Breath_AbsSpectrum_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->heartWfm_Spectrum_FftSize);

    MMW_ALLOC_BUF(pFilterCoefsBreath, float,
                  pVitalSigns_Heart_AbsSpectrum_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  IIR_FILTER_BREATH_NUM_STAGES * IIR_FILTER_COEFS_SECOND_ORDER);

    MMW_ALLOC_BUF(pScaleValsBreath, float,
                  pFilterCoefsBreath_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  IIR_FILTER_BREATH_NUM_STAGES + 1);

    MMW_ALLOC_BUF(pFilterCoefsHeart_4Hz, float,
                  pScaleValsBreath_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  IIR_FILTER_HEART_NUM_STAGES * IIR_FILTER_COEFS_SECOND_ORDER);

    MMW_ALLOC_BUF(pScaleValsHeart_4Hz, float,
                  pFilterCoefsHeart_4Hz_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  IIR_FILTER_HEART_NUM_STAGES + 1);

    MMW_ALLOC_BUF(pXcorr, float,
                  pScaleValsHeart_4Hz_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  XCORR_NUM_LAGS);

    MMW_ALLOC_BUF(pTempReal_Prev, float,
                  pXcorr_4Hz_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->numRangeBins);

    MMW_ALLOC_BUF(pTempImag_Prev, float,
                  pTempReal_Prev_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->numRangeBins);

    MMW_ALLOC_BUF(pRangeProfileClutterRemoved, float,
                  pTempImag_Prev_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->numRangeBins);

    MMW_ALLOC_BUF(pDataOutTemp, float,
                  pTempImag_Prev_end, SYS_MEMORY_ALLOC_DOUBLE_WORD_ALIGN_DSP,
                  obj->heartWfm_Spectrum_FftSize);

//////////////  Parameters :  IIR-Cascade Bandpass Filter  //////////////////////////////////////

    // 0.1 Hz to 0.5 Hz Bandpass Filter coefficients
     float pFilterCoefsBreath[IIR_FILTER_BREATH_NUM_STAGES * IIR_FILTER_COEFS_SECOND_ORDER] = {
                                            1.0000, 0, -1.0000, 1.0000, -1.9632, 0.9644,
                                            1.0000, 0, -1.0000, 1.0000, -1.8501, 0.8681 };
     float pScaleValsBreath[IIR_FILTER_BREATH_NUM_STAGES + 1] = {0.0602, 0.0602, 1.0000};
     memcpy(obj->pFilterCoefsBreath, pFilterCoefsBreath, sizeof(pFilterCoefsBreath));
     memcpy(obj->pScaleValsBreath, pScaleValsBreath, sizeof(pScaleValsBreath));

    // Heart Beat Rate    0.8 - 4.0 Hz
    float pFilterCoefsHeart_4Hz[IIR_FILTER_HEART_NUM_STAGES * IIR_FILTER_COEFS_SECOND_ORDER] = {
                                               1.0000, 0, -1.0000, 1.0000, -0.5306, 0.5888,
                                               1.0000, 0, -1.0000, 1.0000, -1.8069, 0.8689,
                                               1.0000, 0, -1.0000, 1.0000, -1.4991, 0.5887,
                                               1.0000, 0, -1.0000, 1.0000, -0.6654, 0.2099 };
    float pScaleValsHeart_4Hz[IIR_FILTER_HEART_NUM_STAGES + 1] = {0.4188, 0.4188, 0.3611, 0.3611, 1.0000};
    memcpy(obj->pFilterCoefsHeart_4Hz, pFilterCoefsHeart_4Hz, sizeof(pFilterCoefsHeart_4Hz));
    memcpy(obj->pScaleValsHeart_4Hz, pScaleValsHeart_4Hz, sizeof(pScaleValsHeart_4Hz));

    memset(obj->pDelayHeart, 0, sizeof(obj->pDelayHeart));
    memset(obj->pDelayBreath,0, sizeof(obj->pDelayBreath));

    float DopplerWindowCoefs [DOPPLER_WINDOW_SIZE] =  { 0.0800, 0.0894, 0.1173, 0.1624,
                                                        0.2231, 0.2967, 0.3802, 0.4703,
                                                        0.5633, 0.6553, 0.7426, 0.8216,
                                                        0.8890, 0.9422, 0.9789, 0.9976 };
    memcpy(obj->pDopplerWindow, DopplerWindowCoefs, sizeof(DopplerWindowCoefs));

    float pFilterCoefs[FIR_FILTER_SIZE] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };
    memcpy(obj->pFilterCoefs, pFilterCoefs, FIR_FILTER_SIZE*sizeof(float));

#ifdef NO_OVERLAY
    heapUsed = prev_end - heapL3start;
#else
    heapUsed = detMatrix_end - heapL3start;
#endif
    MmwDemo_dssAssert(heapUsed <= sizeof(gMmwL3));
    MmwDemo_printHeapStats("L3", heapUsed, sizeof(gMmwL3));
}


void MmwDemo_dataPathConfigFFTs(MmwDemo_DSS_DataPathObj *obj)
{

    MmwDemo_genWindow((void *)obj->window1D,
                        FFT_WINDOW_INT16,
                        obj->numAdcSamples,
                        obj->numAdcSamples/2,
                        ONE_Q15,
                        MMW_WIN_BLACKMAN);

    /* Generate twiddle factors for 1D FFT. This is one time */
    MmwDemo_gen_twiddle_fft16x16_fast((int16_t *)obj->twiddle16x16_1D, obj->numRangeBins);
	
	/* Generate twiddle factors for vital signs FFT operation. This is one time */
    MmwDemo_gen_twiddle_fft32x32_fast((int32_t *)obj->pVitalSignsSpectrumTwiddle32x32, obj->breathingWfm_Spectrum_FftSize, 2147483647.5);
}

/**
 *  @b Description
 *  @n
 *      Function generates a single FFT window samples. It calls single precision
 *      sine and cosine functions from mathlib library for the first sample only, and then recursively
 *      generates cosine values for other samples.
 *
 *  @param[out] win             Pointer to calculated window samples.
 *  @param[in]  windowDatumType Window samples data format. For windowDatumType = @ref FFT_WINDOW_INT16,
 *              the samples format is int16_t. For windowDatumType = @ref FFT_WINDOW_INT32,
 *              the samples format is int32_t.
 *  @param[in]  winLen          Nominal window length
 *  @param[in]  winGenLen       Number of generated samples
 *  @param[in]  oneQformat      Q format of samples, oneQformat is the value of
 *                              one in the desired format.
 *  @param[in]  winType         Type of window, one of @ref MMW_WIN_BLACKMAN, @ref MMW_WIN_HANNING,
 *              or @ref MMW_WIN_RECT.
 *  @retval none.
 */
void MmwDemo_genWindow(void *win,
                        uint32_t windowDatumType,
                        uint32_t winLen,
                        uint32_t winGenLen,
                        int32_t oneQformat,
                        uint32_t winType)
{
    uint32_t winIndx;
    int32_t winVal;
    int16_t * win16 = (int16_t *) win;
    int32_t * win32 = (int32_t *) win;

    float eR = 1.;
    float eI = 0.;
    float e2R = 1.;
    float e2I = 0.;
    float ephyR, ephyI;
    float e2phyR, e2phyI;
    float tmpR;

    float phi = 2 * PI_ / ((float) winLen - 1);


    ephyR  = cossp(phi);
    ephyI  = sinsp(phi);

    e2phyR  = ephyR * ephyR - ephyI * ephyI;
    e2phyI  = 2 * ephyR * ephyI;


    if(winType == MMW_WIN_BLACKMAN)
    {
        //Blackman window
        float a0 = 0.42;
        float a1 = 0.5;
        float a2 = 0.08;
        for(winIndx = 0; winIndx < winGenLen; winIndx++)
        {
            winVal = (int32_t) ((oneQformat * (a0 - a1*eR + a2*e2R)) + 0.5);
            if(winVal >= oneQformat)
            {
                winVal = oneQformat - 1;
            }
            if (windowDatumType == FFT_WINDOW_INT16)
            {
                win16[winIndx] = (int16_t) winVal;
             }
            if (windowDatumType == FFT_WINDOW_INT32)
            {
                win32[winIndx] = (int32_t) winVal;
             }
            tmpR = eR;
            eR = eR * ephyR - eI * ephyI;
            eI = tmpR * ephyI + eI * ephyR;

            tmpR = e2R;
            e2R = e2R * e2phyR - e2I * e2phyI;
            e2I = tmpR * e2phyI + e2I * e2phyR;
        }
    }
    else if (winType == MMW_WIN_HANNING)
    {
        //Hanning window
        for(winIndx = 0; winIndx < winGenLen; winIndx++)
        {
            winVal = (int32_t) ((oneQformat * 0.5* (1 - eR)) + 0.5);
            if(winVal >= oneQformat)
            {
                winVal = oneQformat - 1;
            }
            if (windowDatumType == FFT_WINDOW_INT16)
            {
                win16[winIndx] = (int16_t) winVal;
            }
            if (windowDatumType == FFT_WINDOW_INT32)
            {
                win32[winIndx] = (int32_t) winVal;
            }
            tmpR = eR;
            eR = eR*ephyR - eI*ephyI;
            eI = tmpR*ephyI + eI*ephyR;
        }
    }
    else if (winType == MMW_WIN_RECT)
    {
        //Rectangular window
        for(winIndx = 0; winIndx < winGenLen; winIndx++)
        {
            if (windowDatumType == FFT_WINDOW_INT16)
            {
                win16[winIndx] =  (int16_t)  (oneQformat-1);
            }
            if (windowDatumType == FFT_WINDOW_INT32)
            {
                win32[winIndx] = (int32_t) (oneQformat-1);
            }
        }
    }
}

void MmwDemo_checkDynamicConfigErrors(MmwDemo_DSS_DataPathObj *obj)
{
// Nothing to do in the vital signs demo. Keeping it in-case needed in future versions
#if 0
    MmwDemo_CliCfg_t *cliCfg = obj->cliCfg;

    MmwDemo_dssAssert(!( (cliCfg->extendedMaxVelocityCfg.enabled == 1) &&
                         (cliCfg->multiObjBeamFormingCfg.enabled == 1)
                       ));

    MmwDemo_dssAssert(!( (cliCfg->extendedMaxVelocityCfg.enabled == 1) &&
                         (cliCfg->nearFieldCorrectionCfg.enabled == 1)
                       ));

    MmwDemo_dssAssert(!( (cliCfg->extendedMaxVelocityCfg.enabled == 1) &&
                         (obj->numTxAntennas == 1)
                       ));
#endif
}
