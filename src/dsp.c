/*
 * Copyright (C) 2026 Kamil Lulko <kamil.lulko@gmail.com>
 *
 * This file is part of hIRoic_zephyr.
 *
 * hIRoic_zephyr is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * hIRoic_zephyr is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with hIRoic_zephyr. If not, see <https://www.gnu.org/licenses/>.
 */

#include "dsp.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "app_conf.h"
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* Calculate maximum FFT size needed: next power of 2 >= (DSP_BUFFER_SIZE + COEF_MAX - 1) */
/* This is used for static buffer allocation */
/* Helper macro to calculate next power of 2 at compile time */
#define NEXT_POWER_OF_2(n) \
	((n) <= 16 ? 16 : \
	 (n) <= 32 ? 32 : \
	 (n) <= 64 ? 64 : \
	 (n) <= 128 ? 128 : \
	 (n) <= 256 ? 256 : \
	 (n) <= 512 ? 512 : \
	 (n) <= 1024 ? 1024 : \
	 (n) <= 2048 ? 2048 : \
	 (n) <= 4096 ? 4096 : 8192)

#define FFT_SIZE_MAX NEXT_POWER_OF_2(DSP_BUFFER_SIZE + COEF_MAX - 1)

/* Overlap-add: output is (FFT_SIZE - DSP_BUFFER_SIZE) samples longer */
/* This will be calculated at runtime based on actual FFT size */

static const q31_t coef[460] = {
1661309, 4930648, 7210357, 7969177, 7723002, 5464464, 1984651, -794875,
-2796633, -3648160, -2653684, -836352, 464958, 1365160, 1972757, 1848331,
1180310, 731415, 717919, 922550, 1275760, 1582588, 1453341, 1049706,
603812, 209076, 107894, 214546, 481429, 761787, 712907, 274775,
59021, 104408, -34271, -191000, 2029, 330768, 529940, 744128,
939604, 984893, 819031, 480627, 155519, 58231, -11306, -169842,
-225034, -269277, -327872, -277797, -167466, 6559, 216122, 344417,
401998, 392012, 287314, 120829, 32078, -42779, -147295, -219221,
-331460, -517333, -633749, -585849, -381404, -107821, 112302, 173400,
140034, 73189, -98481, -327596, -533992, -682236, -691383, -586081,
-377214, -135576, 11395, 35484, -27636, -94583, -156194, -260365,
-372756, -506796, -729598, -1014327, -1227488, -1324328, -1308442, -1147178,
-898656, -675775, -434568, -186190, -34530, -12042, -62766, -169751,
-247655, -224762, -154416, -86099, -34684, -90304, -291921, -577258,
-854364, -1042891, -1125208, -1102981, -960011, -697752, -373692, -68320,
129467, 147499, 38310, -135702, -336464, -508343, -613859, -656458,
-632766, -590802, -554812, -539828, -535479, -552653, -575539, -551254,
-479208, -388482, -280932, -188891, -148600, -187472, -282625, -403278,
-506668, -556003, -574908, -596398, -606886, -621437, -657285, -683028,
-676190, -669889, -663549, -635631, -592443, -551325, -517988, -519606,
-555615, -596282, -626946, -662941, -682516, -685371, -686464, -672695,
-626150, -584311, -557855, -526945, -479535, -425720, -372816, -341146,
-330796, -335401, -337198, -344065, -339651, -309170, -264948, -212230,
-153116, -114407, -105583, -126220, -164109, -205744, -244452, -276518,
-286997, -283497, -277057, -291742, -335523, -397582, -442113, -447574,
-410165, -354889, -298079, -251432, -213423, -184756, -163122, -148486,
-133147, -129174, -133040, -138719, -151226, -175940, -193406, -195089,
-174635, -140435, -106748, -85264, -78475, -84803, -88275, -92405,
-106339, -131515, -152085, -162595, -158666, -144259, -121234, -99659,
-75353, -56763, -60452, -96771, -142862, -183279, -202670, -199021,
-183345, -164038, -139892, -121563, -103946, -84017, -64141, -47798,
-20249, 25239, 83609, 134201, 165847, 169888, 165044, 164379,
164671, 158121, 154150, 158238, 182871, 229292, 280488, 305316,
300563, 284789, 277340, 272924, 261241, 228559, 183737, 142701,
123369, 129259, 157078, 188428, 218321, 250473, 284389, 309007,
315436, 296496, 269260, 247927, 240640, 235263, 221857, 200672,
182100, 167269, 165164, 172153, 174442, 165984, 164527, 175148,
196368, 219246, 244733, 263761, 265484, 244305, 211447, 177163,
158047, 154144, 161598, 173965, 190889, 212654, 243821, 273860,
302251, 330552, 354080, 360813, 347935, 312144, 270099, 235188,
226173, 257372, 312481, 348572, 346610, 313908, 279990, 261089,
253705, 250530, 253648, 256113, 260114, 261465, 262653, 266349,
278073, 301732, 342623, 391215, 434624, 448272, 431477, 400675,
379375, 374685, 383163, 392672, 398699, 393113, 376135, 347456,
314434, 282313, 262187, 252215, 250340, 256549, 271266, 284861,
292427, 300183, 321547, 343605, 349700, 332540, 296851, 248744,
199731, 159437, 139757, 137221, 143568, 149163, 161016, 184377,
214094, 231485, 235729, 234797, 239760, 248530, 261567, 278008,
300532, 320650, 340412, 358180, 365448, 351189, 324367, 300180,
294239, 307454, 329377, 337892, 325254, 298077, 271287, 246776,
226206, 210965, 204223, 205353, 222389, 246375, 265331, 271910,
269346, 262684, 261642, 265233, 268404, 263395, 249472, 221831,
193124, 175676, 171894, 175409, 181147, 187081, 196383, 204761,
214513, 223996, 230166, 230470, 231660, 236498, 249634, 263781,
272982, 271766, 259570, 231948, 191941, 148674, 118520, 105736,
107432, 114281, 125499, 142958, 160892, 165746, 162931, 162017,
169763, 181232, 197076, 214944, 225715, 223426, 219779, 219325,
218365, 209609, 192655, 161471
};

/* Frequency-domain convolution structures */
/* Buffers sized for maximum FFT size, but actual size is determined at runtime */
static q31_t ir_fft[FFT_SIZE_MAX * 2];  /* Complex FFT of IR (interleaved real/imag) */
static q31_t input_fft[FFT_SIZE_MAX * 2];  /* Complex FFT of input block */
static q31_t output_fft[FFT_SIZE_MAX * 2];  /* Complex FFT of output */
static q31_t overlap_buffer[FFT_SIZE_MAX];  /* Overlap-add buffer (max size) */
static q31_t work_buffer[FFT_SIZE_MAX];  /* Shared: input staging + output extraction */

static unsigned int ir_length = 0;
static unsigned int coefSize;
static unsigned int fft_size = 0;  /* Runtime FFT size */
static unsigned int overlap_size = 0;  /* Runtime overlap size */
static const arm_cfft_instance_q31 *fft_instance = NULL;  /* FFT function instance */

/* Calculate next power of 2 >= n, capped at 4096 (max supported FFT size) */
static unsigned int calculate_next_power_of_2(unsigned int n)
{
	unsigned int power = 1;
	unsigned int max_power = 4096;  /* Maximum supported FFT size */

	while (power < n && power < max_power) {
		power <<= 1;
	}

	/* If n exceeds max_power, return max_power */
	if (n > max_power) {
		return max_power;
	}

	return power;
}

/* Get FFT instance based on size */
static const arm_cfft_instance_q31 *get_fft_instance(unsigned int size)
{
	switch (size) {
		case 16:   return &arm_cfft_sR_q31_len16;
		case 32:   return &arm_cfft_sR_q31_len32;
		case 64:   return &arm_cfft_sR_q31_len64;
		case 128:  return &arm_cfft_sR_q31_len128;
		case 256:  return &arm_cfft_sR_q31_len256;
		case 512:  return &arm_cfft_sR_q31_len512;
		case 1024: return &arm_cfft_sR_q31_len1024;
		case 2048: return &arm_cfft_sR_q31_len2048;
		case 4096: return &arm_cfft_sR_q31_len4096;
		default:
			printk("Error: Unsupported FFT size %u\n", size);
			return NULL;
	}
}

/* Calculate and set FFT size based on IR length */
static void set_fft_size(unsigned int ir_len)
{
	unsigned int min_fft_size = DSP_BUFFER_SIZE + ir_len - 1;
	unsigned int max_supported_fft = 4096;

	/* Check if we can fit in a supported FFT size */
	if (min_fft_size > max_supported_fft) {
		printk("Error: Required FFT size %u exceeds maximum %u\n",
		       min_fft_size, max_supported_fft);
		fft_size = 0;
		overlap_size = 0;
		fft_instance = NULL;
		return;
	}

	fft_size = calculate_next_power_of_2(min_fft_size);
	overlap_size = fft_size - DSP_BUFFER_SIZE;
	fft_instance = get_fft_instance(fft_size);

	if (fft_instance == NULL) {
		printk("Error: Cannot find FFT instance for size %u\n", fft_size);
		fft_size = 0;
		overlap_size = 0;
	}
}

void dsp_init(void)
{
	coefSize = sizeof(coef) / sizeof(coef[0]);

	/* Initialize overlap buffer to zero */
	memset(overlap_buffer, 0, sizeof(overlap_buffer));

	/* Load default IR (bitSize=32, fs=44100) - coef is already in Q31 format */
	dsp_loadIR((const int *)coef, coefSize, 24, 44100);
}

static void dsp_process_freq_domain(int16_t *out, int16_t *in)
{
	unsigned int i;

	if (fft_instance == NULL || fft_size == 0) {
		/* Fallback: just copy input to output */
		for (i = 0; i < DSP_BUFFER_SIZE; i++) {
			out[i] = in[i];
		}
		return;
	}

	/* Convert 16-bit input to Q31 format (left shift by 16) */
	for (i = 0; i < DSP_BUFFER_SIZE; i++) {
		work_buffer[i] = (q31_t)in[i] << 16;
	}

	/* Zero-pad input to fft_size */
	memset(&work_buffer[DSP_BUFFER_SIZE], 0,
	       (fft_size - DSP_BUFFER_SIZE) * sizeof(q31_t));

	/* Convert real input to complex format (interleaved real/imag) */
	/* Real part in even indices, imag part (zero) in odd indices */
	for (i = 0; i < fft_size; i++) {
		input_fft[i * 2] = work_buffer[i];      /* Real part */
		input_fft[i * 2 + 1] = 0;                 /* Imaginary part */
	}

	/* Convert input to frequency domain (FFT) */
	arm_cfft_q31(fft_instance, input_fft, 0, 1);

	/* Multiply with IR frequency response (complex multiply) */
	arm_cmplx_mult_cmplx_q31(input_fft, ir_fft, output_fft, fft_size);

	for (i = 0; i < fft_size; i++) {
		output_fft[i] = output_fft[i] << 9;
	}

	/* Convert back to time domain (IFFT) */
	arm_cfft_q31(fft_instance, output_fft, 1, 1);

	for (i = 0; i < fft_size; i++) {
		output_fft[i] = output_fft[i] << 8;
	}

	/* Extract real part from complex output */
	for (i = 0; i < fft_size; i++) {
		work_buffer[i] = output_fft[i * 2];
	}

	/* Overlap-add: add previous overlap to current output (with saturation) */
	arm_add_q31(work_buffer, overlap_buffer, work_buffer, overlap_size);

	/* Save overlap for next block */
	for (i = 0; i < overlap_size; i++) {
		overlap_buffer[i] = work_buffer[DSP_BUFFER_SIZE + i];
	}

	/* Convert Q31 output back to 16-bit with saturation */
	for (i = 0; i < DSP_BUFFER_SIZE; i++) {
		q31_t val = work_buffer[i];
		/* Compensate for remaining FFT scaling and convert to int16 */
		int64_t scaled = ((int64_t)val * 32) >> 16;
		/* Clamp to int16_t range: -32768 to 32767 */
		if (scaled > 32767) {
			out[i] = 32767;
		} else if (scaled < -32768) {
			out[i] = -32768;
		} else {
			out[i] = (int16_t)scaled;
		}
	}
}

void dsp_process(int16_t *out, int16_t *in, unsigned int size)
{
	(void)size;  /* Parameter not used, size is always DSP_BUFFER_SIZE */

	dsp_process_freq_domain(out, in);
}

void dsp_loadIR(const int *ir, unsigned int length, unsigned int bitSize, unsigned int fs)
{
	unsigned int i;
	unsigned int actualLength = length;

	if (ir == NULL || length == 0) {
		return;
	}

	if (length > COEF_MAX) {
		printk("IR length %u exceeds maximum %u\n", length, COEF_MAX);
		actualLength = COEF_MAX;
	}

	if (fs != DSP_RATE) {
		printk("Warning: IR sample rate %u != DSP rate %u (resampling not implemented)\n",
		       fs, DSP_RATE);
	}

	ir_length = actualLength;

	/* Calculate and set FFT size based on IR length */
	set_fft_size(actualLength);

	if (fft_instance == NULL) {
		printk("Error: Failed to initialize FFT for IR length %u\n", actualLength);
		return;
	}

	/* Copy IR to padded buffer and zero-pad to fft_size */
	for (i = 0; i < actualLength; i++) {
		work_buffer[i] = (q31_t)ir[i];
	}
	memset(&work_buffer[actualLength], 0,
	       (fft_size - actualLength) * sizeof(q31_t));

	/* Convert real IR to complex format (interleaved real/imag) */
	for (i = 0; i < fft_size; i++) {
		ir_fft[i * 2] = work_buffer[i];      /* Real part */
		ir_fft[i * 2 + 1] = 0;                /* Imaginary part */
	}

	/* Convert IR to frequency domain (FFT) */
	arm_cfft_q31(fft_instance, ir_fft, 0, 1);

	/* Clear overlap buffer when loading new IR */
	memset(overlap_buffer, 0, overlap_size * sizeof(q31_t));

	printk("IR loaded: %u samples, FFT %u, ir[0..3]=%d %d %d %d\n",
	       actualLength, fft_size,
	       (int)work_buffer[0], (int)work_buffer[1],
	       (int)work_buffer[2], (int)work_buffer[3]);
}
