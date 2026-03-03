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

#ifndef INC_APP_CONF_H_
#define INC_APP_CONF_H_

/* ========================================================================= */
/* AUDIO TEST MODES (enable ONE or NONE)                                     */
/* ========================================================================= */

/**
 * Audio processing test modes:
 * - Define AUDIO_TEST_MODE_ECHO to bypass DSP and echo RX to TX unchanged
 * - Define neither for normal DSP processing (IR convolution)
 */
//#define AUDIO_TEST_MODE_ECHO 1

/* ========================================================================= */
/* DSP CONFIGURATION                                                         */
/* ========================================================================= */

#define DSP_BUFFER_SIZE			512
#define DSP_BITSIZE				32
#define DSP_RATE				44100
#define IR_MAX_LEN				1048U
#define COEF_MAX                IR_MAX_LEN
/* FFT-based convolution configuration */
/* Use FFT when IR length exceeds this threshold */
#define FFT_CONVOLUTION_THRESHOLD 512U
#define WAV_BUF_MAX_LEN			((IR_MAX_LEN + 64) * 4)

#endif /* INC_APP_CONF_H_ */
