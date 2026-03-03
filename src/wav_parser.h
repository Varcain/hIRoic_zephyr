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

#ifndef INC_WAV_PARSER_H_
#define INC_WAV_PARSER_H_

#include <stdint.h>

struct wav_riff_header {
	uint32_t chunk_id;      /* "RIFF" = 0x46464952 */
	uint32_t chunk_size;
	uint32_t format;        /* "WAVE" = 0x45564157 */
} __packed;

struct wav_fmt_chunk {
	uint32_t chunk_id;      /* "fmt " = 0x20746D66 */
	uint32_t chunk_size;
	uint16_t audio_format;
	uint16_t num_channels;
	uint32_t sample_rate;
	uint32_t byte_rate;
	uint16_t block_align;
	uint16_t bits_per_sample;
} __packed;

struct wav_data_chunk {
	uint32_t chunk_id;      /* "data" = 0x61746164 */
	uint32_t chunk_size;
} __packed;

typedef enum {
	WAVE_FORMAT_PCM = 0x0001,
	WAVE_FORMAT_IEEE_FLOAT = 0x0003,
	WAVE_FORMAT_ALAW = 0x0006,
	WAVE_FORMAT_MULAW = 0x0007,
	WAVE_FORMAT_EXTENSIBLE = 0xFFFE,
} Format_Code;

typedef struct E_WAV_Data {
	Format_Code formatCode;
	unsigned int nChannels;
	unsigned int nSamplesPerSec;
	unsigned int blockAlign;
	unsigned int bps;
	unsigned int n;
	void *samples;
} WAV_Data;

unsigned char *wav_parser_getWavBuff(void);
int wav_parser_parseWav(WAV_Data *wavData);

#endif /* INC_WAV_PARSER_H_ */
