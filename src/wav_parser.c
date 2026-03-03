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

#include <zephyr/sys/printk.h>
#include <stdint.h>
#include "wav_parser.h"
#include "app_conf.h"

/* WAV file chunk identifiers */
#define CHUNK_ID_RIFF    0x46464952U  /* "RIFF" */
#define CHUNK_ID_WAVE    0x45564157U  /* "WAVE" */
#define CHUNK_ID_FMT     0x20746D66U  /* "fmt " */
#define CHUNK_ID_FACT    0x74636166U  /* "fact" */
#define CHUNK_ID_DATA    0x61746164U  /* "data" */

/* WAV format chunk constants */
#define FMT_CHUNK_SIZE_MIN  16U
#define CHUNK_HEADER_SIZE   8U  /* 4 bytes ID + 4 bytes size */

static unsigned char wavBuffer[WAV_BUF_MAX_LEN] __aligned(4);

unsigned char *wav_parser_getWavBuff(void)
{
	return wavBuffer;
}

int wav_parser_parseWav(WAV_Data *wavData)
{
	const struct wav_riff_header *riff;
	const struct wav_fmt_chunk *fmt = NULL;
	const struct wav_data_chunk *data = NULL;
	const unsigned char *samples = NULL;
	uint32_t offset;
	uint32_t end;

	if (wavData == NULL) {
		printk("Invalid WAV_Data pointer\n");
		return -1;
	}

	/* Parse RIFF header */
	riff = (const struct wav_riff_header *)wavBuffer;

	if (riff->chunk_id != CHUNK_ID_RIFF) {
		printk("Bad RIFF chunk ID\n");
		return -1;
	}

	if (riff->format != CHUNK_ID_WAVE) {
		printk("Bad WAVE ID\n");
		return -1;
	}

	end = sizeof(struct wav_riff_header) + riff->chunk_size
	      - sizeof(riff->format);
	offset = sizeof(struct wav_riff_header);

	/* Walk sub-chunks */
	while (offset + CHUNK_HEADER_SIZE <= end) {
		uint32_t ck_id = *(const uint32_t *)&wavBuffer[offset];
		uint32_t ck_size = *(const uint32_t *)&wavBuffer[offset + 4];

		if (ck_id == CHUNK_ID_FMT) {
			if (ck_size < FMT_CHUNK_SIZE_MIN) {
				printk("fmt chunk too small: %u\n", ck_size);
				return -1;
			}
			fmt = (const struct wav_fmt_chunk *)&wavBuffer[offset];
			printk("fmt chunk: format=%x ch=%u rate=%u bps=%u\n",
			       fmt->audio_format, fmt->num_channels,
			       fmt->sample_rate, fmt->bits_per_sample);
		} else if (ck_id == CHUNK_ID_FACT) {
			printk("fact chunk (compressed audio not supported)\n");
			return -1;
		} else if (ck_id == CHUNK_ID_DATA) {
			data = (const struct wav_data_chunk *)&wavBuffer[offset];
			samples = &wavBuffer[offset + CHUNK_HEADER_SIZE];
			printk("data chunk: %u bytes\n", data->chunk_size);
			break;
		} else {
			printk("skipping unknown chunk 0x%08x (%u bytes)\n",
			       ck_id, ck_size);
		}

		offset += CHUNK_HEADER_SIZE + ck_size;
	}

	/* Sanity checks */
	if (fmt == NULL) {
		printk("Missing fmt chunk\n");
		return -1;
	}

	if (fmt->audio_format != WAVE_FORMAT_PCM) {
		printk("Unsupported audio format: 0x%x\n", fmt->audio_format);
		return -1;
	}

	if (fmt->bits_per_sample != 16 && fmt->bits_per_sample != 24
	    && fmt->bits_per_sample != 32) {
		printk("Unsupported bits per sample: %u\n", fmt->bits_per_sample);
		return -1;
	}

	if (fmt->num_channels < 1) {
		printk("Invalid channel count: %u\n", fmt->num_channels);
		return -1;
	}

	if (fmt->sample_rate == 0) {
		printk("Invalid sample rate\n");
		return -1;
	}

	uint16_t expected_block_align = fmt->num_channels
					* (fmt->bits_per_sample / 8);
	if (fmt->block_align != expected_block_align) {
		printk("block_align mismatch: got %u, expected %u\n",
		       fmt->block_align, expected_block_align);
		return -1;
	}

	uint32_t expected_byte_rate = fmt->sample_rate * fmt->block_align;
	if (fmt->byte_rate != expected_byte_rate) {
		printk("byte_rate mismatch: got %u, expected %u\n",
		       fmt->byte_rate, expected_byte_rate);
		return -1;
	}

	if (data == NULL || samples == NULL || data->chunk_size == 0) {
		printk("No sample data found\n");
		return -1;
	}

	/* Calculate number of samples and apply IR_MAX_LEN truncation */
	uint32_t dataCkSize = data->chunk_size;
	uint32_t numSamples = dataCkSize / fmt->block_align;

	if (numSamples > IR_MAX_LEN) {
		printk("WAV truncated from %u to %u samples\n",
		       numSamples, IR_MAX_LEN);
		numSamples = IR_MAX_LEN;
	}

	wavData->formatCode = (Format_Code)fmt->audio_format;
	wavData->nChannels = fmt->num_channels;
	wavData->nSamplesPerSec = fmt->sample_rate;
	wavData->blockAlign = fmt->block_align;
	wavData->bps = fmt->bits_per_sample;
	wavData->n = numSamples;
	wavData->samples = (void *)samples;

	printk("Done parsing!\n");
	return 0;
}
