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

#include "ir_manager.h"
#include "wav_parser.h"
#include "app_conf.h"
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <zephyr/fs/fs.h>
#include <zephyr/drivers/disk.h>
#include <ff.h>

#define PATH_SIZE 128U
#define BYTES_PER_SAMPLE_MAX 4U

/* Helper function to check if filename has .wav extension (case-insensitive) */
static int is_wav_file(const char *filename)
{
	const char *ext;
	size_t len;

	if (filename == NULL) {
		return 0;
	}

	len = strlen(filename);
	if (len < 4) {
		return 0;  /* Too short to have .wav extension */
	}

	ext = filename + len - 4;  /* Point to last 4 characters */

	/* Case-insensitive comparison */
	return (ext[0] == '.') &&
	       (ext[1] == 'w' || ext[1] == 'W') &&
	       (ext[2] == 'a' || ext[2] == 'A') &&
	       (ext[3] == 'v' || ext[3] == 'V');
}

/* Zephyr FS mount point for SD card */
static FATFS fat_fs;
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = "/SD:",
};

static struct fs_dir_t dir;
static bool dir_open = false;
static char current_ir_name[64] = "default";

int ir_manager_init(void)
{
	int res;
	struct fs_dirent entry;

	/* Debug: check disk status before mount */
	printk("[SD] Calling disk_access_init...\n");
	res = disk_access_init("SD");
	printk("[SD] disk_access_init returned: %d\n", res);

	if (res == 0) {
		int status = disk_access_status("SD");
		printk("[SD] disk_access_status: %d\n", status);
	}

	res = fs_mount(&mp);
	if (res != 0) {
		printk("Failed to mount filesystem: %d\n", res);
		return -1;
	}

	fs_dir_t_init(&dir);
	res = fs_opendir(&dir, "/SD:");
	if (res != 0) {
		printk("Failed to open root directory: %d\n", res);
		return -1;
	}

	/* List all files in root directory */
	do {
		res = fs_readdir(&dir, &entry);
		if (res != 0 || entry.name[0] == 0) {
			break;
		}
		printk("%s\n", entry.name);
	} while (1);

	fs_closedir(&dir);

	/* Re-open directory for iteration by getNextIR */
	fs_dir_t_init(&dir);
	res = fs_opendir(&dir, "/SD:");
	if (res != 0) {
		printk("Failed to reopen directory: %d\n", res);
		return -1;
	}
	dir_open = true;

	return 0;
}

int ir_manager_getNextIR(int *ir, unsigned int *length, unsigned int *fs)
{
	int res;
	int i;
	int j;
	const unsigned char *ptr;
	WAV_Data wavData;
	unsigned int maxReadSize;
	unsigned char shift;
	struct fs_dirent entry;
	struct fs_file_t file;
	char filepath[PATH_SIZE];
	ssize_t bytes_read;

	printk("[IR] getNextIR: Entry\n");

	if (!dir_open) {
		printk("[IR] getNextIR: SD card not initialized\n");
		return -1;
	}

	if (ir == NULL || length == NULL || fs == NULL) {
		printk("[IR] getNextIR: Invalid parameters\n");
		return -1;
	}

	/* Find next .wav file in directory */
	printk("[IR] getNextIR: Starting file search\n");
	do {
		res = fs_readdir(&dir, &entry);
		if (res != 0 || entry.name[0] == 0) {
			/* No more files found, wrap around to beginning */
			printk("[IR] getNextIR: No files, wrapping around\n");
			fs_closedir(&dir);
			fs_dir_t_init(&dir);
			res = fs_opendir(&dir, "/SD:");
			if (res != 0) {
				printk("[IR] getNextIR: Failed to reopen directory: %d\n", res);
				return -1;
			}
			continue;
		}

		printk("[IR] getNextIR: Found file: %s\n", entry.name);

		/* Double-check that it's a .wav file (case-insensitive) */
		if (!is_wav_file(entry.name)) {
			/* Skip non-.wav files and continue searching */
			printk("[IR] getNextIR: Skipping non-wav file\n");
			continue;
		}

		/* Found a valid .wav file */
		printk("[IR] getNextIR: Valid wav file found\n");
		break;
	} while (1);

	printk("[IR] getNextIR: Selected %s\n", entry.name);
	snprintf(current_ir_name, sizeof(current_ir_name), "%s", entry.name);

	/* Build full path with mount point */
	snprintf(filepath, sizeof(filepath), "/SD:/%s", entry.name);

	fs_file_t_init(&file);
	res = fs_open(&file, filepath, FS_O_READ);
	if (res != 0) {
		printk("[IR] getNextIR: Failed to open file: %d\n", res);
		return -1;
	}

	printk("[IR] getNextIR: File opened successfully\n");

	/* Use file size from directory entry (avoid fs_stat which can
	 * corrupt FATFS sector cache on Cortex-M7 with D-cache) */
	size_t fsize = entry.size;

	/* Calculate maximum readable size */
	maxReadSize = (fsize > WAV_BUF_MAX_LEN) ? WAV_BUF_MAX_LEN : (unsigned int)fsize;
	printk("[IR] getNextIR: maxReadSize=%u\n", maxReadSize);
	if (fsize > (IR_MAX_LEN * BYTES_PER_SAMPLE_MAX)) {
		printk("[IR] getNextIR: IR will be truncated to %u\n", maxReadSize);
	}

	ptr = (const unsigned char *)wav_parser_getWavBuff();
	printk("[IR] getNextIR: Reading file, size=%u\n", maxReadSize);
	bytes_read = fs_read(&file, (void *)ptr, maxReadSize);
	if (bytes_read < 0) {
		printk("[IR] getNextIR: Failed to read file: %zd\n", bytes_read);
		fs_close(&file);
		return -1;
	}

	printk("[IR] getNextIR: Read %zd bytes\n", bytes_read);
	if ((unsigned int)bytes_read != maxReadSize) {
		printk("[IR] getNextIR: Warning: read %zd bytes, expected %u\n", bytes_read, maxReadSize);
	}
	fs_close(&file);

	printk("[IR] getNextIR: Parsing WAV data\n");
	res = wav_parser_parseWav(&wavData);
	if (res != 0) {
		printk("[IR] getNextIR: Failed to parse WAV file: %d\n", res);
		return -1;
	}
	printk("[IR] getNextIR: WAV parsed successfully\n");

	/* Validate format code */
	printk("[IR] getNextIR: Validating format, code=0x%04x\n", wavData.formatCode);
	if (wavData.formatCode == WAVE_FORMAT_PCM) {
		printk("[IR] getNextIR: PCM wave format\n");
	} else if (wavData.formatCode == WAVE_FORMAT_IEEE_FLOAT) {
		printk("[IR] getNextIR: IEEE FLOAT format (unsupported)\n");
		return -1;
	} else if (wavData.formatCode == WAVE_FORMAT_ALAW) {
		printk("[IR] getNextIR: ALAW format (unsupported)\n");
		return -1;
	} else if (wavData.formatCode == WAVE_FORMAT_MULAW) {
		printk("[IR] getNextIR: MULAW format (unsupported)\n");
		return -1;
	} else if (wavData.formatCode == WAVE_FORMAT_EXTENSIBLE) {
		printk("[IR] getNextIR: Extensible format (unsupported)\n");
		return -1;
	} else {
		printk("[IR] getNextIR: Unknown format: 0x%04x\n", wavData.formatCode);
		return -1;
	}

	printk("[IR] getNextIR: Channels=%u, Samples=%u, bps=%u, blockAlign=%u\n",
	       wavData.nChannels, wavData.n, wavData.bps, wavData.blockAlign);
	if (wavData.nChannels > 1) {
		printk("[IR] getNextIR: Unsupported number of channels %u\n", wavData.nChannels);
		return -1;
	}

	/* Validate sample count */
	if (wavData.n > IR_MAX_LEN) {
		printk("[IR] getNextIR: Sample count %u exceeds IR_MAX_LEN %d\n", wavData.n, IR_MAX_LEN);
		return -1;
	}

	if (wavData.blockAlign == 0 || wavData.blockAlign > BYTES_PER_SAMPLE_MAX) {
		printk("[IR] getNextIR: Invalid block align: %u\n", wavData.blockAlign);
		return -1;
	}
	printk("[IR] getNextIR: Validation passed\n");

	/* Convert samples from byte array to integer array */
	printk("[IR] getNextIR: Converting samples, n=%u, blockAlign=%u\n", wavData.n, wavData.blockAlign);
	ptr = (const unsigned char *)wavData.samples;
	for (i = 0; i < (int)wavData.n; i++) {
		ir[i] = 0;
		for (j = 0; j < (int)wavData.blockAlign; j++) {
			ir[i] += (int)ptr[j + (i * (int)wavData.blockAlign)] << (j * 8);
		}
	}
	printk("[IR] getNextIR: Sample conversion complete\n");

	/* Convert bit size if needed */
	printk("[IR] getNextIR: Bit size conversion check\n");
	if (wavData.bps != DSP_BITSIZE) {
		if (DSP_BITSIZE > wavData.bps) {
			shift = DSP_BITSIZE - wavData.bps;
			printk("[IR] getNextIR: Upscaling by %u bits\n", shift);
			for (i = 0; i < (int)wavData.n; i++) {
				ir[i] = ir[i] << shift;
			}
		} else {
			shift = wavData.bps - DSP_BITSIZE;
			printk("[IR] getNextIR: Downscaling by %u bits\n", shift);
			for (i = 0; i < (int)wavData.n; i++) {
				ir[i] = ir[i] >> shift;
			}
		}
		printk("[IR] getNextIR: Bit size conversion complete\n");
	}

	printk("[IR] getNextIR: Setting output parameters, length=%u, fs=%u\n", wavData.n, wavData.nSamplesPerSec);
	*length = wavData.n;
	*fs = wavData.nSamplesPerSec;

	printk("[IR] getNextIR: Success, returning 0\n");
	return 0;
}

const char *ir_manager_getCurrentIRName(void)
{
	return current_ir_name;
}
