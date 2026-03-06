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
#include "ir_convert.h"
#include "app_conf.h"
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <zephyr/fs/fs.h>
#include <ff.h>

#define PATH_SIZE 128U

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

	res = fs_mount(&mp);
	if (res != 0) {
		printk("Failed to mount filesystem: %d\n", res);
		return -1;
	}

	fs_dir_t_init(&dir);
	res = fs_opendir(&dir, "/SD:");
	if (res != 0) {
		printk("Failed to open directory: %d\n", res);
		return -1;
	}
	dir_open = true;

	printk("IR manager initialized\n");
	return 0;
}

int ir_manager_getNextIR(int *ir, unsigned int *length, unsigned int *fs)
{
	int res;
	const unsigned char *ptr;
	WAV_Data wavData;
	unsigned int maxReadSize;
	struct fs_dirent entry;
	struct fs_file_t file;
	char filepath[PATH_SIZE];
	ssize_t bytes_read;
	int wrap_count = 0;

	if (!dir_open) {
		printk("getNextIR: SD card not initialized\n");
		return -1;
	}

	if (ir == NULL || length == NULL || fs == NULL) {
		return -1;
	}

	/* Find next .wav file in directory */
	do {
		res = fs_readdir(&dir, &entry);
		if (res != 0 || entry.name[0] == 0) {
			/* No more files found, wrap around to beginning */
			if (++wrap_count > 1) {
				printk("getNextIR: no WAV files found\n");
				return -1;
			}
			fs_closedir(&dir);
			fs_dir_t_init(&dir);
			res = fs_opendir(&dir, "/SD:");
			if (res != 0) {
				printk("getNextIR: failed to reopen directory: %d\n", res);
				return -1;
			}
			continue;
		}

		if (!is_wav_file(entry.name)) {
			continue;
		}

		break;
	} while (1);

	printk("IR selected: %s\n", entry.name);
	snprintf(current_ir_name, sizeof(current_ir_name), "%s", entry.name);

	/* Build full path with mount point */
	snprintf(filepath, sizeof(filepath), "/SD:/%s", entry.name);

	fs_file_t_init(&file);
	res = fs_open(&file, filepath, FS_O_READ);
	if (res != 0) {
		printk("getNextIR: failed to open file: %d\n", res);
		return -1;
	}

	/* Use file size from directory entry (avoid fs_stat which can
	 * corrupt FATFS sector cache on Cortex-M7 with D-cache) */
	size_t fsize = entry.size;

	/* Calculate maximum readable size */
	maxReadSize = (fsize > WAV_BUF_MAX_LEN) ? WAV_BUF_MAX_LEN
	                                         : (unsigned int)fsize;

	ptr = (const unsigned char *)wav_parser_getWavBuff();
	bytes_read = fs_read(&file, (void *)ptr, maxReadSize);
	if (bytes_read < 0) {
		printk("getNextIR: failed to read file: %zd\n", bytes_read);
		fs_close(&file);
		return -1;
	}
	fs_close(&file);

	res = wav_parser_parseWav(&wavData, (unsigned int)bytes_read);
	if (res != 0) {
		printk("getNextIR: failed to parse WAV: %d\n", res);
		return -1;
	}

	res = ir_convert_samples(&wavData, ir, length, fs);
	if (res != 0) {
		printk("getNextIR: failed to convert samples\n");
		return -1;
	}

	printk("IR loaded: %u samples @ %u Hz\n", *length, *fs);
	return 0;
}

const char *ir_manager_getCurrentIRName(void)
{
	return current_ir_name;
}
