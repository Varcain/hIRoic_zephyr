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
 *
 * hIRoic - Real-time guitar cabinet IR convolution processor
 *
 * Threads:
 * - audio_thread (priority 7): i2s_read -> dsp_process -> i2s_write
 * - input_thread (priority 4): serial console, IR loading
 * - heartbeat_thread (priority 5): stats logging, LVGL label update
 * - LVGL workqueue (automatic): display rendering
 */

#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <zephyr/logging/log.h>
#include <zephyr/console/console.h>
#include <string.h>
#include <stdio.h>

#include "app_conf.h"
#include "dsp.h"
#include "ir_manager.h"

#ifdef CONFIG_LVGL
#include <lvgl.h>
#include <lvgl_display.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include "lvgl_ui_core.h"
#endif

LOG_MODULE_REGISTER(hiroic, LOG_LEVEL_DBG);

/* ========================================================================= */
/* I2S DEVICE NODES                                                          */
/* ========================================================================= */

#if DT_NODE_EXISTS(DT_NODELABEL(i2s_rxtx))
#define I2S_RX_NODE DT_NODELABEL(i2s_rxtx)
#define I2S_TX_NODE I2S_RX_NODE
#else
#define I2S_RX_NODE DT_NODELABEL(i2s_rx)
#define I2S_TX_NODE DT_NODELABEL(i2s_tx)
#endif

/* ========================================================================= */
/* AUDIO CONFIGURATION                                                       */
/* ========================================================================= */

#define SAMPLE_FREQUENCY   DSP_RATE
#define SAMPLE_BIT_WIDTH   16
#define BYTES_PER_SAMPLE   sizeof(int16_t)
#define NUMBER_OF_CHANNELS 1
#define SAMPLES_PER_BLOCK  DSP_BUFFER_SIZE
#define INITIAL_BLOCKS     4
#define TIMEOUT            1000

#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
/* Pad slab block size to cache line boundary (32 bytes) for DMA coherency */
#define SLAB_BLOCK_SIZE ((BLOCK_SIZE + 31) & ~31)
#define BLOCK_COUNT (INITIAL_BLOCKS + 4)
K_MEM_SLAB_DEFINE_IN_SECT_STATIC(mem_slab, __dtcm_noinit_section,
				 SLAB_BLOCK_SIZE, BLOCK_COUNT, 32);

/* ========================================================================= */
/* APPLICATION STATE                                                         */
/* ========================================================================= */

/* IR buffer for loading from SD card */
static int ir_buffer[IR_MAX_LEN];
static unsigned int ir_length;
static unsigned int ir_sample_rate;

/* Semaphore: gates audio thread until I2S streams are started */
static K_SEM_DEFINE(i2s_ready_sem, 0, 1);

/* Flag: when true, audio thread bypasses DSP (passthrough) during IR load */
static volatile bool ir_loading;

/* Flag: when true, audio thread bypasses DSP (user-toggled bypass) */
static volatile bool ir_bypass;

/* Processing statistics */
static volatile uint32_t total_processing_cycles;
static volatile uint32_t processing_sample_count;
static volatile uint32_t audio_overrun_count;
static volatile int16_t audio_peak_level;

/* I2S device pointers (set in main, used by threads) */
static const struct device *i2s_dev_rx;
static const struct device *i2s_dev_tx;


/* ========================================================================= */
/* I2S HELPERS                                                               */
/* ========================================================================= */

static bool configure_streams(const struct device *dev_rx, const struct device *dev_tx,
			      const struct i2s_config *config)
{
	int ret;

	if (dev_rx == dev_tx) {
		ret = i2s_configure(dev_rx, I2S_DIR_BOTH, config);
		if (ret == 0) {
			return true;
		}
		if (ret != -ENOSYS) {
			printk("Failed to configure streams: %d\n", ret);
			return false;
		}
	}

	/* RX must be slave when using separate TX/RX with synchronous mode */
	struct i2s_config rx_config = *config;
	rx_config.options = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE;

	ret = i2s_configure(dev_rx, I2S_DIR_RX, &rx_config);
	if (ret < 0) {
		printk("Failed to configure RX stream: %d\n", ret);
		return false;
	}

	ret = i2s_configure(dev_tx, I2S_DIR_TX, config);
	if (ret < 0) {
		printk("Failed to configure TX stream: %d\n", ret);
		return false;
	}

	return true;
}

static bool prepare_transfer(const struct device *dev_rx, const struct device *dev_tx)
{
	int ret;

	for (int i = 0; i < INITIAL_BLOCKS; ++i) {
		void *mem_block;

		ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
		if (ret < 0) {
			printk("Failed to allocate TX block %d: %d\n", i, ret);
			return false;
		}

		memset(mem_block, 0, BLOCK_SIZE);

		ret = i2s_write(dev_tx, mem_block, BLOCK_SIZE);
		if (ret < 0) {
			printk("Failed to write block %d: %d\n", i, ret);
			return false;
		}
	}

	return true;
}

static bool trigger_command(const struct device *dev_rx, const struct device *dev_tx,
			    enum i2s_trigger_cmd cmd)
{
	int ret;

	if (dev_rx == dev_tx) {
		ret = i2s_trigger(dev_rx, I2S_DIR_BOTH, cmd);
		if (ret == 0) {
			return true;
		}
		if (ret != -ENOSYS) {
			printk("Failed to trigger command %d: %d\n", cmd, ret);
			return false;
		}
	}

	/* TX first for START, RX first for STOP/DROP */
	if (cmd == I2S_TRIGGER_START) {
		ret = i2s_trigger(dev_tx, I2S_DIR_TX, cmd);
		if (ret < 0) {
			printk("Failed to trigger command %d on TX: %d\n", cmd, ret);
			return false;
		}
		ret = i2s_trigger(dev_rx, I2S_DIR_RX, cmd);
		if (ret < 0) {
			printk("Failed to trigger command %d on RX: %d\n", cmd, ret);
			return false;
		}
	} else {
		ret = i2s_trigger(dev_rx, I2S_DIR_RX, cmd);
		if (ret < 0) {
			printk("Failed to trigger command %d on RX: %d\n", cmd, ret);
			return false;
		}
		ret = i2s_trigger(dev_tx, I2S_DIR_TX, cmd);
		if (ret < 0) {
			printk("Failed to trigger command %d on TX: %d\n", cmd, ret);
			return false;
		}
	}

	return true;
}

static bool start_i2s_streams(void)
{
	if (!prepare_transfer(i2s_dev_rx, i2s_dev_tx)) {
		return false;
	}
	if (!trigger_command(i2s_dev_rx, i2s_dev_tx, I2S_TRIGGER_START)) {
		return false;
	}
	printk("I2S streams started\n");
	return true;
}


/* ========================================================================= */
/* AUDIO THREAD                                                              */
/* ========================================================================= */

static void audio_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("Audio thread started, waiting for I2S...\n");
	k_sem_take(&i2s_ready_sem, K_FOREVER);
	printk("Audio thread running\n");

	for (;;) {
		void *rx_block;
		uint32_t block_size;
		int ret;

		/* Block until RX data is ready */
		ret = i2s_read(i2s_dev_rx, &rx_block, &block_size);
		if (ret < 0) {
			if (ret == -EIO) {
				/* Stream was stopped (e.g. for IR reload).
				 * 50ms backoff avoids busy-looping while the stream
				 * is being restarted by the input thread. */
				k_sleep(K_MSEC(50));
				continue;
			}
			printk("Failed to read data: %d\n", ret);
			audio_overrun_count++;
			continue;
		}

		uint32_t start_cycles = k_cycle_get_32();

		/* Allocate output block */
		void *tx_block;
		ret = k_mem_slab_alloc(&mem_slab, &tx_block, K_NO_WAIT);
		if (ret < 0) {
			printk("Failed to allocate TX block: %d\n", ret);
			k_mem_slab_free(&mem_slab, rx_block);
			audio_overrun_count++;
			continue;
		}

#if defined(AUDIO_TEST_MODE_ECHO)
		/* Echo mode: copy RX to TX unchanged */
		memcpy(tx_block, rx_block, BLOCK_SIZE);
#else
		if (ir_loading || ir_bypass) {
			/* Passthrough while IR is being loaded or bypass enabled */
			memcpy(tx_block, rx_block, BLOCK_SIZE);
		} else {
			/* Normal mode: apply IR convolution */
			dsp_process((int16_t *)tx_block, (int16_t *)rx_block);
		}
#endif

		uint32_t end_cycles = k_cycle_get_32();
		total_processing_cycles += (end_cycles - start_cycles);
		processing_sample_count++;

		/* Compute peak level of output buffer (outside timing) */
		{
			int16_t peak = 0;
			int16_t *out = (int16_t *)tx_block;
			for (int i = 0; i < DSP_BUFFER_SIZE; i++) {
				int16_t abs_val = (out[i] < 0) ? -out[i] : out[i];
				if (abs_val > peak) {
					peak = abs_val;
				}
			}
			audio_peak_level = peak;
		}

		/* Free the RX block */
		k_mem_slab_free(&mem_slab, rx_block);

		/* Queue TX block for output */
		ret = i2s_write(i2s_dev_tx, tx_block, BLOCK_SIZE);
		if (ret < 0) {
			printk("Failed to write data: %d\n", ret);
			k_mem_slab_free(&mem_slab, tx_block);
			audio_overrun_count++;
		}
	}
}

K_THREAD_DEFINE(audio_thread, 4096,
		audio_thread_fn, NULL, NULL, NULL,
		7, 0, 0);

/* ========================================================================= */
/* INPUT THREAD                                                              */
/* ========================================================================= */

static void input_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("Input thread started\n");

	for (;;) {
		uint8_t c = console_getchar();

		if (c == 'a') {
			printk("Loading next IR...\n");

			/* Bypass DSP during IR load (audio passes through) */
			ir_loading = true;

			/* Load next IR from SD card */
			int res = ir_manager_getNextIR(ir_buffer, &ir_length, &ir_sample_rate);
			if (res == 0) {
				/* Reconfigure DSP with new IR */
				dsp_loadIR(ir_buffer, ir_length, ir_sample_rate);
				printk("Loaded IR: %u samples @ %u Hz\n", ir_length, ir_sample_rate);
#ifdef CONFIG_LVGL
				lv_lock();
				lvgl_ui_update_ir(ir_manager_getCurrentIRName(),
						  ir_length);
				lv_unlock();
#endif
			} else {
				printk("Failed to load IR: %d\n", res);
			}

			ir_loading = false;
		} else if (c == 's') {
			ir_bypass = !ir_bypass;
			printk("DSP bypass %s\n", ir_bypass ? "ON" : "OFF");
		}
	}
}

K_THREAD_DEFINE(input_thread, 8192,
		input_thread_fn, NULL, NULL, NULL,
		4, 0, 0);

/* ========================================================================= */
/* HEARTBEAT THREAD                                                          */
/* ========================================================================= */

static void heartbeat_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("Heartbeat thread started\n");
	uint32_t tick = 0;

	for (;;) {
		k_sleep(K_MSEC(33));
		tick++;

#ifdef CONFIG_LVGL
		/* Update VU bar every 33ms (~30 fps) */
		lv_lock();
		lvgl_ui_update_vu(audio_peak_level);
		lv_unlock();
#endif

		/* CPU stats + label every ~1s (30 ticks) */
		if (tick % 30 != 0) {
			continue;
		}

		/* Calculate average processing time */
		uint32_t avg_time_us = 0;
		uint32_t pct = 0;
		if (processing_sample_count > 0) {
			uint32_t total_us = k_cyc_to_us_floor32(total_processing_cycles);
			avg_time_us = total_us / processing_sample_count;
			/* Reset counters for next period */
			total_processing_cycles = 0;
			processing_sample_count = 0;
		}

		/* Deadline: time per buffer in microseconds */
		uint32_t deadline_us = ((uint32_t)DSP_BUFFER_SIZE * 1000000U) / (uint32_t)DSP_RATE;
		if (deadline_us > 0) {
			pct = (avg_time_us * 100U) / deadline_us;
		}

		printk("DSP: %u.%03u ms / %u.%03u ms (%u%%) | Overruns: %u\n",
		       avg_time_us / 1000, avg_time_us % 1000,
		       deadline_us / 1000, deadline_us % 1000,
		       pct,
		       audio_overrun_count);

#ifdef CONFIG_LVGL
		lv_lock();
		lvgl_ui_update_cpu(pct);
		lv_unlock();
#endif
	}
}

K_THREAD_DEFINE(heartbeat_thread, 2048,
		heartbeat_thread_fn, NULL, NULL, NULL,
		5, 0, 0);

/* ========================================================================= */
/* MAIN                                                                      */
/* ========================================================================= */

int main(void)
{
	struct i2s_config config;

	printk("hIRoic - Guitar Cabinet IR Convolution\n");
	printk("Build %s %s\n", __DATE__, __TIME__);
	printk("Config: %d-bit, %d channel, %d Hz, %d samples/block\n",
	       SAMPLE_BIT_WIDTH, NUMBER_OF_CHANNELS, SAMPLE_FREQUENCY, SAMPLES_PER_BLOCK);

	/* Initialize DSP engine (loads default 460-sample embedded IR) */
	dsp_init();

	/* Initialize console for serial input */
	console_init();

	/* Get I2S devices */
	i2s_dev_rx = DEVICE_DT_GET(I2S_RX_NODE);
	i2s_dev_tx = DEVICE_DT_GET(I2S_TX_NODE);

	if (!device_is_ready(i2s_dev_rx)) {
		printk("ERROR: %s is not ready\n", i2s_dev_rx->name);
		return 0;
	}
	printk("I2S RX device %s is ready\n", i2s_dev_rx->name);

	if (i2s_dev_rx != i2s_dev_tx) {
		if (!device_is_ready(i2s_dev_tx)) {
			printk("ERROR: %s is not ready\n", i2s_dev_tx->name);
			return 0;
		}
		printk("I2S TX device %s is ready\n", i2s_dev_tx->name);
	}

	/* Configure I2S streams */
	config.word_size = SAMPLE_BIT_WIDTH;
	config.channels = NUMBER_OF_CHANNELS;
	config.format = I2S_FMT_DATA_FORMAT_I2S;
	config.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	config.frame_clk_freq = SAMPLE_FREQUENCY;
	config.mem_slab = &mem_slab;
	config.block_size = BLOCK_SIZE;
	config.timeout = TIMEOUT;
	if (!configure_streams(i2s_dev_rx, i2s_dev_tx, &config)) {
		return 0;
	}

	/* Enable SAI2_A to generate MCLK for codec initialization.
	 * The SAI driver doesn't do this, and the WM8994 needs MCLK present
	 * before it can be configured over I2C. */
	volatile uint32_t *sai2a_cr1 = (volatile uint32_t *)0x40015C04;
	*sai2a_cr1 |= (1U << 16);  /* SAI_xCR1_SAIEN */

	/* Configure codec AFTER SAI streams so MCLK is present */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(audio_codec), okay)
	const struct device *const codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));
	struct audio_codec_cfg audio_cfg;

	audio_cfg.dai_route = AUDIO_ROUTE_PLAYBACK_CAPTURE;
	audio_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	audio_cfg.dai_cfg.i2s.word_size = SAMPLE_BIT_WIDTH;
	audio_cfg.dai_cfg.i2s.channels = NUMBER_OF_CHANNELS;
	audio_cfg.dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S;
	audio_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_MASTER;
	audio_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_FREQUENCY;
	audio_cfg.dai_cfg.i2s.mem_slab = &mem_slab;
	audio_cfg.dai_cfg.i2s.block_size = BLOCK_SIZE;
	audio_codec_configure(codec_dev, &audio_cfg);

	audio_codec_start(codec_dev, AUDIO_DAI_DIR_TX);
	audio_codec_start(codec_dev, AUDIO_DAI_DIR_RX);
	/* Allow codec analog outputs to settle after start.
	 * The WM8994 driver init already waits ~632ms internally;
	 * this extra 1s is conservative headroom for the SAI clocks
	 * and analog path to fully stabilize before streaming. */
	k_msleep(1000);
#endif

	/* Initialize IR manager (mount SD card, list files) */
	if (ir_manager_init() == 0) {
		/* Load first IR from SD card */
		if (ir_manager_getNextIR(ir_buffer, &ir_length, &ir_sample_rate) == 0) {
			dsp_loadIR(ir_buffer, ir_length, ir_sample_rate);
			printk("Initial IR: %u samples @ %u Hz\n", ir_length, ir_sample_rate);
		} else {
			printk("No IR loaded from SD, using default\n");
		}
	} else {
		printk("SD card not available, using default IR\n");
	}

#ifdef CONFIG_LVGL
	{
		const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

		if (!device_is_ready(display_dev)) {
			printk("ERROR: display device not ready\n");
		} else {
			lv_lock();
			lvgl_ui_create_widgets("hIRoic (Zephyr)");
			if (ir_length > 0) {
				lvgl_ui_update_ir(ir_manager_getCurrentIRName(),
						  ir_length);
			}
			lv_unlock();

			/* Turn on backlight */
			display_blanking_off(display_dev);

			printk("LVGL display initialized\n");
		}
	}
#endif

	/* Start I2S streaming and ungate the audio thread */
	if (!start_i2s_streams()) {
		printk("ERROR: Failed to start I2S streams\n");
		return 0;
	}
	k_sem_give(&i2s_ready_sem);

	printk("hIRoic running. Press 'a' to cycle IRs.\n");

	/* main() returns, threads continue running */
	return 0;
}
