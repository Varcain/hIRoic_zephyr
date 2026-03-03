# hIRoic

Real-time guitar cabinet impulse response (IR) convolution processor running on
the STM32F746G Discovery board, built with [Zephyr RTOS](https://zephyrproject.org/).

## Features

- Real-time IR convolution using CMSIS-DSP (overlap-save, FFT-based)
- WAV IR loading from SD card (PCM, 16/24/32-bit, up to 1048 samples)
- WM8994 audio codec driver (I2S, 44.1 kHz, 32-bit)
- LVGL display with VU meter and IR name
- Serial console for IR cycling and bypass control

## Hardware

- **Board:** STM32F746G-DISCO
- **Codec:** WM8994 (on-board)
- **Storage:** microSD card with WAV files in root directory

## Building

Requires a [Zephyr workspace](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).

```bash
source /path/to/zephyrproject/.venv/bin/activate
west build -b stm32f746g_disco /path/to/hIRoic_zephyr --pristine
west flash --runner openocd
```

## Usage

Place `.wav` IR files on a FAT-formatted microSD card and insert it into the
board. Use the serial console (115200 baud):

- **a** - Load next IR
- **s** - Toggle bypass (dry signal passthrough)

## Project Structure

```
src/
  main.c          - Threads (audio, input, heartbeat), I2S streaming, LVGL UI
  dsp.c/h         - FFT-based overlap-save convolution (CMSIS-DSP)
  ir_manager.c/h  - SD card mounting, WAV file enumeration, IR loading
  wav_parser.c/h  - WAV file parsing with struct-cast approach
  app_conf.h      - DSP and buffer configuration constants
drivers/
  wm8994/         - Out-of-tree Zephyr audio codec driver for WM8994
boards/
  stm32f746g_disco.conf     - Kconfig for the target board
  stm32f746g_disco.overlay  - Devicetree overlay (I2S, SD card, codec)
```

## License

This project is licensed under the GNU General Public License v3.0 - see the
[LICENSE](LICENSE) file for details.

Copyright (C) 2026 Kamil Lulko <kamil.lulko@gmail.com>
