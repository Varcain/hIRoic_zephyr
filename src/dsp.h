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

#ifndef INC_DSP_H_
#define INC_DSP_H_

#include <stdint.h>
#include "arm_math.h"

void dsp_init(void);
void dsp_process(int16_t *out, int16_t *in, unsigned int size);
void dsp_loadIR(const int *ir, unsigned int length, unsigned int bitSize, unsigned int fs);

#endif /* INC_DSP_H_ */
