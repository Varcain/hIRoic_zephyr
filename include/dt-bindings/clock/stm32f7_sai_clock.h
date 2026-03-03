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
#ifndef STM32F7_SAI_CLOCK_H_
#define STM32F7_SAI_CLOCK_H_

#include <zephyr/dt-bindings/clock/stm32f7_clock.h>

/* SAI clock source selection (DCKCFGR1 register)
 * These are not yet in upstream Zephyr's stm32f7_clock.h */
#define SAI1_SEL(val)	STM32_DT_CLOCK_SELECT((val), 21, 20, DCKCFGR1_REG)
#define SAI2_SEL(val)	STM32_DT_CLOCK_SELECT((val), 23, 22, DCKCFGR1_REG)

#endif /* STM32F7_SAI_CLOCK_H_ */
