# Copyright (C) 2026 Kamil Lulko <kamil.lulko@gmail.com>
#
# This file is part of hIRoic_zephyr.
#
# hIRoic_zephyr is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# hIRoic_zephyr is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with hIRoic_zephyr. If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.20.0)

# Add out-of-tree WM8994 driver
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers/wm8994)

# Add app-level DTS include path (for dt-bindings/clock/stm32f7_sai_clock.h)
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hIRoic)

target_sources(app PRIVATE
    src/main.c
    src/dsp.c
    src/ir_manager.c
    src/wav_parser.c
)
