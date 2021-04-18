/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

<<<<<<< HEAD:Marlin/src/HAL/TEENSY31_32/inc/Conditionals_post.h
#if USE_FALLBACK_EEPROM
  #define USE_WIRED_EEPROM 1
=======
#ifndef __TYPES_H__
#define __TYPES_H__

#include <stdint.h>

typedef unsigned long millis_t;

typedef struct {
  int8_t x_index, y_index;
  float distance; // When populated, the distance from the search location
} mesh_index_pair;

>>>>>>> 1314b31d97bba8cd74c6625c47176d4692f57790:Marlin/types.h
#endif
