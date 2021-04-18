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

<<<<<<< HEAD:Marlin/src/libs/private_spi.h
=======
#ifndef __MARLIN_SPI_H__
#define __MARLIN_SPI_H__

#include <stdint.h>
>>>>>>> 1314b31d97bba8cd74c6625c47176d4692f57790:Marlin/MarlinSPI.h
#include "softspi.h"
#include <stdint.h>

template<uint8_t MisoPin, uint8_t MosiPin, uint8_t SckPin>
class SPIclass {
  static SoftSPI<MisoPin, MosiPin, SckPin> softSPI;
  public:
    FORCE_INLINE static void init() { softSPI.begin(); }
    FORCE_INLINE static void send(uint8_t data) { softSPI.send(data); }
    FORCE_INLINE static uint8_t receive() { return softSPI.receive(); }
};

// Hardware SPI
template<>
class SPIclass<MISO_PIN, MOSI_PIN, SCK_PIN> {
  public:
    FORCE_INLINE static void init() {
      OUT_WRITE(SCK_PIN, LOW);
      OUT_WRITE(MOSI_PIN, HIGH);
      SET_INPUT_PULLUP(MISO_PIN);
    }
    FORCE_INLINE static uint8_t receive() {
<<<<<<< HEAD:Marlin/src/libs/private_spi.h
      #if defined(__AVR__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
        SPDR = 0;
        for (;!TEST(SPSR, SPIF););
        return SPDR;
      #else
        return spiRec();
      #endif
=======
      SPDR = 0;
      while (!TEST(SPSR, SPIF)) { /* nada */ }
      return SPDR;
>>>>>>> 1314b31d97bba8cd74c6625c47176d4692f57790:Marlin/MarlinSPI.h
    }
};
<<<<<<< HEAD:Marlin/src/libs/private_spi.h
=======

#endif // __MARLIN_SPI_H__
>>>>>>> 1314b31d97bba8cd74c6625c47176d4692f57790:Marlin/MarlinSPI.h
