/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Melzi pin assignments
 */

#define BOARD_NAME "Melzi"

#if defined(__AVR_ATmega1284P__)
  #define LARGE_FLASH true
#endif

#define SANGUINOLOLU_V_1_2
/**
* Marlin 3D Printer Firmware
* Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
*
* Based on Sprinter and grbl.
* Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*/

/**
* Sanguinololu board pin assignments
*/

#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega1284P__)
  #error "Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu."
#endif

#ifndef BOARD_NAME
  #define BOARD_NAME "Wanhao Melzi V1.0"
#endif

#define IS_MELZI (MB(MELZI) || MB(MELZI_MAKR3D))

//
// Steppers
//
#define X_STEP_PIN         15
#define X_DIR_PIN          21
#define X_MIN_PIN          18

#define Y_STEP_PIN         22
#define Y_DIR_PIN          23
#define Y_MIN_PIN          19

#define Z_STEP_PIN         3
#define Z_DIR_PIN          2
#define Z_MIN_PIN          20

#define E0_STEP_PIN         1
#define E0_DIR_PIN          0

//
// Temperature Sensors
//
#define TEMP_0_PIN          7   // Analog Input (pin 33 extruder)
#define TEMP_BED_PIN        6   // Analog Input (pin 34 bed)

//
// Heaters / Fans
//
#define HEATER_0_PIN       13 // (extruder)
#define HEATER_BED_PIN     10 // (bed)

#define X_ENABLE_PIN     14
#define Y_ENABLE_PIN     14
#define Z_ENABLE_PIN     26
#define E0_ENABLE_PIN    14
#define FAN_PIN           4
#define LED_PIN          27 // On some broken versions of the Sanguino libraries the pin definitions are wrong, so LED_PIN needs to be 28. But you should upgrade your Sanguino libraries! See #368.

