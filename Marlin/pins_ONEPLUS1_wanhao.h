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
 * Wanhao OnePlus1 pin assignments
 * https://www.aliexpress.com/item/motherboard-i3mini-0ne-motherboard-New-2017-Wanhao-printer-i3-Mini/32849200836.html?
 */

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#ifndef BOARD_NAME
  #define BOARD_NAME "Wanhao OnePlus1"
#endif

#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "i3 Mini"
#endif

//
// Steppers
//
#define X_STEP_PIN         22
#define X_DIR_PIN          23
#define X_ENABLE_PIN       57
#define X_MAX_PIN          19

#define Y_STEP_PIN         25
#define Y_DIR_PIN          26
#define Y_ENABLE_PIN       24
#define Y_MIN_PIN          18

#define Z_STEP_PIN         29
#define Z_DIR_PIN          39
#define Z_ENABLE_PIN       28
#define Z_MIN_PIN          38

#define E0_STEP_PIN        55
#define E0_DIR_PIN         56
#define E0_ENABLE_PIN      54

//
// Temperature Sensors
//
#define TEMP_0_PIN         13   // ANALOG NUMBERING
#define TEMP_BED_PIN       14   // ANALOG NUMBERING

//
// Heaters / Fans
//
#define HEATER_0_PIN        4
#define HEATER_BED_PIN     44   // BED
#define FAN_PIN            12   // IO pin. Buffer needed

//
// LCD Display input pins
//

// Pins for DOGM SPI LCD Support
#define DOGLCD_A0          40
#define DOGLCD_CS          41
#define LCD_BACKLIGHT_PIN  65 // Backlight LED on A11/D65
#define LCD_RESET_PIN      27
#define SDSS               53

#define KILL_PIN           64
#define LCD_CONTRAST      255

//
// LCD / Controller
//
#define BEEPER_PIN 37

// Buttons
#define BTN_EN1             2
#define BTN_EN2             3
#define BTN_ENC             5  // The click switch

//
// Misc. Functions
//
#define SD_DETECT_BYMYSELF
#define SD_DETECT_PIN_INIT	DDRD &= ~_BV(PIND6)
#define SD_DETECT_SET(x)	if(x)PORTD|=_BV(PIND6);else PORTD&=~_BV(PIND6)
#define SD_DETECT_PIN_VAL	(!(bool)(PIND&_BV(PIND6)))

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13
