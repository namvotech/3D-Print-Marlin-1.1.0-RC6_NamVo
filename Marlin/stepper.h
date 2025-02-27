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
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef stepper_h
#define stepper_h

#include "planner.h"
#include "stepper_indirection.h"

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  extern bool abort_on_endstop_hit;
#endif

// Initialize and start the stepper motor subsystem
void st_init();

// Block until all buffered steps are executed
void st_synchronize();

// Set current position in steps
void st_set_position(const long& x, const long& y, const long& z, const long& e);
void st_set_e_position(const long& e);

// Get current position in steps
long st_get_position(AxisEnum axis);

// Get current axis position in mm
float st_get_axis_position_mm(AxisEnum axis);

//NAMADD
// Get current position in mm
float st_get_position_mm(AxisEnum axis);

// The stepper subsystem goes to sleep when it runs out of things to execute. Call this
// to notify the subsystem that it is time to go to work.
void st_wake_up();

#if ENABLED(GALVO_LASER) //Galvo Control NAMFIX
	void scan_XY_galvo(unsigned long x1, unsigned long y1, unsigned long x2, unsigned long y2);
	void coordinate_XY_move(unsigned long X, unsigned long Y);
	short World_to_Galvo(long value);

	void update_X_galvo(int step_dir);
	void update_Y_galvo(int step_dir);
	static void galvo_sw_spi_send(uint8_t val);
	void galvos_digitalPotWrite(int channel, int value);
	void move_galvos(unsigned long X, unsigned long Y);
	void set_galvo_pos(unsigned long X, unsigned long Y);
	void move_X_galvo(unsigned short X);
	void move_Y_galvo(unsigned short Y);
	void timed_refresh_of_galvos(void);
#endif

void checkHitEndstops(); //call from somewhere to create an serial error message with the locations the endstops where hit, in case they were triggered
void endstops_hit_on_purpose(); //avoid creation of the message, i.e. after homing and before a routine call of checkHitEndstops();

void enable_endstops(bool check); // Enable/disable endstop checking

void enable_endstops_globally(bool check);
void endstops_not_homing();

void checkStepperErrors(); //Print errors detected by the stepper

void finishAndDisableSteppers();

extern block_t* current_block;  // A pointer to the block currently being traced

void quickStop();

#if HAS_DIGIPOTSS
  void digitalPotWrite(int address, int value);
#endif
void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2);
void microstep_mode(uint8_t driver, uint8_t stepping);
void digipot_init();
void digipot_current(uint8_t driver, int current);
void microstep_init();
void microstep_readings();

#if ENABLED(Z_DUAL_ENDSTOPS)
  void In_Homing_Process(bool state);
  void Lock_z_motor(bool state);
  void Lock_z2_motor(bool state);
#endif

//NAMADD Z TRIAD
#if ENABLED(Z_TRIAD_ENDSTOPS)
  void In_Homing_Process(bool state);
  void Lock_z_motor(bool state);
  void Lock_z2_motor(bool state);
  void Lock_z3_motor(bool state);
#endif

#if ENABLED(BABYSTEPPING)
  void babystep(const uint8_t axis, const bool direction); // perform a short step with a single stepper motor, outside of any convention
#endif

#endif
