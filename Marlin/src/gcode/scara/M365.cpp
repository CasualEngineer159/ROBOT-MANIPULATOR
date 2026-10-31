/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "../../inc/MarlinConfig.h"

#if IS_SCARA

#include "../gcode.h"
#include "../parser.h"
#include "../../module/motion.h"
#include "../../module/scara.h"

bool correction_active;

namespace {
  constexpr uint8_t correction_point_count = 3;
  xyz_pos_t correction_points[correction_point_count];
  uint8_t saved_correction_points;

  void report_correction_state() {
    SERIAL_ECHOPGM("SCARA correction active: ");
    SERIAL_ECHOLNPGM(correction_active ? "TRUE" : "FALSE");
  }

  void set_correction_active(const bool active) {
    if (correction_active == active) return;
    correction_active = active;
    report_correction_state();
  }
}

/**
 * M365: Save, reset, or toggle SCARA correction points and correction.
 */
void GcodeSuite::M365() {
  if (parser.seen_test('R')) {
    saved_correction_points = 0;
    set_correction_active(false);
    SERIAL_ECHOLNPGM("SCARA correction points reset.");
    return;
  }

  if (parser.seen_test('P')) {
    if (saved_correction_points < correction_point_count) {
      set_correction_active(false);
      SERIAL_ECHOLNPGM("SCARA correction remains inactive until three points are saved.");
    }
    else {
      set_correction_active(!correction_active);
    }
    return;
  }

  if (!parser.seen_test('S')) {
    SERIAL_ECHOLNPGM("M365 requires S (save), R (reset), or P (toggle).");
    return;
  }

  if (saved_correction_points >= correction_point_count) {
    SERIAL_ECHOLNPGM("SCARA correction points already saved.");
    return;
  }

  correction_points[saved_correction_points].x = current_position.x;
  correction_points[saved_correction_points].y = current_position.y;
  correction_points[saved_correction_points].z = current_position.z;
  ++saved_correction_points;

  SERIAL_ECHOPGM("SCARA correction point ");
  SERIAL_ECHO(saved_correction_points);
  SERIAL_ECHOPGM("/");
  SERIAL_ECHO(correction_point_count);
  SERIAL_ECHOPGM(" saved: X");
  SERIAL_ECHO_F(current_position.x, 3);
  SERIAL_ECHOPGM(" Y");
  SERIAL_ECHO_F(current_position.y, 3);
  SERIAL_ECHOPGM(" Z");
  SERIAL_ECHO_F(current_position.z, 3);
  SERIAL_EOL();

  if (saved_correction_points == correction_point_count) {
  if (saved_correction_points == correction_point_count) {
    SERIAL_ECHOLNPGM("SCARA correction points complete:");
    for (uint8_t i = 0; i < correction_point_count; ++i) {
      SERIAL_ECHOPGM("Point ");
      SERIAL_ECHO(i + 1);
      SERIAL_ECHOPGM(": X");
      SERIAL_ECHO_F(correction_points[i].x, 3);
      SERIAL_ECHOPGM(" Y");
      SERIAL_ECHO_F(correction_points[i].y, 3);
      SERIAL_ECHOPGM(" Z");
      SERIAL_ECHO_F(correction_points[i].z, 3);
      SERIAL_EOL();
    }
  }
}

#endif // IS_SCARA
