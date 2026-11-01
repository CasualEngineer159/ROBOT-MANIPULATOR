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

#include "../gcode.h"
#include "../parser.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/scara.h"

bool correction_active;

namespace {
  uint8_t saved_correction_points;

  void report_correction_state() {
    SERIAL_ECHOPGM("SCARA correction active: ");
    SERIAL_ECHOLNPGM(correction_active ? "TRUE" : "FALSE");
  }

  void set_correction_active(const bool active) {
    if (correction_active == active) return;

    const xyz_pos_t current = {
      current_position.x,
      current_position.y,
      current_position.z
    };
    const xyz_pos_t converted = scara_apply_correction(current, active);
    current_position.x = converted.x;
    current_position.y = converted.y;
    current_position.z = converted.z;
    correction_active = active;
    sync_plan_position();
    report_correction_state();
  }
}

/**
 * M365: Save, reset, or toggle SCARA correction points and correction.
 */
void GcodeSuite::M365() {
  planner.synchronize();

  if (parser.seen_test('R')) {
    saved_correction_points = 0;
    set_correction_active(false);
    scara_reset_correction_matrix();
    SERIAL_ECHOLNPGM("SCARA correction points reset.");
    return;
  }

  if (parser.seen_test('P')) {
    if (saved_correction_points < SCARA_CORRECTION_POINT_COUNT) {
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

  if (saved_correction_points >= SCARA_CORRECTION_POINT_COUNT) {
    SERIAL_ECHOLNPGM("SCARA correction points already saved.");
    return;
  }

  xyz_pos_t point;
  point.x = current_position.x;
  point.y = current_position.y;
  point.z = current_position.z;
  scara_set_correction_point(saved_correction_points, point);
  ++saved_correction_points;

  SERIAL_ECHOPGM("SCARA correction point ");
  SERIAL_ECHO(saved_correction_points);
  SERIAL_ECHOPGM("/");
  SERIAL_ECHO(SCARA_CORRECTION_POINT_COUNT);
  SERIAL_ECHOPGM(" saved: X");
  SERIAL_ECHO_F(current_position.x, 3);
  SERIAL_ECHOPGM(" Y");
  SERIAL_ECHO_F(current_position.y, 3);
  SERIAL_ECHOPGM(" Z");
  SERIAL_ECHO_F(current_position.z, 3);
  SERIAL_EOL();

  if (saved_correction_points == SCARA_CORRECTION_POINT_COUNT) {
    set_correction_active(true);
    SERIAL_ECHOLNPGM("SCARA correction points complete:");
    for (uint8_t i = 0; i < SCARA_CORRECTION_POINT_COUNT; ++i) {
      const xyz_pos_t &point = scara_get_correction_point(i);
      SERIAL_ECHOPGM("Point ");
      SERIAL_ECHO(i + 1);
      SERIAL_ECHOPGM(": X");
      SERIAL_ECHO_F(point.x, 3);
      SERIAL_ECHOPGM(" Y");
      SERIAL_ECHO_F(point.y, 3);
      SERIAL_ECHOPGM(" Z");
      SERIAL_ECHO_F(point.z, 3);
      SERIAL_EOL();
    }
  }
}
