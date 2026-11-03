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
#include "../../module/bed_correction.h"

/**
 * M365 S: Save the current real position as the next bed correction corner.
 *         After three corners are saved, the bed correction is calculated and applied to
 *         incoming G0-G3 XYZ and to outgoing XYZ reports (M114).
 * M365 R: Reset the saved corners and disable the bed correction.
 */
void GcodeSuite::M365() {
  planner.synchronize();

  if (parser.seen_test('R')) {
    bed_correction_reset();
    SERIAL_ECHOLNPGM("Bed correction reset.");
    return;
  }

  if (parser.seen_test('T')) {
    if (!bed_correction_toggle()) {
      SERIAL_ECHOLNPGM("Bed correction is not calibrated. Save three corners with M365 S first.");
      return;
    }
    SERIAL_ECHOLNPGM("Bed correction turned ", bed_correction_is_active() ? "on." : "off.");
    return;
  }

  if (!parser.seen_test('S')) {
    SERIAL_ECHOLNPGM("M365 requires S (save corner), T (toggle) or R (reset).");
    return;
  }

  if (bed_correction_saved_count() >= BED_CORRECTION_POINT_COUNT) {
    SERIAL_ECHOLNPGM("Three corners already saved. Use M365 R to start over.");
    return;
  }

  const uint8_t index = bed_correction_saved_count();
  const xyz_pos_t point = { current_position.x, current_position.y, current_position.z };
  bed_correction_set_point(index, point);

  SERIAL_ECHOPGM("Bed correction corner ");
  SERIAL_ECHO(index + 1);
  SERIAL_ECHOLNPGM("/", BED_CORRECTION_POINT_COUNT, " saved.");

  if (bed_correction_is_calibrated())
    SERIAL_ECHOLNPGM("Bed correction calibrated and active.");
  else if (bed_correction_saved_count() == BED_CORRECTION_POINT_COUNT)
    SERIAL_ECHOLNPGM("Corners are collinear, bed correction not calibrated. Use M365 R and retry.");
}
