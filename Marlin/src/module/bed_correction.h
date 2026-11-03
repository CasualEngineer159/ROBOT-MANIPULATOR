/**
 * bed_correction.h - Rigid bed correction between the ideal (user) frame and the real (robot) frame.
 *
 * Three corner points measured in the real frame define the bed plane. The ideal frame has its
 * origin at the bed center, which is the midpoint of the longest side of the triangle formed by
 * the corners. The transform is a rotation plus translation, so distances and angles are preserved.
 */
#pragma once

#include "../core/types.h"

constexpr uint8_t BED_CORRECTION_POINT_COUNT = 3;

bool bed_correction_set_point(const uint8_t index, const xyz_pos_t &point);  // Real-frame point
void bed_correction_reset();
bool bed_correction_toggle();
uint8_t bed_correction_saved_count();
bool bed_correction_is_calibrated();  // Matrix computed from three non-collinear corners
bool bed_correction_is_active();      // Calibrated and toggled on (M365 T)

xyz_pos_t bed_correction_ideal_to_real(const xyz_pos_t &ideal);  // Applied to incoming G-code XYZ
xyz_pos_t bed_correction_real_to_ideal(const xyz_pos_t &real);   // Applied to outgoing XYZ reports
