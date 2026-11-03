/**
 * bed_correction.cpp - Rigid bed correction (see bed_correction.h)
 */

#include "../inc/MarlinConfig.h"

#include "bed_correction.h"

namespace {
  xyz_pos_t points[BED_CORRECTION_POINT_COUNT];
  uint8_t point_count;

  // Real-frame to ideal-frame transform: ideal = R * (real - center)
  // R rows are the orthonormal basis (u, v, n); the rows are stored as-is.
  float rot[3][3];
  xyz_pos_t center;
  bool calibrated;  // Matrix computed from three non-collinear corners
  bool enabled;     // User toggle (M365 T); only meaningful when calibrated

  float dist(const xyz_pos_t &a, const xyz_pos_t &b) {
    return SQRT(sq(a.x - b.x) + sq(a.y - b.y) + sq(a.z - b.z));
  }

  void compute() {
    calibrated = false;
    enabled = false;
    if (point_count < BED_CORRECTION_POINT_COUNT) return;

    // Find the longest side of the triangle; its midpoint is the ideal origin.
    const float d01 = dist(points[0], points[1]),
                d12 = dist(points[1], points[2]),
                d20 = dist(points[2], points[0]);
    const xyz_pos_t *a, *b, *opposite;
    if (d01 >= d12 && d01 >= d20)      { a = &points[0]; b = &points[1]; opposite = &points[2]; }
    else if (d12 >= d20)               { a = &points[1]; b = &points[2]; opposite = &points[0]; }
    else                               { a = &points[2]; b = &points[0]; opposite = &points[1]; }

    center.set((a->x + b->x) * 0.5f, (a->y + b->y) * 0.5f, (a->z + b->z) * 0.5f);

    // Ideal x axis: from the origin toward the opposite corner (in the bed plane).
    // Ideal y axis: in the bed plane, perpendicular to x, on the side of the triangle.
    // Ideal z axis: bed normal, pointing away from the triangle's side.
    const xyz_pos_t ex = { opposite->x - center.x, opposite->y - center.y, opposite->z - center.z };
    const float ex_len = SQRT(sq(ex.x) + sq(ex.y) + sq(ex.z));
    if (ex_len < 0.0001f) return;  // Degenerate (collinear) points

    const xyz_pos_t ab = { b->x - a->x, b->y - a->y, b->z - a->z };
    float nx = ab.y * ex.z - ab.z * ex.y,   // normal = ab x ex (any triangle normal)
          ny = ab.z * ex.x - ab.x * ex.z,
          nz = ab.x * ex.y - ab.y * ex.x;
    const float n_len = SQRT(sq(nx) + sq(ny) + sq(nz));
    if (n_len < 0.0001f) return;  // Collinear corners
    nx /= n_len; ny /= n_len; nz /= n_len;

    const float ux = ex.x / ex_len, uy = ex.y / ex_len, uz = ex.z / ex_len;
    // In-plane y axis = n x u (orthogonal to both, so the basis is right-handed)
    const float vx = ny * uz - nz * uy,
                vy = nz * ux - nx * uz,
                vz = nx * uy - ny * ux;

    rot[0][0] = ux; rot[0][1] = uy; rot[0][2] = uz;
    rot[1][0] = vx; rot[1][1] = vy; rot[1][2] = vz;
    rot[2][0] = nx; rot[2][1] = ny; rot[2][2] = nz;
    calibrated = true;
    enabled = true;
  }
}

bool bed_correction_set_point(const uint8_t index, const xyz_pos_t &point) {
  if (index >= BED_CORRECTION_POINT_COUNT || index != point_count) return false;
  points[index] = point;
  ++point_count;
  compute();
  return true;
}

void bed_correction_reset() {
  point_count = 0;
  calibrated = false;
  enabled = false;
}

bool bed_correction_toggle() {
  if (!calibrated) return false;
  enabled = !enabled;
  return true;
}

uint8_t bed_correction_saved_count() { return point_count; }

bool bed_correction_is_calibrated() { return calibrated; }

bool bed_correction_is_active() { return calibrated && enabled; }

xyz_pos_t bed_correction_real_to_ideal(const xyz_pos_t &real) {
  if (!enabled) return real;
  const float dx = real.x - center.x, dy = real.y - center.y, dz = real.z - center.z;
  return {
    rot[0][0] * dx + rot[0][1] * dy + rot[0][2] * dz,
    rot[1][0] * dx + rot[1][1] * dy + rot[1][2] * dz,
    rot[2][0] * dx + rot[2][1] * dy + rot[2][2] * dz
  };
}

xyz_pos_t bed_correction_ideal_to_real(const xyz_pos_t &ideal) {
  if (!enabled) return ideal;
  // Inverse of an orthonormal rotation is its transpose
  return {
    rot[0][0] * ideal.x + rot[1][0] * ideal.y + rot[2][0] * ideal.z + center.x,
    rot[0][1] * ideal.x + rot[1][1] * ideal.y + rot[2][1] * ideal.z + center.y,
    rot[0][2] * ideal.x + rot[1][2] * ideal.y + rot[2][2] * ideal.z + center.z
  };
}
