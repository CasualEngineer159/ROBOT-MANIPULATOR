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

    // 1. Find the longest side of the triangle (the rectangle's diagonal);
    //    its midpoint is the center of the rectangular bed.
    const float d01 = dist(points[0], points[1]),
                d12 = dist(points[1], points[2]),
                d20 = dist(points[2], points[0]);
    const xyz_pos_t *a, *b, *opposite;
    if (d01 >= d12 && d01 >= d20)      { a = &points[0]; b = &points[1]; opposite = &points[2]; }
    else if (d12 >= d20)               { a = &points[1]; b = &points[2]; opposite = &points[0]; }
    else                               { a = &points[2]; b = &points[0]; opposite = &points[1]; }

    center.set((a->x + b->x) * 0.5f, (a->y + b->y) * 0.5f, (a->z + b->z) * 0.5f);

    // 2. Compute the bed normal from the triangle edges.
    const xyz_pos_t ab = { b->x - a->x, b->y - a->y, b->z - a->z };
    const xyz_pos_t ac = { opposite->x - a->x, opposite->y - a->y, opposite->z - a->z };

    float nx = ab.y * ac.z - ab.z * ac.y,
          ny = ab.z * ac.x - ab.x * ac.z,
          nz = ab.x * ac.y - ab.y * ac.x;
    const float n_len = SQRT(sq(nx) + sq(ny) + sq(nz));
    if (n_len < 0.0001f) return; // Degenerate (collinear) corners
    nx /= n_len; ny /= n_len; nz /= n_len;

    // Rule: The new Z axis must always point up (angle with old +Z < 90 deg => nz > 0).
    // If nz < 0, flipping the normal is equivalent to swapping points 1 and 2.
    if (nz < 0.0f) {
      nx = -nx; ny = -ny; nz = -nz;
    }

    // 3. Rule: Point 0 lies at +45 degrees between the new +X and +Y axes.
    // Vector from center to points[0], projected onto the plane for numerical stability:
    float dx = points[0].x - center.x,
          dy = points[0].y - center.y,
          dz = points[0].z - center.z;
    const float d_dot_n = dx * nx + dy * ny + dz * nz;
    dx -= d_dot_n * nx;
    dy -= d_dot_n * ny;
    dz -= d_dot_n * nz;

    const float d_len = SQRT(sq(dx) + sq(dy) + sq(dz));
    if (d_len < 0.0001f) return;
    dx /= d_len; dy /= d_len; dz /= d_len;

    // In-plane unit vector q = n x d (+90 degrees CCW from d around +n)
    const float qx = ny * dz - nz * dy,
                qy = nz * dx - nx * dz,
                qz = nx * dy - ny * dx;

    // Rotate d by -45 deg around n to get new +X (u), and +45 deg to get new +Y (v):
    // u = cos(45)*d - sin(45)*q
    // v = cos(45)*d + sin(45)*q  (which equals n x u)
    constexpr float INV_SQRT2 = 0.70710678f;
    const float ux = INV_SQRT2 * (dx - qx),
                uy = INV_SQRT2 * (dy - qy),
                uz = INV_SQRT2 * (dz - qz);

    const float vx = INV_SQRT2 * (dx + qx),
                vy = INV_SQRT2 * (dy + qy),
                vz = INV_SQRT2 * (dz + qz);

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
