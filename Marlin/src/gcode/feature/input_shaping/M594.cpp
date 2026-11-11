#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"

#if EITHER(INPUT_SHAPING_X, INPUT_SHAPING_Y)

#include "../../../module/planner.h"
#include "../../../module/stepper.h"
#include "../../../module/motion.h"
#include "../../../MarlinCore.h"

constexpr float SWEEP_START_FREQ      = 1.0f;
constexpr float SWEEP_END_FREQ        = 20.0f;
constexpr float SWEEP_HZ_PER_SEC      = 0.5f;
constexpr float RESONANCE_DURATION_S  = 2.5f;
constexpr float SWEEP_AMPLITUDE_DEG   = 0.05f; // Joint amplitude (deg) applied to A, B or C alike

constexpr float TEST_MAX_ACCEL        = 20000.0f;
constexpr float TEST_MAX_FEEDRATE     = 500.0f;
constexpr float TEST_JUNCTION_DEV     = 0.05f;

static float old_max_acceleration_x, old_max_acceleration_y, old_max_acceleration_z;
static float old_max_feedrate_x, old_max_feedrate_y, old_max_feedrate_z;
static float old_print_acceleration;
static float old_travel_acceleration;
#if ENABLED(CLASSIC_JERK)
  static float old_jerk_x, old_jerk_y, old_jerk_z;
#else
  static float old_junction_deviation;
#endif
#if ENABLED(INPUT_SHAPING_X)
  static float old_shaping_freq_x;
#endif
#if ENABLED(INPUT_SHAPING_Y)
  static float old_shaping_freq_y;
#endif
#if ENABLED(INPUT_SHAPING_Z)
  static float old_shaping_freq_z;
#endif

void M594_setup(const bool shaping_on) {
  SERIAL_ECHOLNPGM("M594: Preparing for frequency sweep...");

  old_max_acceleration_x = planner.settings.max_acceleration_mm_per_s2[X_AXIS];
  old_max_acceleration_y = planner.settings.max_acceleration_mm_per_s2[Y_AXIS];
  old_max_acceleration_z = planner.settings.max_acceleration_mm_per_s2[Z_AXIS];
  old_print_acceleration = planner.settings.acceleration;
  old_travel_acceleration = planner.settings.travel_acceleration;
  old_max_feedrate_x = planner.settings.max_feedrate_mm_s[X_AXIS];
  old_max_feedrate_y = planner.settings.max_feedrate_mm_s[Y_AXIS];
  old_max_feedrate_z = planner.settings.max_feedrate_mm_s[Z_AXIS];

  #if ENABLED(CLASSIC_JERK)
    old_jerk_x = planner.max_jerk.x;
    old_jerk_y = planner.max_jerk.y;
    old_jerk_z = planner.max_jerk.z;
  #else
    old_junction_deviation = planner.junction_deviation_mm;
  #endif

  #if ENABLED(INPUT_SHAPING_X)
    old_shaping_freq_x = stepper.get_shaping_frequency(X_AXIS);
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    old_shaping_freq_y = stepper.get_shaping_frequency(Y_AXIS);
  #endif

  planner.set_max_acceleration(X_AXIS, TEST_MAX_ACCEL);
  planner.set_max_acceleration(Y_AXIS, TEST_MAX_ACCEL);
  planner.set_max_acceleration(Z_AXIS, TEST_MAX_ACCEL);
  planner.set_max_feedrate(X_AXIS, TEST_MAX_FEEDRATE);
  planner.set_max_feedrate(Y_AXIS, TEST_MAX_FEEDRATE);
  planner.set_max_feedrate(Z_AXIS, TEST_MAX_FEEDRATE);

  #if ENABLED(CLASSIC_JERK)
    planner.set_max_jerk(X_AXIS, 1.0f);
    planner.set_max_jerk(Y_AXIS, 1.0f);
    planner.set_max_jerk(Z_AXIS, 1.0f);
  #else
    planner.junction_deviation_mm = TEST_JUNCTION_DEV;
  #endif

  if (shaping_on) {
    SERIAL_ECHOLNPGM(" > Input shaping: ON");
  } else {
    SERIAL_ECHOLNPGM(" > Input shaping: OFF");
    #if ENABLED(INPUT_SHAPING_X)
      stepper.set_shaping_frequency(X_AXIS, 0.0f);
    #endif
    #if ENABLED(INPUT_SHAPING_Y)
      stepper.set_shaping_frequency(Y_AXIS, 0.0f);
    #endif
  }

  #if ENABLED(S_CURVE_ACCELERATION)
    planner.settings.s_curve_enabled = false;
  #endif
}

// Maps the S parameter (0=A, 1=B, 2=C) to the joint's planner axis
static AxisEnum joint_axis(const int axis) {
  return axis == 0 ? A_AXIS : axis == 1 ? B_AXIS : C_AXIS;
}

static char joint_char(const int axis) {
  return axis == 0 ? 'A' : axis == 1 ? 'B' : 'C';
}

static float &joint_ref(const int axis) {
  return axis == 0 ? delta.a : axis == 1 ? delta.b : delta.c;
}

// Moves one joint to an absolute target in degrees. The planner length hint is the
// joint displacement, so feedrate and acceleration are consistent with joint units.
static void M594_move_joint(const int axis, const float target, const float velocity) {
  const float length = ABS(target - joint_ref(axis));
  joint_ref(axis) = target;

  PlannerHints ph(length);
  ph.move_to_delta = true;

  forward_kinematics(delta.a, delta.b, delta.c);

  xyze_pos_t dest = current_position;
  dest.x = cartes.x;
  dest.y = cartes.y;
  dest.z = cartes.z;

  planner.buffer_line(dest, velocity, active_extruder, ph);
  current_position = dest;
}

void M594_sweep(int axis) {
  SERIAL_ECHOPGM("M594: Sweeping ");
  SERIAL_CHAR(joint_char(axis));
  SERIAL_ECHOLNPGM(" joint...");

  float current_freq = SWEEP_START_FREQ;
  float sign = 1.0f;

  inverse_kinematics(current_position);
  const float center = joint_ref(axis);

  while (current_freq <= SWEEP_END_FREQ) {
    while (planner.movesplanned() >= BLOCK_BUFFER_SIZE - 4) {
      idle();
    }

    const float t_seg = 0.25f / current_freq;
    const float accel = SWEEP_AMPLITUDE_DEG / sq(t_seg);

    planner.set_max_acceleration(joint_axis(axis), accel);
    planner.settings.acceleration = accel;
    planner.settings.travel_acceleration = accel;

    const float velocity = accel * t_seg;

    M594_move_joint(axis, center + sign * SWEEP_AMPLITUDE_DEG, velocity);

    current_freq += 2.0f * t_seg * SWEEP_HZ_PER_SEC;
    sign = -sign;
  }

  planner.synchronize();
}

void M594_resonate(int axis, float freq) {
  SERIAL_ECHOPGM("M594: Resonating ");
  SERIAL_CHAR(joint_char(axis));
  SERIAL_ECHOPGM(" joint at ");
  SERIAL_ECHO(freq);
  SERIAL_ECHOLNPGM(" Hz...");

  float total_time = 0.0f;
  float sign = 1.0f;

  inverse_kinematics(current_position);
  const float center = joint_ref(axis);

  const float t_seg = 0.25f / freq;
  const float accel = SWEEP_AMPLITUDE_DEG / sq(t_seg);

  planner.set_max_acceleration(joint_axis(axis), accel);
  planner.settings.acceleration = accel;
  planner.settings.travel_acceleration = accel;

  const float velocity = accel * t_seg;

  while (total_time <= RESONANCE_DURATION_S) {
    while (planner.movesplanned() >= BLOCK_BUFFER_SIZE - 4) {
      idle();
    }

    M594_move_joint(axis, center + sign * SWEEP_AMPLITUDE_DEG, velocity);

    total_time += 2.0f * t_seg;
    sign = -sign;
  }

  planner.synchronize();
}

void M594_teardown() {
  SERIAL_ECHOLNPGM("M594: Restoring original settings...");

  planner.set_max_acceleration(X_AXIS, old_max_acceleration_x);
  planner.set_max_acceleration(Y_AXIS, old_max_acceleration_y);
  planner.set_max_acceleration(Z_AXIS, old_max_acceleration_z);
  planner.settings.acceleration = old_print_acceleration;
  planner.settings.travel_acceleration = old_travel_acceleration;
  planner.set_max_feedrate(X_AXIS, old_max_feedrate_x);
  planner.set_max_feedrate(Y_AXIS, old_max_feedrate_y);
  planner.set_max_feedrate(Z_AXIS, old_max_feedrate_z);

  #if ENABLED(CLASSIC_JERK)
    planner.set_max_jerk(X_AXIS, old_jerk_x);
    planner.set_max_jerk(Y_AXIS, old_jerk_y);
    planner.set_max_jerk(Z_AXIS, old_jerk_z);
  #else
    planner.junction_deviation_mm = old_junction_deviation;
  #endif

  #if ENABLED(INPUT_SHAPING_X)
    stepper.set_shaping_frequency(X_AXIS, old_shaping_freq_x);
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    stepper.set_shaping_frequency(Y_AXIS, old_shaping_freq_y);
  #endif

  #if ENABLED(S_CURVE_ACCELERATION)
    planner.settings.s_curve_enabled = true;
  #endif

  SERIAL_ECHOLNPGM("M594: Finished.");
}

void GcodeSuite::M594() {
  if (!parser.seenval('S')) {
    SERIAL_ECHOLNPGM("M594 requires S parameter (S0 for A, S1 for B, S2 for C)");
    return;
  }

  const int axis = parser.value_int();
  if (axis < 0 || axis > 2) {
    SERIAL_ECHOLNPGM("Invalid axis. Use S0 for A, S1 for B, S2 for C");
    return;
  }

  float freq = 0.0f;
  if (parser.seenval('F')) {
    freq = parser.value_float();
  }

  bool shaping_on = false;
  if (parser.seenval('P')) {
    shaping_on = (parser.value_int() == 1);
  }

  planner.synchronize();
  M594_setup(shaping_on);

  if (freq > 0.0f) {
    M594_resonate(axis, freq);
  } else {
    M594_sweep(axis);
  }

  planner.synchronize();
  M594_teardown();
  planner.reset_small_move_alert();
}

#else

void GcodeSuite::M594() {
  SERIAL_ECHOLNPGM("M594 requires INPUT_SHAPING_X or INPUT_SHAPING_Y.");
}

#endif