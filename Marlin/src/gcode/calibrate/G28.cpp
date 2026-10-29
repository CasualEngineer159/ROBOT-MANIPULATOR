/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#include "../gcode.h"

#include "../../module/endstops.h"
#include "../../module/planner.h"
#include "../../module/stepper.h" // for various

#if HAS_MULTI_HOTEND
  #include "../../module/tool_change.h"
#endif

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(BD_SENSOR)
  #include "../../feature/bedlevel/bdl/bdl.h"
#endif

#if ENABLED(SENSORLESS_HOMING)
  #include "../../feature/tmc_util.h"
#endif

#include "../../module/probe.h"

#if ENABLED(BLTOUCH)
  #include "../../feature/bltouch.h"
#endif

#include "../../lcd/marlinui.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extui/ui_api.h"
#elif ENABLED(DWIN_CREALITY_LCD)
  #include "../../lcd/e3v2/creality/dwin.h"
#elif ENABLED(DWIN_LCD_PROUI)
  #include "../../lcd/e3v2/proui/dwin.h"
#endif

#if ENABLED(LASER_FEATURE)
  #include "../../feature/spindle_laser.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

#if ENABLED(Z_SAFE_HOMING)

  inline void home_z_safely() {
    DEBUG_SECTION(log_G28, "home_z_safely", DEBUGGING(LEVELING));

    // Disallow Z homing if X or Y homing is needed
    if (homing_needed_error(_BV(X_AXIS) | _BV(Y_AXIS))) return;

    sync_plan_position();

    /**
     * Move the Z probe (or just the nozzle) to the safe homing point
     * (Z is already at the right height)
     */
    constexpr xy_float_t safe_homing_xy = { Z_SAFE_HOMING_X_POINT, Z_SAFE_HOMING_Y_POINT };
    #if HAS_HOME_OFFSET
      xy_float_t okay_homing_xy = safe_homing_xy;
      okay_homing_xy -= home_offset;
    #else
      constexpr xy_float_t okay_homing_xy = safe_homing_xy;
    #endif

    destination.set(okay_homing_xy, current_position.z);

    TERN_(HOMING_Z_WITH_PROBE, destination -= probe.offset_xy);

    if (position_is_reachable(destination)) {

      if (DEBUGGING(LEVELING)) DEBUG_POS("home_z_safely", destination);

      // Free the active extruder for movement
      TERN_(DUAL_X_CARRIAGE, idex_set_parked(false));

      TERN_(SENSORLESS_HOMING, safe_delay(500)); // Short delay needed to settle

      do_blocking_move_to_xy(destination);
      homeaxis(Z_AXIS);
    }
    else {
      LCD_MESSAGE(MSG_ZPROBE_OUT);
      SERIAL_ECHO_MSG(STR_ZPROBE_OUT_SER);
    }
  }

#endif // Z_SAFE_HOMING

#if ENABLED(IMPROVE_HOMING_RELIABILITY)

  motion_state_t begin_slow_homing() {
    motion_state_t motion_state{0};
    motion_state.acceleration.set(planner.settings.max_acceleration_mm_per_s2[X_AXIS],
                                 planner.settings.max_acceleration_mm_per_s2[Y_AXIS]
                                 OPTARG(DELTA, planner.settings.max_acceleration_mm_per_s2[Z_AXIS])
                               );
    planner.settings.max_acceleration_mm_per_s2[X_AXIS] = 100;
    planner.settings.max_acceleration_mm_per_s2[Y_AXIS] = 100;
    TERN_(DELTA, planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = 100);
    #if HAS_CLASSIC_JERK
      motion_state.jerk_state = planner.max_jerk;
      planner.max_jerk.set(0, 0 OPTARG(DELTA, 0));
    #endif
    planner.refresh_acceleration_rates();
    return motion_state;
  }

  void end_slow_homing(const motion_state_t &motion_state) {
    planner.settings.max_acceleration_mm_per_s2[X_AXIS] = motion_state.acceleration.x;
    planner.settings.max_acceleration_mm_per_s2[Y_AXIS] = motion_state.acceleration.y;
    TERN_(DELTA, planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = motion_state.acceleration.z);
    TERN_(HAS_CLASSIC_JERK, planner.max_jerk = motion_state.jerk_state);
    planner.refresh_acceleration_rates();
  }

#endif // IMPROVE_HOMING_RELIABILITY

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 *  L<bool>   Force leveling state ON (if possible) or OFF after homing (Requires RESTORE_LEVELING_AFTER_G28 or ENABLE_LEVELING_AFTER_G28)
 *  O         Home only if the position is not known and trusted
 *  R<linear> Raise by n mm/inches before homing
 *
 * Cartesian/SCARA parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 */

bool BC_endstol_check = false;

void GcodeSuite::G28() {
  DEBUG_SECTION(log_G28, "G28", DEBUGGING(LEVELING));
  if (DEBUGGING(LEVELING)) log_machine_info();

  TERN_(BD_SENSOR, bdl.config_state = 0);

  if ((extDigitalRead(71) == LOW) || (extDigitalRead(72) == LOW)) {
      SERIAL_ECHOLNPGM("Prosím přeněte brzdy obou ramen na automatické ovládání pomocí příkazu M50 S0");
      return;
  }

  /**
   * Set the laser power to false to stop the planner from processing the current power setting.
   */

  #if ENABLED(MARLIN_DEV_MODE)
    if (parser.seen_test('S')) {
      LOOP_NUM_AXES(a) set_axis_is_at_home((AxisEnum)a);
      sync_plan_position();
      SERIAL_ECHOLNPGM("Simulated Homing");
      report_current_position();
      return;
    }
  #endif

  // Home (O)nly if position is unknown
  if (!axes_should_home() && parser.seen_test('O')) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> homing not needed, skip");
    return;
  }

  #if ENABLED(FULL_REPORT_TO_HOST_FEATURE)
    const M_StateEnum old_grblstate = M_State_grbl;
    set_and_report_grblstate(M_HOMING);
  #endif

  TERN_(HAS_DWIN_E3V2_BASIC, DWIN_HomingStart());
  TERN_(EXTENSIBLE_UI, ExtUI::onHomingStart());

  planner.synchronize();          // Wait for planner moves to finish!

  SET_SOFT_ENDSTOP_LOOSE(false);  // Reset a leftover 'loose' motion state

  // Disable the leveling matrix before homing
  #if CAN_SET_LEVELING_AFTER_G28
    const bool leveling_restore_state = parser.boolval('L', TERN1(RESTORE_LEVELING_AFTER_G28, planner.leveling_active));
  #endif

  // Cancel any prior G29 session
  TERN_(PROBE_MANUALLY, g29_in_progress = false);

  // Disable leveling before homing
  TERN_(HAS_LEVELING, set_bed_leveling_enabled(false));

  // Reset to the XY plane
  TERN_(CNC_WORKSPACE_PLANES, workspace_plane = PLANE_XY);

  // Count this command as movement / activity
  reset_stepper_timeout();

  #if ENABLED(IMPROVE_HOMING_RELIABILITY)
    motion_state_t saved_motion_state = begin_slow_homing();
  #endif

  TERN_(HAS_DUPLICATION_MODE, set_duplication_enabled(false));

  remember_feedrate_scaling_off();

  endstops.enable(true); // Enable endstops for next homing move

  //Příprava na HOMING manévr
  all_axis_unhomed();
  int loop_counter = 0;
  // Homing z počáteční polohy senzorů [0, 0]
  // Pokud z [0, 0] přecházíme na [1, 0], nechceme znovu nahrubo homovat B, to už je hotové
  bool two_zero_home = false;
 
  while (!is_axis_home_(B_AXIS))
  {
    

    // Provedení případu [1, -]
    if (endstop_pressed(B_AXIS)){
      if (!two_zero_home){
        homeaxis(B_AXIS,false);
      }
      set_axis_home(B_AXIS);
    }
    // Provedení případu [0, 0]
    else if (!endstop_pressed(B_AXIS) && (!endstop_pressed(C_AXIS)))
    {
      two_zero_home = true;
      BC_endstol_check = true;
      homeaxis(B_AXIS,false);
      BC_endstol_check = false;
    }
    // Provedení případu [0, 1]
    else if (!endstop_pressed(B_AXIS) && (endstop_pressed(C_AXIS)))
    {
      homeaxis(B_AXIS,false,true);
      set_axis_home(B_AXIS);
    }
    // Pojistka proti zacyklení
    loop_counter += 1;
    if (loop_counter>10) {
      SERIAL_ECHOLNPGM("Infinite loop");
      break;
    }
  }
  two_zero_home = false;
  
  homeaxis(C_AXIS,false);
  set_axis_home(C_AXIS);

  // Homing načisto
   
  homeaxis(A_AXIS,true);
  homeaxis(B_AXIS,true);
  homeaxis(C_AXIS,true);
  /*
  if (endstop_pressed(B_AXIS)) {
    SERIAL_ECHOLNPGM("Jsem v homing loopu");
    homeaxis(C_AXIS,false);
    do {
      homeaxis(B_AXIS,true);
      homeaxis(C_AXIS,is_axis_home_(B_AXIS));
    } while (!is_axis_home_(B_AXIS));
  } 
  else{
    SERIAL_ECHOLNPGM("Nejsem v homing loopu");
    homeaxis(B_AXIS,true);
    homeaxis(C_AXIS,true);
  }
  */
  set_axis_is_at_home(A_AXIS);
  set_axis_is_at_home(B_AXIS);
  set_axis_is_at_home(C_AXIS);
  sync_plan_position();
  inverse_kinematics(current_position);

  /**
   * Preserve DXC mode across a G28 for IDEX printers in DXC_DUPLICATION_MODE.
   * This is important because it lets a user use the LCD Panel to set an IDEX Duplication mode, and
   * then print a standard GCode file that contains a single print that does a G28 and has no other
   * IDEX specific commands in it.
   */

  endstops.not_homing();

  // Clear endstop state for polled stallGuard endstops
  TERN_(SPI_ENDSTOPS, endstops.clear_endstop_state());

  // Move to a height where we can use the full xy-area
  TERN_(DELTA_HOME_TO_SAFE_ZONE, do_blocking_move_to_z(delta_clip_start_height));

  TERN_(CAN_SET_LEVELING_AFTER_G28, if (leveling_restore_state) set_bed_leveling_enabled());

  restore_feedrate_and_scaling();

  // Restore the active tool after homing
  #if HAS_MULTI_HOTEND && (DISABLED(DELTA) || ENABLED(DELTA_HOME_TO_SAFE_ZONE))
    tool_change(old_tool_index, TERN(PARKING_EXTRUDER, !pe_final_change_must_unpark, DISABLED(DUAL_X_CARRIAGE)));   // Do move if one of these
  #endif

  ui.refresh();

  TERN_(HAS_DWIN_E3V2_BASIC, DWIN_HomingDone());
  TERN_(EXTENSIBLE_UI, ExtUI::onHomingDone());

  //report_current_position();

  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(old_grblstate));

}
