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

/**
 * scara.cpp
 */

#include "../inc/MarlinConfig.h"

#if IS_SCARA

#include "scara.h"
#include "motion.h"
#include "planner.h"

#if ENABLED(AXEL_TPARA)
  #include "endstops.h"
  #include "../MarlinCore.h"
#endif

float add_x;
float add_y;
float add_z;

extern xyze_pos_t current_position;
extern bool kinematic_calc_failiure;

float segments_per_second = TERN(AXEL_TPARA, TPARA_SEGMENTS_PER_SECOND, DEFAULT_SCARA_SEGMENTS_PER_SECOND);

#if EITHER(MORGAN_SCARA, MP_SCARA)

  static constexpr xy_pos_t scara_offset = { SCARA_OFFSET_X, SCARA_OFFSET_Y };

  void forward_kinematics(const_float_t a, const_float_t b, const_float_t c) {
    const float alfa = RADIANS(a), // prevadime na radiany
                beta = M_PI/2 + RADIANS(b), // prevadime na radiany
                gama = M_PI/2 + RADIANS(c);

    cartes.x = cos(alfa)*(cos(beta)*k2 + sin((gama - beta) - (M_PI/2 - beta))*k3+lx);
    cartes.y = -sin(alfa)*(cos(beta)*k2 + sin((gama - beta) - (M_PI/2 - beta))*k3+lx);
    cartes.z = k1 + sin(beta)*k2 - cos((gama - beta) - (M_PI/2 - beta))*k3 -(lz);
  }

#endif

bool are_angles_possible(const_float_t &a, const_float_t &b, const_float_t &c){
  const float alfa = a,
              beta = b,
              gamma = c;

  // Kontrola na imaginární části pro alfa, beta, gamma
  if (std::isnan(alfa) || std::isnan(beta) || std::isnan(gamma)){
    //SERIAL_ECHOLNPGM("Je nan");
    return false;
  }
  // Kontrola na rozsah pro alfa, beta, gamma
  if (!WITHIN(alfa,alfa_min,alfa_max)||!WITHIN(beta,beta_min,beta_max)||!WITHIN(gamma,gamma_min,gamma_max)){
    //SERIAL_ECHOLNPGM("Uhel mimo rozsah");
    return false;
  }
  if (!(gamma - beta > theta1_min) || !(gamma - beta < theta1_max)) {
    //SERIAL_ECHOLNPGM("Theha mimo rozsah");
    return false;
  }
  //SERIAL_ECHOLNPGM("ABC zkontrolováno");
  return true;
}


bool are_xyz_coordinates_possible(const_float_t &x, const_float_t &y, const_float_t &z){

  float r = HYPOT(x, y);

  if (r < r_min || r < (z-q)/k) {
    //SERIAL_ECHOLNPGM("Vzálenost mimo rozsah");
    return false;
  }
  //SERIAL_ECHOLNPGM("XYZ zkontrolováno");
  return true;
}


#if ENABLED(MP_SCARA)

  void scara_set_axis_is_at_home(const AxisEnum axis) {
    //if (axis == Z_AXIS)
    //  current_position.z = Z_HOME_POS;
    //else {
      // MORGAN_SCARA uses a Cartesian XY home position
      xyz_pos_t homeposition = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
      //DEBUG_ECHOLNPGM_P(PSTR("homeposition X"), homeposition.x, SP_Y_LBL, homeposition.y);

      delta = homeposition;
      forward_kinematics(delta.a, delta.b, delta.c);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPGM_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);
      update_software_endstops(axis);
    //}
  }

  void inverse_kinematics(
    const xyz_pos_t &raw,
    bool is_only_a_question,
    bool already_checked)
    {
    float alfa, alfares, beta, gamma, received_x,received_y,received_z, d1, d2, beta1, beta2, o1;

    const xyz_pos_t spos = raw;

    //ERIAL_ECHOLNPGM("jede z  x:", current_position.x, " y:",current_position.y," z:",current_position.z + lz);
    //SERIAL_ECHOLNPGM("jede do x:", received_x, " y:",received_y," z:",received_z);
    
    if ((extDigitalRead(71) == LOW) || (extDigitalRead(72) == LOW)) {
      kinematic_calc_failiure = true;
      SERIAL_ECHOLNPGM("Prosím přeněte brzdy obou ramen na automatické ovládání pomocí příkazu M50 S0");
      return;
    }

    if (!already_checked && !are_xyz_coordinates_possible(spos.x, spos.y, spos.z)){
      kinematic_calc_failiure = true;
      SERIAL_ECHOLNPGM("Chyba: Souřadnice je mimo povolený rozsah robota.");
      return;
    }
    
    received_x = spos.x + 0.0001;
    received_y = spos.y + 0.0001;
    received_z = spos.z + lz;

    alfares = atan2(-received_y, received_x);
    d1 = (-received_y)/(sin(alfares)) -lx;
    d2 = sqrt(d1*d1 + (received_z-k1)*(received_z-k1));
    beta1 = (acos((k2*k2+d2*d2-k3*k3)/(2*k2*d2)));
    beta2 = (asin((received_z - k1)/(d2)));
    o1= (acos((k2*k2-d2*d2+k3*k3)/(2*k2*k3)));

    alfa = alfares*(180/M_PI);
    beta = (beta1 + beta2 - M_PI/2)*(180/M_PI);
    gamma = (beta1 + beta2 + o1 - M_PI/2)*(180/M_PI);
    //SERIAL_ECHOLNPGM("kinematic_failiure:", kinematic_calc_failiure);

    if (!already_checked && !are_angles_possible(alfa,beta,gamma)){
      kinematic_calc_failiure = true;
      SERIAL_ECHOLNPGM("Chyba: Úhel je mimo povolený rozsah robota.");
      return;
    }

    if (is_only_a_question){
      return;
    }
    
    delta.set(alfa, beta, gamma);
  }

#elif ENABLED(MP_SCARA)

  //void scara_set_axis_is_at_home(const AxisEnum axis) {
    //if (axis == Z_AXIS)
    //  current_position.z = Z_HOME_POS;
    //else {
      // MP_SCARA uses arm angles for AB home position
     // #ifndef SCARA_OFFSET_THETA1
     //   #define SCARA_OFFSET_THETA1  12 // degrees
      //#endif
      //#ifndef SCARA_OFFSET_THETA2
      //  #define SCARA_OFFSET_THETA2 131 // degrees
      //#endif
     // ab_float_t homeposition = { SCARA_OFFSET_THETA1, SCARA_OFFSET_THETA2 };
      //DEBUG_ECHOLNPGM("homeposition A:", homeposition.a, " B:", homeposition.b);

      //inverse_kinematics(homeposition);
      //forward_kinematics(delta.a, delta.b);
      //current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPGM_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);
      //update_software_endstops(axis);
    //}
  //}

  //void inverse_kinematics(const xyz_pos_t &raw) {
   // const float x = raw.x, y = raw.y, c = HYPOT(x, y),
    //            THETA3 = ATAN2(y, x),
    //            THETA1 = THETA3 + ACOS((sq(c) + sq(L1) - sq(L2)) / (2.0f * c * L1)),
      //          THETA2 = THETA3 - ACOS((sq(c) + sq(L2) - sq(L1)) / (2.0f * c * L2));

   // delta.set(DEGREES(THETA1), DEGREES(THETA2), raw.z);

    /*
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      SERIAL_ECHOLNPGM("  SCARA (x,y) ", x, ",", y," Theta1=", THETA1, " Theta2=", THETA2);
    //*/
 // }

#elif ENABLED(AXEL_TPARA)

  static constexpr xyz_pos_t robot_offset = { TPARA_OFFSET_X, TPARA_OFFSET_Y, TPARA_OFFSET_Z };

  void scara_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = Z_HOME_POS;
    else {
      xyz_pos_t homeposition = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
      //DEBUG_ECHOLNPGM_P(PSTR("homeposition X"), homeposition.x, SP_Y_LBL, homeposition.y, SP_Z_LBL, homeposition.z);

      inverse_kinematics(homeposition);
      forward_kinematics(delta.a, delta.b, delta.c);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPGM_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);
      update_software_endstops(axis);
    }
  }

  // Convert ABC inputs in degrees to XYZ outputs in mm
  void forward_kinematics(const_float_t a, const_float_t b, const_float_t c) {
    const float w = c - b,
                r = L1 * cos(RADIANS(b)) + L2 * sin(RADIANS(w - (90 - b))),
                x = r  * cos(RADIANS(a)),
                y = r  * sin(RADIANS(a)),
                rho2 = L1_2 + L2_2 - 2.0f * L1 * L2 * cos(RADIANS(w));

    cartes = robot_offset + xyz_pos_t({ x, y, SQRT(rho2 - sq(x) - sq(y)) });
  }

  // Home YZ together, then X (or all at once). Based on quick_home_xy & home_delta
  void home_TPARA() {
    // Init the current position of all carriages to 0,0,0
    current_position.reset();
    destination.reset();
    sync_plan_position();

    // Disable stealthChop if used. Enable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      TERN_(X_SENSORLESS, sensorless_t stealth_states_x = start_sensorless_homing_per_axis(X_AXIS));
      TERN_(Y_SENSORLESS, sensorless_t stealth_states_y = start_sensorless_homing_per_axis(Y_AXIS));
      TERN_(Z_SENSORLESS, sensorless_t stealth_states_z = start_sensorless_homing_per_axis(Z_AXIS));
    #endif

    //const int x_axis_home_dir = TOOL_X_HOME_DIR(active_extruder);

    //const xy_pos_t pos { max_length(X_AXIS) , max_length(Y_AXIS) };
    //const float mlz = max_length(X_AXIS),

    // Move all carriages together linearly until an endstop is hit.
    //do_blocking_move_to_xy_z(pos, mlz, homing_feedrate(Z_AXIS));

    current_position.x = 0 ;
    current_position.y = 0 ;
    current_position.z = max_length(Z_AXIS) ;
    line_to_current_position(homing_feedrate(Z_AXIS));
    planner.synchronize();

    // Re-enable stealthChop if used. Disable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      TERN_(X_SENSORLESS, end_sensorless_homing_per_axis(X_AXIS, stealth_states_x));
      TERN_(Y_SENSORLESS, end_sensorless_homing_per_axis(Y_AXIS, stealth_states_y));
      TERN_(Z_SENSORLESS, end_sensorless_homing_per_axis(Z_AXIS, stealth_states_z));
    #endif

    endstops.validate_homing_move();

    // At least one motor has reached its endstop.
    // Now re-home each motor separately.
    homeaxis(A_AXIS);
    homeaxis(C_AXIS);
    homeaxis(B_AXIS);

    // Set all carriages to their home positions
    // Do this here all at once for Delta, because
    // XYZ isn't ABC. Applying this per-tower would
    // give the impression that they are the same.
    LOOP_NUM_AXES(i) set_axis_is_at_home((AxisEnum)i);

    sync_plan_position();
  }

  void inverse_kinematics(const xyz_pos_t &raw) {
    const xyz_pos_t spos = raw - robot_offset;

    const float RXY = SQRT(HYPOT2(spos.x, spos.y)),
                RHO2 = NORMSQ(spos.x, spos.y, spos.z),
                //RHO = SQRT(RHO2),
                LSS = L1_2 + L2_2,
                LM = 2.0f * L1 * L2,

                CG = (LSS - RHO2) / LM,
                SG = SQRT(1 - POW(CG, 2)), // Method 2
                K1 = L1 - L2 * CG,
                K2 = L2 * SG,

                // Angle of Body Joint
                THETA = ATAN2(spos.y, spos.x),

                // Angle of Elbow Joint
                //GAMMA = ACOS(CG),
                GAMMA = ATAN2(SG, CG), // Method 2

                // Angle of Shoulder Joint, elevation angle measured from horizontal (r+)
                //PHI = asin(spos.z/RHO) + asin(L2 * sin(GAMMA) / RHO),
                PHI = ATAN2(spos.z, RXY) + ATAN2(K2, K1),   // Method 2

                // Elbow motor angle measured from horizontal, same frame as shoulder  (r+)
                PSI = PHI + GAMMA;

    delta.set(DEGREES(THETA), DEGREES(PHI), DEGREES(PSI));

    //SERIAL_ECHOLNPGM(" SCARA (x,y,z) ", spos.x , ",", spos.y, ",", spos.z, " Rho=", RHO, " Rho2=", RHO2, " Theta=", THETA, " Phi=", PHI, " Psi=", PSI, " Gamma=", GAMMA);
  }

#endif

void scara_report_positions() {
  SERIAL_ECHOLNPGM(" a:", planner.get_axis_position_degrees(A_AXIS)
    #if ENABLED(AXEL_TPARA)
      , "  Phi:", planner.get_axis_position_degrees(B_AXIS)
      , "  Psi:", planner.get_axis_position_degrees(C_AXIS)
    #else
      , " b:", planner.get_axis_position_degrees(B_AXIS)
      , " g:", planner.get_axis_position_degrees(C_AXIS)
    #endif
  );
  SERIAL_EOL();
}

#endif // IS_SCARA
