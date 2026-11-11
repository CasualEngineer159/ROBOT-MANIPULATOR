# Project Outline

This document provides a high-level overview of the ROBOT-MANIPULATOR project, based on the Marlin firmware. The project is configured to control a 3-axis Palletizing robot (Paletizační robot) utilizing parallel linkages to maintain tool parallelism. **Because Marlin does not natively support palletizing robots, this project uses a workaround by adapting and modifying the existing SCARA architecture within the firmware.**

### Project Directory Structure

The core of the ROBOT-MANIPULATOR firmware resides within the `Marlin/` directory. Here's a breakdown of its important subdirectories:

- `Marlin/`
    - `Configuration.h` - Main configuration file for basic settings.
    - `Configuration_adv.h` - Advanced configuration settings.
    - `Marlin.ino` - The main Arduino sketch file, typically containing `setup()` and `loop()`.
    - `src/`
        - `MarlinCore.cpp`/`.h` - Core functionalities and main program loop.
        - `gcode/` - Handles G-code parsing and execution.
            - `gcode.cpp`/`.h` - Core G-code processing.
            - `motion/` - G-code commands related to movement (e.g., `G0_G1.cpp`, `Custom_ATC.cpp`, `G2_G3.cpp`, `G7.cpp`, `G8_G9.cpp`, `M50_M51.cpp`).
            - `can/` - G-code commands for CAN bus communication (e.g., `M700_M701.cpp`).
            - `calibrate/` - G-code commands for calibration (e.g., `G28.cpp`).
            - `control/` - G-code commands for various controls (e.g., `M42.cpp`, `M120_M121.cpp`).
            - `host/` - G-code commands for host communication (e.g., `M114.cpp`).
            - `config/` - G-code commands for configuration (e.g., `M43.cpp`).
            - `feature/` - Implementations of various features (e.g., `pause/G60.cpp`, `pause/G61.cpp`, `input_shaping/M593.cpp`, `input_shaping/M594.cpp`).
        - `module/` - Contains modular components for different hardware and software functionalities.
            - `motion.cpp`/`.h` - High-level motion control and homing.
            - `planner.cpp`/`.h` - Motion planning and step generation.
            - `scara.cpp`/`.h` - SCARA kinematics implementation.
            - `canbus.cpp`/`.h` - CAN bus communication handling.
            - `endstops.cpp`/`.h` - Endstop management.
            - `stepper.cpp`/`.h` - Low-level stepper motor control.
            - `temperature.cpp`/`.h` - Temperature management.
        - `pins/` - Pin definitions for various motherboards.
            - `stm32f4/` - Pin definitions specific to STM32F4 microcontrollers, including `pins_BTT_OCTOPUS_V1_common.h`.
        - `inc/` - Contains include files and conditional definitions (e.g., `Conditionals_post.h`).
        - `lcd/` - LCD and user interface related files.

### 1. Configuration

- **`Marlin/Configuration.h`**
    - **`MOTHERBOARD`**: `BOARD_BTT_OCTOPUS_PRO_V1_0` - A powerful 32-bit controller board.
    - **`USE_CANBUS`**: Enabled to activate CAN bus functionality.
    - **Kinematics**: Currently utilizing/adapted from `MP_SCARA` to drive the 3-axis palletizing parallel linkage system. **It is crucial to note that while `MP_SCARA` is enabled as a structural workaround in the firmware, the physical robot is a 3-axis Palletizing robot with parallel linkages, not a standard SCARA.**
    - **`SCARA_LINKAGE_1`, `SCARA_LINKAGE_2`, `SCARA_LINKAGE_3`**: Defines physical dimensions of robot arms for accurate movement.
    - **Stepper Drivers & Motors**: Nema 34 (9Nm) motors driven by `CL86T` closed-loop external drivers (STEP/DIR interface).
    - **Steps/Resolution**: Configured for ultra-high resolution at 32,000 steps per revolution (0.01125° per step).
    - **Endstops**: Standard min-endstops used for homing.
        - `X_MIN_ENDSTOP_INVERTING`, `Y_MIN_ENDSTOP_INVERTING`, `Z_MIN_ENDSTOP_INVERTING` are set to `false`, indicating the use of Normally Open (NO) inductive sensors.
        - `VALIDATE_HOMING_ENDSTOPS` is enabled.
    - **Home Positions**: `MANUAL_X_HOME_POS`, `MANUAL_Y_HOME_POS`, `MANUAL_Z_HOME_POS` are set to specific float values, defining the hardcoded home coordinates.
    - **Homing Feedrate**: `HOMING_FEEDRATE_MM_M` is increased to `{400, 400, 400}` for faster homing.
    - **`EEPROM_SETTINGS`**: Enabled, allowing settings to be saved to the board's memory.

- **`Marlin/Configuration_adv.h`**
    - **`INPUT_SHAPING`**: Enabled to allow for vibration cancellation.
    - **`SAVED_POSITIONS`**: Increased to `10`, allowing more positions to be saved and recalled with `G60`/`G61`.
    - **`CNC_COORDINATE_SYSTEMS`**: Enabled, supporting G53 and G54-G59.3 commands for selecting coordinate systems.
    - **`DIRECT_PIN_CONTROL`**: Enabled, allowing direct control of pin states using `M42`.
    - **`PINS_DEBUGGING`**: Enabled, allowing for pin status display, toggling, and watching using `M43`.
    - **`BABYSTEPPING`**: Enabled, allowing for fine-tuning of the robot's position.
    - **`ADAPTIVE_STEP_SMOOTHING`**: Enabled, helps reduce vibrations and improve movement quality.
    - **`SQUARE_WAVE_STEPPING`**: Enabled, optimal for the stepper drivers being used.
    - **TMC Driver Settings**: Heavily configured with specific currents, microsteps, and parameters for Trinamic stepper drivers, crucial for smooth and quiet operation.

- **`Marlin/src/inc/Conditionals_post.h`**
    - **`SCARA_PRINTABLE_RADIUS`**: Hardcoded to `1200`, overriding the standard SCARA derivation from linkage lengths. This defines a custom, fixed printable radius for the palletizing robot.
    - **`USE_GCODE_SUBCODES`**: Unconditionally enabled, ensuring G-code subcodes are always supported.

### 2. Core Logic

- **`Marlin/src/MarlinCore.cpp`**
    - Contains the main `setup()` and `loop()` functions.
    - Initializes all hardware and continuously processes G-code commands and manages the robot's state.
    - **CAN Bus Initialization**: Calls `canbus.setup_canbus()` to initialize the CAN hardware.
    - **Brake Control Pin Initialization**: `PE7` and `PE8` are configured as outputs and set to `HIGH` (likely disengaged state for electromagnetic brakes).
    - **Sensor Pull-up Initialization**: Pull-up resistors are enabled for `TOOL_ID_BIT0_PIN` to `TOOL_ID_BIT3_PIN` (POGO pins for tool ID) and `DOCK_0_SENSOR_PIN` to `DOCK_2_SENSOR_PIN` (dock presence sensors).

- **`Marlin/src/gcode/gcode.cpp`**
    - Contains the G-code parser and dispatcher (`process_parsed_command`).
    - Takes incoming G-code commands and calls appropriate functions for execution.
    - **`break_command_pending`**: A new global boolean variable (`bool break_command_pending = false;`) is introduced to manage the two-step `M50`/`M51` brake control process.
    - **`process_parsed_command` Modification**: Includes logic to check for `M50`, `M51`, `M105` and, if `break_command_pending` is true and another command is received, it cancels the pending `M50` action with a serial message "Příkaz M51 zrušen" (Command M51 canceled).
    - **New G-code Cases**: Integrates handlers for `G7`, `G8`, `G9`, `M6`, `M50`, `M51`, `M593`, `M594`, `M666`, `M667`, `M700`, `M701`.

- **`Marlin/src/gcode/gcode.h`**
    - Declares the new G-code functions: `G7()`, `G8()`, `G9()`, `M6()`, `M50()`, `M51()`, `M593()`, `M594()`, `M666()`, `M667()`, `M700()`, `M701()`.
    - `G7()` is commented as "Set robot angles alfa, beta, gamma dirrectly A B C".
    - `G60`/`G61` comments updated to reflect `SAVED_POSITIONS` requirement.

### 3. CAN Bus Communication

- **`Marlin/src/module/canbus.h` (New File)**
    - Defines the `CANBus` class, which encapsulates CAN communication logic.
    - Declares methods for setup (`setup_canbus`), sending (`send_message`), and receiving (`receive_message`).

- **`Marlin/src/module/canbus.cpp` (New File)**
    - Implements the `CANBus` class methods.
    - **`setup_canbus()`**: Initializes the STM32F4's CAN1 peripheral, configuring GPIO pins (PD0 for RX, PD1 for TX), setting the baud rate to 500 kbps, and establishing a filter to accept all incoming messages. It also sends a test message ("HELLOCAN") on startup.
    - **`send_message()`**: Transmits a CAN message with a given ID and data payload.
    - **`receive_message()`**: Checks for and retrieves a message from the CAN receive FIFO.

- **`Marlin/src/gcode/can/M700_M701.cpp` (New File)**
    - Implements the G-code interface for CAN bus communication.
    - **`M700`**: Sends a CAN message. It takes an ID (`I` parameter) and a data payload as a hexadecimal string (`H` parameter).
    - **`M701`**: Receives and prints a CAN message. If a message is available, it prints the message ID and its data payload formatted as hexadecimal bytes.

### 4. Kinematics

- **`Marlin/src/module/scara.h`**
    - Defines the SCARA kinematics and physical constraints (arm lengths, joint ranges).
    - **Custom Kinematic Boundaries**: Introduces `alfa_min`/`max`, `beta_min`/`max`, `gamma_min`/`max` for joint angle limits, and `theta1_min`/`max` for the angle between the two main arms.
    - **Workspace Limits**: Defines `zo`, `yo`, `k`, `q`, and `r_min` to establish a conical workspace boundary (`r < (z-q)/k`).
    - Declares `are_angles_possible()` and `are_xyz_coordinates_possible()` for pre-movement validation.

- **`Marlin/src/module/scara.cpp`**
    - **WARNING**: The standard SCARA kinematic equations in `forward_kinematics()` and `inverse_kinematics()` within this file have been **intentionally modified and replaced** to calculate the specific kinematics for the **palletizing robot's parallel linkages**. Any future modifications to this file must account for these custom palletizing kinematics, and **NOT** standard SCARA kinematics.
    - **`forward_kinematics()`**: Calculates X, Y, Z position of the end-effector from joint angles.
        - **Y-Axis Inversion**: The `cartes.y` calculation is inverted (`-sin(alfa) * ...`) to change the robot from a left-handed to a right-handed coordinate system.
    - **`inverse_kinematics()`**: Critical function: translates target X, Y, Z into required joint angles.
        - **Safety Checks**: Integrates calls to `are_xyz_coordinates_possible()` and `are_angles_possible()` to validate target positions against custom kinematic boundaries.
        - **`kinematic_calc_failiure` Flag**: A global `bool kinematic_calc_failiure` is set to `true` if a target position is out of bounds, preventing unsafe movements.
        - **Optional Parameters**: `is_only_a_question` and `already_checked` parameters allow for testing reachability without committing to a move.
        - **Brake Check**: Checks `extDigitalRead(71)` and `extDigitalRead(72)` (brake status) and sets `kinematic_calc_failiure` if brakes are not in automatic mode.

### 5. Motion Control

- **`Marlin/src/module/motion.cpp`**
    - Contains high-level motion commands like `do_blocking_move_to()`.
    - Used to move the robot to specific positions.
    - Handles homing and other motion-related tasks.
    - **Kinematic Failure Integration**: `prepare_fast_move_to_destination()` and `line_to_destination_kinematic()` now check the `kinematic_calc_failiure` flag and abort movement if set.
    - **`direct_angle_change()`**: A new function to directly change robot angles, used by `G7`.

- **`Marlin/src/module/planner.cpp`**
    - The motion planner.
    - Takes target positions from `motion.cpp` and generates step pulses for stepper motors.
    - Responsible for acceleration and deceleration for smooth and accurate movement.

- **`Marlin/src/module/stepper.cpp`**
    - **Input Shaping Implementation**: The low-level stepper ISR (`Stepper::isr`) contains the core logic for Input Shaping. It intercepts step requests for each axis (X and Y) and applies the ZV (Zero Vibration) shaping algorithm.
    - **How it Works**: Instead of a single step pulse, the shaper splits the impulse into two parts, separated by a delay calculated from the resonant frequency (`M593 F...`). The magnitude of these two impulses is determined by the damping ratio (`M593 D...`). This is achieved using a "secondary Bresenham" error accumulator (`shaping_x.delta_error`) which translates the fractional, time-delayed impulses into whole, physical step pulses. This entire process is independent of the robot's kinematics, as it operates on the final Cartesian step commands.

- **`Marlin/src/gcode/motion/G2_G3.cpp`**
    - **Arc Movement Safety**: Modified `plan_arc()` function now incorporates `inverse_kinematics(raw, true)` and checks `kinematic_calc_failiure` to ensure arc movements respect the custom kinematic boundaries.

- **`Marlin/src/gcode/motion/G7.cpp` (New File)**
    - Implements the `G7` G-code command.
    - **Function**: "Kloubovy JOG (PTP)" (Joint JOG (Point-to-Point)). Allows direct control of robot's individual axes (A, B, C) in its native coordinate system.
    - **Details**: Reads `A`, `B`, `C` parameters, applies A-axis inversion (`* -1.0f`), handles relative/absolute modes, and calls `direct_angle_change()` to execute the move. **Note: Input Shaping is NOT applied to `G7` moves.**

- **`Marlin/src/gcode/motion/G8_G9.cpp` (New File)**
    - Implements `G8` and `G9` G-code commands.
    - **`G8` (Polar PTP Approach)**: Converts polar coordinates (A, R) to Cartesian (X, Y) and performs a fast, arc-like move using `prepare_fast_move_to_destination()`.
    - **`G9` (Polar Linear Approach)**: Converts polar coordinates (A, R) to Cartesian (X, Y) and performs a precise linear move using `prepare_line_to_destination()`.
    - **Details**: Both commands parse `A` (angle) and `R` (radius) parameters, convert them to `destination.x` and `destination.y`, and then execute the move.

- **`Marlin/src/gcode/motion/M50_M51.cpp` (New File)**
    - Implements `M50` and `M51` G-code commands for brake control.
    - **`M50` (Set Brake State)**:
        - `S0`: Connects both main and secondary arm brakes to the driver (automatic control).
        - `S1`: Connects both brakes to a 24V source (disengaged).
        - `M50.1 S<state>`: Controls the main arm brake.
        - `M50.2 S<state>`: Controls the secondary arm brake.
        - Sets `break_command_pending = true` and prompts for `M51` confirmation.
    - **`M51` (Confirm Brake State)**:
        - Executes the pending brake state change set by `M50` if `break_command_pending` is true.
        - Writes the `PE7_state` and `PE8_state` to the respective pins.

### 6. Automatic Tool Changer (ATC)

- **`Marlin/src/gcode/motion/Custom_ATC.cpp`**
    - Custom file implementing ATC functionality using a Maxwell kinematic coupling.
    - **Toolheads**: Supports interchangeable tools including a dial indicator, a 3D printing head (FDM), and a pen plotter.
    - **Hardware Interaction**: Defines pins for reading a 4-bit tool ID from POGO pins (`TOOL_ID_BITx_PIN`) and for dock presence sensors (`DOCK_x_SENSOR_PIN`).
    - **Tool Data Management**: Uses `ToolData` struct (`tool_table`) and `ToolDock` struct (`magazine`) to store tool offsets, clamp angles, names, and dock positions.
    - **Calibration (`M667 C<n>`)**: Allows "teach-in" calibration to define `ref_R`, `ref_A`, `ref_Z` (reference coordinates for the tool magazine).
    - **Auto-Scan (`M667 S`)**: Scans docks to identify tools, handles current tool unloading, and optimizes scan order.
    - **Unload (`M667 U`)**: Unloads the currently held tool into the nearest available dock.
    - **`M6` (Tool Change)**: Orchestrates the entire tool change process, including:
        - Checking calibration status.
        - Finding the target tool's dock.
        - Unloading the current tool (using `unload_current_tool()`).
        - Moving to the target dock.
        - Activating the servo gripper (`M280 P0 S<angle>`).
        - Verifying the picked-up tool's ID via POGO pins.
        - Handling ID mismatches as fatal errors.
    - **`M666` (Tool Library Management)**: Allows defining/updating tool properties (X, Y, Z offsets, clamp angle, name) and listing all stored tools.

### 7. Kinematics & Homing Adjustments

- **`Marlin/src/module/endstops.cpp`**
    - **New Member Variables**: `old_live_state`, `endstop_changed`, `endstop_poll_count` added to `Endstops` class for advanced endstop state tracking.
    - **`update()` Function Overhaul**:
        - **Endstop State Detection**: The `PROCESS_ENDSTOP` macro is redefined to trigger on `ENDSTOP_CHANGED` (state change) rather than `TEST_ENDSTOP` (current state), crucial for inductive sensors.
        - **Cycle-based Debouncing**: Integrates `endstop_poll_count` and `old_live_state` within `change_state()` to implement a cycle-based debouncing mechanism, allowing the arm to drive deeper into the sensor field to prevent false triggers.
        - **Conditional Endstop Processing**: `BC_endstol_check` is used to conditionally process endstops for B and C axes during specific homing phases.
        - **Removed Axis Processing**: Endstop processing for I, J, K, U, V, W axes has been removed.

- **`Marlin/src/module/endstops.h`**
    - Declares `extern bool BC_endstol_check;` and `extern bool final_home_move;` for use in the custom homing routine.
    - Defines the new static member variables (`old_live_state`, `endstop_changed`, `endstop_poll_count`) within the `Endstops` class.
    - The `change_state()` function's declaration reflects its new role in cycle-based endstop detection.

- **`Marlin/src/gcode/calibrate/G28.cpp`**
    - **Custom Homing Routine (`G28()`)**: Overhauled to implement "Algorithm 2" for the palletizing robot.
        - **Initial Brake Check**: Verifies that arm brakes are in automatic mode (`PE7`, `PE8` pins).
        - **`all_axis_unhomed()`**: Resets homing status for all axes.
        - **Iterative B/C Homing**: Includes a `while (!is_axis_home_(B_AXIS))` loop that iteratively calls `homeaxis(B_AXIS, false)` and conditionally `homeaxis(C_AXIS, ...)` based on endstop states and `two_zero_home`/`BC_endstol_check` flags. This handles the complex interaction between B and C axes during homing.
        - **`homeaxis(AxisEnum axis, bool final_home, bool BC_homing)`**: A modified homing function that supports direction-specific moves, applies bump maneuvers only during final positioning, and allows simultaneous homing of axes B and C.
        - **`endstop_pressed(const AxisEnum axis)`**: A helper function to check the state of specific endstop pins (`PG6`, `PG9`, `PG10`).
        - **`set_axis_home(const AxisEnum axis)` / `is_axis_home_(const AxisEnum axis)`**: Helper functions to track the homed status of individual axes.
        - **Final Homing**: Performs final homing for A, B, and C axes.
        - **Kinematic Update**: Calls `inverse_kinematics(current_position)` after homing to ensure the robot's Cartesian position is correctly calculated from the homed joint angles.

### 8. Key G-Code Commands

- **`G0`/`G1`**: Linear Move.
    - **Function**: Most common G-code for all linear movements. Now integrates custom kinematic boundary checks.
    - **File**: `Marlin/src/gcode/motion/G0_G1.cpp`
- **`G2`/`G3`**: Arc Moves (Clockwise/Counter-Clockwise).
    - **Function**: Moves the robot along an arc. Marlin automatically interpolates these into small linear segments. Now integrates custom kinematic boundary checks.
    - **File**: `Marlin/src/gcode/motion/G2_G3.cpp`
- **`G7`**: Joint JOG (PTP).
    - **Function**: Direct control of robot's individual axes (A, B, C) in its native coordinate system, supporting relative movement.
    - **File**: `Marlin/src/gcode/motion/G7.cpp`
- **`G8`**: Polar PTP Approach.
    - **Function**: Performs a smooth, arc-like move to a target polar coordinate (Angle A, Radius R).
    - **File**: `Marlin/src/gcode/motion/G8_G9.cpp`
- **`G9`**: Polar Linear Approach.
    - **Function**: Performs a precise linear move to a target polar coordinate (Angle A, Radius R).
    - **File**: `Marlin/src/gcode/motion/G8_G9.cpp`
- **`G28`**: Home one or more axes.
    - **Function**: Initiates the custom homing routine (Algorithm 2) for the palletizing robot.
    - **File**: `Marlin/src/gcode/calibrate/G28.cpp`
- **`G60`**: Save Current Position.
    - **Function**: Saves the current coordinates of the robot to a specified slot.
    - **File**: `Marlin/src/gcode/feature/pause/G60.cpp`
- **`G61`**: Return to Saved Position.
    - **Function**: Moves the robot to the coordinates stored in a specified slot.
    - **File**: `Marlin/src/gcode/feature/pause/G61.cpp`
- **`G90`**: Absolute Positioning.
    - **Function**: All coordinates are interpreted as absolute positions in the machine's coordinate system.
    - **File**: `Marlin/src/gcode/gcode.cpp` (Handled in the main G-code parser: `set_relative_mode(false)`)
- **`G91`**: Relative Positioning.
    - **Function**: All coordinates are interpreted as relative to the current position.
    - **File**: `Marlin/src/gcode/gcode.cpp` (Handled in the main G-code parser: `set_relative_mode(true)`)
- **`M6`**: Automatic Tool Change (ATC).
    - **Function**: Orchestrates dropping off the current tool and picking up a new one from the tool magazine.
    - **File**: `Marlin/src/gcode/motion/Custom_ATC.cpp`
- **`M42`**: Set Pin State.
    - **Function**: Allows for direct control of any pin on the board, useful for debugging and controlling external devices.
    - **File**: `Marlin/src/gcode/control/M42.cpp`
- **`M43`**: Pin Status.
    - **Function**: Reports the status of all pins on the board, useful for debugging.
    - **File**: `Marlin/src/gcode/config/M43.cpp`
- **`M50`**: Set Brake State.
    - **Function**: Initiates a change in the state of the electromagnetic brakes (automatic or disengaged). Requires `M51` to confirm.
    - **File**: `Marlin/src/gcode/motion/M50_M51.cpp`
- **`M51`**: Confirm Brake State.
    - **Function**: Confirms and executes the pending brake state change set by `M50`.
    - **File**: `Marlin/src/gcode/motion/M50_M51.cpp`
- **`M114`**: Get Current Position.
    - **Function**: Reports the current position of the robot in the machine's coordinate system.
    - **File**: `Marlin/src/gcode/host/M114.cpp`
- **`M120`**: Enable Endstops.
    - **Function**: Enables the endstops.
    - **File**: `Marlin/src/gcode/control/M120_M121.cpp`
- **`M121`**: Disable Endstops.
    - **Function**: Disables the endstops.
    - **File**: `Marlin/src/gcode/control/M120_M121.cpp`
- **`M593`**: Input Shaping.
    - **Function**: Sets the frequency (`F`) and damping ratio (`D`) for the Input Shaping algorithm to cancel vibrations on the X and Y axes.
    - **File**: `Marlin/src/gcode/feature/input_shaping/M593.cpp`
- **`M594`**: Frequency Sweep.
    - **Function**: Frequency Sweep to determine resonant frequencies.
    - **File**: `Marlin/src/gcode/feature/input_shaping/M594.cpp`
- **`M666`**: Tool Library Management.
    - **Function**: Defines or lists tool properties (offsets, clamp angle, name).
    - **File**: `Marlin/src/gcode/motion/Custom_ATC.cpp`
- **`M667`**: Master ATC Command.
    - **Function**: Controls ATC calibration, auto-scanning, unloading, and diagnostics.
    - **File**: `Marlin/src/gcode/motion/Custom_ATC.cpp`
- **`M700`**: Send CAN Message.
    - **Function**: Sends a CAN message with a specified ID and a hexadecimal data payload.
    - **File**: `Marlin/src/gcode/can/M700_M701.cpp`
- **`M701`**: Receive CAN Message.
    - **Function**: Polls for and prints a received CAN message, displaying the ID and data in hexadecimal format.
    - **File**: `Marlin/src/gcode/can/M700_M701.cpp`