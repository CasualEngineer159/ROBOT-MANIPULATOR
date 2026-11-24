# ROBOT-MANIPULATOR — Project Overview

> **Thesis project (diplomka):** A custom 3-axis **Palletizing Robot** (*Paletizační robot*) with parallel linkages, implemented as heavily modified **Marlin 2.1** firmware running on the **BigTreeTech Octopus Pro V1** (STM32F4). Because Marlin has no native palletizing robot support, the **MP_SCARA kinematic plugin is structurally enabled but its entire mathematical core is replaced** with custom parallel-linkage kinematics.

---

## System Architecture

```
Host PC / G-code sender
        │  USB / Serial (250000 baud)
        ▼
STM32F4 — BTT Octopus Pro V1  (Marlin 2.1 firmware)
  ├── STEP/DIR ──────────────► NEMA 34 motors (CL86T closed-loop drivers)
  ├── GPIO PE7/PE8 ──────────► Electromagnetic brakes
  ├── GPIO PG6/PG9/PG10 ─────► Inductive endstops (NO)
  ├── GPIO PE12–PE15 ─────────► POGO-pin tool ID readback (4-bit)
  ├── GPIO PG11–PG13 ─────────► Dock presence sensors
  ├── CAN1 (PD0 RX / PD1 TX) ◄► Seeeduino XIAO (secondary controller)
  └── Servo (M280) ───────────► Tool magazine gripper
```

---

## Directory Layout

| Path | Purpose |
|---|---|
| `Marlin/` | Firmware source root (`src_dir` in PlatformIO) |
| `Marlin/Configuration.h` | Main settings — board, kinematics, motors, endstops |
| `Marlin/Configuration_adv.h` | Advanced settings — input shaping, babystepping, ATC |
| `Marlin/src/module/` | Hardware modules: motion, planner, stepper, kinematics, CAN bus |
| `Marlin/src/gcode/motion/` | Custom G-code implementations: G7, G8/G9, M50/M51, ATC |
| `Marlin/src/gcode/can/` | CAN bus G-code commands: M700, M701 |
| `Marlin/src/gcode/calibrate/` | Custom homing routine (G28 — "Algorithm 2") |
| `Marlin/src/pins/stm32f4/` | BTT Octopus pin definitions |
| `config/` | Additional board configuration files |
| `renode/` | Renode simulation files for the STM32 target |
| `ini/` | PlatformIO per-platform environment fragments |
| `docker/` | Docker build environment |
| `OUTLINE.md` | Detailed project outline |
| `OUTLINE_short.md` | Condensed project outline |
| `CAN_changes.diff` | Git diff of CAN bus integration changes |

---

## Hardware

| Component | Value |
|---|---|
| **MCU board** | BigTreeTech Octopus Pro V1 — `BOARD_BTT_OCTOPUS_PRO_V1_0` |
| **Motors** | NEMA 34, 9 Nm |
| **Drivers** | CL86T — closed-loop, STEP/DIR interface |
| **Steps/unit** | 133.33 (A/X), 88.89 (B/Y), 88.89 (C/Z) steps per unit |
| **Endstops** | Inductive, Normally Open — X/Y/Z min pins |
| **Homing feedrate** | 400 mm/min on all axes |
| **Home positions** | X=569.92 mm, Y=9.35 mm, Z=817.37 mm |
| **Brakes** | Electromagnetic, GPIO PE7 (main arm) / PE8 (secondary arm) |
| **CAN bus** | 500 kbps on PD0 (RX) / PD1 (TX); auto-retransmission disabled |
| **Tool coupling** | Maxwell kinematic coupling, POGO-pin tool ID readback (4-bit, PE12–PE15) |
| **Dock sensors** | 3 dock presence sensors (PG11, PG12, PG13) |

---

## 1. Configuration

### `Marlin/Configuration.h`

- **`MOTHERBOARD`**: `BOARD_BTT_OCTOPUS_PRO_V1_0`
- **`MP_SCARA`**: Enabled — used as a structural hook only; actual math is replaced.
- **Linkage constants** (used in custom kinematic equations):
  - `SCARA_LINKAGE_1 = 450.0 mm` (physical arm L3)
  - `SCARA_LINKAGE_2 = 435.0 mm` (physical arm L4)
  - `SCARA_LINKAGE_3 = 512.0 mm` (table height reference)
- **`USE_CANBUS`**: Defined — enables the CAN bus module.
- **`EEPROM_SETTINGS`**: Enabled — settings persist across power cycles.
- **`DEFAULT_AXIS_STEPS_PER_UNIT`**: `{ 133.33333333, 88.88888889, 88.88888889, 97.89 }`
- **`MANUAL_X/Y/Z_HOME_POS`**: `569.92`, `9.35`, `817.37` mm
- **`HOMING_FEEDRATE_MM_M`**: `{ 400, 400, 400 }`

### `Marlin/Configuration_adv.h`

- **`INPUT_SHAPING_X`** and **`INPUT_SHAPING_Y`**: Enabled — ZV vibration cancellation on X and Y axes.
- **`ADAPTIVE_STEP_SMOOTHING`**: Enabled.
- **`BABYSTEPPING`**: Enabled — fine position adjustments.
- **`SAVED_POSITIONS`**: `10` — slots for `G60`/`G61`.
- **`DIRECT_PIN_CONTROL`**: Enabled (`M42`).
- **`PINS_DEBUGGING`**: Enabled (`M43`).
- **`SQUARE_WAVE_STEPPING`**: **Commented out** (not active in current build).
- **`CNC_COORDINATE_SYSTEMS`**: **Commented out** (not active in current build).

### `Marlin/src/inc/Conditionals_post.h`

- **`SCARA_PRINTABLE_RADIUS`**: Hardcoded to `1200` mm — overrides standard SCARA derivation.
- **`USE_GCODE_SUBCODES`**: Unconditionally defined to `1` — required for `M50.1`/`M50.2` subcodes.

---

## 2. Kinematics — Custom Parallel-Linkage (not SCARA!)

> [!IMPORTANT]
> `MP_SCARA` is enabled **only as a structural hook**. The kinematic math in `scara.cpp` has been **completely replaced** with custom palletizing-robot equations for a 3-axis parallel linkage. Any future changes to this file must account for this — standard SCARA equations are commented out.

### Source: [`Marlin/src/module/scara.cpp`](Marlin/src/module/scara.cpp) & [`scara.h`](Marlin/src/module/scara.h)

**Coordinate system constants** (defined as `constexpr float` in `scara.h`):

| Symbol | Value | Meaning |
|---|---|---|
| `k2` | `SCARA_LINKAGE_1` = 450 mm | Upper arm length |
| `k3` | `SCARA_LINKAGE_2` = 435 mm | Forearm length |
| `k1` | `SCARA_LINKAGE_3` = 512 mm | Vertical offset (table height) |
| `lx` | 139.5 mm | X offset to tool |
| `lz` | 129.43 mm | Z offset |

**Joint limits:**

| Variable | Range |
|---|---|
| `alfa` (A-axis) | −179.99° … +180° |
| `beta` (B-axis) | −90° … +45° |
| `gamma` (C-axis) | −20° … +115° |
| `theta1` = γ − β | 5° … 165° |

**Workspace boundary (conical constraint):**  
`r ≥ r_min` AND `r ≥ (z − q) / k`, where `r_min = 260 mm`, `zo = 950`, `yo = 485`, `k = -zo/yo`, `q = zo`.

### `forward_kinematics(a, b, c)`

Converts joint angles α, β, γ to Cartesian X, Y, Z:

```
alfa  = deg2rad(a)
beta  = π/2 + deg2rad(b)
gamma = π/2 + deg2rad(c)

X = +cos(alfa) × (cos(beta)×k2 + sin((γ−β)−(π/2−β))×k3 + lx)
Y = −sin(alfa) × (cos(beta)×k2 + sin((γ−β)−(π/2−β))×k3 + lx)   ← Y inverted (right-hand coord)
Z = k1 + sin(beta)×k2 − cos((γ−β)−(π/2−β))×k3 − lz
```

### `inverse_kinematics(xyz, is_only_a_question, already_checked)`

Converts target X, Y, Z to joint angles. Full derivation with intermediate variables `alfares`, `d1`, `d2`, `beta1`, `beta2`, `o1`:

1. **Brake check**: reads `extDigitalRead(71)` and `extDigitalRead(72)` — if either is `LOW`, sets `kinematic_calc_failiure = true` and aborts.
2. **Workspace check**: calls `are_xyz_coordinates_possible()` unless `already_checked`.
3. **Solve**: computes α, β, γ.
4. **Angle check**: calls `are_angles_possible()` — sets `kinematic_calc_failiure = true` on violation.
5. **Optional dry-run**: `is_only_a_question = true` returns after validation without updating `delta`.

The `kinematic_calc_failiure` flag propagates to `prepare_fast_move_to_destination()`, `line_to_destination_kinematic()`, and arc planning (`G2`/`G3`) to abort unsafe moves.

---

## 3. CAN Bus Communication

### Source: [`Marlin/src/module/canbus.cpp`](Marlin/src/module/canbus.cpp) / [`canbus.h`](Marlin/src/module/canbus.h)

The `CANBus` class encapsulates all CAN communication:

| Method | Signature | Purpose |
|---|---|---|
| `setup_canbus()` | `void` | Initializes STM32F4 CAN1 peripheral |
| `send_message()` | `(uint32_t id, uint8_t* data, uint8_t len)` | Transmits a CAN frame |
| `receive_message()` | `(uint32_t* id, uint8_t* data, uint8_t* len) → bool` | Polls CAN FIFO0 |

**Hardware configuration:**
- Pins: `PD0` (RX, pull-up enabled), `PD1` (TX), alternate function `GPIO_AF9_CAN1`
- Prescaler: 15; `CAN_BS1_10TQ` / `CAN_BS2_4TQ` → sample point ≈ 73 % at 500 kbps
- `SyncJumpWidth`: `CAN_SJW_2TQ` (increased for clock tolerance between chips)
- `AutoRetransmission`: **DISABLED** (prevents infinite retries on missing ACK)
- `AutoBusOff`: ENABLED
- Filter: bank 0, mask-mode 32-bit, all-pass (ID/mask both 0x0000)
- Sends a `"HELLOCAN"` test message on boot

### G-Code interface: [`Marlin/src/gcode/can/M700_M701.cpp`](Marlin/src/gcode/can/M700_M701.cpp)

| Command | Syntax | Function |
|---|---|---|
| `M700` | `M700 I<id> H<hexdata>` | Send CAN message; hex string pairs e.g. `H1122AABB` (up to 8 bytes) |
| `M701` | `M701` | Poll and print one received CAN message |

---

## 4. Motion Control

### High-level motion: [`Marlin/src/module/motion.cpp`](Marlin/src/module/motion.cpp)

- `prepare_fast_move_to_destination()` and `line_to_destination_kinematic()` check `kinematic_calc_failiure` before executing any move.
- **`direct_angle_change(robot_angles)`**: New function that sets joint angles directly in `delta` and synchronizes the planner — used exclusively by `G7`.

### Low-level stepper / Input Shaping: [`Marlin/src/module/stepper.cpp`](Marlin/src/module/stepper.cpp)

Input Shaping is implemented via a **`ShapingQueue` + `shaping_isr()`** mechanism called from the stepper ISR:

- `INPUT_SHAPING_X` and `INPUT_SHAPING_Y` are both enabled.
- Each axis maintains a queue of time-tagged step echoes (`ShapingQueue`).
- The ISR calls `shaping_isr()` which replays delayed fractional impulses with the ZV (Zero Vibration) weighting.
- Parameters set via `M593 F<freq> D<damping>`.
- Completely abstracted from robot kinematics — operates on final Cartesian steps.

> [!NOTE]
> Input Shaping is **not applied** to `G7` (joint JOG) moves, since those bypass Cartesian motion planning.

### Arc safety: [`Marlin/src/gcode/motion/G2_G3.cpp`](Marlin/src/gcode/motion/G2_G3.cpp)

`plan_arc()` now calls `inverse_kinematics(raw, true)` at each interpolation step and checks `kinematic_calc_failiure` to ensure arc moves respect custom workspace limits.

---

## 5. Custom G-Code Commands

### Motion commands

| G-code | File | Description |
|---|---|---|
| `G0`/`G1 X Y Z F` | `G0_G1.cpp` | Linear move — integrates kinematic boundary check |
| `G2`/`G3` | `G2_G3.cpp` | Arc move — validates each interpolation step against workspace |
| **`G7 A B C`** | `G7.cpp` | **Joint JOG (PTP)** — direct control of angles α, β, γ. A-axis is software-inverted (`× −1`). Supports relative/absolute mode. No input shaping. |
| **`G8 A R [Z] [F]`** | `G8_G9.cpp` | **Polar PTP** — fast arc-like move to polar target `(A°, R mm)`. Converts to Cartesian, then calls `prepare_fast_move_to_destination()`. |
| **`G9 A R [Z] [F]`** | `G8_G9.cpp` | **Polar Linear** — precise linear move to polar target. Converts to Cartesian, then calls `prepare_line_to_destination()`. |
| `G60 S<slot>` | `G60.cpp` | Save current position (up to 10 slots) |
| `G61 S<slot>` | `G61.cpp` | Return to saved position |
| `G28` | `G28.cpp` | Custom multi-stage homing — see §7 |
| `G90` / `G91` | `gcode.cpp` | Absolute / relative positioning |

### Brake control: [`M50_M51.cpp`](Marlin/src/gcode/motion/M50_M51.cpp)

Two-step safety protocol:

| Subcommand | Effect |
|---|---|
| `M50 S0` | Stage: set **both** brakes to driver (automatic — brakes release when motors powered) |
| `M50 S1` | Stage: set **both** brakes to 24 V source (always disengaged) |
| `M50.1 S<x>` | Stage: main arm brake only (PE7) |
| `M50.2 S<x>` | Stage: secondary arm brake only (PE8) |
| `M51` | **Confirm** — executes the staged brake state via `extDigitalWrite(PE7/PE8, state)` |

The global `break_command_pending` flag (declared in `gcode.cpp`) is set by `M50` and cleared by `M51`. If any other command (except `M105`) is received while pending, the staged change is cancelled and prints `"Příkaz M51 zrušen"`.

### Input Shaping commands

| Command | Purpose |
|---|---|
| `M593 F<hz> D<ratio>` | Set shaping frequency and damping ratio for X/Y |
| `M594` | Frequency sweep to identify resonant frequencies |

### Diagnostic commands

| Command | Purpose |
|---|---|
| `M42 P<pin> S<state>` | Set pin state directly |
| `M43` | Report pin statuses |
| `M114` | Report current position |
| `M120` / `M121` | Enable / disable endstops |

---

## 6. Automatic Tool Changer (ATC)

### Source: [`Marlin/src/gcode/motion/Custom_ATC.cpp`](Marlin/src/gcode/motion/Custom_ATC.cpp)

Implements a tool magazine with **Maxwell kinematic coupling** docking:

**Hardware:**
- 4-bit tool ID: POGO pins `PE12`–`PE15` (`TOOL_ID_BIT0_PIN`–`TOOL_ID_BIT3_PIN`)
- 3 dock presence sensors: `PG11` / `PG12` / `PG13`
- Special values: `HEAD_EMPTY = 15`, `HEAD_ERROR = 0`

**Data structures:**
- `ToolData tool_table[16]` — stores offsets, clamp angles, names for up to 16 tools
- `ToolDock magazine[3]` — angular offsets from calibration reference: `{−22.0°, 0.0°, +22.0°}`
- `int8_t tool_in_hand` — currently held tool index (−1 if none)
- `int8_t dock_tool[3]` — which tool is in each dock (−1 if empty)
- Calibration state: `is_atc_calibrated`, `ref_R`, `ref_A`, `ref_Z`

**Supported tools:** dial indicator, FDM print head, pen plotter (and others up to 16)

**G-code interface:**

| Command | Function |
|---|---|
| `M6 T<n>` | Full tool change: unload current → move to target dock → activate servo → verify ID via POGO pins. ID mismatch → fatal error. |
| `M666 T<n> [X Y Z A name]` | Define tool properties (offsets, clamp servo angle, name string) |
| `M666` (no args) | List all stored tools |
| `M667 C<n>` | Teach-in calibration — sets `ref_R`, `ref_A`, `ref_Z` for dock `n` |
| `M667 S` | Auto-scan: scan all docks, identify tools, unload current if needed, optimize scan order |
| `M667 U` | Unload current tool to nearest available dock |

**Internal move primitive:** `exec_blocking_move(a, r, z, f)` converts polar coordinates to Cartesian via `do_blocking_move_to()`, with default feedrate 3000 mm/min.

---

## 7. Homing — "Algorithm 2"

### Source: [`Marlin/src/gcode/calibrate/G28.cpp`](Marlin/src/gcode/calibrate/G28.cpp)

The standard Marlin `G28` function is extended with a fully custom multi-stage homing routine for the palletizing robot:

**Helper functions (defined within G28.cpp):**
- `endstop_pressed(axis)` — reads GPIO `PG6` (A), `PG9` (B), `PG10` (C) directly
- `set_axis_home(axis)` / `is_axis_home_(axis)` — per-axis homed status tracking
- `homeaxis(axis, final_home, BC_homing)` — modified homing: applies bump maneuver only on `final_home=true`; `BC_homing=true` enables simultaneous B/C movement

**Homing sequence:**

1. **Brake pre-check**: reads PE7 and PE8 — aborts if brakes are not in automatic mode.
2. **Reset**: `all_axis_unhomed()` clears all axis homed flags.
3. **Iterative B/C homing loop** (`while (!is_axis_home_(B_AXIS))`):
   - If B endstop pressed: home B axis (`homeaxis(B_AXIS, false)`), mark B homed.
   - If neither B nor C pressed (`two_zero_home` case): set `BC_endstol_check = true`, home both B/C simultaneously.
   - If only C pressed: home B with `BC_homing=true` (`homeaxis(B_AXIS, false, true)`), mark B homed.
4. **C final approach**: `homeaxis(C_AXIS, false)`, mark C homed.
5. **Final homing** (with bump): `homeaxis(A_AXIS, true)`, `homeaxis(B_AXIS, true)`, `homeaxis(C_AXIS, true)`.
6. **Post-home recovery**: if B endstop still pressed, re-home C, then repeat fine B/C homing.
7. **Kinematic sync**: calls `inverse_kinematics(current_position)` to reconcile Cartesian state from homed joint angles.

**External flags** (declared in `endstops.h`):
- `extern bool BC_endstol_check` — conditionally suppresses B/C endstop processing during simultaneous homing
- `extern bool final_home_move` — controls bump maneuver behavior

### Endstop overhaul: [`Marlin/src/module/endstops.cpp`](Marlin/src/module/endstops.cpp)

- `PROCESS_ENDSTOP` macro redefined to trigger on **state change** (`ENDSTOP_CHANGED`) rather than current state — essential for inductive sensors.
- **Cycle-based debouncing**: `endstop_poll_count` / `old_live_state` in `change_state()` — drives the arm deeper into the sensor field to prevent false triggers.
- `BC_endstol_check` suppresses B/C endstop processing when both axes home simultaneously.
- Endstop processing for I, J, K, U, V, W axes has been removed.

---

## 8. Startup Initialization

### [`Marlin/src/MarlinCore.cpp`](Marlin/src/MarlinCore.cpp) — `setup()`

Initialization order relevant to this project:

1. `hal.init()`
2. **CAN bus**: `canbus.setup_canbus()` ← enabled via `USE_CANBUS`
3. **Brake pins**: `PE7`/`PE8` set as `OUTPUT`, written `HIGH` (automatic/safe state)
4. **Tool ID POGO pins**: `PE12`–`PE15` set as `INPUT_PULLUP`
5. **Dock sensors**: `PG11`–`PG13` set as `INPUT_PULLUP`

---

## 9. Build System

| Item | Value |
|---|---|
| **Build tool** | PlatformIO (VSCode extension or CLI) |
| **Framework** | Arduino (STM32 HAL underneath) |
| **Default environment** | `BIGTREE_OCTOPUS_V1` |
| **Simulation environment** | `BIGTREE_OCTOPUS_V1_renode` (Renode, flash offset `0x8000`) |
| **Monitor speed** | 250 000 baud |
| **Monitor settings** | LF line endings, colorize + time + send_on_enter filters |

```bash
# Build for hardware
pio run -e BIGTREE_OCTOPUS_V1

# Build for Renode simulation
pio run -e BIGTREE_OCTOPUS_V1_renode

# Upload
pio run -e BIGTREE_OCTOPUS_V1 --target upload
```

### Key source file exclusions (from `platformio.ini`)

The build explicitly excludes most standard Marlin features not needed for this robot:
LCD menus, bed leveling, temperature G-codes, filament management, standard SCARA module (`-<src/module/scara.cpp>`), delta, polargraph, probe, servo (the custom servo via `M280` is re-included separately), Trinamic TMC driver tools, SD card, etc.

Custom additions explicitly included:
- `+<src/gcode/scara/M365.cpp>` — SCARA-related G-code kept
- `+<src/module/bed_correction.cpp>` — custom bed correction module

---

## 10. Notable Source Files

| File | Why it matters |
|---|---|
| [`scara.cpp`](Marlin/src/module/scara.cpp) | ⚠️ All kinematic math is **custom palletizing logic** — do not treat as standard SCARA |
| [`scara.h`](Marlin/src/module/scara.h) | Joint limits, workspace boundary constants, function declarations |
| [`G28.cpp`](Marlin/src/gcode/calibrate/G28.cpp) | Complete custom multi-stage homing algorithm |
| [`Custom_ATC.cpp`](Marlin/src/gcode/motion/Custom_ATC.cpp) | Full ATC: tool table, dock management, M6/M666/M667 |
| [`G7.cpp`](Marlin/src/gcode/motion/G7.cpp) | Joint JOG — direct angle control |
| [`G8_G9.cpp`](Marlin/src/gcode/motion/G8_G9.cpp) | Polar PTP and polar linear moves |
| [`M50_M51.cpp`](Marlin/src/gcode/motion/M50_M51.cpp) | Two-step electromagnetic brake control |
| [`stepper.cpp`](Marlin/src/module/stepper.cpp) | ZV input shaping via `ShapingQueue` / `shaping_isr()` |
| [`canbus.cpp`](Marlin/src/module/canbus.cpp) | CAN bus integration (STM32F4 CAN1, 500 kbps) |
| [`M700_M701.cpp`](Marlin/src/gcode/can/M700_M701.cpp) | CAN send/receive G-code interface |
| [`endstops.cpp`](Marlin/src/module/endstops.cpp) | Cycle-based debouncing for inductive sensors |
| [`endstops.h`](Marlin/src/module/endstops.h) | `BC_endstol_check`, `final_home_move` extern declarations |
| [`motion.cpp`](Marlin/src/module/motion.cpp) | `direct_angle_change()`, kinematic failure integration |
| [`gcode.cpp`](Marlin/src/gcode/gcode.cpp) | `break_command_pending` flag, custom G-code dispatch |
| [`MarlinCore.cpp`](Marlin/src/MarlinCore.cpp) | Startup init: CAN, brakes, ATC pins |
| [`Configuration.h`](Marlin/Configuration.h) | All hardware parameters |
| [`Configuration_adv.h`](Marlin/Configuration_adv.h) | Input shaping, babystepping, ATC settings |
| [`Conditionals_post.h`](Marlin/src/inc/Conditionals_post.h) | `SCARA_PRINTABLE_RADIUS=1200`, `USE_GCODE_SUBCODES=1` |
