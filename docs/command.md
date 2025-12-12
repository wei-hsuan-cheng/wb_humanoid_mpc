# Command Interfaces and Pose References (Unitree G1)

This note explains how external commands (GUI/keyboard/joystick) are converted into **base pose/velocity references** that the centroidal and whole‑body MPC controllers track.

Relevant code:

- Centroidal keyboard pose command:  
  `humanoid_nmpc/humanoid_centroidal_mpc_ros2/src/CentroidalMpcKeyboardPoseCommandNode.cpp`
- Centroidal MPC interface:  
  `humanoid_nmpc/humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp`
- Whole‑body MPC interface:  
  `humanoid_nmpc/humanoid_wb_mpc/src/WBMpcInterface.cpp`
- Common reference manager and swing logic:  
  `humanoid_nmpc/humanoid_common_mpc/include/humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h`

High‑level picture:

- External UI (GUI / keyboard / joystick) emits **desired base motion**:
  - Either as a relative pose displacement $(\Delta x, \Delta y, \Delta z, \Delta \theta_z)$.
  - Or as a base velocity command $(v_x, v_y, \omega_z)$.
- A **target‑trajectory calculator** (centroidal or WB) converts that command and the current observation into a `TargetTrajectories` object:

  $$
    \{ t_i, x^\text{ref}(t_i), u^\text{ref}(t_i) \}_{i=0}^N
  $$
- The MPC interfaces (`CentroidalMpcInterface`, `WBMpcInterface`) feed these trajectories to the optimal control problem, and the quadratic costs in `Q`, `R`, `Q_final` make the robot track them.

---

## What Is in the Reference State?

Both MPC stacks build $x^\text{ref}(t)$ from the **current base pose**, a **default joint posture**, and the **commanded base motion**.

### Centroidal MPC (`g1_centroidal_mpc`)

Code: [`CentroidalMpcTargetTrajectoriesCalculator.cpp`](../humanoid_nmpc/humanoid_centroidal_mpc/src/command/CentroidalMpcTargetTrajectoriesCalculator.cpp)

For each knot of the `TargetTrajectories`:

- State layout is  
  $x = [\bar{h}\;\; p_b\;\; \theta_{b,zyx}\;\; q_j]$ (centroidal momentum, base pose, joint angles).
- Base pose:
  - `getCurrentBasePoseTarget()` reads the current base pose from the MPC state and zeroes roll/pitch.
  - `getDeltaBaseTarget()` or `integrateTargetBasePose()` applies the commanded displacement / velocity.
  - Vertical position is built around `defaultBaseHeight` from  
    `g1_centroidal_mpc/config/command/reference.info`.
- Joints:
  - All MPC‑active joints (legs, waist, arms) are set to `defaultJointState` from the same `reference.info`.
  - Swing / contact variations for feet and arms are then added by the switched‑model reference manager (see `swing_strategy.md`).
- Momentum:
  - In `commandedVelocityToTargetTrajectories`, the first 6 entries are a **target centroidal momentum** derived from the commanded base twist and the centroidal momentum matrix.

In short, the reference state for centroidal MPC contains:

- Base COM pose following the commanded displacement/velocity.
- A nominal full‑body joint posture (legs + torso + both arms).
- A centroidal momentum profile consistent with the commanded base motion.

### Whole‑Body MPC (`g1_wb_mpc`)

Code: [`WBMpcTargetTrajectoriesCalculator.cpp`](../humanoid_nmpc/humanoid_wb_mpc/src/command/WBMpcTargetTrajectoriesCalculator.cpp)

Here the state is  
$x = [p_b\;\; \theta_{b,zyx}\;\; q_j\;\; v_b\;\; \omega_b\;\; \dot{q}_j]$ (base pose, joint angles, base velocities, joint velocities), and the calculator fills:

- Base pose:
  - Same `getCurrentBasePoseTarget()` / `integrateTargetBasePose()` logic as centroidal, using `defaultBaseHeight`.
- Joint angles:
  - `q_j` is set to the same `defaultJointState` block from  
    `g1_wb_mpc/config/command/reference.info`.
  - Swing/gait modulation (legs + arm swing) is handled by the reference manager.
- Base velocity:
  - `targetBaseVel` is built from the filtered commanded velocity `[v_x, v_y, 0, ω_z, 0, 0]`.
- Joint velocities:
  - Filled with zeros in the reference (tracking weights in `Q`, `Q_final` encourage smooth, low‑velocity motion).

So for WB MPC the reference state contains:

- A base pose trajectory consistent with the commanded walking velocity.
- A nominal full‑body joint posture (including both arms) that the swing logic perturbs.
- A base‑velocity target aligned with the commanded velocity; joint velocities are implicitly driven by the swing/gait and the `Q`, `R` weights.

The numeric tracking weights for all of these components live in the `Q`, `Q_final`, and `R` blocks of:

- Centroidal: `robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task.info`.
- Whole‑body: `robot_models/unitree_g1/g1_wb_mpc/config/mpc/task.info`.

---

## Centroidal MPC: Keyboard Pose Commands

File: `humanoid_nmpc/humanoid_centroidal_mpc_ros2/src/CentroidalMpcKeyboardPoseCommandNode.cpp`

This node implements a **keyboard‑driven pose command** for the pelvis:

- It constructs a `CentroidalMpcInterface`, which gives access to:
  - `getMpcRobotModel()` – the centroidal robot model (mapping centroidal state → joint space).
  - `getPinocchioInterface()` – for kinematics.
  - `getCentroidalModelInfo()` – state/input dimensions, etc.
- It then creates a `CentroidalMpcTargetTrajectoriesCalculator`:

  ```cpp
  CentroidalMpcTargetTrajectoriesCalculator mpcTargetTrajectoriesCalculator(
      referenceFile, interface.getMpcRobotModel(), interface.getPinocchioInterface(),
      interface.getCentroidalModelInfo(), interface.mpcSettings().timeHorizon_);
  ```

- The node defines a callback mapping **commanded displacements** to `TargetTrajectories`:

  ```cpp
  TargetTrajectoriesKeyboardPublisher::CommandLineToTargetTrajectories targetTrajectoriesFunc =
      [&mpcTargetTrajectoriesCalculator](const vector4_t& commandedVelocities,
                                         const SystemObservation& observation) mutable {
        return mpcTargetTrajectoriesCalculator.commandedPositionToTargetTrajectories(
            commandedVelocities, observation.time, observation.state);
      };
  ```

  Here:

  - `commandedVelocities` is actually a 4‑vector $(\Delta x, \Delta y, \Delta z, \Delta \theta_z)$ (see the node’s prompt string).
  - `observation` contains the current time and state from the dummy sim or real robot.

- `TargetTrajectoriesKeyboardPublisher` handles the user interface:
  - It prompts the user: “Enter XYZ and Yaw (deg) displacements for the PELVIS, separated by spaces”.
  - When the user enters a displacement, it calls `targetTrajectoriesFunc`, gets a `TargetTrajectories` object, and publishes it to the MPC.

Effect on MPC:

- `CentroidalMpcTargetTrajectoriesCalculator::commandedPositionToTargetTrajectories`:
  - Takes the current base pose from the observation.
  - Adds the user‑specified displacement to define a new desired base pose trajectory over the MPC horizon.
  - Generates consistent joint references (via the centroidal robot model and reference file) so the whole body moves toward the new pose.
- The centroidal MPC then tracks this new base pose via:
  - The **centroidal state cost** (base pose entries in `Q`, `Q_final`).
  - Any task‑space feet/link costs configured in the centroidal `task.info`.

In short, the keyboard pose node lets you specify “move the pelvis by $(\Delta x, \Delta y, \Delta z, \Delta \theta_z)$”, and MPC will replan trajectories that move and stabilize the model around the new base pose.

---

## Whole‑Body MPC: Velocity‑Based Commands (GUI / Joystick)

The whole‑body MPC side uses velocity targets rather than pose displacements, routed through a **procedural motion manager**.

### Target Trajectories from Velocity Commands

In `humanoid_nmpc/humanoid_wb_mpc_ros2/src/WBMpcRobotSim.cpp`, the whole‑body sim sets up:

- A `WBMpcTargetTrajectoriesCalculator`:

  ```cpp
  WBMpcTargetTrajectoriesCalculator mpcTargetTrajectoriesCalculator(
      referenceFile, interface.getMpcRobotModel(), interface.mpcSettings().timeHorizon_);
  ```

- A conversion function from **velocity command** to `TargetTrajectories`:

  ```cpp
  ProceduralMpcMotionManager::VelocityTargetToTargetTrajectories targetTrajectoriesFunc =
      [&mpcTargetTrajectoriesCalculator](const vector4_t& velocityTarget,
                                         scalar_t initTime, scalar_t finalTime,
                                         const vector_t& initState) mutable {
        return mpcTargetTrajectoriesCalculator.commandedVelocityToTargetTrajectories(
            velocityTarget, initTime, initState);
      };
  ```

  where:

  - `velocityTarget` is a 4‑vector of commanded base velocities (typically $(v_x, v_y, v_z, \omega_z)$, with $v_z$ often zero).
  - `initTime`, `initState` are the current MPC time and state.

- A `Ros2ProceduralMpcMotionManager`:

  ```cpp
  auto ros2ProceduralMpcMotionManager = std::make_shared<Ros2ProceduralMpcMotionManager>(
      gaitFile, referenceFile, interface.getSwitchedModelReferenceManagerPtr(),
      interface.getMpcRobotModel(), targetTrajectoriesFunc);

  ros2ProceduralMpcMotionManager->subscribe(nodeHandle, qos);
  ```

The motion manager subscribes to a ROS 2 **velocity command topic** (published by GUI/joystick nodes) and, whenever a new velocity command arrives, uses `targetTrajectoriesFunc` to generate a new `TargetTrajectories` for the MPC to track.

Conceptually:

- `velocityTarget` defines the desired **base COM velocity** and yaw rate.
- `commandedVelocityToTargetTrajectories` integrates that over the horizon to generate a predicted base path:

  $$
    \mathbf{p}_b^\text{ref}(t) \approx \mathbf{p}_b(t_0) +
      \int_{t_0}^{t} R_z(\theta_z(\tau)) \, \mathbf{v}_\text{cmd} \, d\tau,
  $$

  with appropriate gait and swing patterns baked in.

- The MPC then tracks this path via:
  - Base pose and velocity entries in `Q`, `Q_final` (pose tracking).
  - Foot task‑space costs (feet swing/contact kinematics).

### GUI / Joystick Publishers

The **remote_control** package contains the front‑end nodes that actually provide the `velocityTarget` signals:

- Package: `humanoid_nmpc/remote_control`
  - GUI + gamepad node for base velocity control (used by **both centroidal and WB** launch files via `MPCLaunchConfig.base_velocity_controller_gui_node`):  
    `remote_control/base_velocity_controller_gui.py`
  - Keyboard/Xbox publishers:  
    - `remote_control/keyboard_walking_command_publisher.py`  
    - `remote_control/xbox_walking_command_publisher.py`

These nodes:

- Provide a UI (Tkinter GUI, keyboard CLI, or joystick input).
- Let the user select a desired walking velocity (e.g. forward/lateral speed and yaw rate).
- Publish these commands (typically as ROS 2 messages) at some rate on a topic the motion manager subscribes to.

From the MPC point of view:

- The **GUI/joystick** only specifies *desired base motion* via `humanoid_mpc_msgs/WalkingVelocityCommand` on `/humanoid/walking_velocity_command`.
- `Ros2ProceduralMpcMotionManager` converts each message into a `vector4_t velocityTarget`, then calls the appropriate
  target‑trajectory calculator:
  - Centroidal: [`CentroidalMpcTargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories`](../humanoid_nmpc/humanoid_centroidal_mpc/src/command/CentroidalMpcTargetTrajectoriesCalculator.cpp).
  - Whole‑body: [`WBMpcTargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories`](../humanoid_nmpc/humanoid_wb_mpc/src/command/WBMpcTargetTrajectoriesCalculator.cpp).
- The resulting `TargetTrajectories` provide full reference states (base pose/velocity + joint posture) that the centroidal or WB MPC tracks using the costs described in `pose_tracking.md`.

---

## Summary

- Centroidal keyboard node:
  - User enters relative pelvis pose $(\Delta x, \Delta y, \Delta z, \Delta \theta_z)$.
  - `CentroidalMpcTargetTrajectoriesCalculator::commandedPositionToTargetTrajectories` builds a new base trajectory.
  - Centroidal MPC tracks this trajectory via base/joint state costs and task‑space feet costs.

- Velocity control (GUI/joystick or keyboard) for **both centroidal and WB**:
  - User commands base velocity $(v_x, v_y, v_z, \omega_z)$ and pelvis height via `WalkingVelocityCommand`.
  - `Ros2ProceduralMpcMotionManager` and the corresponding target‑trajectory calculator turn this into a full reference state (base pose/velocity + full‑body posture).
  - The centroidal or WB MPC then tracks it via the quadratic state–input costs and task‑space foot/link costs described in `pose_tracking.md`.

This is the current “reference state and command” pipeline: GUI/keyboard → `WalkingVelocityCommand` or pose keyboard node → `TargetTrajectories` → MPC pose tracking.
