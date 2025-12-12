# Whole-Body Humanoid MPC MuJoCo Simulation Integration

This document describes how the humanoid MPC examples in this repo use **MuJoCo** as their physics backend (both whole‑body and centroidal variants), how robot and environment properties are defined, and how the main pieces of code relate to each other.

## High‑Level Overview

- **OCS2 MPC** for Unitree G1
  - The code [`CentroidalMpcRobotSim.cpp`](../humanoid_nmpc/humanoid_centroidal_mpc_ros2/src/CentroidalMpcRobotSim.cpp) and [`WBMpcRobotSim.cpp`](../humanoid_nmpc/humanoid_wb_mpc_ros2/src/WBMpcRobotSim.cpp) run the **OCS2 centroidal MPC** and **whole‑body MPC stack**, respectively.
  
  - The torque commands are sent to a custom **hardware control interface** `robot_runtime/robot_model` that implements a [`RobotHWInterfaceBase.h`](../robot_runtime/robot_model/include/robot_model/RobotHWInterfaceBase.h) (not through `ros2_control hardware_interface`) 
  
  - MuJoCo as the **physics backend** via [`MujocoSimInterface.cpp`](../robot_runtime/mujoco_sim_interface/src/MujocoSimInterface.cpp).

- Robot configurations (kinematical structure and joint limits) come from a **URDF** (*e.g.,* [`g1_29dof.urdf`](../robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf)), parsed into [`RobotDescription.cpp`](../robot_runtime/robot_model/src/RobotDescription.cpp).

- MuJoCo dynamics, collision, actuators, sensors, and the ground plane come from a **MuJoCo XML** model (*e.g.,* [`g1_29dof.xml`](../robot_models/unitree_g1/g1_description/urdf/g1_29dof.xml)), loaded by [`MujocoSimInterface.cpp`](../robot_runtime/mujoco_sim_interface/src/MujocoSimInterface.cpp).

### In each iteration of control loop
  1. MuJoCo advances the full‑body dynamics (the physical interactions).
  2. The sim state is mapped into a [`RobotState.cpp`](../robot_runtime/robot_model/src/RobotState.cpp).
  3. MPC computes joint torque commands from desired and feedback joint states via [`RobotJointAction.h`](../robot_runtime/robot_model/include/robot_model/RobotJointAction.h).
  4. [`MujocoSimInterface.cpp`](../robot_runtime/mujoco_sim_interface/src/MujocoSimInterface.cpp) writes those torque commands back to MuJoCo actuators (`mjData->ctrl`).

---

## Whole‑Body MPC + MuJoCo

### Simulation Entrypoint: `WBMpcRobotSim`

File: [`humanoid_nmpc/humanoid_wb_mpc_ros2/src/WBMpcRobotSim.cpp`](../humanoid_nmpc/humanoid_wb_mpc_ros2/src/WBMpcRobotSim.cpp)

- Command‑line arguments:
  - `robotName`, `taskFile`, `referenceFile`, `urdfFile`, `gaitFile`, `mjxFile`.
  - `urdfFile` is the robot description used by both MPC and `RobotDescription`.
  - `mjxFile` is the MuJoCo scene/model file (`MujocoSimConfig::scenePath`).
- MPC stack:
  - Builds `WBMpcInterface` from the task and reference files.
  - Instantiates `SqpMpc` and sets up reference managers and ROS 2 visualization.
- Initial state for simulation:
  - Builds a `RobotDescription` and `RobotState` from `urdfFile`.
  - Uses the MPC robot model (`interface.getMpcRobotModel()`) to:
    - Set the base position from the OCS2 initial state.
    - Populate joint angles for the MPC model joints.
- MuJoCo interface:
  - Fills a `robot::mujoco_sim_interface::MujocoSimConfig` with:
    - `scenePath = mjxFile`.
    - `initStatePtr_` pointing to the initialized `RobotState`.
    - `dt` and `verbose` as desired.
  - Constructs `MujocoSimInterface robotInterface(config, urdfFile)`.
  - Creates `WBMpcMrtJointController` with:
    - The same `RobotDescription` used by MuJoCo.
    - MPC model settings and Pinocchio interface.
- Runtime loop:
  1. `robotInterface.initSim()` performs one MuJoCo step and optionally starts rendering.
  2. `mpcJointController.startMpcThread(robotInterface.getRobotState())` spawns the MPC thread.
  3. After the first MPC policy is received, `robotInterface.startSim()` starts the MuJoCo sim thread.
  4. The main loop runs at `mrtDeltaTMicroSeconds_` (here hard‑coded to 500 Hz):
     - `robotInterface.updateInterfaceStateFromRobot()` copies the internal MuJoCo state into the public `RobotState`.
     - `mpcJointController.computeJointControlAction(...)` fills a `RobotJointAction` with PD gains and feedforward torques.
     - `robotInterface.applyJointAction()` publishes the new joint actions to the simulation thread.

---

## Centroidal MPC + MuJoCo

The centroidal MPC example reuses the same MuJoCo backend and robot abstractions, but solves a reduced‑order centroidal OCP and then maps its output back to joint‑space torques.

### Simulation Entrypoint: `CentroidalMpcRobotSim`

File: [`humanoid_nmpc/humanoid_centroidal_mpc_ros2/src/CentroidalMpcRobotSim.cpp`](../humanoid_nmpc/humanoid_centroidal_mpc_ros2/src/CentroidalMpcRobotSim.cpp)

- Command‑line arguments:
  - `robotName`, `taskFile`, `referenceFile`, `urdfFile`, `gaitFile`, `mjxFile` (same pattern as the whole‑body case).
  - `taskFile` is `robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task.info`, which defines the centroidal state, input, horizon, and weights.
  - `urdfFile` and `mjxFile` are the same `g1_29dof.urdf`/`g1_29dof.xml` pair used by the whole‑body MPC.
- MPC stack:
  - Builds a `CentroidalMpcInterface` from the task and reference files ([`humanoid_nmpc/humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp`](../humanoid_nmpc/humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp)).
  - Instantiates `SqpMpc` for the centroidal OCP.
  - Sets up the same `Ros2ProceduralMpcMotionManager` and `HumanoidVisualizer` as in the whole‑body case, but using the centroidal robot model and centroidal model info.
- Centroidal model (see also `docs/g1_centroidal_mpc.md`):
  - State includes normalized centroidal momentum, base pose (position + ZYX orientation), and MPC‑active joint angles.
  - Input includes foot contact wrenches and joint velocities.
  - Dynamics, costs, and constraints are configured via the centroidal MPC task file and `CentroidalMpcInterface`.

### Initialization of the Full‑Body State

Lines `93-107` of `CentroidalMpcRobotSim.cpp` mirror the whole‑body initialization:

- Builds a `RobotDescription` from `urdfFile` and an empty `RobotState`.
- Uses the centroidal MPC robot model (`interface.getMpcRobotModel()`) to:
  - Set the base position from the OCS2 initial centroidal state (`getBasePosition(initMpcState)`).
  - Extract MPC joint angles (`getJointAngles(initMpcState)`) and write them into the full‑body `RobotState` at the corresponding joint indices `robotDescription.getJointIndices(interface.modelSettings().mpcModelJointNames)`.
- Wraps this `RobotState` in a `std::shared_ptr` and assigns it to `MujocoSimConfig::initStatePtr_`, exactly as in the whole‑body simulation.

### MuJoCo Interface and Centroidal MRT Controller

- The MuJoCo configuration and interface setup is identical:
  - `config.scenePath = mjxFile;`
  - `MujocoSimInterface robotInterface(config, urdfFile);`
  - `robotInterface.initSim();` then `robotInterface.startSim();`.
- The main difference is in the MRT controller:
  - `CentroidalMpcMrtJointController` is constructed with:
    - The same `RobotDescription` used by the MuJoCo backend.
    - Centroidal `ModelSettings` (including `mpcModelJointNames` and `fixedJointNames` from `g1_centroidal_mpc/config/mpc/task.info`).
    - The centroidal `CentroidalMpcRobotModel`, the MPC solver, and Pinocchio interface.
  - It owns:
    - A cloned centroidal robot model (`mpcRobotModelPtr_`).
    - A `MpcMrtInterface` for running the rollouts and SQP solver on a separate thread.
    - Two joint index sets:
      - `mpcJointIndices_` for MPC‑controlled joints.
      - `otherJointIndices_` for remaining joints that receive a stabilizing PD.

### Observation and Action Mapping (Centroidal)

See [`humanoid_nmpc/humanoid_centroidal_mpc/src/mrt/CentroidalMpcMrtJointController.cpp`](../humanoid_nmpc/humanoid_centroidal_mpc/src/mrt/CentroidalMpcMrtJointController.cpp).

- Observation build:
  - The controller reads the latest `RobotState` from the MuJoCo backend.
  - It maps:
    - Base pose and twist into the centroidal state layout (normalized centroidal momentum, base position/orientation).
    - Joint positions/velocities for `mpcJointIndices_` into the joint part of the state.
    - Contact flags from `RobotState::getContactFlags()` into the mode/phase definition used by the centroidal OCP.
  - This logic is summarized (with code pointers and math) in `docs/g1_centroidal_mpc.md` under “Observation”.
- MPC policy evaluation and torque synthesis:
  - Once a policy is available (`CentroidalMpcMrtJointController::updateJointController`):
    - The controller evaluates the centroidal policy to obtain desired joint positions/velocities and foot contact wrenches.
    - It computes desired joint accelerations via simple PD in the centroidal joint space:
      - `qdd_j_des = Kp * (q_j_des - q_j) + Kd * (qd_j_des - qd_j)` (currently `Kp = Kd = 0` by default, see constructor).
    - It calls `computeJointTorques` (from `humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h`) to map:
      - Generalized coordinates and velocities (`q`, `qd`),
      - Desired joint accelerations,
      - And commanded contact wrenches,
      - Into a vector of joint torques by inverse dynamics.
  - Mapping into `RobotJointAction`:
    - For each MPC joint index:
      - `q_des` and `qd_des` are set from the centroidal reference.
      - `kp` and `kd` are set (*e.g.,* `1200` and `10`).
      - `feed_forward_effort` is set to the corresponding component of the inverse‑dynamics torque.
    - For “other” joints (not in `mpcJointIndices_`), the controller:
      - Applies a stabilizing PD toward zero position/velocity with smaller gains.
- Before any MPC policy is available:
  - The controller uses a weight‑compensation input computed from the centroidal model and Pinocchio, then maps it to feed‑forward torques via the same `computeJointTorques` helper.

---

## Runtime Loop (Centroidal vs Whole‑Body)

The main loop in [`CentroidalMpcRobotSim.cpp:142-159`](../humanoid_nmpc/humanoid_centroidal_mpc_ros2/src/CentroidalMpcRobotSim.cpp) is structurally identical to the whole‑body version:

1. `robotInterface.updateInterfaceStateFromRobot();`
2. `mpcJointController.computeJointControlAction(0.0, robotInterface.getRobotState(), robotInterface.getRobotJointAction());`
3. `robotInterface.applyJointAction();`
4. `rclcpp::spin_some(nodeHandle);`

The only difference is the internal logic of `CentroidalMpcMrtJointController` described above (centroidal state, centroidal dynamics, and torque mapping).

---

## MuJoCo Simulation Layer [`MujocoSimInterface.cpp`](../robot_runtime/mujoco_sim_interface/src/MujocoSimInterface.cpp)


### Construction and Setup

- `MujocoSimInterface` derives from `robot::model::RobotHWInterfaceBase` and is configured through:
  - `MujocoSimConfig` (`scenePath`, `initStatePtr_`, `dt`, `renderFrequencyHz`, `headless`, `verbose`).
- In the constructor:
  - Loads the MuJoCo model from `config.scenePath` via `mj_loadXML` (`MujocoSimInterface.cpp:55-60`).
  - Allocates `mjData` with `mj_makeData` (`:62-63`).
  - Sets the simulation time step `mujocoModel_->opt.timestep = config_.dt` and caches `timeStepMicro_` (`:74-76`).
  - Calls `setupJointIndexMaps()` to map between MuJoCo joints/actuators and the abstract robot joints (`:80`, `:167-199`).
  - Builds an initial `RobotState`:
    - Uses `config_.initStatePtr_` if provided, otherwise zero config and a default base height of 1.0 m (`:82-90`).
    - Writes that state into MuJoCo via `setSimState` (`:267-297`).
  - Applies a default joint damping of 10.0 to all dynamic DOFs (`:92-99`).
  - Scans available MuJoCo sensors to cache indices for foot sensors (if present) (`:101-113`).
  - Saves a copy of the initial `qpos`/`qvel` into `qpos_init_` and `qvel_init_` for later reset (`:115-128`).

### Joint and Actuator Mapping

Function: [`MujocoSimInterface::setupJointIndexMaps()`](../robot_runtime/mujoco_sim_interface/src/MujocoSimInterface.cpp)

- Joints:
  - Loops over all MuJoCo joints (`mujocoModel_->njnt`).
  - For each joint name, checks if `RobotDescription` contains it:
    - If yes, records it in `activeMuJoCoJointNames_`.
    - If not, prints a warning (joint exists in MuJoCo but is not exposed through the runtime API).
  - Uses `RobotDescription::getJointIndices(activeMuJoCoJointNames_)` to compute `activeRobotJointStateIndices_`.
- Actuators:
  - Loops over MuJoCo actuators (`mujocoModel_->nu`), and gets each actuator’s name via `mj_id2name(..., mjOBJ_ACTUATOR, i)`.
  - The actuator is associated with a robot joint if a joint with the same name exists in `RobotDescription`.
  - The resulting vector `activeRobotActuatorIndices_` maps MuJoCo actuator index → abstract joint index.
- These mappings are used in:
  - `setSimState` and `updateThreadSafeRobotState` to map joint positions/velocities.
  - `simulationStep` to map `RobotJointAction` torques into `mjData_->ctrl[i]`.

### State Mapping: MuJoCo $\leftrightarrow$ `RobotState`

Functions:
- `setSimState(const model::RobotState&)` (`MujocoSimInterface.cpp:267-297`)
- `updateThreadSafeRobotState()` (`MujocoSimInterface.cpp:303-331`)

Mapping robot state into MuJoCo (`setSimState`):

- Root pose:
  - Writes base position into `qpos[0..2]`.
  - Writes base orientation quaternion (local → world) into `qpos[3..6]`.
- Root twist:
  - Converts root linear velocity from local to world frame and writes into `qvel[0..2]`.
  - Writes local angular velocity into `qvel[3..5]`.
- Joint state:
  - For each active joint index `i`, writes:
    - `qpos[i + 7] = robotStateInternal_.getJointPosition(activeRobotJointStateIndices_[i])`.
    - `qvel[i + 6] = robotStateInternal_.getJointVelocity(activeRobotJointStateIndices_[i])`.

Mapping MuJoCo back into a thread‑safe `RobotState` (`updateThreadSafeRobotState`):

- For each active joint:
  - Reads `qpos[i + 7]`, `qvel[i + 6]` from MuJoCo and updates the internal `RobotState`.
- Base pose and twist:
  - Builds a quaternion from `qpos[3..6]` and stores it as root orientation.
  - Reads angular velocity from `qvel[3..5]`.
  - Converts linear velocity from world frame (`qvel[0..2]`) back to the local frame with the inverse quaternion.
- Contacts:
  - Currently sets both foot contact flags to `true` (placeholder; MuJoCo foot touch sensors could be used instead).
- Time:
  - Copies `mujocoData_->time` into `RobotState::time`.
- Finally pushes this state into `threadSafeRobotState_`, which is what the MPC thread reads via `RobotHWInterfaceBase::getRobotState()` and `updateInterfaceStateFromRobot()`.

### Control Mapping and Simulation Loop

Torque computation per joint is encoded in `robot::model::JointAction`:

File: [`robot_runtime/robot_model/include/robot_model/RobotJointAction.h`](../robot_runtime/robot_model/include/robot_model/RobotJointAction.h)

- Each joint has:
  - Desired position/velocity `q_des`, `qd_des`.
  - Gains `kp`, `kd`.
  - Feedforward torque `feed_forward_effort`.
- Effective torque:
  - `getTotalFeedbackTorque(q, qd)` returns `kp * (q_des - q) + kd * (qd_des - qd) + feed_forward_effort`.
  $$
      \tau_{motor} = \tau_{ff} + K_p(q^* - q) + K_d(\dot{q}^* - \dot{q})
  $$

Simulation loop (`MujocoSimInterface::simulationStep`, `MujocoSimInterface.cpp:357-387`):

1. Copies `RobotJointAction` into a thread‑local copy `robotJointActionInternal_` from `threadSafeRobotJointAction_`.
2. For each active actuator:
   - Looks up its joint index `idx`.
   - Evaluates `getTotalFeedbackTorque` using the current joint position/velocity from `robotStateInternal_`.
   - Writes the result into `mujocoData_->ctrl[i]`.
3. Locks `mujocoMutex_` and calls `mj_step(mujocoModel_, mujocoData_)` to advance the physics.
4. Updates the internal `RobotState` via `updateThreadSafeRobotState()` and refreshes timing metrics.
5. Optional auto‑reset:
   - If the base height `qpos[2] < 0.2`, it:
     - Calls `reset()` to restore `qpos`/`qvel` from the initial state.
     - Zeroes all actuator commands and steps once more.
     - Resets FPS and drift metrics.
     - Sleeps 1 s to let the controller recover.

`simulationLoop` (`MujocoSimInterface.cpp:393-404`) repeatedly calls `simulationStep()` at the configured `dt` using `timeStepMicro_`. `initSim()` and `startSim()` (`:410-423`) wrap the initial step and spawn the simulation thread, optionally also starting the OpenGL renderer.

---

## Robot Model Abstractions (`robot_model`)

The MuJoCo interface builds on a small generic hardware interface abstraction:

- [`RobotHWInterfaceBase.h`](../robot_runtime/robot_model/include/robot_model/RobotHWInterfaceBase.h)
  - Holds:
    - A `RobotDescription` parsed from a URDF.
    - A `RobotState` used as the public read‑only view.
    - A `RobotJointAction` container for commanded actions.
  - Provides:
    - `getRobotDescription()` — joint names, limits, indexing.
    - `getRobotState()` — current state snapshot for controllers.
    - `getRobotJointAction()` — reference to fill with new commands.
    - `applyJointAction()` — publishes new actions to a thread‑safe buffer.
- [`RobotDescription.cpp`](../robot_runtime/robot_model/src/RobotDescription.cpp)
  - Input: URDF path (`urdfPath`).
  - Uses `urdf::parseURDF` to:
    - Discover all revolute and prismatic joints.
    - Build a mapping from joint name to:
      - Joint index (`id`).
      - Min/max angle, max velocity, max effort from URDF limits.
  - Exposes joint names and index vectors used to dimension `RobotState` and `RobotJointAction`.
- [`RobotState.cpp`](../robot_runtime/robot_model/include/robot_model/RobotState.cpp)
  - Holds:
    - Base position, orientation, linear and angular velocity.
    - Per‑joint position, velocity, and measured effort.
    - Contact flags and a time stamp.
  - Provides convenience functions to get vectors of joint positions/velocities by index list.
- [`RobotJointAction.h`](../robot_runtime/robot_model/include/robot_model/RobotJointAction.h)
  - A `JointIdMap<JointAction>` pre‑sized based on `RobotDescription`.
  - Used by MPC to write one PD+feedforward action per actuated joint.

These abstractions decouple the MPC logic from the specific backend (MuJoCo here, but could be another simulator or real hardware).

---

## Physical Properties: Robot and Environment

### Robot Physical Properties

URDF model ([`robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf`](../robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf) and following):

- Defines the Unitree G1 kinematic tree:
  - Links (`<link>` tags) with inertial properties:
    - Mass (`<mass value="..."/>`).
    - Inertia tensor (`<inertia .../>`).
  - Visual and collision meshes per link (`<mesh filename="package://g1_description/meshes/..."/>`).
  - Joints (`<joint>` tags) with:
    - Type (revolute).
    - Axis of rotation (`<axis xyz="..."/>`).
    - Limits: lower/upper angle, maximum effort, and velocity (`<limit .../>`).
- `RobotDescription` reads this URDF and stores:
  - Joint limits and maximum efforts/velocities.
  - Name ↔ index maps used by MPC and `MujocoSimInterface`.

MuJoCo dynamic model ([`robot_models/unitree_g1/g1_description/urdf/g1_29dof.xml`](../robot_models/unitree_g1/g1_description/urdf/g1_29dof.xml) and following):

- Encodes the same link and joint structure in MuJoCo’s native format:
  - `worldbody/body` hierarchy defines link poses and inertials (`<inertial ... mass="..." diaginertia="..."/>`).
  - Each `<joint>` includes:
    - The joint name (matching the URDF name).
    - Axis, motion range (`range="min max"`), and `actuatorfrcrange` for torque limits.
- Actuators:
  - `<actuator>` block lists `<motor name="..._joint" joint="..._joint"/>` (`g1_29dof.xml:244-273`).
  - These names are used by `MujocoSimInterface::setupJointIndexMaps` to determine which joints are actuated and how many actuators (`nActuators_`).

The URDF is used for kinematics, joint limits, and inverse dynamics (via Pinocchio in the MPC stack), whereas the MuJoCo XML is the authoritative source for simulated dynamics and contacts.

### Environment Physical Properties

The environment for the humanoid sim is defined directly in the MuJoCo XML (`g1_29dof.xml:1-5`, `:284-304`):

- Global contact/friction settings:
  - A default geom element (`<default>`) sets friction and contact solver parameters:
    - `friction="3.0 0.1 0.001"` (tangential, torsional, rolling).
    - `solimp` and `solref` define the contact impedance and restitution.
- Ground plane:
  - Under `<worldbody>`:
    - A `floor` geom of type `plane` with a textured material `groundplane` (`g1_29dof.xml:291-304`).
  - This plane is what the robot’s feet make contact with.
- Visual setup:
  - `statistic` center/extent, `visual` headlight and haze color, and a skybox texture (`:284-293`) define the rendered scene.

Gravity and other global options use MuJoCo’s defaults unless overridden (no explicit `<option gravity="...">` is set in this XML).

### Sensors

In the MuJoCo model (`g1_29dof.xml:276-281`):

- An IMU is modeled by gyro and accelerometer sensors:
  - `imu-torso-angular-velocity`, `imu-torso-linear-acceleration`.
  - `imu-pelvis-angular-velocity`, `imu-pelvis-linear-acceleration`.
- `MujocoSimInterface` is prepared to also use contact and force sensors named:
  - `"right_foot_touch_sensor"`, `"left_foot_touch_sensor"`.
  - `"right_foot_force_sensor"`, `"left_foot_force_sensor"` (`MujocoSimInterface.cpp:101-113`).
- In the current G1 XML, only IMU sensors are defined, so:
  - `updateThreadSafeRobotState` currently sets both foot contact flags to `true` as a placeholder.
  - Replacing that with actual sensor thresholds is straightforward once corresponding MuJoCo sensors are added.

### Time Step and Damping

- The simulation time step is configured via `MujocoSimConfig::dt` and written into `mujocoModel_->opt.timestep`.
- Additional joint‑space damping is applied by setting `mujocoModel_->dof_damping[i] = 10.0` for all DOFs beyond the floating base (`MujocoSimInterface.cpp:92-99`).
- Metrics (`Metrics`, tracked by `FPSTracker`) record:
  - Simulation FPS.
  - Real‑time factor.
  - Cumulative drift between wall‑clock time and simulated time (`MujocoSimInterface.cpp:338-351`).

---

## Code Structure

Relevant parts of the repo for the humanoid MuJoCo simulation:

```text
wb_humanoid_mpc/
├── humanoid_nmpc/
│   ├── humanoid_wb_mpc_ros2/
│   │   └── src/
│   │       └── WBMpcRobotSim.cpp              # Whole-body MPC + MuJoCo
│   └── humanoid_centroidal_mpc_ros2/
│       └── src/
│           └── CentroidalMpcRobotSim.cpp      # Centroidal MPC + MuJoCo
├── robot_runtime/
│   ├── mujoco_sim_interface/
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   └── mujoco_sim_interface/
│   │   │       ├── MujocoSimInterface.h
│   │   │       ├── MujocoRenderer.h
│   │   │       └── MujocoUtils.h
│   │   ├── src/
│   │   │   ├── MujocoSimInterface.cpp
│   │   │   └── MujocoRenderer.cpp
│   ├── robot_model/
│   │   ├── include/robot_model/
│   │   │   ├── RobotHWInterfaceBase.h
│   │   │   ├── RobotDescription.h
│   │   │   ├── RobotState.h
│   │   │   ├── RobotJointAction.h
│   │   │   ├── JointIDMap.h
│   │   │   └── ControllerBase.h (and related helpers)
│   │   └── src/
│   │       └── RobotDescription.cpp
│   └── robot_core/
│       └── include/robot_core/
│           ├── Types.h
│           ├── ThreadSafe.h
│           └── FPSTracker.h
└── robot_models/
    └── unitree_g1/
        ├── g1_description/
        │   ├── urdf/
        │   │   ├── g1_29dof.urdf              # URDF used by RobotDescription & Pinocchio
        │   │   └── g1_29dof.xml               # MuJoCo XML used by MujocoSimInterface
        │   └── meshes/                        # STL meshes referenced in URDF/XML
        ├── g1_wb_mpc/
        │   └── config/
        │       ├── mpc/
        │       │   └── task.info              # Whole-body MPC model, horizon, gains, initial state
        │       └── command/
        │           └── reference.info         # Whole-body MPC reference trajectories
        └── g1_centroidal_mpc/
            └── config/
                ├── mpc/
                │   └── task.info              # Centroidal MPC model, horizon, gains, initial state
                └── command/
                    └── reference.info         # Centroidal MPC reference trajectories
```

Use this tree as a starting point when navigating or extending the humanoid MuJoCo simulation.
