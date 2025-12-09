# Unitree G1 Centroidal MPC

This file summarizes the centroidal NMPC stack exercised by `make launch-g1-dummy-sim`, with code pointers and compact math for the model, rollout, constraints, sensing, and actuation.

## State, Input, and Dynamics

- State layout (normalized centroidal momentum, base pose, joints): `humanoid_nmpc/humanoid_centroidal_mpc/include/humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h:52-139`
  $$
    x = \begin{bmatrix} \bar{\mathbf{h}} \\ \mathbf{p}_b \\ \boldsymbol{\theta}_{b,zyx} \\ \mathbf{q}_j \end{bmatrix},\quad
    \bar{\mathbf{h}} = \frac{1}{m}\begin{bmatrix}\mathbf{p}_{\text{lin}}\\ \mathbf{l}\end{bmatrix}
    \in \mathbb{R}^{12+n_j}
  $$

  where $\mathbf{p}_b \in \mathbb{R}^3$, $\boldsymbol{\theta}_{b,zyx}$ are ZYX Euler angles, and $\mathbf{q}_j$ are the $n_j= \text{mpc\_joint\_dim}$ active joints (fixed wrist joints are excluded per `robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task.info`).

- Input layout (contact wrenches + joint velocities): `...CentroidalMpcRobotModel.h:64-147`
  $$
    u = \begin{bmatrix}\mathbf{w}_L \\ \mathbf{w}_R \\ \dot{\mathbf{q}}_j\end{bmatrix},\quad
    \mathbf{w}_i=\begin{bmatrix}\mathbf{f}_i \\ \boldsymbol{\tau}_i\end{bmatrix}
  $$

- Continuous-time centroidal dynamics used in rollout (`CentroidalDynamicsAD`): `humanoid_nmpc/humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp:151-237`
  $$
    m\,\dot{\bar{\mathbf{h}}} =
    \sum_{i\in\{\mathrm{L},\mathrm{R}\}}\begin{bmatrix}\mathbf{f}_i \\ \mathbf{r}_i\times \mathbf{f}_i + \boldsymbol{\tau}_i\end{bmatrix} +
    \begin{bmatrix}m\mathbf{g}\\ \mathbf{0}\end{bmatrix}
  $$

  $$
    \dot{\mathbf{p}}_b = \mathbf{v}_b,\quad
    \dot{\boldsymbol{\theta}}_{b,zyx} = T(\boldsymbol{\theta}_{b,zyx})\,\boldsymbol{\omega}_b,\quad
    \dot{\mathbf{q}}_j = \dot{\mathbf{q}}_j \;(\text{from } u)
  $$

  where $T(\cdot)$ is the standard ZYX mapping. Base twist $[\mathbf{v}_b;\boldsymbol{\omega}_b]$ and contact frames are obtained from Pinocchio each rollout step.

- Rollout integrator: `TimeTriggeredRollout` with `rollout.timeStep = 0.015`, `ODE45` (`robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task.info`).
- MPC horizon and rates (same task file): $T_h = 1.2\,\text{s}$, SQP step $dt=0.02$, MPC loop 80 Hz, MRT 100 Hz.

## Costs and Constraints (Centroidal OCP)

Configured in `CentroidalMpcInterface::setupOptimalControlProblem` (`humanoid_nmpc/humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp:151-237`):

- Quadratic state/input tracking (`stateInputQuadraticCost`, `terminalCost`) using weights in `task.info` (matrices `Q`, `R`, `Q_final`).

- ICP cost for pelvis stability (`icp_Cost`).

- Task-space tracking costs for feet and any extra links defined under `task_space_costs` (per-foot terms added at `:200-224` and generic loop at `:318-348`).

- Soft constraints:
  - Joint limits and foot collision avoidance (`stateSoftConstraintPtr`: `:187-189`).

  - Friction force cone per contact and contact moment XY bounds (`softConstraintPtr`: `:208-210`):
    $$
      \mu \sqrt{f_x^2+f_y^2} \le f_z,\quad |\tau_x| \le \tau_{x,\max},\;|\tau_y| \le \tau_{y,\max}
    $$

- Equality constraints per stance foot (`:211-214`):
  - Zero wrench when the foot is in swing (`zeroWrench`).
  
  - Zero twist (linear/rotational velocity) on stance feet with tunable gains (`zeroVelocity`, `getStanceFootConstraint` at `:243-264`):
    $$
      \mathbf{J}_i \dot{\mathbf{q}} = \mathbf{0}
    $$
  
  - Normal velocity constraint to enforce no motion along the contact normal (`getNormalVelocityConstraint` at `:266-272`).
  - Optional knee mimic kinematics (left/right) enforcing $q_{\text{child}} - \alpha q_{\text{parent}} = 0$ (`:277-313`).


## Observation (What sensors feed the MPC)

- The MRT controller builds the MPC state from simulated proprioception (`humanoid_nmpc/humanoid_centroidal_mpc/src/mrt/CentroidalMpcMrtJointController.cpp:99-136`):
  - Base position/orientation (quaternion $\rightarrow$ Euler ZYX) and base linear/angular velocity.
  - Joint positions and velocities for active MPC joints.
  - Contact flags (2 feet) $\rightarrow$ mode number.
  - Input is zeroed before policy is received.
- In MuJoCo dummy sim, these signals come from the sim state (`robot_runtime/mujoco_sim_interface/src/MujocoSimInterface.cpp:303-331`):
  - Joint pos/vel from `qpos/qvel`.
  - Base pose and twist from the free-flyer DOFs.
  - Contact flags currently hard-coded `true` (placeholder for foot touch sensors at `:315-327`).
  - Timestamp from MuJoCo time.


## Policy Rollout and Control Output

- MPC policy evaluation and torque synthesis (`humanoid_nmpc/humanoid_centroidal_mpc/src/mrt/CentroidalMpcMrtJointController.cpp:142-230`):
  - `evaluatePolicy` gives desired state/input $(x^*, u^*)$ with joint targets $\mathbf{q}_j^*, \dot{\mathbf{q}}_j^*$ and contact wrenches $\mathbf{w}_i^*$.
  - Feedforward joint torques computed by inverse dynamics with the commanded contact wrenches:
    $$
      \boldsymbol{\tau}_{\text{ff}} = \text{RNEA}\big(q,\dot{q},\ddot{q}_j^*, \{\mathbf{w}_i^*\}\big)
    $$
    (`computeJointTorques` in `humanoid_nmpc/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp:214-252`).
  
  - Joint PD + torque command for active joints:
    $$
      \tau_i = k_p(q_i^*-q_i) + k_d(\dot{q}_i^*-\dot{q}_i) + \tau_{\text{ff},i}
    $$
    with $k_p=1200$, $k_d=10$ set in code (`:183-189`).
  
  - Before the first policy arrives, a weight-compensating wrench/torque is applied (`:197-219`).

- The hardware/sim interface applies these torques each sim step (`robot_runtime/mujoco_sim_interface/src/MujocoSimInterface.cpp:357-379`), mapping `feed_forward_effort + PD` into `mujocoData_->ctrl`.

## Quick Signals-to-Actuation Path

1. MuJoCo provides base pose/twist + joint states (`MujocoSimInterface.cpp:303-331`).
2. MPC observation is assembled and passed to MRT (`CentroidalMpcMrtJointController.cpp:126-136`).
3. SQP MPC solves the centroidal OCP with dynamics/constraints above (`CentroidalMpcInterface.cpp:151-237`).
4. Policy $\rightarrow$ joint PD + feedforward torques (`CentroidalMpcMrtJointController.cpp:142-230`, `DynamicsHelperFunctions.cpp:214-252`).
5. Torques sent to actuators (`MujocoSimInterface.cpp:357-379`).
