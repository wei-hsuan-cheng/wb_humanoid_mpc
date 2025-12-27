# Outer-Loop Humanoid Centroidal MPC → Inner-Loop WBC via Whole-Body Inverse Dynamics

This note summarizes the architecture of the **MPC-WBC** two-layer controller in [`humanoid_centroidal_mpc`](../../humanoid_nmpc/humanoid_centroidal_mpc/).
The **outer-loop centroidal NMPC** optimizes ground-reaction forces (GRFs) and joint velocities trajectories based on centroidal dynamics + whole-body kinematics **(CD+WBK)**, and then sends them as commnads for the **inner-loop whole-body controller (WBC)** to track. The WBC is a **joint torque controller** realized via **whole-body inverse-dynamics**.
Note that there are also torque-related costs designed at MPC-level to implicitly influence the ground-reaction forces.

---

## 1. Centroidal NMPC (outer-loop)

### 1.1 State and input

Defined in [`CentroidalMpcRobotModel.h:54`](../../humanoid_nmpc/humanoid_centroidal_mpc/include/humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h).

**State**

The state is

$$
  x =
  \begin{bmatrix}
    h_{\mathrm{com}} \\
    q_b \\
    q_j
  \end{bmatrix},
$$

where

- $h_{\mathrm{com}} = [v_{\mathrm{com}};\, \ell_{\mathrm{com}}/m] \in \mathbb{R}^6$ is centroidal momentum (linear velocity + angular momentum divided by mass),
- $q_b \in \mathbb{R}^6$ is the floating-base pose (position + ZYX orientation),
- $q_j \in \mathbb{R}^{n_a}$ are the actuated joint angles (arm + legs).

**Input**

There are 2 contacts, each with a 6D wrench:

- $W_i = [f_i;\, \tau_i] \in \mathbb{R}^6$, force $f_i$ and torque $\tau_i$ at foot $i$.

The (kino-dynamic) input is

$$
  u =
  \begin{bmatrix}
    W_L \\
    W_R \\
    \dot q_j
  \end{bmatrix},
$$

*i.e.*, 6D left and right foot wrenches plus joint velocities.

In code this is reflected by:

- `N_CONTACTS = 2`
- `CONTACT_WRENCH_DIM = 6`
- helper `getContactWrench(input, contactIndex)` that slices a 6D wrench out of `u`.

### 1.2 Centroidal dynamics

Centroidal momentum dynamics about the CoM:

$$
  \dot h_{\mathrm{com}} =
  \sum_{i=1}^{N_c}
  \begin{bmatrix}
    f_i \\
    (r_i - c) \times f_i + \tau_i
  \end{bmatrix}
  +
  \begin{bmatrix}
    m g \\
    0
  \end{bmatrix},
$$

where

- $c$ is CoM position,
- $r_i$ is the contact position of foot $i$,
- $W_i = [f_i;\, \tau_i]$ comes directly from the MPC input.
- Defined in [`CentroidalDynamicsAD::computeFlowMap()`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/dynamics/CentroidalDynamicsAD.cpp).

The relationship between centroidal momentum and generalized velocities is

$$
  h_{\mathrm{com}} = 
  A(q) 
  \begin{bmatrix} 
    \dot q_b \\ 
    \dot q_j 
  \end{bmatrix} = 
  A_b(q)\dot q_b 
  + 
  A_j(q)\dot q_j,
$$

so

$$
  \dot q_b = A_b^{-1}(q)\big(h_{\mathrm{com}} - A_j(q)\dot q_j\big),
  \qquad
  \dot q_j = v_j
$$

(with $v_j$ taken from the input).

- Defined in functions
  - `PinocchioCentroidalDynamics::getValue()` in [`PinocchioCentroidalDynamics.cpp`](../../lib/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/src/PinocchioCentroidalDynamics.cpp), and
  - `PinocchioCentroidalDynamicsAD::getValue()` in [`PinocchioCentroidalDynamicsAD.cpp`](../../lib/ocs2_ros2/ocs2_pinocchio/ocs2_centroidal_model/src/PinocchioCentroidalDynamicsAD.cpp),
  - which built the normalized centroidal momentum rate and joint velocities and then send to [`CentroidalDynamicsAD::computeFlowMap()`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/dynamics/CentroidalDynamicsAD.cpp) as the control input.

These relations define

$$
  \dot x = f(x,u)
$$

used by OCS2.

### 1.3 Contact and swing-foot constraints

**Stance feet**: pose wrt ground and twist must satisfy

$$
  f_{\mathrm{stance}}(x,u,t) =
  A_x \, \mathrm{pose}_{\mathrm{foot}}(q)
  +
  A_v \, \mathrm{twist}_{\mathrm{foot}}(q,\dot q)
  = 0,
$$

so the stance feet have zero twist and fixed orientation w.r.t. the ground plane.

**Swing feet**: normal velocity follows a swing profile $ \dot z_{\mathrm{swing}}(t) $ from the planner:

$$
  f_{\mathrm{swing}}(x,u,t) =
  n^\top v_{\mathrm{foot}}(q,\dot q) - \dot z_{\mathrm{swing}}(t) = 0,
$$

with $n$ the ground normal.

Together with friction cones, joint-velocity bounds, etc., these form the NMPC constraints.

**Code implementions:**

- **Stance** feet (zero twist/orientation fixed):
  - [`ZeroVelocityConstraintCppAd + EndEffectorKinematicsTwistConstraint`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/constraint/ZeroVelocityConstraintCppAd.cpp) for stance feet.
  - This wraps [`EndEffectorKinematicsTwistConstraint`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/constraint/ZeroVelocityConstraintCppAd.cpp), updates the ground-height offset via [`HumanoidPreComputation`](../../humanoid_nmpc/humanoid_common_mpc/src/HumanoidPreComputation.cpp), and is active when the **contact** flag is **true**.

- **Swing** feet (normal velocity follows planner profile):
  - [`NormalVelocityConstraintCppAd + EndEffectorKinematicsLinearVelConstraint`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/constraint/NormalVelocityConstraintCppAd.cpp) for **swing** feet.
  - This wraps [`EndEffectorKinematicsLinearVelConstraint`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/constraint/NormalVelocityConstraintCppAd.cpp) with a 1D normal-velocity config from [`HumanoidPreComputation`](../../humanoid_nmpc/humanoid_common_mpc/src/HumanoidPreComputation.cpp), and is active when the **contact** flag is **false**.

- Configuration/planning:
  - Swing profiles and constraint configs are prepared in [`HumanoidPreComputation`](../../humanoid_nmpc/humanoid_common_mpc/src/HumanoidPreComputation.cpp) and [`SwingTrajectoryPlanner`](../../humanoid_nmpc/humanoid_common_mpc/src/swing_foot_planner/SwingTrajectoryPlanner.cpp). These feed the normal-velocity constraints for swing feet.
  - The constraints are wired into the problem in [`CentroidalMpcInterface.cpp:212-213`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp) (`getStanceFootConstraint()` and `getNormalVelocityConstraint`).

---

## 2. Torque-related cost in centroidal MPC (GRF shaping)

Centroidal MPC (implicitly) **biases the GRFs** using a torque-related cost.

### 2.1 Configuration in task file

In [`g1_centroidal_mpc/config/mpc/task.info`](../../robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task.info):

```ini
left_leg_torque_cost
{
  activeJointNames
  {
    [0] "left_hip_pitch_joint"
    [1] "left_hip_roll_joint"
    [2] "left_hip_yaw_joint"
    [3] "left_knee_joint"
    [4] "left_ankle_pitch_joint"
    [5] "left_ankle_roll_joint"
  }

  weights {
    scaling 1e-4

    (0,0)  2.0     ; 
    (1,0)  2.0     ; 
    (2,0)  1.0     ; 
    (3,0)  8.0     ; 
    (4,0)  0.2     ; 
    (5,0)  0.2     ; 
  }
}

right_leg_torque_cost { ... }
```

### 2.2 Factory and cost class

In [`HumanoidCostConstraintFactory.cpp`](../../humanoid_nmpc/humanoid_common_mpc/src/HumanoidCostConstraintFactory.cpp):

```cpp
// Cost is: HumanoidCostConstraintFactory::getExternalTorqueQuadraticCost()
std::make_unique<ExternalTorqueQuadraticCostAD>(
  contactPointIndex, config, 
  *referenceManagerPtr_, *pinocchioInterfacePtr_,
  *mpcRobotModelADPtr_, modelSettings_);
```

[`ExternalTorqueQuadraticCostAD::costVectorFunction()`](../../humanoid_nmpc/humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp):

- uses an AD Pinocchio interface and the AD robot model to map $(x,u)$ to an approximate vector of **leg joint torques** due to the contact wrenches via Jacobian mapping,
  $$
    \tau_{\text{ext}} = J_\text{ee}^\top W_{\text{contact}},
  $$

- builds a Gauss–Newton quadratic cost of the form

  $$
    \ell_{\mathrm{torque}}(x,u)
    = \big\| W_\tau^{1/2}\, \tau_{\mathrm{ext}}(x,u) \big\|^2,
  $$

- which are implemented in [`ExternalTorqueQuadraticCostAD.cpp:110-135`](../../humanoid_nmpc/humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp), and are automatically handled by [`StateInputCostGaussNewtonAd()`](../../humanoid_nmpc/humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp).

This cost discourages GRF patterns that would generate large leg torques according to this approximate mapping.

> *Important: this is a **soft cost**, not a hard box constraint. It shapes MPC’s choice of wrenches but does not guarantee hard torque limits.*

---

## 3. MPC output at control time $t_k$

OCS2 solves the OCP and returns trajectories
$$
  \{x^*(t_k+i\Delta t), u^*(t_k+i\Delta t)\}.
$$

In [`CentroidalMpcMrtJointController.cpp`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/mrt/CentroidalMpcMrtJointController.cpp), we evaluate the policy at the current time and take the first sample:

```cpp
mpcMrtInterface_.evaluatePolicy(t, currentMpcObservation_, mpcPolicyState, mpcPolicyInput);

// Desired joints from MPC
vector_t mpc_q_j_des  = mpcRobotModelPtr_->getJointAngles(mpcPolicyState);
vector_t mpc_qd_j_des = mpcRobotModelPtr_->getJointVelocities(mpcPolicyState, mpcPolicyInput);

// Desired contact wrenches from MPC
std::array<vector6_t, 2> footWrenches{
    mpcRobotModelPtr_->getContactWrench(mpcPolicyInput, 0), // W_L*
    mpcRobotModelPtr_->getContactWrench(mpcPolicyInput, 1)  // W_R*
};
```

At time $t_k$ you therefore have:

- joint references $q_{j,\mathrm{des}}^*(t_k)$, $\dot q_{j,\mathrm{des}}^*(t_k)$,
- contact wrenches $W_L^*(t_k)$, $W_R^*(t_k)$.

> *Caution: original author left a comment in [`CentroidalMpcMrtJointController.cpp:158`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/mrt/CentroidalMpcMrtJointController.cpp): `// TODO something seems wrong with the inverse dynamics. You should correct that.`. May need to fix this or tackle it carefully.*

---

## 4. Inner-loop: joint-space PD → whole-body inverse dynamics

### 4.1 Joint-space PD to get desired accelerations

Current joint state ([`CentroidalMpcMrtJointController.cpp:161-162`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/mrt/CentroidalMpcMrtJointController.cpp)):

```cpp
vector_t q_j = mpcRobotModelPtr_->getJointAngles(currentMpcObservation_.state);
vector_t qd_j = mpcRobotModelPtr_->getJointVelocities(currentMpcObservation_.state, currentMpcObservation_.input);
```

Desired joint acceleration ([`CentroidalMpcMrtJointController.cpp:163`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/mrt/CentroidalMpcMrtJointController.cpp)):

```cpp
vector_t qdd_j_des = inverse_dynamics_kp_ * (mpc_q_j_des - q_j) +
                     inverse_dynamics_kd_ * (mpc_qd_j_des - qd_j);
```

Mathematically it is

$$
  \ddot q_{j,\mathrm{des}} =
  K_p (q_{j,\mathrm{des}}^* - q_j) + 
  K_d (\dot q_{j,\mathrm{des}}^* - \dot q_j).
$$

If inverse dynamics enforces $\ddot q_j \approx \ddot q_{j,\mathrm{des}}$, then the joint error dynamics approximately satisfy

$$
  \ddot e + K_d \dot e + K_p e \approx 0,
  \qquad
  e = q_j - q_{j,\mathrm{des}}^*.
$$

### 4.2 Whole-body inverse dynamics (Pinocchio)

We also fetch the full generalized coordinates and velocities ([`CentroidalMpcMrtJointController.cpp:168-169`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/mrt/CentroidalMpcMrtJointController.cpp)):

```cpp
vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(currentMpcObservation_.state);
vector_t qd = mpcRobotModelPtr_->getGeneralizedVelocities(currentMpcObservation_.state, currentMpcObservation_.input);
```

Then call ([`CentroidalMpcMrtJointController.cpp:175`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/mrt/CentroidalMpcMrtJointController.cpp)):

```cpp
vector_t mpcJointTorques = computeJointTorques<scalar_t>(q, qd, qdd_j_des, footWrenches, pinocchioInterface_);
```

Inside [`computeJointTorques()`](../../humanoid_nmpc/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp):

1. **Mass matrix and nonlinear effects** via CRBA + RNEA-style calls ([`DynamicsHelperFunctions.cpp:241-242`](../../humanoid_nmpc/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp)):

   ```cpp
   pinocchio::crba(model, data, q);              // M(q)
   pinocchio::nonLinearEffects(model, data, q, qd);  // h(q, qd)
   ```

   giving $M(q)$ and $h(q,\dot q)$.

2. **Foot Jacobians** ([`DynamicsHelperFunctions.cpp:250-253`](../../humanoid_nmpc/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp)):

   ```cpp
   computeFrameJacobian(..., "left_foot_l_contact",  J_foot_l);
   computeFrameJacobian(..., "right_foot_r_contact", J_foot_r);
   ```

3. **Generalized external forces** from MPC wrenches ([`DynamicsHelperFunctions.cpp:257`](../../humanoid_nmpc/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp)):

   ```cpp
   VECTOR_T externalForcesInJointSpace =
       J_foot_l.transpose() * footWrenches[0] +
       J_foot_r.transpose() * footWrenches[1];
   ```

   *i.e.*,

   $$
    Q_{\mathrm{ext}} = J_{LF}^\top W_L^* + J_{RF}^\top W_R^*.
   $$

4. **Generalized acceleration** ([`DynamicsHelperFunctions.cpp:261-262`](../../humanoid_nmpc/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp)):

   A helper computes the floating-base acceleration; then:

   ```cpp
   VECTOR_T q_dd(qd.size());
   q_dd << baseAccelerations, qdd_j_des;
   ```

   so

   $$
    \ddot q =
    \begin{bmatrix}
      \ddot q_{\mathrm{base}} \\
      \ddot q_{j,\mathrm{des}}
    \end{bmatrix}.
   $$

5. **Joint torques** from rigid-body dynamics ([`DynamicsHelperFunctions.cpp:263-265`](../../humanoid_nmpc/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp)):

   The floating-base dynamics are

   $$
    M(q)\ddot q + h(q,\dot q) = S^\top \tau + Q_{\mathrm{ext}},
   $$

   with $S$ the actuator selection matrix. Taking the joint rows:

   ```cpp
   size_t n_joints = qdd_j_des.size();
   VECTOR_T jointTorques =
       data.M.bottomRows(n_joints) * q_dd +
       data.nle.tail(n_joints) -
       externalForcesInJointSpace.tail(n_joints);
   ```

   *i.e.*,

   $$
    \tau_{\mathrm{joints}} =
    M_{jj}(q)\ddot q_j + 
    M_{jb}(q)\ddot q_{\mathrm{base}} + 
    h_j(q,\dot q) - 
    Q_{\mathrm{ext},j}.
   $$

So the inner-loop realizes the mapping

$$
  \tau =
  M(q)\ddot q_{\mathrm{des}} + h(q,\dot q) - J_c^\top W_c^*,
$$

with $\ddot q_{\mathrm{des}} = [\ddot q_{\mathrm{base}};\ddot q_{j,\mathrm{des}}]$ and $W_c^* = [W_L^*;W_R^*]$ from the outer-loop centroidal MPC.

---

## 5. Torque approximation in MPC vs. actual inverse-dynamics torques

- At the **MPC-level**, [`ExternalTorqueQuadraticCostAD`](../../humanoid_nmpc/humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp) uses a Jacobian mapping to estimate leg torques $\tau_{\mathrm{ext}}(x,u)$ from the current state and contact wrenches, and penalizes them with a quadratic cost.
- This **shapes** the GRFs (wrenches $W_L, W_R$) to be more torque-friendly, but is still an approximate, reduced-order model and a *soft* constraint.

- At the **inverse-dynamics level**, [`computeJointTorques()`](../../humanoid_nmpc/humanoid_centroidal_mpc/src/mrt/CentroidalMpcMrtJointController.cpp) uses the full floating-base (whole-body) rigid-body dynamics

  $$
    M(q)\ddot q + h(q,\dot q) = S^\top \tau + J_c^\top W_c^*
  $$

  to compute actual torques $\tau$. These can **differ** from the torques implied by the MPC cost, and can exceed hardware limits if not further constrained.

In practice:

- MPC torque cost = guidance to keep GRFs in a torque-feasible region.
- Hard torque limits must still be enforced in:
  - a QP-style whole-body controller **(QP-WBC)** with box constraints on $\tau$, or
  - the hardware driver (saturation / current limits).

---

## 6. Overall pipeline

1. **Centroidal NMPC (outer-loop)**
   - State: $x = (h_{\mathrm{com}}, q_b, q_j)$
   - Input: $u = (W_L, W_R, \dot q_j)$
   - System dynamics: centroidal dynamics + whole-body kinematics (CD+WBK)
   - Constraints: stance/swing kinematics, friction cones, joint-vel bounds
   - Costs: tracking (CoM/base/feet), wrench regularization, **torque-related cost** via [`ExternalTorqueQuadraticCostAD`](../../humanoid_nmpc/humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp)

2. **MPC output at $t_k$**
   - Joint references $q_{j,\mathrm{des}}^*, \dot q_{j,\mathrm{des}}^*$
   - Contact wrenches $W_L^*, W_R^*$

3. **WBC via inverse dynamics (inner-loop)**
   - Joint-space PD → $\ddot q_{j,\mathrm{des}}$
   - Whole-body inverse dynamics (CRBA + nonLinearEffects + $J_c^\top W_c^*$) → joint torques $\tau$

4. **Execution**
   - Send $\tau$ to hardware / low-level controller
   - Optionally add QP torque constraints or saturation for strict safety.

This is the implementation path for **MPC-WBC** in [`humanoid_centroidal_mpc`](../../humanoid_nmpc/humanoid_centroidal_mpc/), including how torque costs in MPC influence GRFs and how actual torques are computed in the inner-loop.

---

## 7. Supplementary

- [The Use of the *True* Center of Mass in Centroidal MPC](./supplement_true_com_computation.md)
- [Pattern Types for CoM/Torso/Feet Reference Generation](./supplement_pattern_types.md)