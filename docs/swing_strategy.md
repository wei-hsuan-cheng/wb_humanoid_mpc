# Swing Strategy: Feet and Arms (Unitree G1)

This note summarizes how swing motions are generated in the humanoid MPC stack for Unitree G1, both for the **feet** (swing‑foot trajectories) and for the **arms** (arm‑swing reference), with parameter meanings and tuning tips.

It applies to both:

- Centroidal MPC: `robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task.info`
- Whole‑body MPC: `robot_models/unitree_g1/g1_wb_mpc/config/mpc/task.info`

Core code:

- Swing‑foot planner:
  - Header: `humanoid_nmpc/humanoid_common_mpc/include/humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h`
  - Implementation: `humanoid_nmpc/humanoid_common_mpc/src/swing_foot_planner/SwingTrajectoryPlanner.cpp`
- Integration into MPC:
  - Centroidal: `humanoid_nmpc/humanoid_centroidal_mpc/src/CentroidalMpcInterface.cpp`
  - Whole‑body: `humanoid_nmpc/humanoid_wb_mpc/src/WBMpcInterface.cpp`
- Arm swing reference:
  - `humanoid_nmpc/humanoid_common_mpc/src/reference_manager/SwitchedModelReferenceManager.cpp`
  - `humanoid_nmpc/humanoid_common_mpc/src/common/ModelSettings.cpp`

---

## Foot Swing Trajectory

### High‑Level Behavior

For each foot, the swing‑foot planner constructs a vertical trajectory over time, based on:

- The **mode schedule** (contact sequence of stance/swing phases).
- The terrain height at lift‑off and touchdown.
- The swing parameters from the `swing_trajectory_config` block in the MPC `task.info`.

The trajectory is represented as a sequence of cubic splines in time, each parameterized by:

- Lift‑off node: time, height, and vertical velocity at lift‑off.
- Mid‑swing height: peak height of the swing.
- Touchdown node: time, height, and vertical velocity at touchdown.

The planner also builds an **impact‑proximity factor** trajectory per foot—a scalar in `[0, 1]` that is 1 in stance and varies during swing. This factor is used as a scalar multiplier in some costs (e.g. to taper external torque penalties near impact).

### Configuration Parameters

Defined in `swing_trajectory_config` of each `task.info`, loaded into `SwingTrajectoryPlanner::Config`:

- `liftOffVelocity`
  - Desired vertical velocity at lift‑off (foot leaving ground).
  - Units: m/s.
  - Positive values make the foot leave the ground faster; larger magnitudes → more “snappy” lift‑off.

- `touchDownVelocity`
  - Desired vertical velocity just before touchdown.
  - Usually small negative (foot moving downwards slowly).
  - More negative → more aggressive impact; closer to zero → softer landing.

- `swingHeight`
  - Nominal vertical amplitude of the swing.
  - Sets how high the foot arcs above the terrain.
  - Larger values increase clearance but also cost (and may stress kinematic limits).

- `touchDownHeightOffset`
  - Vertical offset applied at touchdown relative to terrain height.
  - Slightly negative values (e.g. `-0.001`) encourage a gentle “settle” into the ground.

- `swingTimeScale`
  - Time threshold used to scale height and velocities when swing phases are short.
  - If swing duration `T_swing` is shorter than this threshold, the planner scales down the peak height and velocities to avoid overly aggressive motions.

- `impactProximityFactorLiftOffVelocity ≤ 0`
  - Velocity term for the **impact‑proximity factor** at lift‑off.
  - Negative values mean the proximity factor decreases slightly after lift‑off (moving away from impact).

- `impactProximityFactorTouchDownVelocity ≥ 0`
  - Velocity term for the impact‑proximity factor at touchdown.
  - Positive values mean the proximity factor increases as we approach impact.

- `impactProximityFactorMidPointValue ∈ [0, 1]`
  - Value of the impact‑proximity factor at mid‑swing.
  - Shapes the “valley” of the factor during swing:
    - Close to 0 → strong reduction of impact‑weighted terms in the middle of swing.
    - Closer to 1 → keep those terms active even mid‑swing.

Centroidal vs whole‑body:

- The only difference between the two configs is `impactProximityFactorMidPointValue`, which slightly changes how strongly impact‑weighted costs are suppressed mid‑swing.

### Trajectory Construction (Feet)

Core code: `humanoid_nmpc/humanoid_common_mpc/src/swing_foot_planner/SwingTrajectoryPlanner.cpp`.

Given a `ModeSchedule` with:

- `modeSequence` (contact mode IDs),
- `eventTimes` (transition times),

the planner:

1. Converts each mode ID into a **stance pattern** (which feet are in contact) using `modeNumber2StanceLeg`.
2. For each foot, extracts a boolean array `[numPhases]` indicating stance (`true`) vs swing (`false`):  
   see `SwingTrajectoryPlanner::extractContactFlags`.
3. For each phase index where the leg is swinging, locates:
   - The lift‑off time index (last stance before swing).
   - The touchdown time index (first stance after swing).  
   This is done by `updateFootSchedule` and `findIndex`.

For a single swing phase for leg `j`, with:

- Lift‑off time `t_L` and height `h_L` (from `liftOffHeightSequence`).
- Touchdown time `t_T` and height `h_T` (from `touchDownHeightSequence`).

the planner builds a cubic spline in time for height `z_j(t)`:

- Lift‑off node:

  \[
    z_j(t_L) = h_L,\quad \dot{z}_j(t_L) = s \cdot v_\text{lift}
  \]

- Mid‑swing height:

  \[
    z_\text{mid} = \min(h_L, h_T) + s \cdot h_\text{swing}
  \]

  where `s` is a scaling factor from `swingTrajectoryScaling(t_L, t_T, swingTimeScale)`.

- Touchdown node:

  \[
    z_j(t_T) = h_T,\quad \dot{z}_j(t_T) = s \cdot v_\text{touch}
  \]

Here:

- \( v_\text{lift} = \text{liftOffVelocity} \)
- \( v_\text{touch} = \text{touchDownVelocity} \)
- \( h_\text{swing} = \text{swingHeight} \)

In code (simplified), for the “swing only in current mode” case:

- Height spline nodes (see `SwingTrajectoryPlanner.cpp`, around the first `if (!eesContactFlagStocks[j][p])` block):
  - `CubicSpline::Node liftOffHeight{t_L, h_L, s * liftOffVelocity};`
  - `CubicSpline::Node touchDownHeight{t_T, h_T, s * touchDownVelocity};`
  - `midHeight = min(h_L, h_T) + s * swingHeight;`
  - `feetHeightTrajectories_[j].emplace_back(liftOffHeight, midHeight, touchDownHeight);`

### Impact Proximity Factor

In parallel, the planner builds an impact‑proximity spline `p_j(t)`:

- For a pure swing phase:

  \[
    p_j(t_L) = 1,\quad \dot{p}_j(t_L) = s \cdot \alpha_L
  \]
  \[
    p_j(t_T) = 1,\quad \dot{p}_j(t_T) = s \cdot \alpha_T
  \]

  with mid‑point:

  \[
    p_\text{mid} = \text{impactProximityFactorMidPointValue}
  \]

  where:

  - \(\alpha_L = \text{impactProximityFactorLiftOffVelocity} \le 0\),
  - \(\alpha_T = \text{impactProximityFactorTouchDownVelocity} \ge 0\).

- For various edge cases (start/end of swing, long swing):
  - The start and end values or their derivatives are adjusted as in the implementation (see the four swing cases in `SwingTrajectoryPlanner.cpp:140-196`).

During stance:

  \[
    p_j(t) \equiv 1
  \]

Usage:

- Costs can scale certain terms by `p_j(t)` so that:
  - They are fully active in stance.
  - Reduced in mid‑swing, where interaction w.r.t. ground is low.

Example: `humanoid_nmpc/humanoid_common_mpc/src/cost/ExternalTorqueQuadraticCostAD.cpp` uses `getImpactProximityFactor` to weight an external torque cost.

### Tuning Foot Swing Parameters

Practical effects when tuning `swing_trajectory_config`:

- `liftOffVelocity`
  - Increase to make the foot “snap” off the ground faster.
  - Too high can lead to large accelerations and potential chattering in MuJoCo.

- `touchDownVelocity`
  - Make more negative for firmer touchdown; closer to zero for soft landing.
  - On uneven terrain or with noisy contact detection, softer landings are safer.

- `swingHeight`
  - Increase for more clearance, especially if the robot catches toes.
  - May need to increase joint velocity/acceleration weights or lower joint limits to avoid unrealistic motion.

- `touchDownHeightOffset`
  - Slight negative offset helps ensure contact is established even if model/terrain estimate is slightly off.
  - Large offsets can cause foot penetration or unrealistic impulses.

- `swingTimeScale`
  - If you shorten swing phases in the gait schedule, large `swingHeight` and `liftOffVelocity` can become too aggressive.
  - This parameter automatically scales down height and velocities for short swing windows; adjust if swing feels too timid or too harsh in short steps.

- `impactProximityFactorLiftOffVelocity`, `impactProximityFactorTouchDownVelocity`, `impactProximityFactorMidPointValue`
  - Control how strongly “impact‑related” costs (e.g. external torques, impact shaping) are attenuated mid‑swing.
  - Lower mid‑point value → strongly relax these costs in mid‑swing.
  - Larger mid‑point value → keep them active, leading to more conservative motions.

For G1, the difference between centroidal and whole‑body configs is primarily this mid‑swing factor; WB MPC tends to prefer slightly different shaping due to the richer joint‑space dynamics.

---

## Arm Swing Strategy

Arm swing is handled by the reference manager, not by a dedicated trajectory planner. The goal is to create a simple, velocity‑dependent arm swing synchronized with the gait phase.

### Configuration (Arm Joints)

In both `task.info` files, arm joint names are specified under:

```text
model_settings
{
  ...
  armJointNames {
    left_shoulder_y       left_shoulder_pitch_joint
    right_shoulder_y      right_shoulder_pitch_joint
    left_elbow_y          left_elbow_joint
    right_elbow_y         right_elbow_joint
  }
}
```

These are read in `humanoid_nmpc/humanoid_common_mpc/src/common/ModelSettings.cpp`:

- The names are stored and mapped to joint indices:
  - `j_l_shoulder_y_index`, `j_r_shoulder_y_index`,
  - `j_l_elbow_y_index`, `j_r_elbow_y_index`.

These indices are used when modifying the desired joint angles to include arm swing.

### Arm Swing in ReferenceManager

Core code: `humanoid_nmpc/humanoid_common_mpc/src/reference_manager/SwitchedModelReferenceManager.cpp`, function

```cpp
vector_t SwitchedModelReferenceManager::getDesiredState(
    const TargetTrajectories& targetTrajectories,
    const vector_t& state,
    scalar_t time) const;
```

Arm swing logic (simplified):

1. Start from the nominal state reference:

   \[
   x_\text{nom}(t) = \text{targetTrajectories.getDesiredState}(t)
   \]

2. If arm swing is active (`armSwingReferenceActive_` set to `true` by WB and centroidal interfaces):

   - Compute a **phase variable** for the gait:

     \[
       \phi(t) = \text{getPhaseVariable}(t)  \in [0,1) \quad \text{(cyclic)}
     \]

   - Extract current joint angles from the nominal state:

     \[
       q_j^\text{nom} = \text{mpcRobotModel.getJointAngles}(x_\text{nom})
     \]

   - Compute base COM linear velocity and orientation:

     \[
       v_\text{COM} = \text{mpcRobotModel.getBaseComLinearVelocity}(x_\text{nom})
     \]
     \[
       \theta_z = \text{yaw angle from getBasePose(state)}
     \]

   - Compute **forward velocity in base frame**:

     \[
       v_x^\text{local} = \cos(\theta_z) v_x + \sin(\theta_z) v_y
     \]

   - Define a **gait cycle factor**:

     \[
       g(t) = \sin\big(2\pi(\phi(t) - 0.15)\big)\, v_x^\text{local}
     \]

   - Modify arm joints (using indices from `ModelSettings`):

     \[
       q^\text{des}_{L\_shoulder\_y} \mathrel{+}= -0.15\, g(t)
     \]
     \[
       q^\text{des}_{R\_shoulder\_y} \mathrel{+}= +0.15\, g(t)
     \]
     \[
       q^\text{des}_{L\_elbow\_y}    \mathrel{+}= -0.15\, g(t)
     \]
     \[
       q^\text{des}_{R\_elbow\_y}    \mathrel{+}= +0.15\, g(t)
     \]

3. The modified joint angles are written back into `x_nominal`, and this becomes the **desired** state used by costs and constraints.

Effect:

- Arms swing out of phase with each other (right arm forward when left leg forward, and vice versa).
- Swing amplitude scales with forward walking velocity; stationary robot → minimal arm swing.
- Phase offset `0.15` aligns arm swing timing with the gait phase.

### Tuning Arm Swing

The main knobs are inside `SwitchedModelReferenceManager::getDesiredState`:

- Phase shift inside `sin(2π(φ − 0.15))`
  - Adjust to make arms swing “earlier” or “later” relative to leg motion.
  - Changing 0.15 to 0.25, for example, would delay arm swing relative to the gait phase.

- Multipliers on `g(t)` (currently `±0.15`)
  - Increase magnitudes for bigger arm swing amplitudes.
  - Decrease for more subtle motion.

- Dependence on `v_x^local`
  - Arm swing amplitude is proportional to forward speed.
  - If you want arm swing at low speeds, you could add a bias term or use a saturating function of speed instead of pure linear scaling.

Because arm swing is encoded directly into the **reference** state, it doesn’t change the plant dynamics or constraints; it just biases the cost toward arm motions that match the desired periodic pattern.

---

This file should serve as a starting point when adjusting swing behavior for feet and arms, or when porting this strategy to a different humanoid model. 

