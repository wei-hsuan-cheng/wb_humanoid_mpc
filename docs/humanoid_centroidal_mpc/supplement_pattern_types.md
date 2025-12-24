# Supplementary: Pattern Types for CoM/Torso/Feet Reference Generation

> *This page is the supplement of [humanoid_centroidal_mpc.md](./humanoid_centroidal_mpc.md).*

This note classifies three common *pattern types* for how high–level reference trajectories
(CoM, torso/base, feet) are generated and fed into a **centroidal MPC** or **whole–body MPC**
stack. It also clarifies that **arm configurations** contribute to the whole–body CoM.

---

## 1. Preliminaries

We assume a floating–base robot model with:

- State
  $$
    x =
    \begin{bmatrix}
      h_G \\
      q_b \\
      q_j
    \end{bmatrix},
  $$
  where
  - $h_G$ is centroidal momentum about the **true CoM**,
  - $q_b$ is the floating–base (torso) pose,
  - $q_j$ are joint angles.

- Contact sequence / gait schedule: stance vs swing phases of feet.
- A high–level command, e.g. desired torso velocity or pose.

We distinguish *what is generated ahead of time* and fed as **reference trajectories** into MPC:

- CoM reference $c_{\text{des}}(t)$,
- torso/base reference $(p_{b,\text{des}}(t), R_{b,\text{des}}(t))$,
- swing/stance foot references $r_{\text{foot,des}}(t)$,
- possibly desired centroidal momentum $h_G^{\text{des}}(t)$.

---

## 2. Pattern A – Full Pattern Generator (CoM + Torso + Feet)

**Idea.**  
A separate high–level **pattern generator** computes *all* of the following:

- CoM trajectory $c_{\text{des}}(t)$ (often via LIPM, SRBD, or linearized centroidal models),
- torso/pelvis trajectory $(p_{b,\text{des}}(t), R_{b,\text{des}}(t))$,
- swing foot trajectories $r_{\text{foot,des}}(t)$,
- possibly nominal centroidal momentum $h_G^{\text{des}}(t)$.

The pattern generator uses the **gait schedule** and the motion command (e.g. desired walking
velocity) to ensure that:

- CoM, torso, and feet are **mutually consistent**, e.g. CoM projection stays inside the support
  polygon, pelvis follows a smooth path over footsteps, etc.

The centroidal MPC then:

1. **Tracks** these references with soft costs:
   $$
     \ell_{\text{CoM}}(x) = \|c(q) - c_{\text{des}}(t)\|^2_{W_c},\quad
     \ell_{\text{base}}(x) = \|p_b(q_b) - p_{b,\text{des}}(t)\|^2_{W_p} + \dots
   $$
2. Enforces dynamic feasibility:
   - centroidal dynamics,
   - friction cones, contact constraints,
   - joint velocity/torque limits (soft or hard).

**Pros:**

- Very stable if the pattern generator model is good and modeling errors are moderate.
- CoM, torso, and feet are jointly planned ahead of time.

**Cons:**

- Requires maintaining a more complex pattern generator.
- May be less flexible for highly dynamic or unstructured teleoperation, where the operator
  directly commands base/hand motions.

---

## 3. Pattern B – Torso–Driven CoM (CoM as Offset from Torso)

**Idea.**  
High–level commands (teleop GUI, joystick, etc.) primarily specify **torso/base motion**. The CoM
trajectory is then generated from the torso reference by a simple kinematic rule, for example:

$$
  c_{\text{des}}(t) = p_{b,\text{des}}(t) + R_{b,\text{des}}(t)\,d_0,
$$

where $d_0$ is a nominal torso→CoM offset measured from a standard standing pose.

Feet trajectories $r_{\text{foot,des}}(t)$ are generated via a gait module based on the same base
motion and schedule, but the CoM is not separately planned by a full dynamic model; it is tied to
the torso with a simple offset.

The centroidal MPC then:

- Tracks both torso and CoM references:
  $$
    \ell_{\text{CoM}}(x) = \|c(q) - c_{\text{des}}(t)\|^2_{W_c},\quad
    \ell_{\text{base}}(x) = \dots
  $$
- Refines contact forces and momentum to make this feasible.

**Pros:**

- Simpler to implement than a full pattern generator (Pattern A).
- Ensures CoM stays roughly “under the torso” in a consistent way.

**Cons:**

- Still only approximate: it ignores detailed limb mass distribution.
- Can be problematic for large arm motions or when carrying heavy loads, where the true CoM
  deviates significantly from a fixed offset.

---

## 4. Pattern C – Implicit CoM (Torso + Feet Only, CoM Emerges)

**Idea.**  
The high–level module generates **torso/base** and **foot** references, plus possibly a nominal
joint posture, but **does not explicitly generate a CoM trajectory**.

In this case:

- The MPC cost includes:
  - torso/base tracking,
  - foot/swing tracking,
  - posture regularization,
  - and **momentum regularization** (e.g. penalizing large $h_G$ or its components),
- but **no explicit term** of the form $\|c(q) - c_{\text{des}}(t)\|^2$.

The CoM trajectory is then **implicit**:

- it is whatever emerges from the commanded torso + feet motion,
- constrained by centroidal dynamics, contact constraints, and momentum penalties.

**Pros:**

- Very flexible for teleoperation or loco–manipulation:
  - the operator commands high–level torso/hand motions,
  - MPC figures out the compatible CoM, GRFs, and joint motion.
- Less brittle to mis–specified or overly constrained CoM references.

**Cons:**

- No explicit guarantee that CoM will follow a particular geometric path.
- Stability relies more on:
  - friction constraints,
  - momentum regularization,
  - and the design of base/feet tasks.

**Classification of the teleop GUI in the described humanoid centroidal MPC:**

- The **procedural motion manager + gait** generate base and feet references:
  - that part looks like a pattern–generator approach.
- **However, CoM is not given its own reference trajectory**:
  - CoM is computed from $q_b, q_j$ and appears only via dynamics and momentum costs.
- Therefore, *with respect to CoM handling*, this implementation is **Pattern C**.

---

## 5. Arms and Their Effect on CoM

Yes: **arm configuration is fully taken into account in the true centroidal model**.

- The whole–body CoM is defined as
  $$
    c(q) = \frac{1}{m}\sum_i m_i\,c_i(q),
  $$
  where the sum is over **all links**:
  - legs,
  - torso,
  - head,
  - arms,
  - any payloads attached to the robot.

- Each arm link contributes:
  - its mass $m_i$,
  - its own CoM position $c_i(q)$,
  which depends on joint angles in the arms.

Consequences:

1. **Arm motion shifts the whole–body CoM.**
   - Swinging an arm forward or sideways moves the CoM a few centimeters relative to the torso.
   - This effect is captured automatically when you compute CoM from the URDF inertias.

2. **Arms therefore influence centroidal dynamics.**
   - Centroidal momentum $h_G = A_G(q)\dot q$ includes contributions from arm velocities.
   - Changing arm pose changes $A_G(q)$, and thus how joint velocities map to momentum.

3. **Patterns A/B/C can all exploit this:**
   - Pattern A: a sophisticated pattern generator *can* plan CoM and arm motion jointly.
   - Pattern B: if your “CoM = torso + offset” approximation ignores arm mass, it becomes less
     accurate for large arm motions.
   - Pattern C: arms can be used as an internal degree of freedom to adjust balance (e.g. arm
     flailing or counter–swing) without a prescribed CoM trajectory.

In summary:

- In a **true centroidal MPC** implementation using the URDF inertias, *all* links—including arms—
  contribute to CoM and centroidal momentum.
- Arm configurations are not “just for legs”; they are part of the dynamical system that determines
  CoM, momentum, and feasible GRFs.

---

## 6. Summary of Pattern Types

- **Pattern A – Full pattern generator:**
  - Explicitly generates CoM, torso, and foot trajectories,
  - MPC refines them under full dynamics and constraints,
  - Very stable when the pattern generator model is accurate.

- **Pattern B – Torso–driven CoM:**
  - High–level command defines torso; CoM derived from torso via a fixed offset or simple rule,
  - Feet generated by a gait module,
  - MPC tracks both torso and CoM with less complexity than a full pattern generator.

- **Pattern C – Implicit CoM:**
  - High–level module specifies torso and foot trajectories only,
  - CoM is implicit, shaped by centroidal dynamics and momentum costs,
  - Well–suited to teleoperation and loco–manipulation, where explicit CoM paths are not required.

In all three patterns, **arms** are part of the multi–body model and affect CoM and centroidal
momentum whenever a *true* centroidal model (based on the URDF inertias) is used.
