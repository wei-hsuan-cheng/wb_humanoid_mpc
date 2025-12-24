# Supplementary: The Use of the *True* Center of Mass in Centroidal MPC

> *This page is a supplement of [humanoid_centroidal_mpc.md](./humanoid_centroidal_mpc.md).*

This section clarifies how the **true whole–body center of mass (CoM)** is used in a centroidal MPC formulation, and how it co–exists with tracking tasks defined on the **torso/base frame** and the limbs (feet, hands).

---

## 1. Notation

- $q = [q_b; q_j]$: generalized configuration
  - $q_b$: floating–base pose of the **torso** (position + orientation in world)
  - $q_j$: actuated joint angles

- $\dot q = [\dot q_b; \dot q_j]$: generalized velocities

- $c(q)$: **true whole–body CoM position** in world, computed from all link masses **(arms, legs, head, etc.)** and their current poses.

- $h_G(q,\dot q) \in \mathbb{R}^6$: **centroidal momentum** about the instantaneous CoM, expressed in a chosen world–aligned frame:
  $$
    h_G =
    \begin{bmatrix}
      k_G \\ l_G
    \end{bmatrix},
  $$

  with angular momentum $k_G$ and linear momentum $l_G$.

- $A_G(q) \in \mathbb{R}^{6\times(6+n)}$: **centroidal momentum matrix (CMM)**, such that
  $$
    h_G(q,\dot q) = A_G(q)\,\dot q.
  $$

---

## 2. Moving CoM vs. SRBD Approximation

In a **full multi–body centroidal model**, the CoM is *not fixed* in the torso frame:

- Each link $i$ has its own mass and inertia.
- As the limbs move, the mass distribution changes.
- The system CoM
  $$
    c(q) = \frac{1}{m}\sum_i m_i\,c_i(q)
  $$
  moves relative to the torso.

The CMM is constructed using this *instantaneous* CoM:

$$
  A_G(q) = \sum_{i=1}^N X_{Gi}^*\,I_i\,J_i(q),
$$
where

- $J_i(q)$ is the body Jacobian of link $i$,
- $I_i$ is the spatial inertia of link $i$,
- $X_{Gi}^*$ is the **wrench transform** from link frame $i$ to the CoM frame $G$.

The centroidal momentum is then

$$
  h_G(q,\dot q) = A_G(q)\,\dot q,
$$

and its time derivative obeys the clean balance equation

$$
  \dot h_G =
  \sum_{c}
  \begin{bmatrix}
    f_c \\
    (r_c - c(q))\times f_c + \tau_c
  \end{bmatrix}
  +
  \begin{bmatrix}
    m g \\ 0
  \end{bmatrix},
$$
where $f_c, \tau_c$ are the contact forces and torques applied at contact point $r_c$.

By contrast, the **single rigid body dynamics (SRBD)** approximation places all mass in the torso and treats the CoM as fixed in that body. This simplifies the model but no longer matches the exact centroidal balance. The full centroidal MPC therefore uses the **true moving CoM** for its dynamics.

---

## 3. State Used in Centroidal MPC

A typical centroidal MPC state is

$$
  x =
  \begin{bmatrix}
    h_G \\
    q_b \\
    q_j
  \end{bmatrix},
$$

where:

- $h_G$ is **always** defined about the *current* whole–body CoM.
- $q_b$ is the torso/base pose (*e.g.*, a link such as `base_link`), not the CoM pose.
- The CoM position $c(q)$ is **not a separate state variable**; it is a function of $q$ that can be evaluated when needed inside the dynamics, constraints, and costs.

The centroidal dynamics used in the MPC are then

$$
  \dot x =
  \begin{bmatrix}
    \dot h_G \\
    \dot q_b \\
    \dot q_j
  \end{bmatrix} =
  \begin{bmatrix}
    f_{\text{cent}}(q,\dot q, W_c) \\
    f_{\text{kin,base}}(q,\dot q) \\
    \dot q_j
  \end{bmatrix},
$$
where $W_c$ are the contact wrenches, and $\dot h_G$ is computed using the true CoM in the moment arm $(r_c - c(q))\times f_c$.

---

## 4. Tracking: CoM vs. Torso and Limbs

Although the dynamics are expressed around the **true CoM**, tracking tasks in MPC are defined
as *functions of the state* and can refer to *any* body frame or point:

- **CoM tracking task**:
  $$
    \ell_{\text{CoM}}(x) =
    \big\| c(q) - c_{\text{des}}(t) \big\|_{W_c}^2.
  $$
- **Torso/base pose tracking task**:
  $$
    \ell_{\text{base}}(x) =
    \big\| p_b(q_b) - p_{b,\text{des}}(t) \big\|_{W_p}^2
    +
    \big\| \text{log}\big(R_{b,\text{des}}(t)^\top R_b(q_b)\big) \big\|_{W_R}^2.
  $$
- **Swing foot tasks**:
  $$
    \ell_{\text{foot}}(x) =
    \big\| r_{\text{foot}}(q) - r_{\text{foot,des}}(t) \big\|_{W_f}^2.
  $$

Crucially:

- The **reference frame for momentum** is the CoM.
- The **reference frames for tracking** (torso, feet, hands) are simply *other functions* of
  the same state $q$.
- You do **not** need to “fix” the CoM at the torso to define a consistent reference trajectory.
  Instead, you:
  - use the true CoM in the centroidal dynamics and in CoM-related costs,
  - use the torso frame and limb frames in their own costs.

This separation is what is meant by:

> *“For tracking, you can still track torso pose, hands, feet, etc. — they’re just separate task variables.”*

---

## 5. Why a Moving CoM Does *Not* Break $h_G$ Tracking

Suppose you want to regulate the centroidal momentum to a desired trajectory $h_G^{\text{des}}(t)$. Even though the CoM position $c(q)$ varies with limb motion, this does **not** create a conceptual inconsistency:

- At each time, the definition of centroidal momentum is
  $$
    h_G(q,\dot q) = A_G(q)\,\dot q,
  $$
  with $A_G(q)$ built from the *current* CoM.
- When solving the OCP, the MPC uses this exact mapping at each state $x$.
- The cost
  $$
    \ell_{h}(x) = \big\| h_G(q,\dot q) - h_G^{\text{des}}(t) \big\|_{W_h}^2
  $$
  always compares two quantities defined about **the same instantaneous CoM**, so the problem
  is well-posed.

The fact that $c(q)$ changes over time simply means that $A_G(q)$ and $I_G(q)$ are configuration-dependent, which is exactly what the full centroidal model is designed to capture.

---

## 6. Practical Impact vs. SRBD

- In **SRBD**, the CoM is artificially tied to the torso:
  - simpler, often linearizable models (LIPM-like),
  - but internal limb motions are misrepresented.
- In **true centroidal MPC**, the CoM is computed from the URDF’s link inertias at each step:
  - $c(q)$ typically moves a few centimeters around the torso as the robot walks or swings arms,
  - but the relationship between external wrenches and momentum rate
    $\dot h_G$ remains physically exact.

The MPC then:

1. Uses the true CoM in the centroidal dynamics and contact moment arms $(r_c - c(q))$.
2. Tracks CoM, torso, and limb trajectories via separate cost terms defined on the same state.
3. Achieves higher model fidelity than SRBD without sacrificing the ability to use torso-based
   reference trajectories.

This is the intended use of the **true CoM** during centroidal MPC.
