# MPC Time Parameter Tuning (Unitree G1)

This note summarizes how the main time‑related parameters are set and tuned for the G1 centroidal and whole‑body MPC controllers, and how they relate to MuJoCo simulation.

Relevant config files:

- Centroidal MPC: `robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task.info`
- Whole‑body MPC: `robot_models/unitree_g1/g1_wb_mpc/config/mpc/task.info`

## Key Time Parameters

Both task files define the same key quantities:

- `multiple_shooting.dt` – grid spacing of the OCP (SQP shooting nodes).
- `mpc.timeHorizon` – horizon length of the optimal control problem.
- `rollout.timeStep` – integration step for the continuous‑time rollout.
- `mpc.mpcDesiredFrequency` – how often the MPC solve is run (outer loop).
- `mpc.mrtDesiredFrequency` – frequency of the MRT loop that evaluates the latest policy.

There is no hard requirement that all of these be exactly divisible, but their ratios strongly affect numerical cost and closed‑loop stability.

## Centroidal MPC Settings

From `g1_centroidal_mpc/config/mpc/task.info`:

- `timeHorizon = 1.2 s`
- `multiple_shooting.dt = 0.02 s` → about 60 grid nodes.
- `rollout.timeStep = 0.015 s`
- `mpcDesiredFrequency = 100 Hz` → 0.01 s between MPC calls.
- `mrtDesiredFrequency = 500 Hz` → 0.002 s between MRT updates.

Notes:

- `timeHorizon / dt = 1.2 / 0.02 = 60` is an integer, giving a clean OCP grid.
- The rollout integrator uses a slightly smaller step (`0.015 s`) than the OCP grid (`0.02 s`), which is acceptable since ODE45 adapts internally.
- MPC runs at 100 Hz, faster than the grid frequency (50 Hz). The controller effectively evaluates/interpolates the policy twice per grid interval.
- The centroidal state is relatively low‑dimensional, so this fine grid is affordable even with `sqpIteration = 1`.

## Whole‑Body MPC Settings

From `g1_wb_mpc/config/mpc/task.info`:

- `timeHorizon = 1.2 s`
- `multiple_shooting.dt = 0.035 s` → about 34–35 grid nodes.
- `rollout.timeStep = 0.015 s` (commented “should be smaller or equal to dt”).
- `mpcDesiredFrequency = 50 Hz` → 0.02 s between MPC calls.
- `mrtDesiredFrequency = 500 Hz` → 0.002 s between MRT updates.

Why `dt = 0.035` instead of `0.02`:

- The whole‑body MPC problem is much larger and stiffer than the centroidal one:
  - Full base + joint state and velocities.
  - Foot constraints, joint limits, and task‑space tracking on many joints.
- With `sqpIteration = 1` (real‑time iteration), each MPC cycle performs only a single SQP update.
- Reducing `dt` to `0.02 s` would increase the number of nodes from ~34 to 60:
  - More linearizations and a larger Riccati/LQ solve per iteration.
  - For a fixed CPU budget, the SQP step becomes less converged each cycle.
  - In MuJoCo this shows up as unstable behavior (robot tends to fall).
- Using `dt = 0.035 s` is a practical compromise:
  - Fewer nodes → cheaper per‑step computation.
  - A single SQP iteration per MPC call stays closer to the true optimal step.
  - Combined with the tuned costs and constraints, this yields stable whole‑body behavior in MuJoCo.

## Practical Tuning Guidelines

When changing time‑related parameters:

- **OCP grid vs horizon**
  - Prefer `timeHorizon / dt` close to an integer, but it does not need to be exact.
  - Smaller `dt` increases resolution but also the per‑step computational cost.

- **Rollout step**
  - Choose `rollout.timeStep ≤ multiple_shooting.dt`.
  - For adaptive integrators, it is enough that `timeStep` is of the same order or smaller.

- **MPC and MRT frequencies**
  - `mpcDesiredFrequency` sets how often you recompute the policy; it does not have to equal `1 / dt`, but:
    - If `1 / dt ≈ mpcDesiredFrequency`, each MPC call advances roughly one grid step.
    - If `mpcDesiredFrequency` is higher, you evaluate/interpolate the policy multiple times between grid points.
  - `mrtDesiredFrequency` should typically be ≥ `mpcDesiredFrequency`, often a small integer multiple.

- **Centroidal vs whole‑body**
  - Centroidal MPC can run with smaller `dt` and higher `mpcDesiredFrequency` because the model is lower‑dimensional.
  - Whole‑body MPC usually needs a coarser grid to keep real‑time iteration stable with the same hardware and `sqpIteration = 1`.

If you change `dt` or `mpcDesiredFrequency`, you should always:

- Check that one SQP iteration still fits in the real‑time budget.
- Test closed‑loop behavior in MuJoCo (standing/walking stability).
- Adjust some cost weights if the controller becomes too aggressive or sluggish under the new timings.

## MRT and Receding Horizon Interpretation

In this setup the control architecture is split into:

- **MPC solve (slow loop, `mpcDesiredFrequency`)**
  - At time `t_k` solves an OCP over `[t_k, t_k + T]` and produces a time‑stamped policy
    (nominal state/input trajectory and, if enabled, linear feedback gains).
  - In your configs this happens every `T_mpc = 1 / mpcDesiredFrequency` seconds.

- **MRT loop (fast loop, `mrtDesiredFrequency`)**
  - Runs at a higher rate.
  - At each MRT tick between `t_k` and `t_{k+1}`:
    - Reads the current observation (time, state, mode).
    - Evaluates the *current* policy at that time (and state, if feedback is enabled).
    - Sends the resulting control to the plant (MuJoCo or dummy simulator).
  - When a new MPC solution becomes available at `t_{k+1}`, MRT switches to the new policy
    and never uses the tail of the old one again.

This is still a receding‑horizon controller:

- Only the **front part** of each optimized horizon is ever applied on the real system; the rest
  is used to plan ahead and is discarded when the next solve completes.
- Because `T_mpc` is smaller than `dt` in your configs, you effectively only use the segment near
  the first grid point of each policy before re‑solving; later samples `u[1…N‑1]` shape the
  optimization but are not executed verbatim.

Effect of `useFeedbackPolicy`:

- With `useFeedbackPolicy = true`:
  - MRT applies `u(t) = u_ff(t) + K(t) (x(t) − x_ref(t))`, using both feed‑forward and local
    linear feedback from the MPC solution.
- With `useFeedbackPolicy = false` (your current setting):
  - MRT applies **pure feed‑forward** `u(t) = u_ff(t)` (still time‑varying and interpolated),
    and robustness relies on frequent re‑optimization rather than on the local LQR term.

