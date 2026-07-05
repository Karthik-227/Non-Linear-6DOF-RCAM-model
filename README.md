# RCAM (Research Civil Aircraft Model) — MATLAB/Simulink Implementation

This repository contains a MATLAB/Simulink implementation of the **RCAM (Research Civil Aircraft Model)**, a nonlinear 6-degree-of-freedom (6-DOF) rigid-body aircraft dynamics model widely used as a benchmark for flight dynamics, control design, and simulation studies. The model and structure of this implementation are inspired by **Professor Christopher Lum's** YouTube lecture series on RCAM modeling and flight dynamics.

RCAM was originally developed as a GARTEUR (Group for Aeronautical Research and Technology in Europe) benchmark model representing a generic wide-body civil transport aircraft, and is commonly used to study nonlinear flight dynamics, trim/linearization techniques, and flight control law design.

---

## Repository Contents

| File | Description |
|---|---|
| `RCAM_model.m` | Core nonlinear state-derivative function. Computes `XDOT = f(X, U)` — the time derivatives of the 9 aircraft states given the current state vector and control inputs. |
| `initialize_constants.m` | Script defining all fixed aircraft geometric, inertial, aerodynamic, and environmental constants used by the model. |
| `RCAM_model_run.slx` | Simulink model that wraps `RCAM_model.m` (typically via a MATLAB Function block or S-Function) to allow time-domain simulation, trimming, linearization, and visualization. |

---

## Model Overview

RCAM is a nonlinear, 6-DOF, rigid-body aircraft model formulated in the **body-fixed reference frame**. It captures:

- Rigid-body translational and rotational dynamics (Newton-Euler equations)
- Nonlinear aerodynamic force and moment models (lift, drag, sideforce, and moment coefficients, including a piecewise linear/nonlinear lift curve to represent stall behavior)
- Twin-engine thrust forces and moments (including moment arms due to engine offset from the center of gravity)
- Gravitational forces resolved into the body frame
- Standard aircraft Euler angle kinematics (roll `φ`, pitch `θ`, yaw `ψ`)

The model does **not** include actuator dynamics, atmospheric turbulence, or flexible-body effects — it represents the aircraft as a rigid body with ideal (instantaneous, saturation-limited) control surface deflections and throttle inputs.

---

## State Vector

The state vector `X` is a 9-element column vector expressed in the body-fixed frame:

| Index | Symbol | Description | Units |
|---|---|---|---|
| `X(1)` | `u` | Body-axis velocity component (forward) | m/s |
| `X(2)` | `v` | Body-axis velocity component (lateral) | m/s |
| `X(3)` | `w` | Body-axis velocity component (vertical) | m/s |
| `X(4)` | `p` | Roll rate about body x-axis | rad/s |
| `X(5)` | `q` | Pitch rate about body y-axis | rad/s |
| `X(6)` | `r` | Yaw rate about body z-axis | rad/s |
| `X(7)` | `φ` (phi) | Bank/roll Euler angle | rad |
| `X(8)` | `θ` (theta) | Pitch Euler angle | rad |
| `X(9)` | `ψ` (psi) | Heading/yaw Euler angle | rad |

## Control Input Vector

The control vector `U` is a 5-element column vector:

| Index | Symbol | Description | Range |
|---|---|---|---|
| `U(1)` | `δA` | Aileron deflection | ±25° |
| `U(2)` | `δT` | Stabilizer (tailplane) deflection | −25° to +10° |
| `U(3)` | `δR` | Rudder deflection | ±30° |
| `U(4)` | `δth1` | Engine 1 throttle | 0.5° to 10° (equivalent) |
| `U(5)` | `δth2` | Engine 2 throttle | 0.5° to 10° (equivalent) |

All control inputs are automatically saturated to their physical limits inside `RCAM_model.m` before being used in the dynamics.

## Output

`RCAM_model.m` returns:

```
XDOT = [x1to3dot;   % u_dot, v_dot, w_dot
        x4to6dot;   % p_dot, q_dot, r_dot
        x7to9dot];  % phi_dot, theta_dot, psi_dot
```

which represents the time derivative of the full 9-state vector and can be integrated (e.g., via `ode45`, a fixed-step solver, or the Simulink model) to propagate the aircraft's motion over time.

---

## Modeling Details

### 1. Control Saturation
Aileron, stabilizer, rudder, and throttle inputs are clipped to their physically realizable ranges before being used anywhere else in the model.

### 2. Airflow Angles
Airspeed `Va`, angle of attack `α`, and sideslip angle `β` are computed from the body-axis velocity components:

```
Va    = sqrt(u^2 + v^2 + w^2)
alpha = atan2(w, u)
beta  = asin(v / Va)
```

### 3. Aerodynamic Force Coefficients
- **Lift** (`CL_wb`) uses a linear lift-curve slope below the stall angle (`alpha_switch`) and a cubic polynomial fit above it, capturing post-stall nonlinearity.
- **Tail lift** (`CL_t`) accounts for downwash effects and stabilizer deflection.
- **Drag** (`CD`) is modeled as a quadratic function of angle of attack (induced + parasitic drag).
- **Sideforce** (`CY`) is a function of sideslip and rudder deflection.

### 4. Force Transformation
Aerodynamic forces are computed in the stability axis (`F_s`) and rotated into the body axis (`F_b`) via the standard angle-of-attack rotation matrix `C_bs`.

### 5–7. Aerodynamic Moments
Moments are first computed about the aerodynamic center (`MAac_b`) using stability/control derivatives (`eta`, `dCMdx`, `dCMdu`), then transferred to the center of gravity using the offset between the CG and aerodynamic center (`rcg_b - rac_b`).

### 8. Engine Forces and Moments
Each engine produces thrust proportional to its throttle setting (`F = δth · m · g`), aligned with the body x-axis. Because the engines are mounted off the aircraft centerline/CG, their thrust also produces a moment about the CG.

### 9. Gravity
Gravitational acceleration is resolved from the inertial frame into the body frame using the current bank (`φ`) and pitch (`θ`) angles.

### 10. Equations of Motion
Newton-Euler rigid-body equations are used to compute the full state derivative:

```
u,v,w_dot   = (1/m)*F_b - (wbe_b × V_b)
p,q,r_dot   = Ib^-1 * (M_cg,b - wbe_b × (Ib * wbe_b))
phi,theta,psi_dot = H(phi,theta) * wbe_b
```

where `Ib` is the aircraft inertia tensor, `F_b` is the total body-axis force (aerodynamic + thrust + gravity), and `M_cg,b` is the total moment about the CG (aerodynamic + thrust).

---

## Constants (`initialize_constants.m`)

This script should define all fixed physical parameters used by `RCAM_model.m`, including:

- **Mass & geometry**: aircraft mass, mean aerodynamic chord, wing/tail planform areas, tail moment arm
- **CG, aerodynamic center, and engine mounting positions** (in the body/mean-aerodynamic-chord reference frame)
- **Environmental constants**: air density, gravitational acceleration
- **Aerodynamic model parameters**: downwash gradient, zero-lift angle of attack, lift-curve slope, stall polynomial coefficients, stall angle threshold
- **Control surface limits** for aileron, stabilizer, rudder, and throttle

> Ensure `initialize_constants.m` is run (or its variables are loaded into the workspace/Simulink model) **before** simulating `RCAM_model.m`, since the state-derivative function depends on these constants being defined in scope.

---

## Simulink Model (`RCAM_model_run.slx`)

`RCAM_model_run.slx` provides a simulation harness around the nonlinear model, typically consisting of:

- A **MATLAB Function block** (or S-Function) calling `RCAM_model.m` to compute `XDOT` from the current state and control inputs
- **Integrator blocks** to propagate the state vector `X` over time
- **Input blocks** (e.g., step, constant, or manual switch blocks) to specify control surface deflections and throttle settings
- **Scopes/output ports** to visualize states such as velocity, attitude, and angular rates over the simulation run

This Simulink model enables:
- Open-loop nonlinear time-domain simulation
- Trimming (finding steady-state `X`, `U` for a desired flight condition)
- Linearization about a trim point (for control design, e.g., stability/control augmentation systems)

---

## Usage

1. Run `initialize_constants.m` in the MATLAB workspace to load all model constants.
2. Open and run `RCAM_model_run.slx` in Simulink, or call `RCAM_model.m` directly from a MATLAB script/ODE solver:

```matlab
initialize_constants;   % load constants into base workspace

X0 = [85; 0; 0; 0; 0; 0; 0; 0.1; 0];   % initial state guess
U0 = [0; -0.1; 0; 0.08; 0.08];          % initial control input guess

[t, X] = ode45(@(t, X) RCAM_model(X, U0), [0 60], X0);
```

3. Post-process the resulting state history to analyze aircraft response, or use the Simulink model to interactively adjust control inputs and observe the aircraft's nonlinear response in real time.

---

## Known Notes / Caveats

- The model currently computes `Mcg_b` and `x4to6dot` twice in succession in `RCAM_model.m` (a redundant/duplicate calculation). This does not affect correctness but could be cleaned up for efficiency.
- The rudder/throttle control limit checks are implemented with `if`/`elseif` saturation blocks; note that `u5` (engine 2 throttle) saturation is not explicitly enforced in the current code (only `u1`–`u4` are clamped) — consider adding a matching saturation block for `u5` for consistency.
- This model assumes rigid-body dynamics only; no structural flexibility, actuator lag, or sensor dynamics are included.

---

## References

- Lum, C. — RCAM Modeling Lecture Series (YouTube), which this implementation is based on/inspired by.
- GARTEUR Action Group FM(AG08) — *Design Challenge: A Robust Flight Control Design Challenge Problem Formulation and Manual: The Research Civil Aircraft Model (RCAM)*, the original RCAM benchmark specification.

---

