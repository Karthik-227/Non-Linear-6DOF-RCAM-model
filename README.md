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

## Simulation Driver Script (`initialize_constants.m`)

This script initializes the **initial state and control trim values**, runs the Simulink model, and plots the resulting time histories. It performs three jobs:

1. **Defines the initial state vector `x0`** (9×1) and **initial/constant control vector `u`** (5×1) used to kick off the simulation.
2. **Runs the Simulink model** `RCAM_model_run.slx` via the `sim()` command.
3. **Extracts and plots** the logged control inputs (`simU`) and states (`simX`) from the Simulink `ans` output structure.

```matlab
%Initialize constants for the RCAM simulation
clear
clc
close all
%% Define constants
x0 = [85;      %approx 165 knots
      0;
      0;
      0;
      0;
      0;
      0;
      0.1;     %approx 5.73 deg
      0];
u = [0;
     -0.1;     %approx -5.73 deg
     0;
     0.08;     %recall minimum for throttles are 0.5*pi/180 = 0.0087
     0.08];
%% run the model
sim('RCAM_model_run.slx')
%% Plot the results
t = ans.simX.Time;
u1 = ans.simU.Data(:,1);
u2 = ans.simU.Data(:,2);
u3 = ans.simU.Data(:,3);
u4 = ans.simU.Data(:,4);
u5 = ans.simU.Data(:,5);
x1 = ans.simX.Data(:,1);
x2 = ans.simX.Data(:,2);
x3 = ans.simX.Data(:,3);
x4 = ans.simX.Data(:,4);
x5 = ans.simX.Data(:,5);
x6 = ans.simX.Data(:,6);
x7 = ans.simX.Data(:,7);
x8 = ans.simX.Data(:,8);
x9 = ans.simX.Data(:,9);

figure
subplot(5,1,1); plot(t,u1); legend('u_1'); grid on
subplot(5,1,2); plot(t,u2); legend('u_2'); grid on
subplot(5,1,3); plot(t,u3); legend('u_3'); grid on
subplot(5,1,4); plot(t,u4); legend('u_4'); grid on
subplot(5,1,5); plot(t,u5); legend('u_5'); grid on

%Plot the states
figure
%u, v, w
subplot(3,3,1); plot(t,x1); legend('x_1'); grid on
subplot(3,3,4); plot(t,x2); legend('x_2'); grid on
subplot(3,3,7); plot(t,x3); legend('x_3'); grid on
%p, q, r
subplot(3,3,2); plot(t,x4); legend('x_4'); grid on
subplot(3,3,5); plot(t,x5); legend('x_5'); grid on
subplot(3,3,8); plot(t,x6); legend('x_6'); grid on
%phi, theta, psi
subplot(3,3,3); plot(t,x7); legend('x_7'); grid on
subplot(3,3,6); plot(t,x8); legend('x_8'); grid on
subplot(3,3,9); plot(t,x9); legend('x_9'); grid on
```

### Example trim values used

| Variable | Value | Meaning |
|---|---|---|
| `x0(1) = u` | 85 m/s | Approx. 165 knots forward airspeed |
| `x0(8) = θ` | 0.1 rad | Approx. 5.73° initial pitch angle |
| `u(2) = δT` | −0.1 rad | Approx. −5.73° stabilizer deflection |
| `u(4), u(5) = δth1, δth2` | 0.08 rad | Throttle settings (minimum allowed is 0.5°≈0.0087 rad) |

All other states and controls (`v, w, p, q, r, φ, ψ, δA, δR`) are initialized to zero.

> **Note:** The Simulink model must log the control and state signals to workspace variables named `simU` and `simX` (e.g., via "To Workspace" blocks or signal logging configured with these names) for the plotting section of this script to work correctly.
>
> Also note the `sim()` call references the Simulink file by name (`RCAM_model_run.slx`) — make sure the filename casing matches exactly, since MATLAB/Simulink file references can be case-sensitive on some operating systems (e.g., Linux).

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

1. Make sure all three files (`RCAM_model.m`, `initialize_constants.m`, `RCAM_model_run.slx`) are in the same MATLAB working directory (or on the MATLAB path).
2. Open `RCAM_model_run.slx` in Simulink and confirm it references `RCAM_model.m` (e.g., via a MATLAB Function block) and logs the state and control signals to workspace variables named `simX` and `simU` respectively (e.g., using "To Workspace" blocks or Simulink signal logging).
3. Run `initialize_constants.m` directly from the MATLAB Command Window:

```matlab
initialize_constants
```

This will:
- Set the initial state `x0` (85 m/s forward speed, 0.1 rad pitch, all other states zero) and the control vector `u` (stabilizer at −0.1 rad, both throttles at 0.08) in the base workspace.
- Automatically launch the Simulink simulation via `sim('RCAM_model_run.slx')`.
- Automatically generate two figures once the simulation completes:
  - **Figure 1**: time histories of the 5 control inputs (`u1`–`u5`)
  - **Figure 2**: a 3×3 grid of the 9 state time histories, grouped as body-axis velocities (`x1–x3`), body rates (`x4–x6`), and Euler angles (`x7–x9`)

4. To explore different flight conditions or control inputs, edit the `x0` and `u` vectors at the top of `initialize_constants.m` and re-run the script. Remember the physical limits enforced inside `RCAM_model.m`:

| Control | Min | Max |
|---|---|---|
| `δA` (aileron) | −25° | +25° |
| `δT` (stabilizer) | −25° | +10° |
| `δR` (rudder) | −30° | +30° |
| `δth1`, `δth2` (throttles) | 0.5° (≈0.0087 rad) | 10° |

---

## Known Notes / Caveats

- The model currently computes `Mcg_b` and `x4to6dot` twice in succession in `RCAM_model.m` (a redundant/duplicate calculation). This does not affect correctness but could be cleaned up for efficiency.
- The rudder/throttle control limit checks are implemented with `if`/`elseif` saturation blocks; note that `u5` (engine 2 throttle) saturation is not explicitly enforced in the current code (only `u1`–`u4` are clamped) — consider adding a matching saturation block for `u5` for consistency.
- This model assumes rigid-body dynamics only; no structural flexibility, actuator lag, or sensor dynamics are included.

---

## References

- Lum, C. — RCAM Modeling Lecture Series (YouTube), which this implementation is based on/inspired by.
- GARTEUR Action Group FM(AG08) — *Design Challenge: A Robust Flight Control Design Challenge Problem Formulation and Manual: The Research Civil Aircraft Model (RCAM)*, the original RCAM benchmark specification.
