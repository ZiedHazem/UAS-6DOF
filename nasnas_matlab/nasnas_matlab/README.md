# TAT Nasnas Mk.1 UAV: Nonlinear 6-DOF Model, MATD3 Controller, and Baseline Controllers

**Author:** Dr. Zied Ben Hazem

This repository accompanies the manuscript *“Multi-Agent Reinforcement Learning for Trajectory Tracking of the TAT Nasnas Mk.1 UAV”* (Drones, Manuscript ID: 4592782).

The project provides a MATLAB implementation of a nonlinear six-degree-of-freedom (6-DOF) UAV model, model-based flight controllers, a multi-agent reinforcement learning (MARL) framework using Multi-Agent Twin Delayed Deep Deterministic Policy Gradient (MATD3), benchmark controllers, and simulation-based performance evaluation.

## 1. Software Requirements

- MATLAB R2023b or later.
- Deep Learning Toolbox for reinforcement learning and neural-network training.
- Optimization Toolbox if required by the selected trim and optimization routines.
- Parallel Computing Toolbox and a compatible CUDA-enabled GPU for optional accelerated training.

The required toolboxes depend on the MATLAB functions used by the selected execution path. Confirm toolbox availability before running the corresponding scripts.

## 2. Quick Start

Run the following commands from the project root directory:

```matlab
cd nasnas_matlab

% Run the demonstration and preliminary simulations
run_all_demo

% Train the reinforcement learning controllers
train_all_agents

% Evaluate trained controllers
evaluate_all
```

`run_all_demo` is intended for preliminary model and controller checks, including trim, open-loop responses, conventional controllers, and a short reinforcement learning run.

`train_all_agents` is intended for full training runs using the configured episode counts and controller variants.

`evaluate_all` is intended for comparative evaluation across random seeds, nominal conditions, parameter uncertainties, and wind scenarios.

**Execution note:** The availability and successful execution of these entry-point scripts must be confirmed in the target MATLAB environment before the repository is described as fully tested.

## 3. Project Structure and Manuscript Mapping

| File or directory | Main functionality | Manuscript reference |
|---|---|---|
| `params/nasnas_params.m` | UAV, actuator, wind, reward, and safety parameters | Tables 1 and 2; Sections 2.10–2.11 and 3.4.4 |
| `model/nasnas_dynamics.m` | Nonlinear 12-state 6-DOF equations | Equations 9–58, 71, and 76 |
| `model/nasnas_step.m` | Numerical integration and actuator constraints | Equations 61–63 |
| `model/nasnas_trim.m` | Level-flight trim calculation | Section 2.14 |
| `model/nasnas_linearize.m` | Linearization and modal analysis | Section 2.14 |
| `model/dryden_init.m`, `model/dryden_step.m` | Dryden turbulence initialization and propagation | Section 2.11 |
| `model/perturb_params.m` | Aerodynamic, inertia, mass, and CG uncertainty | Equation 75 |
| `env/NasnasEnv.m` | Reinforcement learning environment, observations, actions, rewards, curriculum, and termination | Section 3.4 |
| `env/make_trajectory.m` | Reference trajectory generation | Section 4.2 |
| `rl/train_rl.m` | MATD3, TD3, independent-agent TD3, and concatenated-observation variants | Equations 77–88 |
| `rl/rl_*.m`, `ReplayBuffer.m` | Actor–critic networks, replay buffer, and learning configuration | Sections 3.6 and relevant parameter tables |
| `controllers/pid_controller.m` | Cascaded PID controller | Section 4.1.5 |
| `controllers/backstepping_controller.m` | Nonlinear backstepping controller | Equations 115–119 |
| `controllers/rl_policy.m` | Execution of trained reinforcement learning policies | Section 3.4.5 |
| `eval/*.m` | Simulation, performance metrics, and statistical analysis | Section 4.1.4 |
| `sim/run_open_loop.m` | Open-loop doublet and pulse-response simulations | Figures 3–10 |

The manuscript references above should be checked against the final published numbering.

## 4. Model Conventions

The implementation uses the North-East-Down (NED) coordinate convention, with the vertical position \(z_E\) positive downward. Consequently, \(z_E-z_{\mathrm{ref}}>0\) indicates that the UAV is below its reference altitude.

The state vector is defined as

\[
\mathbf{x}=
[x_E,y_E,z_E,u,v,w,\phi,\theta,\psi,p,q,r]^T.
\]

The control command is

\[
\mathbf{u}=[\delta_e,\delta_a,\delta_r,\delta_T]^T,
\]

where the elevator, aileron, and rudder commands are expressed in radians and the normalized throttle lies between 0 and 1.

The elevator sign convention defines positive deflection as trailing-edge down.

## 5. Reinforcement Learning Configurations

| Configuration | Agents | Observation and action configuration | Purpose |
|---|---|---|---|
| `matd3` | Two coordinated agents | Longitudinal actor: 8 observations and 2 actions; lateral actor: 10 observations and 2 actions | Proposed multi-agent controller |
| `td3`, `singleObs='full'` | One agent | 25-dimensional observation and 4 actions | Centralized TD3 baseline |
| `independent` | Two independent agents | Individual observations and actions | Independent-learning ablation |
| `td3`, `singleObs='concat'` | One agent | Concatenated 18-dimensional observation and 4 actions | Concatenated-observation ablation |

The MATD3 centralized critic is configured to use the global state and both agents' actions during training, while the trained actors are intended for decentralized execution.

The network architecture, replay-buffer configuration, exploration noise, target-policy smoothing, delayed actor updates, and other hyperparameters should be consistent with the final manuscript tables.

## 6. Reported Trim and Dynamic Characteristics

The following numerical results are reported as corresponding to the model configuration and Section 2.14 of the manuscript. They should be regenerated from the MATLAB implementation before being treated as independently reproduced results.

| Parameter | Reported value |
|---|---:|
| Trim angle of attack | 2.31° |
| Trim elevator deflection | −0.68° |
| Trim throttle | 0.388 |
| Trim thrust | 67.9 N |
| Trim lift coefficient | 0.449 |
| Short-period natural frequency | 8.04 rad/s |
| Short-period damping ratio | 0.51 |
| Phugoid natural frequency | 0.335 rad/s |
| Phugoid damping ratio | 0.063 |
| Dutch-roll natural frequency | 4.61 rad/s |
| Dutch-roll damping ratio | 0.14 |
| Roll-subsidence time constant | 0.18 s |
| Spiral-mode time constant | 37.7 s |

A preliminary PID result is also reported for a 30° heading-change maneuver, with a final heading error of approximately 0.04° and altitude deviation within 2 m. These values should be confirmed using a reproducible simulation run.

## 7. Implementation Notes and Modeling Assumptions

The following details should be documented and checked against the final code and manuscript.

1. **Aerodynamic derivatives:** The implementation includes the additional derivatives `CLq`, `CLde`, `CDde`, `CYp`, `CYr`, `CYda`, `CYdr`, `Clr`, `Cldr`, `Cnp`, and `Cnda`. Their values and physical justification should be documented explicitly because they are not all listed consistently in the original Table 1.

2. **Safety constraints:** The reported limits are 15° angle of attack, 20° sideslip, 60° bank angle, and 45° pitch angle. Exceeding a configured safety limit terminates an episode.

3. **Reference trajectories:** The configured trajectories include a straight path with a 30° heading change, lawnmower pattern, spiral, Gerono figure-eight, and helix. Their dimensions and guidance equations should match Section 4.2.

4. **Observation and reward scaling:** The implementation includes normalized observations, velocity-related scaling constants, reward scaling, curriculum weights, and a 1,000-step warm-up configuration. These values should be checked against the final training configuration.

5. **PID controller:** The outer-loop attitude-to-rate gains are reported as 2.0 and 3.0, respectively, in addition to the gains listed in Table 8.

6. **Episode duration:** The manuscript contains inconsistent episode-length descriptions: 250 steps in Table 3 and 6,000 steps in the older Table 10 numbering. The training and evaluation configurations must be reconciled with the reported experimental protocol.

7. **Backstepping controller:** The implementation contains modifications to the printed equations, including the sign of the \(z_1\) term in the desired pitch-rate expression, use of the trim angle of attack, and inclusion of an inertia/dynamic-pressure factor in the roll control law. These changes require explicit justification and numerical validation before the controller can be claimed to reproduce the published formulation.

8. **Performance metrics:** The evaluation framework is intended to calculate 3D trajectory RMSE, heading RMSE, altitude RMSE, speed RMSE, maximum 3D tracking error, settling time, and control effort. Any additional metrics reported in the manuscript must have explicit computational definitions and corresponding implementations.

## 8. Verification and Reproducibility

A complete verification process should include:

- **Model verification:** Check state derivatives, coordinate conventions, actuator limits, numerical integration, and physical parameter consistency.
- **Trim verification:** Confirm that the calculated equilibrium satisfies the steady-flight equations and produces the reported trim values.
- **Linearization verification:** Check the numerical Jacobians, eigenvalues, modal frequencies, and damping ratios.
- **Controller verification:** Execute the PID and backstepping controllers using identical initial conditions and reference trajectories.
- **Training verification:** Confirm that all reinforcement learning configurations complete training without runtime errors and that checkpoints are saved correctly.
- **Evaluation verification:** Run the configured seeds and uncertainty scenarios, reproduce the reported metrics, and verify statistical calculations.
- **Reproducibility verification:** Record the MATLAB release, required toolboxes, random seeds, configuration files, and any hardware-dependent settings.

The repository should be marked **fully tested and verified** only after these checks have been completed successfully and the reported results can be reproduced.

## 9. Statistical Evaluation

The intended evaluation includes multi-seed comparisons using means, standard deviations, 95% confidence intervals, and Welch's statistical test. The implementation and reported results should be checked to ensure consistent sample definitions, appropriate assumptions, and correct treatment of multiple controller comparisons.

The paper's terms *coupling rejection* and *recovery time* require explicit operational definitions and corresponding code if they are to be reported as implemented evaluation metrics.

## 10. Reproducibility and Scientific Scope

The project is intended to support simulation-based investigation of nonlinear UAV dynamics and multi-agent reinforcement learning for trajectory tracking. Model parameters that have been estimated rather than experimentally identified should be reported as such. Simulation performance should not be presented as experimental flight validation.

The final release should include the source files, configuration files, training and evaluation instructions, and any generated results necessary to reproduce the reported experiments.

**Author:** Dr. Zied Ben Hazem  
**Research area:** Nonlinear flight dynamics, intelligent control, and multi-agent reinforcement learning.

