# TAT Nasnas Mk.1 – nonlinear 6-DOF model, MARL (MATD3) controller and baselines (MATLAB) Dr. ZIED BEN HAZEM

Code accompanying the paper *"Multi-agent reinforcement learning for trajectory tracking of the
TAT Nasnas Mk.1 UAV"* (Drones, manuscript 4592782).

## Requirements
* MATLAB R2023b or newer
* Deep Learning Toolbox (dlnetwork / dlgradient / adamupdate) – needed for the RL part only
* Optional: Parallel Computing Toolbox + CUDA GPU (training), Optimization Toolbox (`fsolve`; trim
  falls back to `fminsearch` without it)
* No Statistics Toolbox needed (Welch test and t-quantiles are implemented with base MATLAB)

## Quick start
```matlab
cd nasnas_matlab
run_all_demo          % trim + modes, open-loop figures, PID/backstepping, 30-episode smoke training
train_all_agents      % full 5000-episode runs: MATD3, TD3, independent TD3, TD3-concat (GPU, ~12-14 h each)
evaluate_all          % Monte-Carlo comparison (5 seeds), nominal / +-10 % uncertainty / wind
```

## Folder map (paper section -> file)
| Folder / file | Content | Paper |
|---|---|---|
| `params/nasnas_params.m` | all physical, actuator, wind, reward and safety parameters | Table 1, Sec. 2.10-2.11, 3.4.4 |
| `model/nasnas_dynamics.m` | 12-state nonlinear 6-DOF model `xdot=f(x,u,w,Theta)` | Eq. 9-58, 71, 76 |
| `model/nasnas_step.m` | RK4 step with first-order actuators, deflection and rate limits | Eq. 61-63 |
| `model/nasnas_trim.m`, `nasnas_linearize.m` | level-flight trim at 36.11 m/s, Jacobians, modal analysis | Sec. 2.14 |
| `model/dryden_init.m`, `dryden_step.m` | Dryden turbulence (sigma 1.5/1.0/0.5 m/s, L 100/80/50 m) | Sec. 2.11 |
| `model/perturb_params.m` | +-10 % aero/inertia, +-5 % mass/CG uncertainty | Eq. 75 |
| `env/NasnasEnv.m` | RL environment: observations (Eq. 89-91), actions (92-93), rewards (94-101), curriculum stages, wind, termination | Sec. 3.4 |
| `env/make_trajectory.m` | line (+30 deg heading change), lawnmower, spiral, figure-eight (Gerono), helix | Sec. 4.2 |
| `rl/train_rl.m` | MATD3 (Algorithm 1) + single-agent TD3 + ablations, twin critics, smoothing, delayed updates | Eq. 77-88 |
| `rl/rl_*.m`, `ReplayBuffer.m` | networks [256,256], hyper-parameters (Tables 2/4), buffer, action scaling | Sec. 3.6 |
| `controllers/pid_controller.m` | cascaded PID, Table 8 gains | Sec. 4.1.5 |
| `controllers/backstepping_controller.m` | Lyapunov backstepping, Table 9 gains | Eq. 115-119 |
| `controllers/rl_policy.m` | decentralised execution of the trained actors | Sec. 3.4.5 |
| `eval/*.m` | episode runner, metrics, multi-seed statistics (mean, std, 95 % CI, Welch p) | Sec. 4.1.4 |
| `sim/run_open_loop.m` | open-loop doublet/pulse response | Fig. 3-10 |

Conventions: NED axes, `z_E` positive **down** (so `Z - Z_ref > 0` means *below* the reference);
state `x=[xE yE zE u v w phi theta psi p q r]`; command `[de da dr dT]` (rad, rad, rad, 0..1);
positive elevator = trailing edge down.

## Modes of `train_rl`
| `cfg.mode` | Agents | Critic input | Paper |
|---|---|---|---|
| `'matd3'` | actor L (8 obs, 2 act), actor D (10 obs, 2 act) | `[s_global(14); a_L; a_D]` | proposed MARL |
| `'td3'`, `singleObs='full'` | 1 actor, 25-dim state, 4 act | `[S25; a]` | single-agent TD3 baseline |
| `'independent'` | two TD3 agents | `[o_i; a_i]` | ablation (i) |
| `'td3'`, `singleObs='concat'` | 1 actor, `[o_L;o_D]` (18) | `[o18; a]` | ablation (ii) |



trim alpha = 2.31 deg, de = -0.68 deg, dT = 0.388 (T = 67.9 N), CL = 0.449;
short period wn = 8.04 rad/s, zeta = 0.51; phugoid wn = 0.335 rad/s, zeta = 0.063;
Dutch roll wn = 4.61 rad/s, zeta = 0.14; roll subsidence tau = 0.18 s; spiral tau = 37.7 s – all
agree with Sec. 2.14 of the paper. The PID loop with the Table 8 gains was also checked on the model
(30 deg heading change tracked with 0.04 deg final error, altitude within 2 m).
**Not verified:** the MATLAB files themselves were written without access to MATLAB and have not been
executed; the RL training (`train_rl`) and the evaluation scripts in particular have never been run.
Expect to fix small syntax/toolbox-version issues on first run, and run `run_all_demo` first.

## 
1. Derivatives missing from Table 1: `CLq=3.0, CLde=0.30, CDde=0.02, CYp=-0.05, CYr=0.15, CYda=0,
   CYdr=0.15, Clr=0.05, Cldr=0.02, Cnp=-0.02, Cnda=-0.02`. They were chosen so that trim and modes
   reproduce the values quoted in Sec. 2.14 (CLde = 0.30 follows from CL = 0.449 at trim). Table 1's
   symbol column is shifted by one row; the *names/values* columns were used.
2. Safety limits of Eq. 99: alpha_stall 15 deg, beta_max 20 deg, phi_max 60 deg, theta_max 45 deg;
   a violation ends the episode.
3. Trajectory dimensions (lawnmower 400 m legs/200 m turns, spiral R0=150 m, figure-eight a=350 m,
   b=175 m, helix R=250 m / 6 deg climb, line with 30 deg heading change) and the guidance that turns the
   trajectory into `theta_ref, phi_ref, psi_ref`.
4. Observation scaling constants, `wv = ww = 0.1`, reward scale 0.01 (`cfg.rewardScale`), curriculum
   weighting (stage 1: attitude x2, position x0.5), warm-up of 1000 steps, 25-dim single-agent state
   (`s_global(14)` + position/speed errors + references + surface deflections).
5. PID outer attitude->rate gains (2.0 and 3.0) – Table 8 does not list them.
6. Episode length: Table 3 says 250 steps (2.5 s) but Table 10 (old numbering) says 6000 steps (60 s).
   The code trains with 250 steps (`cfg.env.episodeSteps`) and evaluates with 6000; please reconcile
   the paper.

## Deviations in the backstepping controller (Eq. 115-119)
The printed equations diverged when tested on the 6-DOF model, so three changes were made
(documented in `backstepping_controller.m`): (1) `q_d = thd_dot - k2 z2 + z1` (sign of the z1 term);
(2) `theta_d` uses the trim angle of attack; (3) the roll law (117) includes the factor
`Ix/(qbar S b)` like (115)-(116). With these changes it holds altitude and tracks a 30 deg heading
change in a quick test (about 17 m peak altitude error), but it is **not tuned**; Table 9 gains are used as given.

## Metrics
3D RMSE, heading RMSE, altitude RMSE, speed RMSE, maximum 3D error ("overshoot"), settling time
(last time the 3D error exceeds 2 m) and control effort (total variation of the four commands).
"Coupling rejection" and "recovery time" of the paper are **not** implemented (definitions not given).



