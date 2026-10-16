%RUN_ALL_DEMO  End-to-end smoke test of the NASNAS package (a few minutes on a laptop).
%   1) parameters, trim and linear modes     2) open-loop response (Figs. 3-10)
%   3) PID / backstepping tracking            4) short MATD3 + TD3 training (smoke test only)
%   For the full 5000-episode experiments use  train_all_agents.m  and  evaluate_all.m.
clear; clc; close all;
startup_nasnas;

%% 1) trim and modes ---------------------------------------------------------------------
P = nasnas_params();  trim = nasnas_trim(P);
[~,~,modes] = nasnas_linearize(P, trim, true); %#ok<ASGLU>

%% 2) open-loop response -----------------------------------------------------------------
run_open_loop();

%% 3) analytic controllers on the straight-line trajectory (30 s) ------------------------
ctrls = struct('name',{'PID','Backstepping'}, 'fcn',{@pid_controller, @backstepping_controller});
env = NasnasEnv(struct('episodeSteps',3000));
for c = 1:numel(ctrls)
    res = run_episode(env, ctrls(c).fcn, struct('traj','line','seed',1,'episodeSteps',3000));
    m = compute_metrics(res);
    fprintf('%-13s 3D RMSE = %6.2f m | heading RMSE = %5.2f deg | altitude RMSE = %5.2f m | failed = %d\n', ...
        ctrls(c).name, m.rmse3d, m.headDeg, m.altRmse, m.failed);
end

%% 4) very short training run (smoke test) -----------------------------------------------
cfg = struct('numEpisodes',30,'useGPU',false,'warmup',500,'verbose',5,'saveFile','');
cfg.mode = 'matd3';  [agentM, ~] = train_rl(cfg);
cfg.mode = 'td3';    [agentS, ~] = train_rl(cfg);
ctrls = struct('name',{'MATD3','TD3'}, 'fcn',{rl_policy(agentM), rl_policy(agentS)});
for c = 1:numel(ctrls)
    res = run_episode(env, ctrls(c).fcn, struct('traj','line','seed',1,'episodeSteps',3000));
    m = compute_metrics(res);
    fprintf('%-13s 3D RMSE = %6.2f m (untrained smoke-test policy, failed = %d)\n', ctrls(c).name, m.rmse3d, m.failed);
end
