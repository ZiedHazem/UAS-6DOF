%EVALUATE_ALL  Reproduce the comparisons of Sec. 4.2-4.3 (needs the agents from train_all_agents.m).
startup_nasnas;
A1 = load('results/agent_matd3.mat');  A2 = load('results/agent_td3.mat');
ctrls = struct('name',{'MATD3 (proposed)','TD3 (single)','Backstepping','PID'}, ...
               'fcn', {rl_policy(A1.agent), rl_policy(A2.agent), @backstepping_controller, @pid_controller});
cfg = struct('seeds',1:5,'episodeSteps',6000);
cfg.scenario = 'nominal';      R_nom = evaluate_controllers(ctrls, cfg);               %#ok<NASGU>
cfg.scenario = 'uncertainty';  cfg.trajs = {'eight'};  R_unc  = evaluate_controllers(ctrls, cfg); %#ok<NASGU>
cfg.scenario = 'wind';         cfg.trajs = {'eight'};  R_wind = evaluate_controllers(ctrls, cfg); %#ok<NASGU>
if ~exist('results','dir'), mkdir('results'); end
save('results/evaluation.mat','R_nom','R_unc','R_wind');
