%TRAIN_ALL_AGENTS  Full training runs used in the paper (5000 episodes each, GPU recommended).
%   Trains: MATD3 (proposed), single-agent TD3 baseline, and the two CTDE ablations of Sec. 4.1.5.
%   ~12-14 h per run on an RTX 3090 (see Sec. 4.1.6).  Results are saved in ./results/.
startup_nasnas;
runs = { ...
  struct('mode','matd3',       'singleObs','full',   'saveFile','results/agent_matd3.mat'), ...
  struct('mode','td3',         'singleObs','full',   'saveFile','results/agent_td3.mat'), ...
  struct('mode','independent', 'singleObs','full',   'saveFile','results/agent_independent_td3.mat'), ...
  struct('mode','td3',         'singleObs','concat', 'saveFile','results/agent_td3_concat.mat')};
for i = 1:numel(runs)
    cfg = runs{i};  cfg.numEpisodes = 5000;  cfg.seed = 1;
    fprintf('\n##### Training %s (%s) #####\n', cfg.mode, cfg.singleObs);
    train_rl(cfg);
end
