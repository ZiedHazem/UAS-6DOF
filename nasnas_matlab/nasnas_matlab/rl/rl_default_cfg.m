function cfg = rl_default_cfg(cfg)
%RL_DEFAULT_CFG  Default MATD3 / TD3 training configuration (Tables 2, 3, 4 of the paper).
%   mode      : 'matd3'       two actors (L, D) + centralised twin critics on the global state
%               'independent' two TD3 agents, critics see only their own obs/action (ablation i)
%               'td3'         single agent, 4 actions;  singleObs = 'full' (25-dim, baseline) or
%                             'concat' (18-dim [oL;oD], ablation ii)
d.mode = 'matd3';  d.singleObs = 'full';
d.numEpisodes = 5000;                 % Table 4
d.hidden = [256 256];                 % actor / critic hidden layers
d.lr = 3e-4;  d.gamma = 0.99;  d.tau = 0.005;
d.batch = 256;  d.bufferSize = 1e6;  d.policyDelay = 2;
d.expSigma = [0.2 0.1 0.0];           % exploration noise per curriculum stage (Sec. 4.1.3)
d.tgtStd = 0.1;  d.tgtClip = 0.5;     % target-policy smoothing noise (sigma=0.1, c=0.5)
d.stageFrac = [0.1 0.4];              % stage boundaries = episodes 500 and 2000 of 5000
d.warmup = 1000;                      % [ASSUMED] steps before the first update
d.useGPU = true;
d.saveFile = '';  d.saveEvery = 100;  d.seed = 1;  d.verbose = 10;
d.env = struct('episodeSteps',250);   % Table 3: 250 steps (2.5 s) per training episode
for f = fieldnames(d).', if ~isfield(cfg,f{1}), cfg.(f{1}) = d.(f{1}); end, end
cfg.env = NasnasEnv.defaults(cfg.env);
end
