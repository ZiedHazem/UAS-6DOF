function R = evaluate_controllers(ctrls, cfg)
%EVALUATE_CONTROLLERS  Monte-Carlo comparison of controllers (Sec. 4.1.4, 4.2, 4.3).
%   ctrls : struct array with fields  name  and  fcn  ( [cmd,st] = fcn(out,st,dt) )
%   cfg   : trajs (cellstr, default all five), seeds (default 1:5), scenario
%           ('nominal'|'uncertainty'|'wind'), episodeSteps (default 6000 = 60 s)
%   Reports mean +/- std, 95 % CI (Student t, n-1 dof) and Welch t-test p-values of the FIRST
%   controller against every other one (put the proposed MATD3 controller first).
if nargin < 2, cfg = struct(); end
if ~isfield(cfg,'trajs'),        cfg.trajs = {'line','lawnmower','spiral','eight','helix'}; end
if ~isfield(cfg,'seeds'),        cfg.seeds = 1:5; end
if ~isfield(cfg,'scenario'),     cfg.scenario = 'nominal'; end
if ~isfield(cfg,'episodeSteps'), cfg.episodeSteps = 6000; end
env = NasnasEnv(struct('episodeSteps',cfg.episodeSteps));
nC = numel(ctrls); nT = numel(cfg.trajs); nS = numel(cfg.seeds);
names = {'rmse3d','headDeg','altRmse','velRmse','maxErr','settle','tv'};
M = nan(nC, nT, nS, numel(names));
for c = 1:nC
    for t = 1:nT
        for s = 1:nS
            o = struct('traj',cfg.trajs{t},'seed',cfg.seeds(s),'episodeSteps',cfg.episodeSteps);
            switch cfg.scenario
                case 'nominal',     o.uncertainty = false; o.wind = 'none';
                case 'uncertainty', o.uncertainty = true;  o.wind = 'none';
                case 'wind',        o.uncertainty = false; o.wind = 'pulses';
            end
            res = run_episode(env, ctrls(c).fcn, o);
            m = compute_metrics(res);
            for q = 1:numel(names), M(c,t,s,q) = m.(names{q}); end
        end
    end
end
R.metrics = names;  R.raw = M;  R.cfg = cfg;  R.ctrl = {ctrls.name};
R.mean = mean(M,3);  R.std = std(M,0,3);
fprintf('\n=== Scenario: %s  (n = %d seeds) ===\n', cfg.scenario, nS);
for t = 1:nT
    fprintf('\nTrajectory: %s\n', cfg.trajs{t});
    fprintf('%-18s %-16s %-16s %-16s %-10s %-12s\n','Controller','3D RMSE [m]','Heading [deg]','Altitude [m]','Settle [s]','Control TV');
    for c = 1:nC
        mu = squeeze(R.mean(c,t,:)); sd = squeeze(R.std(c,t,:));
        a = squeeze(M(c,t,:,1)); [~,ci] = tci(a);
        fprintf('%-18s %5.2f+/-%-5.2f(CI %.2f) %5.2f+/-%-8.2f %5.2f+/-%-8.2f %-10.2f %-12.2f\n', ...
            ctrls(c).name, mu(1), sd(1), ci, mu(2), sd(2), mu(3), sd(3), mu(6), mu(7));
    end
    for c = 2:nC
        p = welch_p(squeeze(M(1,t,:,1)), squeeze(M(c,t,:,1)));
        fprintf('  Welch p (%s vs %s, 3D RMSE) = %.4g\n', ctrls(1).name, ctrls(c).name, p);
    end
end
end

function [tcrit, ci] = tci(a)
n = numel(a); df = n-1;
tcrit = fzero(@(t) betainc(df/(df+t^2), df/2, 0.5) - 0.05, [0.5 50]);   % two-sided 95 %
ci = tcrit*std(a)/sqrt(n);
end

function p = welch_p(a, b)
n1 = numel(a); n2 = numel(b); v1 = var(a); v2 = var(b);
se2 = v1/n1 + v2/n2;
if se2 == 0, p = double(mean(a) == mean(b)); return; end
t = (mean(a)-mean(b))/sqrt(se2);
df = se2^2 / ((v1/n1)^2/(n1-1) + (v2/n2)^2/(n2-1));
p = betainc(df/(df+t^2), df/2, 0.5);
end
