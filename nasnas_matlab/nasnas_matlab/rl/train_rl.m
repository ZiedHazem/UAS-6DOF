function [agent, logs] = train_rl(cfg)
%TRAIN_RL  MATD3 training for the TAT Nasnas Mk.1 (Algorithm 1) and the TD3 baselines.
%   [agent, logs] = train_rl(cfg)      (see rl_default_cfg for options)
%   Requires: Deep Learning Toolbox (dlnetwork, dlarray, dlgradient, adamupdate).
%   Optional: Parallel Computing Toolbox + GPU.
%
%   Centralised training / decentralised execution (CTDE, Sec. 3.4.5):
%     mode 'matd3'       critic_i(s_global, a_L, a_D),  actor_i(o_i)
%     mode 'independent' critic_i(o_i, a_i)             (no centralised critic)
%     mode 'td3'         critic(S, a),  actor(S)         (single agent, 4 actions)
%   Twin critics, target-policy smoothing and delayed actor/target updates as in Eqs.(77)-(88).
if nargin < 1, cfg = struct(); end
cfg  = rl_default_cfg(cfg);
mode = cfg.mode;  M = cfg.numEpisodes;
rng(cfg.seed);
env  = NasnasEnv(cfg.env);
[obsDims, actDims] = rl_dims(mode, cfg.singleObs);
nAg = numel(obsDims);  nS = 14;
obsOff = [0 cumsum(obsDims)];  actOff = [0 cumsum(actDims)];
useGPU = cfg.useGPU && canUseGPU;
nR = nAg;
buf = ReplayBuffer(cfg.bufferSize, nS, sum(obsDims), sum(actDims), nR);

% ---------------- networks, targets, Adam state ------------------------------------
actors = cell(1,nAg); actorsT = cell(1,nAg); critics = cell(nAg,2); criticsT = cell(nAg,2);
optA = cell(1,nAg); optC = cell(nAg,2);
for i = 1:nAg
    actors{i}  = rl_make_net(obsDims(i), actDims(i), cfg.hidden, true);
    cin = criticInDim(mode, i, nS, obsDims, actDims);
    for k = 1:2, critics{i,k} = rl_make_net(cin, 1, cfg.hidden, false); end
    actorsT{i} = actors{i};
    for k = 1:2, criticsT{i,k} = critics{i,k}; optC{i,k} = struct('avg',[],'sq',[]); end
    optA{i} = struct('avg',[],'sq',[]);
end
if useGPU
    for i = 1:nAg
        actors{i} = dlupdate(@gpuArray, actors{i}); actorsT{i} = dlupdate(@gpuArray, actorsT{i});
        for k = 1:2, critics{i,k} = dlupdate(@gpuArray, critics{i,k}); criticsT{i,k} = dlupdate(@gpuArray, criticsT{i,k}); end
    end
end

% ---------------- curriculum (Sec. 4.1.3) --------------------------------------------
stageEnd = round(cfg.stageFrac*M);                    % [500 2000] for M = 5000
trajSet  = {'line','lawnmower','spiral','eight','helix'};
logs = struct('episodeReward',zeros(M,nAg),'rmse',zeros(M,1),'headingErr',zeros(M,1), ...
              'stage',zeros(M,1),'steps',zeros(M,1));
stepCount = 0;  tStart = tic;

for ep = 1:M
    if ep <= stageEnd(1), stage = 1; elseif ep <= stageEnd(2), stage = 2; else, stage = 3; end
    env.setStage(stage);
    sigma = cfg.expSigma(stage);
    switch stage
        case 1, ropts = struct('traj','level','uncertainty',false,'wind','none');
        case 2, ropts = struct('traj','line','uncertainty',false,'wind','none');
        case 3, ropts = struct('traj',trajSet{randi(numel(trajSet))},'uncertainty',true,'wind','random');
    end
    ropts.seed = cfg.seed*100000 + ep;
    out = env.reset(ropts);
    epR = zeros(1,nAg); e2 = 0; h2 = 0; nSt = 0;
    done = false;
    while ~done
        % ---- action selection with exploration noise (Algorithm 1, step 7) ----------
        o = cell(1,nAg); a = zeros(sum(actDims),1);
        for i = 1:nAg
            o{i} = rl_agent_obs(mode, cfg.singleObs, out, i);
            ai = predict(actors{i}, toDl(o{i}, useGPU));
            ai = double(gather(extractdata(ai)));
            ai = min(max(ai + sigma*randn(size(ai)), -1), 1);
            a(actOff(i)+1:actOff(i+1)) = ai;
        end
        [out2, r, done, ~] = env.step(rl_scale_action(mode, a));
        o2 = cell(1,nAg);
        for i = 1:nAg, o2{i} = rl_agent_obs(mode, cfg.singleObs, out2, i); end
        if strcmp(mode,'td3'), rv = r.glob; else, rv = [r.L; r.D]; end
        % 'done' used in the bootstrap only for unsafe/diverged terminations (not for time-outs)
        terminal = done && ~(env.k >= env.cfg.episodeSteps);
        buf.add(out.sGlobal, vertcat(o{:}), a, rv, out2.sGlobal, vertcat(o2{:}), double(terminal));
        out = out2;  stepCount = stepCount + 1;
        epR = epR + rv(:).';  e2 = e2 + out.raw.err3d^2;
        h2 = h2 + (mod(out.raw.psi - out.raw.psir + pi, 2*pi) - pi)^2;  nSt = nSt + 1;
        if buf.n >= max(cfg.batch, cfg.warmup), update(stepCount); end
    end
    logs.episodeReward(ep,:) = epR;  logs.rmse(ep) = sqrt(e2/nSt);
    logs.headingErr(ep) = rad2deg(sqrt(h2/nSt));  logs.stage(ep) = stage;  logs.steps(ep) = nSt;
    if cfg.verbose > 0 && (mod(ep,cfg.verbose) == 0 || ep == 1)
        fprintf('Ep %5d/%d | stage %d | R = %9.2f | 3D RMSE = %7.2f m | psi err = %6.2f deg | %6.1f min\n', ...
            ep, M, stage, sum(epR), logs.rmse(ep), logs.headingErr(ep), toc(tStart)/60);
    end
    if ~isempty(cfg.saveFile) && (mod(ep,cfg.saveEvery) == 0 || ep == M)
        agent = packAgent(); %#ok<NASGU>
        d = fileparts(cfg.saveFile); if ~isempty(d) && ~exist(d,'dir'), mkdir(d); end
        save(cfg.saveFile, 'agent', 'logs', 'cfg');
    end
end
agent = packAgent();

% =======================  nested helpers  ============================================
    function update(it)
        b  = buf.sample(cfg.batch);
        S  = toDl(b.S, useGPU);  S2 = toDl(b.S2, useGPU);
        Oc = cell(1,nAg); O2c = cell(1,nAg); Ac = cell(1,nAg);
        for i = 1:nAg
            Oc{i}  = toDl(b.O(obsOff(i)+1:obsOff(i+1),:),  useGPU);
            O2c{i} = toDl(b.O2(obsOff(i)+1:obsOff(i+1),:), useGPU);
            Ac{i}  = toDl(b.A(actOff(i)+1:actOff(i+1),:),  useGPU);
        end
        Dn = toDl(b.D, useGPU);
        % ---- target actions with clipped smoothing noise, Eq.(86) ---------------------
        aT = cell(1,nAg);
        for j = 1:nAg
            mu  = predict(actorsT{j}, O2c{j});
            nz  = min(max(cfg.tgtStd*randn(size(mu),'single'), -cfg.tgtClip), cfg.tgtClip);
            if useGPU, nz = gpuArray(nz); end
            aT{j} = min(max(mu + dlarray(nz,'CB'), -1), 1);
        end
        % ---- critic updates, Eq.(84)-(85) ----------------------------------------------
        for i = 1:nAg
            qt1 = predict(criticsT{i,1}, criticInput(mode, i, S2, O2c, aT));
            qt2 = predict(criticsT{i,2}, criticInput(mode, i, S2, O2c, aT));
            Ri  = toDl(b.R(i,:), useGPU);
            y   = Ri + cfg.gamma*(1 - Dn).*min(qt1, qt2);
            [~, g1, g2] = dlfeval(@criticGrad, critics{i,1}, critics{i,2}, criticInput(mode,i,S,Oc,Ac), y);
            [critics{i,1}, optC{i,1}.avg, optC{i,1}.sq] = adamupdate(critics{i,1}, g1, optC{i,1}.avg, optC{i,1}.sq, it, cfg.lr);
            [critics{i,2}, optC{i,2}.avg, optC{i,2}.sq] = adamupdate(critics{i,2}, g2, optC{i,2}.avg, optC{i,2}.sq, it, cfg.lr);
        end
        % ---- delayed actor + target updates, Eq.(87)-(88) -----------------------------
        if mod(it, cfg.policyDelay) == 0
            for i = 1:nAg
                [~, ga] = dlfeval(@actorGrad, actors{i}, critics{i,1}, mode, i, S, Oc, Ac);
                nUp = it/cfg.policyDelay;
                [actors{i}, optA{i}.avg, optA{i}.sq] = adamupdate(actors{i}, ga, optA{i}.avg, optA{i}.sq, nUp, cfg.lr);
                actorsT{i} = softUpdate(actorsT{i}, actors{i}, cfg.tau);
                for k = 1:2, criticsT{i,k} = softUpdate(criticsT{i,k}, critics{i,k}, cfg.tau); end
            end
        end
    end

    function ag = packAgent()
        ag.mode = mode;  ag.singleObs = cfg.singleObs;  ag.cfg = cfg;
        ag.actors = cellfun(@(n) dlupdate(@gather, n), actors, 'UniformOutput', false);
        ag.obsDims = obsDims;  ag.actDims = actDims;
    end
end

% =======================  local functions  ===========================================
function n = criticInDim(mode, i, nS, obsDims, actDims)
switch mode
    case 'matd3',       n = nS + sum(actDims);
    case 'independent', n = obsDims(i) + actDims(i);
    case 'td3',         n = obsDims(1) + actDims(1);
end
end

function x = criticInput(mode, i, S, Oc, Ac)
switch mode
    case 'matd3',       x = cat(1, S, Ac{:});          % Q_i(s_global, a_L, a_D)
    case 'independent', x = cat(1, Oc{i}, Ac{i});      % Q_i(o_i, a_i)
    case 'td3',         x = cat(1, Oc{1}, Ac{1});      % Q(S, a)
end
end

function [loss, g1, g2] = criticGrad(n1, n2, in, y)
q1 = forward(n1, in);  q2 = forward(n2, in);
loss = mean((q1 - y).^2, 'all') + mean((q2 - y).^2, 'all');      % Eq.(84)
[g1, g2] = dlgradient(loss, n1.Learnables, n2.Learnables);
end

function [loss, grad] = actorGrad(actor, critic, mode, i, S, Oc, Ac)
Ac{i} = forward(actor, Oc{i});                                    % a_i = pi_i(o_i)
q = forward(critic, criticInput(mode, i, S, Oc, Ac));             % other agents' actions from batch
loss = -mean(q, 'all');                                           % ascent on Eq.(87)
grad = dlgradient(loss, actor.Learnables);
end

function tgt = softUpdate(tgt, src, tau)
tgt.Learnables.Value = cellfun(@(t,s) tau*s + (1-tau)*t, ...
    tgt.Learnables.Value, src.Learnables.Value, 'UniformOutput', false);   % Eq.(88)
end

function d = toDl(x, useGPU)
x = single(x);  if useGPU, x = gpuArray(x); end
d = dlarray(x, 'CB');
end
