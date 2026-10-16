classdef NasnasEnv < handle
%NASNASENV  Closed-loop simulation / RL environment for the TAT Nasnas Mk.1 (Sec. 3 and 4).
%   env = NasnasEnv(cfg);  out = env.reset(opts);  [out,r,done,info] = env.step(cmd);
%   cmd = [de da dr dT] in physical units (rad, rad, rad, 0..1).
%   out : struct with scaled observations  sGlobal(14), oL(8), oD(10), S25(25), o18(18),
%         raw tracking data and an 'info' struct for the analytic baseline controllers.
%   cfg fields (defaults in defaults()):
%     dt=0.01, episodeSteps=250, traj='line', uncertainty=false, wind='none'|'steady'|'pulses'|
%     'dryden'|'random', stage=3, rewardScale=0.01, maxErr=200
    properties
        P0; P; trim; cfg; dt; traj; k; xa; dT; wind; rs; stage; tsec; wLast; lastCmd
    end
    methods (Static)
        function c = defaults(c)
            d = struct('dt',0.01,'episodeSteps',250,'traj','line','uncertainty',false, ...
                       'wind','none','stage',3,'rewardScale',0.01,'maxErr',200,'kz',0.01,'h0',500);
            for f = fieldnames(d).', if ~isfield(c,f{1}), c.(f{1}) = d.(f{1}); end, end
        end
    end
    methods
        function obj = NasnasEnv(cfg)
            if nargin < 1, cfg = struct(); end
            obj.cfg = NasnasEnv.defaults(cfg);
            obj.dt  = obj.cfg.dt;
            obj.P0  = nasnas_params();
            obj.trim = nasnas_trim(obj.P0, obj.P0.Vtrim, obj.cfg.h0);
            obj.stage = obj.cfg.stage;
        end
        function setStage(obj, s), obj.stage = s; end

        function out = reset(obj, opts)
            if nargin < 2, opts = struct(); end
            g = @(f,d) fieldOr(opts,f,d);
            name  = g('traj', obj.cfg.traj);
            seed  = g('seed', randi(2^31-1));
            unc   = g('uncertainty', obj.cfg.uncertainty);
            wmode = g('wind', obj.cfg.wind);
            nSteps = g('episodeSteps', obj.cfg.episodeSteps);
            obj.cfg.episodeSteps = nSteps;
            obj.rs = RandStream('mt19937ar','Seed',seed);
            tr = obj.trim;
            obj.traj = make_trajectory(name, tr.V, obj.dt, nSteps+2, -obj.cfg.h0);
            obj.P = obj.P0;
            if unc, obj.P = perturb_params(obj.P0, 0.10, 0.05, obj.rs); end
            x0 = tr.x0;
            x0(1:3) = [obj.traj.X(1); obj.traj.Y(1); obj.traj.Z(1)];
            x0(9)   = obj.traj.psi(1);
            obj.xa  = [x0; tr.de; 0; 0];
            obj.dT  = tr.dT;  obj.lastCmd = [tr.de; 0; 0; tr.dT];
            obj.k = 1;  obj.tsec = 0;
            obj.initWind(wmode, seed, nSteps);
            obj.wLast = obj.wind.steady;
            out = obj.observe();
        end

        function [out, r, done, info] = step(obj, cmd)
            cmd = cmd(:);
            cmd(1:3) = min(max(cmd(1:3), -obj.P.dmax), obj.P.dmax);
            cmd(4)   = min(max(cmd(4), 0), 1);
            wE = obj.windEarth(obj.xa(1:12));
            obj.xa = nasnas_step(obj.xa, cmd, wE, obj.P, obj.dt);
            obj.dT = cmd(4);  obj.lastCmd = cmd;
            obj.k = obj.k + 1;  obj.tsec = (obj.k-1)*obj.dt;
            out = obj.observe();
            [r, unsafe] = obj.reward(out, cmd);
            timeout = obj.k >= obj.cfg.episodeSteps;
            diverged = ~all(isfinite(obj.xa)) || out.raw.err3d > obj.cfg.maxErr;
            done = unsafe || timeout || diverged;
            info = struct('unsafe',unsafe,'timeout',timeout,'diverged',diverged);
        end

        function out = observe(obj)
            P = obj.P;  x = obj.xa(1:12);  del = obj.xa(13:15);  tr = obj.traj;
            wrap = @(a) mod(a+pi, 2*pi) - pi;
            [VA, al, be] = nasnas_airdata(x, obj.wLast);
            k = min(obj.k, numel(tr.X));
            Xr = tr.X(k); Yr = tr.Y(k); Zr = tr.Z(k); psir = wrap(tr.psi(k));
            psid = tr.psidot(k); gam = tr.gam(k); Vr = tr.V;
            Xe = x(1)-Xr; Ye = x(2)-Yr; Ze = x(3)-Zr; Ve = VA - Vr;
            ph = x(7); th = x(8); psi = wrap(x(9)); p = x(10); q = x(11); r = x(12);
            thr = obj.trim.theta + gam + min(max(-obj.cfg.kz*Ze, -0.15), 0.15);
            phr = min(max(atan(Vr*psid/P.g) + 0.5*wrap(psir-psi), -0.7), 0.7);
            ec  = -(x(1)-Xr)*sin(psir) + (x(2)-Yr)*cos(psir);       % cross-track error
            % ----- scaled observations (scaling constants are [ASSUMED]) ----------
            sG = [x(1)/1e3; x(2)/1e3; x(3)/1e3; x(4)/40; x(5)/10; x(6)/10; ph; th; psi/pi; p; q; r; al; be];
            oL = [Ze/50; Ve/5; th; q; thr; Vr/40; del(1); obj.dT];                       % Eq.(90)
            oD = [Ye/50; ph; psi/pi; p; r; be; phr; psir/pi; del(2); del(3)];            % Eq.(91)
            S25 = [sG; Xe/50; Ye/50; Ze/50; Ve/5; thr; phr; psir/pi; Vr/40; del(1); del(2); del(3)];
            out.sGlobal = sG;  out.oL = oL;  out.oD = oD;  out.S25 = S25;  out.o18 = [oL; oD];
            out.raw = struct('Xe',Xe,'Ye',Ye,'Ze',Ze,'Ve',Ve,'err3d',sqrt(Xe^2+Ye^2+Ze^2), ...
                'phi',ph,'theta',th,'psi',psi,'phir',phr,'thetar',thr,'psir',psir,'alpha',al,'beta',be, ...
                'VA',VA,'ec',ec,'u',x(4),'v',x(5),'w',x(6),'p',p,'q',q,'r',r);
            out.t = obj.tsec;
            out.info = struct('x',x,'delta',del,'dT',obj.dT,'alpha',al,'beta',be,'VA',VA, ...
                'Xr',Xr,'Yr',Yr,'Zr',Zr,'psir',psir,'psidot',psid,'gam',gam,'Vr',Vr, ...
                'Zdotr',-Vr*sin(gam),'ec',ec,'trim',obj.trim,'P',P,'t',obj.tsec);
        end

        function [r, unsafe] = reward(obj, out, cmd)
            %  Eq.(94)-(101) with stage-dependent weighting (curriculum, Sec. 4.1.3)
            w = obj.P0.rw;  lim = obj.P0.lim;  R = out.raw;
            wrap = @(a) mod(a+pi, 2*pi) - pi;
            switch obj.stage, case 1, mp = 0.5; ma = 2.0; otherwise, mp = 1.0; ma = 1.0; end
            rpos = -(w.wX*R.Xe^2 + w.wY*R.Ye^2 + w.wZ*R.Ze^2) * mp;
            ratt = -(w.wphi*(R.phi-R.phir)^2 + w.wtheta*(R.theta-R.thetar)^2 + w.wpsi*wrap(R.psi-R.psir)^2) * ma;
            uref = obj.traj.V*cos(obj.trim.theta);
            rvel = -(w.wu*(R.u-uref)^2 + w.wv*R.v^2 + w.ww*R.w^2);
            rctl = -(w.wde*cmd(1)^2 + w.wda*cmd(2)^2 + w.wdr*cmd(3)^2 + w.wdt*cmd(4)^2);
            rsaf = 0; unsafe = false;
            if abs(R.alpha) > lim.alphaStall || abs(R.beta) > lim.betaMax
                rsaf = -100; unsafe = true;
            elseif abs(R.phi) > lim.phiMax || abs(R.theta) > lim.thetaMax
                rsaf = -50; unsafe = true;
            end
            rg = rpos + ratt + rvel + rctl + rsaf;                                        % Eq.(94)
            rL = rg + w.lamL*(-w.wZi*R.Ze^2 - w.wVi*R.Ve^2 - w.wthi*(R.theta-R.thetar)^2);  % Eq.(100)
            rD = rg + w.lamD*(-w.wYi*R.Ye^2 - w.wphii*(R.phi-R.phir)^2 - w.wpsii*wrap(R.psi-R.psir)^2); % Eq.(101)
            s = obj.cfg.rewardScale;
            r = struct('glob',rg*s,'L',rL*s,'D',rD*s);
        end

        function initWind(obj, mode, seed, nSteps)
            w.mode = mode;  w.steady = zeros(3,1);  w.pulses = zeros(0,4);
            switch mode
                case 'none'
                case 'steady',  w.steady = [5;3;2];
                case 'pulses'   % Sec. 4.3: V_W0=[5 3 2] m/s + 5-10 m/s pulses of 0.5-2 s on each axis
                    w.steady = [5;3;2];
                    T = nSteps*obj.dt;  tt = linspace(0.15*T, 0.85*T, 5);
                    for i = 1:numel(tt)
                        for ax = 1:3
                            amp = (5 + 5*rand(obj.rs))*sign(rand(obj.rs)-0.5);
                            w.pulses(end+1,:) = [tt(i)+0.3*ax, 0.5+1.5*rand(obj.rs), amp, ax]; %#ok<AGROW>
                        end
                    end
                case 'dryden'
                    w.g = dryden_init(obj.traj.V, obj.P.dry.sigma, obj.P.dry.L, obj.dt, seed);
                case 'random'   % training stage 3: steady wind 0-10 m/s, random heading, + Dryden gusts
                    U = 10*rand(obj.rs);  a = 2*pi*rand(obj.rs);
                    w.steady = [U*cos(a); U*sin(a); 0];
                    w.g = dryden_init(obj.traj.V, obj.P.dry.sigma, obj.P.dry.L, obj.dt, seed);
                otherwise, error('Unknown wind mode "%s"', mode);
            end
            obj.wind = w;
        end

        function wE = windEarth(obj, x)
            w = obj.wind;  wE = w.steady;
            if isfield(w,'g')
                [obj.wind.g, gB] = dryden_step(w.g);
                wE = wE + nasnas_rot(x(7),x(8),x(9))*gB;
            end
            for i = 1:size(w.pulses,1)
                if obj.tsec >= w.pulses(i,1) && obj.tsec < w.pulses(i,1)+w.pulses(i,2)
                    wE(w.pulses(i,4)) = wE(w.pulses(i,4)) + w.pulses(i,3);
                end
            end
            obj.wLast = wE;
        end
    end
end

function v = fieldOr(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
