function res = run_episode(env, ctrl, opts)
%RUN_EPISODE  Simulate one episode with controller handle  [cmd, st] = ctrl(out, st, dt).
%   opts : fields passed to env.reset (traj, seed, uncertainty, wind, episodeSteps)
out = env.reset(opts);  st = [];  dt = env.dt;
N = env.cfg.episodeSteps;
H = struct('t',zeros(N,1),'err3d',zeros(N,1),'Ze',zeros(N,1),'psiErr',zeros(N,1),'Ve',zeros(N,1), ...
           'cmd',zeros(N,4),'x',zeros(N,12),'ref',zeros(N,3));
wrap = @(a) mod(a+pi, 2*pi) - pi;
n = 0;  done = false;  unsafe = false;
while ~done
    [cmd, st] = ctrl(out, st, dt);
    [out, ~, done, info] = env.step(cmd);
    n = n + 1;
    H.t(n) = out.t;  H.err3d(n) = out.raw.err3d;  H.Ze(n) = out.raw.Ze;
    H.psiErr(n) = wrap(out.raw.psi - out.raw.psir);  H.Ve(n) = out.raw.Ve;
    H.cmd(n,:) = cmd(:).';  H.x(n,:) = env.xa(1:12).';
    H.ref(n,:) = [env.traj.X(min(env.k,end)) env.traj.Y(min(env.k,end)) env.traj.Z(min(env.k,end))];
    unsafe = info.unsafe || info.diverged;
end
for f = fieldnames(H).', H.(f{1}) = H.(f{1})(1:n,:); end
res = H;  res.n = n;  res.failed = unsafe;
end
