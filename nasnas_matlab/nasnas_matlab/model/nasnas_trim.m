function trim = nasnas_trim(P, V, h0)
%NASNAS_TRIM  Level-flight trim at speed V (default P.Vtrim), udot=wdot=qdot=0, theta=alpha.
if nargin < 2 || isempty(V),  V  = P.Vtrim; end
if nargin < 3 || isempty(h0), h0 = 500; end
res = @(z) resid(z, V, P, h0);
z0  = [0.04; -0.01; 0.4];
try
    z = fsolve(res, z0, optimoptions('fsolve','Display','off', ...
        'FunctionTolerance',1e-12,'StepTolerance',1e-12));
catch
    opts = optimset('Display','off','TolFun',1e-14,'TolX',1e-12,'MaxFunEvals',1e4,'MaxIter',1e4);
    z = fminsearch(@(zz) norm(res(zz))^2, z0, opts);     % fallback without Optimization Toolbox
    z = fminsearch(@(zz) norm(res(zz))^2, z, opts);
end
alpha = z(1); de = z(2); dT = z(3);
trim.alpha = alpha;  trim.theta = alpha;  trim.de = de;  trim.dT = dT;
trim.V = V;  trim.h0 = h0;  trim.T = P.Tmax*dT;
trim.x0 = [0; 0; -h0; V*cos(alpha); 0; V*sin(alpha); 0; alpha; 0; 0; 0; 0];
trim.u0 = [de; 0; 0; dT];
[~,aux] = nasnas_dynamics(trim.x0, trim.u0, zeros(3,1), P);
trim.CL = aux.CL;  trim.CD = aux.CD;
end

function r = resid(z, V, P, h0)
alpha = z(1); de = z(2); dT = z(3);
x = [0;0;-h0; V*cos(alpha);0;V*sin(alpha); 0;alpha;0; 0;0;0];
xd = nasnas_dynamics(x, [de;0;0;dT], zeros(3,1), P);
r = [xd(4); xd(6); xd(11)];
end
