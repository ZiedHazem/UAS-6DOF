function g = dryden_init(V, sigma, Lg, dt, seed)
%DRYDEN_INIT  Discrete-time Dryden turbulence generator (MIL-F-8785C forms), Sec. 2.11.
%   H_u = su*sqrt(2Lu/(pi V)) /(1+Lu/V s)
%   H_v,w = s*sqrt(L/(pi V)) (1+sqrt(3) L/V s)/(1+L/V s)^2
%   Driven by unit-PSD white noise (variance 1/dt in discrete time); exact ZOH discretisation.
if nargin < 5, seed = 0; end
g.dt = dt;  g.rs = RandStream('mt19937ar','Seed',seed);
tau = Lg(:)./V;
g.Ad1 = exp(-dt/tau(1));  g.Bd1 = 1 - g.Ad1;  g.K1 = sigma(1)*sqrt(2*Lg(1)/(pi*V));  g.x1 = 0;
g.Ad = cell(3,1); g.Bd = cell(3,1); g.C = cell(3,1); g.x = cell(3,1);
for ax = 2:3
    t  = tau(ax);
    A  = [0 1; -1/t^2 -2/t];  B = [0; 1/t^2];
    M  = expm([A B; 0 0 0]*dt);
    g.Ad{ax} = M(1:2,1:2);  g.Bd{ax} = M(1:2,3);
    g.C{ax}  = sigma(ax)*sqrt(Lg(ax)/(pi*V)) * [1, sqrt(3)*t];
    g.x{ax}  = [0;0];
end
end
