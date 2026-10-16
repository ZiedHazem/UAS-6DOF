function Pp = perturb_params(P, relAero, relMass, rs)
%PERTURB_PARAMS  Random parametric uncertainty, Eq.(75): Theta_i = Theta_i0 (1+eps_i).
%   +/-relAero (default 10%) on CLa, CD0, Cma and Ix, Iy, Iz, Ixz;  +/-relMass (5%) on m and CG.
if nargin < 2 || isempty(relAero), relAero = 0.10; end
if nargin < 3 || isempty(relMass), relMass = 0.05; end
if nargin < 4, rs = RandStream('mt19937ar','Seed',1); end
Pp = P;
for f = {'CLa','CD0','Cma','Ix','Iy','Iz','Ixz'}
    Pp.(f{1}) = P.(f{1})*(1 + relAero*(2*rand(rs)-1));
end
for f = {'m','xCG','zCG'}
    Pp.(f{1}) = P.(f{1})*(1 + relMass*(2*rand(rs)-1));
end
end
