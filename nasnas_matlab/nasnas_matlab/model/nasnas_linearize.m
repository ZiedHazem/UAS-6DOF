function [A, B, modes] = nasnas_linearize(P, trim, verbose)
%NASNAS_LINEARIZE  Numerical Jacobians about trim and modal characteristics (Sec. 2.14).
if nargin < 3, verbose = true; end
x0 = trim.x0;  u0 = trim.u0;  n = 12;  m = 4;  w0 = zeros(3,1);
f = @(x,u) nasnas_dynamics(x, u, w0, P);
A = zeros(n,n); B = zeros(n,m);
for i = 1:n, h = 1e-6; dx = zeros(n,1); dx(i) = h; A(:,i) = (f(x0+dx,u0)-f(x0-dx,u0))/(2*h); end
for i = 1:m, h = 1e-6; du = zeros(m,1); du(i) = h; B(:,i) = (f(x0,u0+du)-f(x0,u0-du))/(2*h); end
lon = [4 6 11 8];   % u w q theta
lat = [5 10 12 7];  % v p r phi
ev_lon = eig(A(lon,lon));  ev_lat = eig(A(lat,lat));
modes = struct('lon',ev_lon,'lat',ev_lat);
if verbose
    fprintf('Trim: alpha=%.2f deg, de=%.2f deg, dT=%.3f (T=%.1f N), CL=%.3f\n', ...
        rad2deg(trim.alpha), rad2deg(trim.de), trim.dT, trim.T, trim.CL);
    print_modes('Longitudinal', ev_lon); print_modes('Lateral-directional', ev_lat);
end
end

function print_modes(name, ev)
fprintf('%s eigenvalues:\n', name);
for k = 1:numel(ev)
    l = ev(k);
    if imag(l) > 1e-9
        fprintf('  oscillatory: wn = %.3f rad/s, zeta = %.3f\n', abs(l), -real(l)/abs(l));
    elseif abs(imag(l)) <= 1e-9
        fprintf('  real: lambda = %.4f 1/s (tau = %.2f s)\n', real(l), -1/real(l));
    end
end
end
