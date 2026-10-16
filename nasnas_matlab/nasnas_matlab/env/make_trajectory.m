function tr = make_trajectory(name, V, dt, N, z0)
%MAKE_TRAJECTORY  Constant-speed reference trajectories used in Section 4 (NED, z down).
%   name : 'level' (stage-1 straight and level), 'line' (straight + 30 deg heading change), 'lawnmower', 'spiral', 'eight' (Gerono
%          lemniscate), 'helix' (helical climb)
%   The paper does not give the geometric dimensions, so the sizes below are [ASSUMED].
%   tr.X,Y,Z,psi,psidot,gam : 1xN arrays sampled every dt;  tr.V = V.
s = V*dt*(0:N-1);
switch lower(name)
    case 'level'                      % straight and level flight (curriculum stage 1)
        psi = zeros(size(s));  gam = zeros(size(s));
    case 'line'                       % straight line, smooth 30 deg heading change
        a = min(max((s-400)/300,0),1);
        psi = deg2rad(30)*(3*a.^2 - 2*a.^3);   gam = zeros(size(s));
    case 'lawnmower'                  % 400 m legs, 200 m radius 180 deg turns (left, right, ...)
        Lleg = 400; Rt = 200; per = 2*(Lleg + pi*Rt);
        p = mod(s, per);  psi = zeros(size(s));  gam = zeros(size(s));
        i2 = p>=Lleg & p<Lleg+pi*Rt;             psi(i2) = (p(i2)-Lleg)/Rt;
        i3 = p>=Lleg+pi*Rt & p<2*Lleg+pi*Rt;     psi(i3) = pi;
        i4 = p>=2*Lleg+pi*Rt;                    psi(i4) = pi - (p(i4)-(2*Lleg+pi*Rt))/Rt;
    case 'spiral'                     % expanding spiral, R(s) = R0 + kk*s
        R0 = 150; kk = 0.15;
        psi = (1/kk)*log(1 + kk*s/R0);  gam = zeros(size(s));
    case 'eight'                      % Gerono lemniscate x=a cos t, y=b sin t cos t
        a = 350; b = 175;
        t  = linspace(pi/2, pi/2+4*pi, 40001);
        dx = -a*sin(t);  dy = b*cos(2*t);
        arc = [0, cumsum(sqrt(dx(1:end-1).^2 + dy(1:end-1).^2).*diff(t))];
        tt  = interp1(arc, t, min(s, arc(end)), 'linear');
        psi = unwrap(atan2(b*cos(2*tt), -a*sin(tt)));
        psi = psi - psi(1);           % start with heading 0 (path rotated accordingly)
        gam = zeros(size(s));
    case 'helix'                      % helical climb, R = 250 m, 6 deg climb angle
        Rt = 250;  psi = s/Rt;  gam = deg2rad(6)*ones(size(s));
    otherwise
        error('Unknown trajectory "%s"', name);
end
psidot = gradient(psi, dt);
X = zeros(1,N); Y = zeros(1,N); Z = zeros(1,N); Z(1) = z0;
for k = 1:N-1
    X(k+1) = X(k) + V*dt*cos(gam(k))*cos(psi(k));
    Y(k+1) = Y(k) + V*dt*cos(gam(k))*sin(psi(k));
    Z(k+1) = Z(k) - V*dt*sin(gam(k));
end
tr = struct('name',name,'V',V,'dt',dt,'X',X,'Y',Y,'Z',Z,'psi',psi,'psidot',psidot,'gam',gam);
end
