function P = nasnas_params()
%NASNAS_PARAMS  Parameters of the TAT Nasnas Mk.1 nonlinear 6-DOF model.
%   Values marked [T1] come from Table 1 of the paper, [TXT] from the paper text.
%   Values marked [ASSUMED] are NOT given in the paper and were chosen so that the
%   trim point and the linear modes reproduce those reported in Section 2.14
%   (alpha=2.32 deg, de=-0.69 deg, dT=0.389; short period wn=8.03 rad/s, zeta=0.51;
%   phugoid wn=0.336 rad/s, zeta=0.063; Dutch roll wn=4.61 rad/s, zeta=0.14;
%   roll tau=0.18 s; spiral tau=37.8 s).  Replace them with identified values if available.

P.name = 'TAT Nasnas Mk.1';

% ---- geometry / mass [T1] -------------------------------------------------
P.Lfus = 2.80;  P.Hfus = 1.30;
P.b  = 3.80;    P.S = 2.05;   P.c = 0.539;   P.AR = P.b^2/P.S;
P.m  = 75;                                  % reference simulation mass [kg]
P.Ix = 18.60;   P.Iy = 15.20; P.Iz = 31.80; P.Ixz = 1.05;   % [kg m^2]
P.xCG = 1.12;   P.yCG = 0;    P.zCG = 0.08;
P.g  = 9.81;    P.rho = 1.225;
P.Tmax = 175;                               % maximum thrust [N]
P.Vtrim = 36.11;                            % cruise / trim speed [m/s] (130 km/h)

% ---- aerodynamic coefficients [T1] ---------------------------------------
P.CL0 = 0.25;  P.CLa = 5.0;
P.CD0 = 0.035; P.CDa = 0.12; P.CDa2 = 0.85;
P.Cm0 = 0.02;  P.Cma = -0.85; P.Cmq = -12.0; P.Cmde = -1.20;
P.CYb = -0.60;
P.Clb = -0.08; P.Clp = -0.30; P.Clda = 0.35;
P.Cnb = 0.10;  P.Cnr = -0.12; P.Cndr = -0.10;

% ---- aerodynamic coefficients NOT listed in the paper [ASSUMED] -----------
P.CLq  = 3.0;   P.CLde = 0.30;  P.CDde = 0.02;
P.CYp  = -0.05; P.CYr  = 0.15;  P.CYda = 0.0;  P.CYdr = 0.15;
P.Clr  = 0.05;  P.Cldr = 0.02;
P.Cnp  = -0.02; P.Cnda = -0.02;

% ---- actuators [TXT] ------------------------------------------------------
P.tau   = [0.05; 0.05; 0.05];    % elevator, aileron, rudder time constants [s]
P.dmax  = 0.35;                  % +/- deflection limit [rad]
P.drate = 1.0;                   % deflection-rate limit [rad/s]

% ---- Dryden turbulence [TXT] ----------------------------------------------
P.dry.sigma = [1.5; 1.0; 0.5];   % m/s
P.dry.L     = [100; 80; 50];     % m

% ---- safety limits used in r_safety, Eq.(99) [ASSUMED] -------------------
P.lim.alphaStall = deg2rad(15);
P.lim.betaMax    = deg2rad(20);
P.lim.phiMax     = deg2rad(60);
P.lim.thetaMax   = deg2rad(45);

% ---- reward weights [TXT, Sec. 3.4.4] ------------------------------------
P.rw.wX = 1.0; P.rw.wY = 1.0; P.rw.wZ = 1.5;
P.rw.wphi = 0.5; P.rw.wtheta = 0.5; P.rw.wpsi = 0.5;
P.rw.wu = 0.1;  P.rw.wv = 0.1; P.rw.ww = 0.1;      % wv, ww [ASSUMED]
P.rw.wde = 0.01; P.rw.wda = 0.01; P.rw.wdr = 0.01; P.rw.wdt = 0.001;
P.rw.lamL = 0.3; P.rw.lamD = 0.3;
P.rw.wZi = 1.0; P.rw.wVi = 0.5; P.rw.wthi = 0.5;
P.rw.wYi = 1.0; P.rw.wphii = 0.5; P.rw.wpsii = 0.5;
end
