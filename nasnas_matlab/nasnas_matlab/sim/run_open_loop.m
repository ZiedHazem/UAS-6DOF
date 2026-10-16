function out = run_open_loop()
%RUN_OPEN_LOOP  Open-loop response from trim to small control perturbations (Figs. 3-10, Sec. 2.14):
%   elevator doublet +0.5 deg (3-6 s) / -0.5 deg (6-9 s), aileron +0.5 deg (8-10 s),
%   rudder +0.5 deg (16-18 s), throttle held at trim; 30 s.
P = nasnas_params();  trim = nasnas_trim(P);
dt = 0.01;  T = 30;  N = round(T/dt);
xa = [trim.x0; trim.de; 0; 0];
hist = zeros(N+1, 15+4);  hist(1,:) = [xa.' trim.u0.'];
d2r = pi/180;
for k = 1:N
    t = (k-1)*dt;
    de = trim.de;  da = 0;  dr = 0;
    if t >= 3 && t < 6,   de = de + 0.5*d2r; end
    if t >= 6 && t < 9,   de = de - 0.5*d2r; end
    if t >= 8 && t < 10,  da = 0.5*d2r; end
    if t >= 16 && t < 18, dr = 0.5*d2r; end
    cmd = [de; da; dr; trim.dT];
    xa = nasnas_step(xa, cmd, zeros(3,1), P, dt);
    hist(k+1,:) = [xa.' cmd.'];
end
t = (0:N)*dt;  x = hist(:,1:12);
out.t = t;  out.x = x;  out.hist = hist;
figure('Name','Position');  plot(t, x(:,1), t, x(:,2), t, -x(:,3)); grid on
legend('X (north)','Y (east)','Altitude'); xlabel('t [s]'); ylabel('m'); title('Position (Fig. 3)');
figure('Name','Velocities');
subplot(2,1,1); plot(t, x(:,4:6)); grid on; legend('u','v','w'); ylabel('m/s'); title('Body velocities (Fig. 4)');
subplot(2,1,2); plot(t, sqrt(sum(x(:,4:6).^2,2)), t, trim.V*ones(size(t)),'--'); grid on
legend('V','V_{cr}'); xlabel('t [s]'); ylabel('m/s'); title('Airspeed (Fig. 5)');
figure('Name','Euler angles'); plot(t, rad2deg(x(:,7:9))); grid on
legend('\phi','\theta','\psi'); xlabel('t [s]'); ylabel('deg'); title('Euler angles (Fig. 6)');
figure('Name','Body rates'); plot(t, rad2deg(x(:,10:12))); grid on
legend('p','q','r'); xlabel('t [s]'); ylabel('deg/s'); title('Angular rates (Fig. 7)');
figure('Name','Controls'); plot(t, [rad2deg(hist(:,16:18)) hist(:,19)*10]); grid on
legend('\delta_e','\delta_a','\delta_r','10\delta_T'); xlabel('t [s]'); title('Commanded inputs (Fig. 8)');
end
