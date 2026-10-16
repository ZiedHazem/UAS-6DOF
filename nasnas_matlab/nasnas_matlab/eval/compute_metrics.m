function m = compute_metrics(res, band)
%COMPUTE_METRICS  Tracking metrics used in Tables 5-11 (see README for definitions).
%   3D RMSE [m], heading RMSE [deg], altitude RMSE [m], speed RMSE [m/s], max overshoot [m]
%   (maximum 3D error), settling time [s] (last time 3D error exceeds 'band', default 2 m),
%   control effort = total variation of the four commands.
if nargin < 2, band = 2.0; end
m.rmse3d   = sqrt(mean(res.err3d.^2));
m.headDeg  = rad2deg(sqrt(mean(res.psiErr.^2)));
m.altRmse  = sqrt(mean(res.Ze.^2));
m.velRmse  = sqrt(mean(res.Ve.^2));
m.maxErr   = max(res.err3d);
idx = find(res.err3d > band, 1, 'last');
if isempty(idx), m.settle = 0; else, m.settle = res.t(idx); end
m.tv = sum(sum(abs(diff(res.cmd,1,1))));
m.failed = res.failed;
end
