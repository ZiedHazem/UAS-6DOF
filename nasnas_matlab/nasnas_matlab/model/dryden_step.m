function [g, gust] = dryden_step(g)
%DRYDEN_STEP  Advance the Dryden generator one step; gust = [ug;vg;wg] in body axes (m/s).
w = randn(g.rs,3,1)/sqrt(g.dt);
g.x1 = g.Ad1*g.x1 + g.Bd1*w(1);
gust = zeros(3,1);
gust(1) = g.K1*g.x1;
for ax = 2:3
    g.x{ax} = g.Ad{ax}*g.x{ax} + g.Bd{ax}*w(ax);
    gust(ax) = g.C{ax}*g.x{ax};
end
end
