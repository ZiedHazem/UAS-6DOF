classdef ReplayBuffer < handle
%REPLAYBUFFER  Uniform experience replay storing (s, o, a, r, s', o', done).
    properties
        cap; n = 0; ptr = 0; S; O; A; R; S2; O2; D
    end
    methods
        function b = ReplayBuffer(cap, nS, nO, nA, nR)
            b.cap = cap;
            b.S = zeros(nS,cap,'single');  b.S2 = zeros(nS,cap,'single');
            b.O = zeros(nO,cap,'single');  b.O2 = zeros(nO,cap,'single');
            b.A = zeros(nA,cap,'single');  b.R = zeros(nR,cap,'single');
            b.D = zeros(1,cap,'single');
        end
        function add(b, s, o, a, r, s2, o2, d)
            b.ptr = mod(b.ptr, b.cap) + 1;
            b.S(:,b.ptr) = s;  b.O(:,b.ptr) = o;  b.A(:,b.ptr) = a;  b.R(:,b.ptr) = r;
            b.S2(:,b.ptr) = s2; b.O2(:,b.ptr) = o2; b.D(:,b.ptr) = d;
            b.n = min(b.n + 1, b.cap);
        end
        function m = sample(b, B)
            idx = randi(b.n, 1, B);
            m = struct('S',b.S(:,idx),'O',b.O(:,idx),'A',b.A(:,idx),'R',b.R(:,idx), ...
                       'S2',b.S2(:,idx),'O2',b.O2(:,idx),'D',b.D(:,idx));
        end
    end
end
