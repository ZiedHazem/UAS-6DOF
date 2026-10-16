function cmd = rl_scale_action(mode, a)
%RL_SCALE_ACTION  Map normalised actions in [-1,1] to [de da dr dT] (rad, rad, rad, 0..1).
%   matd3/independent: a = [de_n dT_n | da_n dr_n]  (a_L = [de dt], a_D = [da dr], Eq.92-93)
%   td3             : a = [de_n da_n dr_n dT_n]
a = max(min(a(:), 1), -1);  dm = 0.35;
switch mode
    case {'matd3','independent'}
        cmd = [dm*a(1); dm*a(3); dm*a(4); (a(2)+1)/2];
    case 'td3'
        cmd = [dm*a(1); dm*a(2); dm*a(3); (a(4)+1)/2];
end
end
