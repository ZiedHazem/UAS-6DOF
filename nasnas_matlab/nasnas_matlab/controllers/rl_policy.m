function fcn = rl_policy(agent)
%RL_POLICY  Wrap trained actors into a controller handle  [cmd, st] = fcn(out, st, dt).
%   Execution is decentralised (Sec. 3.4.5): each actor only sees its own observation.
nA = numel(agent.actors);
fcn = @policy;
    function [cmd, st] = policy(out, st, ~)
        a = zeros(0,1);
        for i = 1:nA
            o = rl_agent_obs(agent.mode, agent.singleObs, out, i);
            ai = predict(agent.actors{i}, dlarray(single(o), 'CB'));
            a = [a; double(extractdata(gather(ai)))]; %#ok<AGROW>
        end
        cmd = rl_scale_action(agent.mode, a);
    end
end
