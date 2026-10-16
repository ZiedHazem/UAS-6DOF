function [obsDims, actDims] = rl_dims(mode, singleObs)
%RL_DIMS  Observation / action dimensions per agent.
switch mode
    case {'matd3','independent'}, obsDims = [8 10]; actDims = [2 2];
    case 'td3'
        if strcmp(singleObs,'concat'), obsDims = 18; else, obsDims = 25; end
        actDims = 4;
    otherwise, error('Unknown RL mode "%s"', mode);
end
end
