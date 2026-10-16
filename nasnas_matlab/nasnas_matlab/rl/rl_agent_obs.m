function o = rl_agent_obs(mode, singleObs, out, i)
%RL_AGENT_OBS  Local observation of agent i (CTDE: actors only use these at execution).
switch mode
    case {'matd3','independent'}
        if i == 1, o = out.oL; else, o = out.oD; end
    case 'td3'
        if strcmp(singleObs,'concat'), o = out.o18; else, o = out.S25; end
end
end
