function net = rl_make_net(inDim, outDim, hidden, tanhOut)
%RL_MAKE_NET  Fully connected ReLU network (Table 4: [256,256]); tanh output for actors.
layers = featureInputLayer(inDim, 'Normalization', 'none', 'Name', 'in');
for h = 1:numel(hidden)
    layers = [layers; fullyConnectedLayer(hidden(h), 'Name', sprintf('fc%d',h)); reluLayer('Name', sprintf('relu%d',h))]; %#ok<AGROW>
end
layers = [layers; fullyConnectedLayer(outDim, 'Name', 'out')];
if tanhOut, layers = [layers; tanhLayer('Name','tanh')]; end
net = dlnetwork(layers);
end
