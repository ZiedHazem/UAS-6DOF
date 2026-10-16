function startup_nasnas()
%STARTUP_NASNAS  Add all sub-folders of this package to the MATLAB path.
root = fileparts(mfilename('fullpath'));
addpath(genpath(root));
fprintf('NASNAS package added to path: %s\n', root);
end
