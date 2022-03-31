%% Copyright 2022 The MathWorks, Inc.
%% Init
clear functions;
proj = currentProject;

%% delete temporary files
cd(proj.RootFolder + filesep + "Cache");
try
    rmdir('*','s');
catch
    % Do Nothing
end

list = dir;
for i = 1:numel(list)
    if ~strcmp(list(i).name, 'readme_cache.txt')
        delete(list(i).name);
    end
end

%% Terminate
cd(proj.RootFolder);

allDocs = matlab.desktop.editor.getAll;
allDocs.close;
clear all;
bdclose all;
clc;
