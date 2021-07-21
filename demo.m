%//%************************************************************************%
%//%*                              Ph.D                                    *%
%//%*                         3D space tile             				   *%
%//%*                                                                      *%
%//%*             Name: Preetham Manjunatha             		           *%
%//%*             Github link: https://github.com/preethamam               *%
%//%*             Written Date: 07/20/2021                                 *%
%//%************************************************************************%


%% Start
%--------------------------------------------------------------------------
clear; close all; clc;
clcwaitbarz = findall(0,'type','figure','tag','TMWWaitbar');
delete(clcwaitbarz);
Start = tic;

%%
% Inputs
fileName = 'teapot.ply';

% Read point cloud
ptCloud = pcread(fileName);

input.zeroCenterAlign2XYZplane = 0; 
input.overlapStyle = 'equalOverlap';  %'unequalOverlap' | 'equalOverlap'
input.sequence = 'ZYZ';               %'ZYX' (default) | 'ZYZ' | 'XYZ'
input.showPCAvectors  = 1;
input.showPlaneViews  = 1;
input.showGridPattern = 0;
input.showPartitions  = 1;

input.saveFragments = 0;
input.zeroCenterFragments = 0;
input.filename = fileName;
input.fileSavePath = '';
input.encodingType = 'ascii';    %'ascii' (default) | 'binary' | 'compressed'

input.overlapX = 0.5;
input.overlapY = 0.5;
input.overlapZ = 0.5;

input.nBlocksX = 2;
input.nBlocksY = 2;
input.nBlocksZ = 1;

%% Function callback
ROIs = tile3Dspace(ptCloud, input);

%% End parameters
%--------------------------------------------------------------------------
clcwaitbarz = findall(0,'type','figure','tag','TMWWaitbar');
delete(clcwaitbarz);
statusFclose = fclose('all');
if(statusFclose == 0)
    disp('All files are closed.')
end
Runtime = toc(Start);
disp(Runtime);
currtime = datetime('now');
display(currtime)
