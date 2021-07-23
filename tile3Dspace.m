function ROIs = tile3Dspace(ptCloudOriginal, input)

%%***********************************************************************%
%*                          3D space tiling                             *%
%* Divides the 3D space to tiles (or blocks) with overlappings          *%
%*                                                                      *%
%* Code author: Preetham Manjunatha                                     *%
%* Github link: https://github.com/preethamam
%* Date: 07/19/2021                                                     *%
%************************************************************************%
%
%************************************************************************%
%
% Usage: ROIs = tile3Dspace(ptCloud, input)
% Inputs: ptCloud   - Point cloud
%         input
%         input.zeroCenterAlign2XYZplane = 1; 0 -- none | 1 - PCA based | 2 - axis rotation based 
%                                             Zero centers the point cloud
%                                             data and aligns to XYZ planes
%         input.overlapStyle = 'unequalOverlap';  % 'unequalOverlap' | 'equalOverlap'
%         input.sequence = 'ZYZ';                 % Euler angle sequence: 'ZYX' (default) | 'ZYZ' | 'XYZ'
%         input.showPCAvectors  = 1;                0 - off | 1 - on
%         input.showPlaneViews  = 1                 0 - off | 1 - on
%         input.showGridPattern = 1;                0 - off | 1 - on
%         input.showPartitions  = 1;                0 - off | 1 - on
% 
%         input.saveFragments = 1;                  0 - off | 1 - on
%         input.zeroCenterFragments = 1;            0 - off | 1 - on
%         input.filename = fileName;                filename to save
%         input.fileSavePath = '';                  file oath to save
%         input.encodingType = 'compressed';        %'ascii' (default) | 'binary' | 'compressed'

%         input.removeFragmentOutliers = 1;         0 - off | 1 - on 
%         input.minDistance = 0.2;                  minimum distance to points to create 
%                                                   segmentation connected components
% 
%         input.overlapX = 0.5;         % Overlap % in X-direction
%         input.overlapY = 0.5;         % Overlap % in Y-direction
%         input.overlapZ = 0.5;         % Overlap % in Z-direction
% 
%         input.nBlocksX = 4;           % Number of blocks in X-direction
%         input.nBlocksY = 3;           % Number of blocks in X-direction
%         input.nBlocksZ = 2;           % Number of blocks in X-direction
%
% Outputs: ROIs - Region of interest, specified as a six-element vector of form 
%                 [xmin, xmax, ymin, ymax, zmin, zmax], where:
%                 xmin and xmax are the minimum and the maximum limits along the x-axis respectively.
%                 ymin and ymax are the minimum and the maximum limits along the y-axis respectively.
%                 zmin and zmax are the minimum and the maximum limits along the z-axis respectively.



% Check if pointCloud class
tf = isa(ptCloudOriginal,'pointCloud');

% Get X, Y and Z limits
if (tf ~= 1)
    error('Expected ptCloudIn to be one of these types: pointCloud. Instead its type was single/double.')
end

if (input.zeroCenterAlign2XYZplane == 1)

    % Zero center the point cloud data
    pcMean  = mean(ptCloudOriginal.Location, 1);
    pcZeroCentered = bsxfun(@minus, ptCloudOriginal.Location, pcMean);   

    % PCA for the plane Eigen vectors
    pcaVectors = pca(pcZeroCentered);
    pcaVectorsScaled = 10 * pcaVectors;

    % Get the rotation angles
    deg = zeros(1,3);
    for i = 1:3
        n = pcaVectors(:,i);
        planeVec = zeros(1,3);
        planeVec(i) = 1;
        n = n*sign(dot(n,planeVec)); %orientation convention
        deg(i) = acosd(dot(n,planeVec));
    end

    % Transformation matrix 
    Radians = deg2rad(deg);
    eul = Radians;
    rotMat = eul2rotm(eul, input.sequence);
    trans = [0 0 0];
    tform = rigid3d(rotMat,trans);

    % Convert to pointCloud class
    ptCloudNew = pointCloud(pcZeroCentered);
    ptCloudNew.Color =  ptCloudOriginal.Color;
    ptCloudNew = pctransform(ptCloudNew,tform);
    
    % Create the fitted plane meshgrid
    x = pcZeroCentered(:,1);
    y = pcZeroCentered(:,2);
    
    % Get normalized vector and plane equation constants
    if abs(pcaVectors(end,end)) < 1e-4
        NormalizedV = -1 * pcaVectors(:,end);
    else
        NormalizedV = -1/pcaVectors(end,end)* pcaVectors(:,end);
    end
    A = NormalizedV(1); B = NormalizedV(2); C = - mean(pcZeroCentered,1) * NormalizedV;

    % Meshgrid
    [X,Y] = meshgrid(linspace(min(x)/2,max(x)/2,20),linspace(min(y)/2,max(y)/2,20));
    Z     = (A*X)+(B*Y)+C;
    
    if (input.showPCAvectors == 1)
        figure; 
        if ~isempty(ptCloudOriginal.Color)
            pcshow(pcZeroCentered,  ptCloudOriginal.Color)
        else
            pcshow(pcZeroCentered)
        end
        hold on
        % PCA vectors
        quiver3(0,0,0,pcaVectorsScaled(1, 1),pcaVectorsScaled(2, 1),pcaVectorsScaled(3, 1), 'g', 'LineWidth', 4);
        quiver3(0,0,0,pcaVectorsScaled(1, 2),pcaVectorsScaled(2, 2),pcaVectorsScaled(3, 2), 'b', 'LineWidth', 4);
        quiver3(0,0,0,pcaVectorsScaled(1, 3),pcaVectorsScaled(2, 3),pcaVectorsScaled(3, 3), 'r', 'LineWidth', 4);

        % Scaled unit vectors
        quiver3(0,0,0, 10, 0, 0, '--g', 'LineWidth', 4);
        quiver3(0,0,0, 0, 10, 0, '--b', 'LineWidth', 4);
        quiver3(0,0,0, 0, 0, 10, '--r', 'LineWidth', 4);
        grid on;
        
        % Create plane
        surf(X,Y,Z,'FaceColor','m'); alpha(0.5);
        hold off

        set(gcf,'color','w');
        set(gca,'color','w');
        set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])

        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('PCA Vectors and fitted plane', 'Color','black');
    end
    
    if (input.showPlaneViews == 1)
        figure; 
        subplot(1,3,1)
        if ~isempty(ptCloudOriginal.Color)
            pcshow(ptCloudNew.Location,  ptCloudOriginal.Color);
        else
            pcshow(ptCloudNew.Location)
        end
        set(gcf,'color','w');
        set(gca,'color','w');
        set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])

        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('XY plane', 'Color','black');
        view(0,90)  % XY
        
        subplot(1,3,2)
        if ~isempty(ptCloudOriginal.Color)
            pcshow(ptCloudNew.Location,  ptCloudOriginal.Color)
        else
            pcshow(ptCloudNew.Location)
        end
        set(gcf,'color','w');
        set(gca,'color','w');
        set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
 
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('XZ plane', 'Color','black');
        view(0,0)   % XZ
        
        subplot(1,3,3)
        if ~isempty(ptCloudOriginal.Color)
            pcshow(ptCloudNew.Location,  ptCloudOriginal.Color)
        else
            pcshow(ptCloudNew.Location)
        end
        set(gcf,'color','w');
        set(gca,'color','w');
        set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
        
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('YZ plane', 'Color','black');
        view(90,0)  % YZ
    end
    
    % Create the point cloud copy
    ptCloud = ptCloudNew;
    
elseif input.zeroCenterAlign2XYZplane == 2
    % Zero center the point cloud data
    pcMean  = mean(ptCloudOriginal.Location, 1);
    pcZeroCentered = bsxfun(@minus, ptCloudOriginal.Location, pcMean); 
    
    % Find the axis-orientation of the point cloud
    OOBB = orientedBox3d(double(pcZeroCentered));

    % Transformation matrix
    deg = [OOBB(7)  OOBB(8)  OOBB(9)];
    mat1 = eulerAnglesToRotation3d(deg,'ZYX');
    rotMatNew = inv(mat1);
    ptCloudNew.Location = transformPoint3d(pcZeroCentered, rotMatNew);

    % Convert to pointCloud class
    ptCloud = pointCloud(ptCloudNew.Location);
    ptCloud.Color = ptCloudOriginal.Color;
else
    
    % Create the point cloud copy
    ptCloud = ptCloudOriginal;
end

% Get X, Y and Z limits
if (tf == 1)
    xLims = ptCloud.XLimits;
    yLims = ptCloud.YLimits;
    zLims = ptCloud.ZLimits;
else
    xLims = [min(ptCloud(:,1)) max(ptCloud(:,1))];
    yLims = [min(ptCloud(:,2)) max(ptCloud(:,2))];
    zLims = [min(ptCloud(:,3)) max(ptCloud(:,3))];
end

% Total lengths of model
lenX = xLims(2) - xLims(1);
lenY = yLims(2) - yLims(1);
lenZ = zLims(2) - zLims(1);

% Split lengths of X, Y and Z directions
lenXsplit = lenX/input.nBlocksX;
lenYsplit = lenY/input.nBlocksY;
lenZsplit = lenZ/input.nBlocksZ;

% Overlap lengths of X, Y and Z directions
overlapX =  lenXsplit * input.overlapX;
overlapY =  lenYsplit * input.overlapY;
overlapZ =  lenZsplit * input.overlapZ;

% Block lengths in X, Y and Z directions
switch input.overlapStyle
    case 'unequalOverlap'
        if input.nBlocksX > 1
           [Xblocks, lenXmin, lenXmax] = unequalOverlap (xLims, lenXsplit, overlapX, input.nBlocksX);
        else
           lenXmax = xLims(2);
           lenXmin = xLims(1);
           Xblocks = [lenXmin lenXmax];
        end

        if input.nBlocksY > 1
           [Yblocks, lenYmin, lenYmax] = unequalOverlap (yLims, lenYsplit, overlapY, input.nBlocksY);
        else
           lenYmax = yLims(2);
           lenYmin = yLims(1);
           Yblocks = [lenYmin lenYmax];
        end

        if input.nBlocksZ > 1
           [Zblocks, lenZmin, lenZmax] = unequalOverlap (zLims, lenZsplit, overlapZ, input.nBlocksZ);
        else
           lenZmax = zLims(2);
           lenZmin = zLims(1);
           Zblocks = [lenZmin lenZmax];
        end
        
    case 'equalOverlap'
        
        if input.nBlocksX > 1
           [Xblocks, lenXmin, lenXmax] = equalOverlap (xLims, lenXsplit, overlapX, input.nBlocksX);           
        else
           lenXmax = xLims(2);
           lenXmin = xLims(1);
           Xblocks = [lenXmin lenXmax];
        end

        if input.nBlocksY > 1
           [Yblocks, lenYmin, lenYmax] = equalOverlap (yLims, lenYsplit, overlapY, input.nBlocksY);           
        else
           lenYmax = yLims(2);
           lenYmin = yLims(1);
           Yblocks = [lenYmin lenYmax];
        end

        if input.nBlocksZ > 1
           [Zblocks, lenYmin, lenYmax] = equalOverlap (zLims, lenZsplit, overlapZ, input.nBlocksZ);           
        else
           lenZmax = zLims(2);
           lenZmin = zLims(1);
           Zblocks = [lenZmin lenZmax];
        end
    
end

% Create ROIs for the partitions
ROIs = zeros(input.nBlocksX*input.nBlocksY*input.nBlocksZ, 6);
cnt = 1;
for i = 1:2:2*input.nBlocksX 
    for j = 1:2:2*input.nBlocksY 
        for k = 1:2:2*input.nBlocksZ
            ROIs(cnt,:) = [Xblocks(:,i:i+1), Yblocks(:,j:j+1), Zblocks(:,k:k+1)];            
            cnt = cnt + 1;
        end
    end    
end

% Plot the grid points
if (input.showGridPattern == 1)
    figure;
    for i=1:numel(Yblocks)
        for j=1:numel(Zblocks)
            plot3(Xblocks, Yblocks(i) * ones(size(Xblocks)), Zblocks(j) * ones(size(Xblocks)),'*')
            hold on
        end
    end
    hold off
    grid on;
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
end
        
% Plot the partitions
if (input.showPartitions == 1)
    figure
    for i = 1:size(ROIs,1)
        roi = ROIs(i,:);
        cubelenX = roi(2) - roi(1);
        cubelenY = roi(4) - roi(3);
        cubelenZ = roi(6) - roi(5);

        cubeOrigin = [roi(1)  roi(3)  roi(5)];

        indices = findPointsInROI(ptCloud,roi);
        ptCloudB = select(ptCloud,indices);  
        
        fragmentColor = rand(1,3);
        if ~(isempty(ptCloudB.Color))
            pcshow(ptCloudB.Location, ptCloudB.Color);
        else
            pcshow(ptCloudB.Location, fragmentColor);
        end
        hold on
        grid on;
        plotcube([cubelenX cubelenY cubelenZ], cubeOrigin, 0.05, fragmentColor);
        set(gcf,'color','w');
        set(gca,'color','w');
        set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])

        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        
%         view(2)
        drawnow;
    end
    hold off
end

% Save the fragments
if (input.saveFragments == 1)
    for i = 1:size(ROIs,1)
        roi = ROIs(i,:);    
        indices = findPointsInROI(ptCloud,roi);
        fragmentOriginal = select(ptCloud,indices);

        if (input.zeroCenterFragments == 1)
            fragmentColor = fragmentOriginal.Color;

            % Zero center the point cloud data
            fragMean = mean(fragmentOriginal.Location, 1);
            fragmentZerocenter = bsxfun(@minus, fragmentOriginal.Location, fragMean);

            fragment = pointCloud(fragmentZerocenter);
            fragment.Color = fragmentColor;
        else
            fragment = fragmentOriginal;
        end
        
        % Remove outliers
        if(input.removeFragmentOutliers == 1)
            
            % Get labels for points
            [labels,numClusters] = pcsegdist(fragment, input.minDistance);
            N = histcounts(labels);
            [maxLabelNum, idx] = max(N); 
            maxClusterIndex = find(labels == idx);

            loc = fragment.Location(maxClusterIndex,:);           
            if ~isempty(ptCloudOriginal.Color)
                clr = fragment.Color(maxClusterIndex,:);
            else
                clr = uint8([]);
            end

            fragment = pointCloud(loc);
            fragment.Color = clr;
        end

        % Save fragments
        [filepath,name,ext] = fileparts(input.filename);
        outputBaseFileName = [name, '_' num2str(i) ext];    
        fileName = fullfile(input.fileSavePath, outputBaseFileName);
        pcwrite(fragment, fileName, 'Encoding', input.encodingType)
    end

end
end
%--------------------------------------------------------------------------------------------------
% Auxillary functions
%--------------------------------------------------------------------------------------------------
% Unequal overlap
function [blocks, lenmin, lenmax] = unequalOverlap (xyzLims, lensplit, overlap, nBlocks)
    lenmax = [xyzLims(1) : lensplit + overlap : xyzLims(2), xyzLims(2)];  
    lenmax = lenmax(2:end);

    lenmin = xyzLims(1) : lensplit : xyzLims(2);
    lenmin = lenmin(1:end-1);
    
    coords = unique(sort([lenmax lenmin]));
    
    lensplits = xyzLims(1):lensplit:xyzLims(2);
    blocks = zeros(1,nBlocks*2);

    blocks(1) = lensplits(1);
    blocks(end) = lensplits(end);
    blocks(2:2:end-2) = coords(3:1:end-1);
    blocks(3:2:end-1) = coords(2:1:end-2);
end

% Equal overlap
function [blocks, lenmin, lenmax] = equalOverlap (xyzLims, lensplit, overlap, nBlocks)
           
    lensplits = xyzLims(1):lensplit:xyzLims(2);
    maxx = lensplits(2:end-1)+overlap/2;
    minn = lensplits(2:end-1)-overlap/2;

    blocks = zeros(1,nBlocks*2);

    blocks(1) = lensplits(1);
    blocks(end) = lensplits(end);
    blocks(2:2:end-2) = maxx;
    blocks(3:2:end-1) = minn;

    lenmax = [maxx lensplits(end)];
    lenmin = [lensplits(1) minn];
end

function plotcube(varargin)

    % Cite As
    % Olivier (2021). PLOTCUBE (https://www.mathworks.com/matlabcentral/fileexchange/15161-plotcube),
    % MATLAB Central File Exchange. Retrieved July 19, 2021.

    % PLOTCUBE - Display a 3D-cube in the current axes
    %
    %   PLOTCUBE(EDGES,ORIGIN,ALPHA,COLOR) displays a 3D-cube in the current axes
    %   with the following properties:
    %   * EDGES : 3-elements vector that defines the length of cube edges
    %   * ORIGIN: 3-elements vector that defines the start point of the cube
    %   * ALPHA : scalar that defines the transparency of the cube faces (from 0
    %             to 1)
    %   * COLOR : 3-elements vector that defines the faces color of the cube
    %
    % Example:
    %   >> plotcube([5 5 5],[ 2  2  2],.8,[1 0 0]);
    %   >> plotcube([5 5 5],[10 10 10],.8,[0 1 0]);
    %   >> plotcube([5 5 5],[20 20 20],.8,[0 0 1]);

    % Default input arguments
    inArgs = { ...
      [10 56 100] , ... % Default edge sizes (x,y and z)
      [10 10  10] , ... % Default coordinates of the origin point of the cube
      .7          , ... % Default alpha value for the cube's faces
      [1 0 0]       ... % Default Color for the cube
      };

    % Replace default input arguments by input values
    inArgs(1:nargin) = varargin;

    % Create all variables
    [edges,origin,alpha,clr] = deal(inArgs{:});

    XYZ = { ...
      [0 0 0 0]  [0 0 1 1]  [0 1 1 0] ; ...
      [1 1 1 1]  [0 0 1 1]  [0 1 1 0] ; ...
      [0 1 1 0]  [0 0 0 0]  [0 0 1 1] ; ...
      [0 1 1 0]  [1 1 1 1]  [0 0 1 1] ; ...
      [0 1 1 0]  [0 0 1 1]  [0 0 0 0] ; ...
      [0 1 1 0]  [0 0 1 1]  [1 1 1 1]   ...
      };

    XYZ = mat2cell(...
      cellfun( @(x,y,z) x*y+z , ...
        XYZ , ...
        repmat(mat2cell(edges,1,[1 1 1]),6,1) , ...
        repmat(mat2cell(origin,1,[1 1 1]),6,1) , ...
        'UniformOutput',false), ...
      6,[1 1 1]);


    cellfun(@patch,XYZ{1},XYZ{2},XYZ{3},...
      repmat({clr},6,1),...
      repmat({'FaceAlpha'},6,1),...
      repmat({alpha},6,1)...
      );

    view(3);
end
