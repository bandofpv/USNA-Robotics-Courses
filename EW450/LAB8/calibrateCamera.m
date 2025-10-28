function [cameraParams,imagesUsed] = calibrateCamera(imBaseName,calFolderName,nImages)
% CALIBRATECAMERA calibrates a camera given image, folder, and number of
% images information.
%   [cameraParams,imagesUsed] = calibrateCamera(imBaseName,calFolderName,nImages)
%
%   Input(s)
%       imBaseName - character array defining image base name (e.g. 'im')
%    calFolderName - character array defining calibration folder name
%          nImages - positive scalar value defining the number of
%                    calibration images
%
%   Output(s)
%     cameraParams - recovered MATLAB camera parameters
%       imagesUsed - 1xN array defining image indices used in calibration
%
%   M. Kutzer, 04Mar2021, USNA

%% Check input(s)
narginchk(3,3);

% TODO - Check inputs


%% Define images to process
for i = 1:nImages
    fname = sprintf('%s%03d.png',imBaseName,i);
    imageFileNames{i} = fullfile(calFolderName,fname);
end

% Initialize index array
imagesUsed = 1:nImages;

% Detect checkerboards in images
[imagePoints, boardSize, tfImagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(tfImagesUsed);
% Update list of indices used
imagesUsed = imagesUsed(tfImagesUsed);
% Images used
fprintf('Images with detected checkerboards: %d\n',numel(imagesUsed));

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Prompt user for Square Size
squareSize = inputdlg({'Enter square size in millimeters'},'SquareSize',...
    [1,35],{'12.70'});
if numel(squareSize) == 0
    warning('Action cancelled by user.');
    cal = [];
    return
end
squareSize = str2double( squareSize{1} );

% Generate coordinates of the corners of the squares
%   relative to the "fiducial frame"
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, tfImagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);
% Update list of indices used
imagesUsed = imagesUsed(tfImagesUsed);
% Images used
fprintf('Images used in calibration: %d\n',numel(imagesUsed));

% % View reprojection errors
% h1=figure; showReprojectionErrors(cameraParams);
% 
% % Visualize pattern locations
% h2=figure; showExtrinsics(cameraParams, 'CameraCentric');
% 
% % Display parameter estimation errors
% displayErrors(estimationErrors, cameraParams);
% 
% % For example, you can use the calibration data to remove effects of lens distortion.
% undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
