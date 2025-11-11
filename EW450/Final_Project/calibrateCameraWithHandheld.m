% CALIBRATECAMERAWITHHANDHELD calibrates an initialized camera with a specified
% number of images.
% Input 1: prv - camera preview object.
% Input 2: nImages - positive scalar integer specifying the number of images to take during
% calibration.
% Output 1: cameraParams - MATLAB camera parameters object
    
function [cameraParams] = calibrateCameraWithHandheld(prv,nImages)

    % Define image base name
    imBaseName = 'im';
    % Define calibration folder name
    calFolderName = 'HandheldCal';
    
    % Get calibration images
    getCalibrationImages(prv,imBaseName,calFolderName,nImages);
    
    % Calibrate the camera
    [cameraParams,imagesUsed] = calibrateCamera(imBaseName,calFolderName,nImages);
    
    % Validate calibration visually
    % Parse Intrinsic Matrix
    A_c2m = cameraParams.IntrinsicMatrix.'; % <-- Note the transpose
    % Parse Extrinsic Matrices
    n = cameraParams.NumPatterns; % <-- Total number of calibration images
    for i = 1:n
        R_f2c = cameraParams.RotationMatrices(:,:,i).'; % <-- Note the transpose
        d_f2c = cameraParams.TranslationVectors(i,:).'; % <-- Note the transpose
        H_f2c{i} = [R_f2c, d_f2c; 0,0,0,1]; % <-- Each extrinsic matrix is contained in a cell
    end
    
    % Recover Body-fixed Fiducial Points
    p_f = cameraParams.WorldPoints.'; % Parse x/y fiducial coordinates (note the transpose)
    p_f(3,:) = 0; % Fiducial z-coordinates (fiducial is 2D, so z is 0)
    p_f(4,:) = 1; % Make points homogeneous
    
    % Reproject Body-fixed Fiducial Points
    for i = 1:n
        % Define filename
        imName = sprintf('%s%03d.png',imBaseName,imagesUsed(i));
        % Load distorted image
        imDist = imread( fullfile(calFolderName,imName) );
        % Undistort image
        im = undistortImage(imDist,cameraParams);
        % Plot image
        fig(i) = figure('Name',imName);
        axs = axes('Parent',fig(i));
        img = imshow(im,'Parent',axs);
        hold(axs,'on');
        axis(axs,'tight');
    
        % Calculate projection matrix
        P_f2m = A_c2m * H_f2c{i}(1:3,:);
        % Project points using intrinsics and extrinsics
        tilde_p_m = P_f2m*p_f; % <-- Scaled pixel coordinates
        p_m = tilde_p_m./tilde_p_m(3,:); % <-- Pixel coordinates
    
        % Plot points
        % - Fiducial origin point
        plt0 = plot(axs,p_m(1,1),p_m(2,1),'ys','LineWidth',3,'MarkerSize',8);
        % - All other fiducial points
        plti = plot(axs,p_m(1,2:end),p_m(2,2:end),'go','LineWidth',2,'MarkerSize',8);
    
        % Label points
        for j = 1:size(p_m,2)
            txt(j) = text(axs,p_m(1,j),p_m(2,j),sprintf('$p_{%d}^{m}$',j),...
                'Interpreter','Latex','Color','m','FontSize',14);
        end
    end
    
    % Define the filename for the robot data
    fnameHandInfo = 'HandCalInfo.mat';
    save(fullfile(calFolderName,fnameHandInfo),...
        'cameraParams','imBaseName','calFolderName','fnameHandInfo');
end