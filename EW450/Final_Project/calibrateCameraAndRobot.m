% Input 1: prv - camera preview object.
% Input 2: ur - initialized URQt object.
% Input 3: cameraParams - MATLAB camera parameters object.
% Input 4: nImages - positive scalar integer specifying the number of images to take during
% calibration.
% Output 1: H o2c - 4x4 array defining the UR3e base frame pose relative to the camera frame.
% Output 2: H c2o - 4x4 array defining the camera frame pose relative to the UR3e base frame.

function [H_o2c,H_c2o] = calibrateCameraAndRobot(prv,ur,cameraParams,nImages)
    
    % Define image base name, folder name, and number of images
    imBaseName = 'coCal';
    calFolderName = 'RobCamCal';
    nImages = 10;
    
    % Prompt user for checkerboard Square Size
    squareSize = inputdlg({'Enter square size in millimeters'},'SquareSize',...
    [1,35],{'10.00'});
    if numel(squareSize) == 0
        warning('Action cancelled by user.');
        out = [];
        return
    end
    squareSize = str2double( squareSize{1} );
    
    % Prompt the user to switch to "local control"
    f = msgbox('Set robot to local control.','Local Control');
    uiwait(f);
    
    % Create the calibration folder
    if ~isfolder(calFolderName)
        mkdir(calFolderName);
    end
    
    % Recover camera preview handles
    handles = recoverPreviewHandles(prv);
    
    % Initialize joint positions
    n = nImages;
    
    q = []; % <--- Joint configuration
    H_e2o = {}; % <--- End-effector pose
    fnames = {}; % <--- Image filenames
    for i = 1:n
        % Define the filename of the image
        fname = sprintf('%s%03d.png',imBaseName,i);
        % Bring preview to the foreground
        figure(handles.Figure);
        % Prompt user to move arm
        msg = sprintf(['Use the "Teach" button to move the arm to a new ',...
            'configuration with the checkerboard fully in the camera FOV. ',...
            'Taking Image %d of %d.'],i,n);
        f = msgbox(msg,'Position for Image');
        uiwait(f);
    
        % Get the image from the preview
        im = get(prv,'CData');
        % Get the joint configuration from the robot
        q(:,end+1) = ur.Joints;
        % Get the end-effector pose from the robot
        H_e2o{end+1} = ur.Pose;
    
        % Show checkerboard on preview
        showCheckerboardOnPreview(prv,im);
        drawnow
    
        % Save the image
        fnames{i} = fullfile(calFolderName,fname);
        imwrite(im,fnames{i},'png');
    end
    
    % Clear checkerboard overlay
    clearPreview(prv);
    
    % Define the filename for the robot data
    fnameRobotInfo = 'URcoCalInfo.mat';
    save(fullfile(calFolderName,fnameRobotInfo),...
        'q','H_e2o','calFolderName','imBaseName','squareSize','cameraParams','fnameRobotInfo');
    
    % Recovering Fiducial Extrinsic Matrices and Corresponding Robot Configuration Information
    
    % Define p_m from the image of the checkerboard
    % NOTE: MATLAB's definition of "imagePoints" relates to p_m as follows:
    %   % Define "imagePoints" from p_m for image i
    %   imagePoints(:,:,i) = p_m(1:2,:).'; % <-- Note the transpose
    %   % Define p_m from "imagePoints" for image i
    %   p_m(1:2,:) = imagePoints(:,:,i).'; % <-- Note the transpose
    %   p_m(3,:) = 1; % <-- Convert to homogeneous, 2D coordinate 
    %                       pixel position
    
    [imagePoints, boardSize, tfImagesUsed] = ...
        detectCheckerboardPoints(fnames,'PartialDetections',false);
    
    % Update list of images used
    fnames_Used = fnames(tfImagesUsed);
    
    % Update corresponding pose and joint configurations
    H_e2o_Used = H_e2o(tfImagesUsed);
    q_Used = q(:,tfImagesUsed);
    
    % Define p f given "boardSize" and "squareSize"
    % NOTE: MATLAB's definition of "worldPoints" relates to p f as follows:
    %   % Define "worldPoints" from p_f
    %   worldPoints = p_f(1:2,:).'; % <-- Note the transpose
    %   % Define p_f from "worldPoints"
    %   p_f(1:2,:) = worldPoints.'; % <-- Note the transpose
    %   p_f(3,:) = 0; % <-- Define z-coordinate
    %   p_f(4,:) = 1; % <-- Convert to homogeneous, 3D
    %   % coordinate relative to the
    %   % fiducial frame
    [worldPoints] = generateCheckerboardPoints(boardSize,squareSize);
    
    % Recover the checkerboard pose relative to the camera frame (H_f2c)
    H_f2c_Used = {};
    for i = 1:size(imagePoints,3)
        % Recover extrinsic information for the ith "used" image
        [R_c2f, tpose_d_f2c] = extrinsics(...
        imagePoints(:,:,i),worldPoints,cameraParams);
        R_f2c = R_c2f.';
        d_f2c = tpose_d_f2c.';
        H_f2c_Used{i} = [R_f2c, d_f2c; 0,0,0,1];
    end
    
    % Finding Useful Extrinsics (H_c2o)
    
    % Initialize parameters
    iter = 0;
    A_o = {};
    B_c = {};
    n = numel(H_f2c_Used);
    for i = 1:n
        for j = 1:n
            % Define:
            %   pose of fiducial in image i *relative to*
            %   pose of fiducial in image j
            H_oi2oj{i,j} = H_e2o_Used{j}*invSE( H_e2o_Used{i} );
            % Define:
            %   end-effector pose for image i *relative to*
            %   end-effector pose for image j
            H_ci2cj{i,j} = H_f2c_Used{j}*invSE( H_f2c_Used{i} );
            if i ~= j && i < j
                % Define transformation pairs to solve for H_c2o given:
                %   H_oi2oj * H_ci2oi = H_cj2oj * H_ci2cj
                %       where H_cj2oj = H_ci2oi = H_c2o
                %
                % We can rewrite this as
                %   A * X = X * B, solve for X
                iter = iter+1;
                A_o{iter} = H_oi2oj{i,j};
                B_c{iter} = H_ci2cj{i,j};
            end
        end
    end

    fprintf('Number of A/B pairs: %d\n',numel(A_o));
    
    % Solve AX = XB
    X = solveAXeqXBinSE(A_o,B_c);
    H_c2o = X;
    
    % Correct possible round-off error in the rotation matrix
    H_c2o = nearestSE(H_c2o);
    
    % Calculate inverse matrix
    H_o2c = inv(H_c2o);

    % Finding Validation Extrinsics (H_e2f)

    % Initialize parameters
    iter = 0;
    A_f = {};
    B_e = {};
    n = numel(H_f2c_Used);
    for i = 1:n
        for j = 1:n
            % Define:
            %   pose of fiducial in image i *relative to*
            %   pose of fiducial in image j
            H_fi2fj{i,j} = invSE( H_f2c_Used{j} )*H_f2c_Used{i};
            % Define:
            %   end-effector pose for image i *relative to*
            %   end-effector pose for image j
            H_ei2ej{i,j} = invSE( H_e2o_Used{j} )*H_e2o_Used{i};
            if i ~= j && i < j
                % Define transformation pairs to solve for H_e2f given:
                %   H_fi2fj * H_ei2fi = H_ej2fj * H_ei2ej
                %       where H_ej2fj = H_ei2fi = H_e2f
                %
                % We can rewrite this as
                %   A * X = X * B, solve for X
                iter = iter+1;
                A_f{iter} = H_fi2fj{i,j};
                B_e{iter} = H_ei2ej{i,j};
            end
        end
    end
    fprintf('Number of A/B pairs: %d\n',numel(A_f));
    
    % Solve AX = XB
    X = solveAXeqXBinSE(A_f,B_e);
    H_e2f = X;
    
    % Correct possible round-off error in the rotation matrix
    H_e2f = nearestSE(H_e2f);

    % Validating Your Calibration

    % Parse Intrinsic Matrix
    A_c2m = cameraParams.IntrinsicMatrix.'; % <-- Note the transpose

    % Recovering Body-fixed Fiducial Points
    % Define p_f from "worldPoints"
    p_f(1:2,:) = worldPoints.'; % <-- Note the transpose
    p_f(3,:) = 0; % <-- Define z-coordinate
    p_f(4,:) = 1; % <-- Convert to homogeneous, 3D
    
    % Reproject Body-fixed Fiducial Points
    for i = 1:n
        % Define filename
        fname = fnames_Used{i};
        % Load distorted image
        imDist = imread( fname );
        % Undistort image
        im = undistortImage(imDist,cameraParams);
        % Plot image
        fig = figure('Name',fname);
        axs = axes('Parent',fig);
        img = imshow(im,'Parent',axs);
        hold(axs,'on');
        axis(axs,'tight');
    
        % Converting H_c2o and H_f2c
        H_o2c = invSE( H_c2o );
        H_f2e = invSE( H_e2f );
        % Calculate extrinsics for image
        H_f2c_i = H_o2c * H_e2o_Used{i} * H_f2e;
        % Calculate projection matrix
        P_f2m = A_c2m * H_f2c_i(1:3,:);
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

end
