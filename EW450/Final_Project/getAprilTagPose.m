% Input 1: im - undistorted image captured from camera.
% Input 2: cameraParams - MATLAB camera parameters object.
% Input 3: tagFamily - character array specifying the tag family.
% Input 4: tagID - scalar integer specifying the tag ID.
% Input 5: tagSize - positive scalar integer specifying the size/scale of the AprilTag.
% Output 1: H_a2c - 4x4 array specifying the pose of the AprilTag frame relative to the camera
% frame.

function [H_a2c] = getAprilTagPose(im,cameraParams,tagFamily,tagID,tagSize)

    % Initilize empy set
    H_a2c = [];
    
    % Parse Intrinsic Matrix
    A_c2m = cameraParams.IntrinsicMatrix.'; % <-- Note the transpose
    
    % Find desired AprilTag(s) in an image and estimate tag pose
    [id,loc,pose,detectedFamily] = readAprilTag(im,tagFamily,cameraParams,tagSize);
    
    % Plot image
    fig = figure;
    axs = axes('Parent',fig);
    hold(axs,'on');
    axis(axs,'tight');
    img = imshow(im,'Parent',axs);
    
    % Highlight and label AprilTags
    for i = 1:numel(id)
        switch id(i)
        case tagID
        % Highlight AprilTag(s) matching tagID in blue
        ptc(i) = patch(axs,'Vertices',loc(:,:,i),'Faces',1:4,...
        'EdgeColor','m','FaceColor','b','FaceAlpha',0.1);
    
        % Define AprilTag label
        str = sprintf('%s\nID: %03d',detectedFamily(i),id(i));
    
        % Label AprilTag
        ang = rad2deg( atan2(...
        loc(2,1,i) - loc(1,1,i),...
        loc(2,2,i) - loc(1,2,i)) - pi/2 );
        txt = text(axs,mean(loc(:,1,i)),mean(loc(:,2,i)),str,...
            'HorizontalAlignment','center','VerticalAlignment','bottom',...
            'Rotation',ang,'FontWeight','Bold','FontSize',10,'Color','b');
    
        % Recover tag extrinsic matrix
        H_a2c = pose(i).T.';
    
        % Define projection matrix
        P_a2m = A_c2m * H_a2c(1:3,:);
    
        % Project x/y axes of Frame a into the image
        xy_a = 1.2*tagSize*[...
        0, 1, 0;...
        0, 0, 1;...
        0, 0, 0];
        xy_a(4,:) = 1;
        sxy_m = P_a2m*xy_a;
        xy_m = sxy_m./sxy_m(3,:);
    
        % Plot x-axis
        pltx = plot(axs,xy_m(1,[1,2]),xy_m(2,[1,2]),'r','LineWidth',1.5);
        % Plot y-axis
        plty = plot(axs,xy_m(1,[1,3]),xy_m(2,[1,3]),'g','LineWidth',1.5);
        otherwise
        % Highlight AprilTag(s) NOT matching tagID in red
        ptc(i) = patch(axs,'Vertices',loc(:,:,i),'Faces',1:4,...
            'EdgeColor','r','FaceColor','r','FaceAlpha',0.1);
        end
    end
end