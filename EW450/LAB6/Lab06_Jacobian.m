%% Initilize UR3e

ur = URQt('UR3e');
ur.Initialize;

%% Grab p0

H_0Tob = ur.Pose;
p_0Tob = H_0Tob(1:3,4);

%% Grab p1

H_1Tob = ur.Pose;
p_1Tob = H_1Tob(1:3,4);

%% Grab p2

H_2Tob = ur.Pose;
p_2Tob = H_2Tob(1:3,4);

%% Calculate H_tTob

x_hat_tTob = (p_1Tob - p_0Tob)/norm(p_1Tob - p_0Tob);
y_tildle_tTob = (p_2Tob - p_0Tob)/norm(p_2Tob - p_0Tob);
z_hat_tTob = cross(x_hat_tTob, y_tildle_tTob)/norm(cross(x_hat_tTob, y_tildle_tTob));
y_hat_tTob = cross(z_hat_tTob, x_hat_tTob)/norm(cross(z_hat_tTob, x_hat_tTob));

R_tTob = [x_hat_tTob, y_hat_tTob, z_hat_tTob];  % Rotation matrix Frame t relative to b
d_tTob = p_0Tob; % Translation vector Frame t relative to b
H_tTob = [R_tTob, d_tTob; 0 0 0 1]; % Homogenous matrix Frame t relative to b

%% Create Drawing

% Run the user drawing interface
X_t = drawOnTarget;

%% Save Drawing Data

% Save drawing
save('Lab06_Data_New.mat','X_t');

% Find the figure handle for the user drawing
fig = findobj('Type','figure','Name','drawOnTarget');

% Save the user drawing
saveas(fig(end),'Lab06_UserDrawing_New.png','png');

%% Calculate X_b (drawing points refrenced to base frame)

% Append ones to the 4th row of "X t" to create a homogeneous representation of position
X_t(4,:) = 1;

% Drawing points referenced to base frame
X_b = H_tTob*X_t;

%% Draw Using the UR3e Manipulator

phi = 0; % fix end effector orientation
Xo_b_above = X_b(1:3,1) + [0; 0; 10]; % point above first drawing point
Xf_b_above = X_b(1:3,end) + [0; 0; 10]; % point above last drawing point

Xo = [Xo_b_above; phi]; % task configuration for first point
L = [151.85, 243.55, 213.2, 131.05, 85.35, 92.10]; % link lengths
q = ikinPickAndPlace(Xo, L); % calculate ikin for first point
ur.Joints = q; % move UR3e manipulator

s = 1; % step size
idx = 1; % index variable

% break movement between each waypoint into small, discrete moves
while true
    % Get current joint configuration
    Qi = ur.Joints;
    % Convert 6-element joint configuration to 4-element joint configuration
    qi = q6Toq4(Qi);
    % Get current end-effector pose
    H_e2o = ur.Pose;
    % Convert current end-effector pose to 4-element task configuration
    Xi = poseToTask(H_e2o);
    % Define desired final task configuration
    Xf = [X_b(1:3,idx); phi];
    % Check if you have reached your goal task configuration
    if norm( Xf - Xi ) < s
        idx = idx + 1;
        % Break condition
        if idx > size(X_b,2)
            break
        end
        % Update desired final task configuration
        Xf = [X_b(1:3,idx); phi];
    end
    % Calculate the change in task configuration
    dX = Xf - Xi;
    % Impose maximum step size
    if s < norm(dX)
        dX = s * (dX./norm(dX));
    end
    % Calculate the change in joint configuration
    dq = ( JacobianPickAndPlace(qi,L)^(-1) )*dX;
    % Calculate the final joint configuration
    qf = qi + dq;
    % Convert 4-element joint configuration to 6-element joint configuration
    Qf = q4Toq6(qf);
    % Send 6-element joint configuration to the robot
    ur.Joints = Qf;
end

Xf = [Xf_b_above; phi]; % task configuration for last point
q = ikinPickAndPlace(Xf, L); % calculate ikin for last point
ur.Joints = q; % move UR3e manipulator