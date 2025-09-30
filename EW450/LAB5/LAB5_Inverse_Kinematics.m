% Define UR3e Link Lengths
L = [151.85, 243.55, 213.2, 131.05, 85.35, 92.10];

%% ikinPickAndPlace Test 1

X = [-250, -350, 80, pi/4];
[q_eu, q_ed] = ikinPickAndPlace(X,L);

%% ikinPickAndPlace Test 2

X = [-450, -250, 100, (3*pi)/4];
[q_eu, q_ed] = ikinPickAndPlace(X,L);

%% ikinPickAndPlace Test 3

X = [250, 450, 120, (5*pi)/4];
[q_eu, q_ed] = ikinPickAndPlace(X,L);

%% ikinPickAndPlace Test 4

X = [450, 250, 140, (7*pi)/4];
[q_eu, q_ed] = ikinPickAndPlace(X,L);

%% Initilize UR3e

ur = URQt('UR3e');
ur.Initialize;

%% Experimentally Testing Inverse Kinematics

L = [151.85, 243.55, 213.2, 131.05, 85.35, 92.10];

% Grab 8 poses to compute with ikin and compare joint angles with gt
fprintf('Press space to start\n');
for i = 1:8
    pause;

    H_e2b{i} = ur.Pose;
    q_exp{i} = ur.Joints; 

    % d_E_b = H_e2b{i}(1:3, 4);
    d_E_b = [H_e2b{i}(1, 4), H_e2b{i}(2, 4), H_e2b{i}(3, 4)];
    phi = atan2(H_e2b{i}(2, 1), H_e2b{i}(1, 1));
    X{i} = [d_E_b, phi];

    q_calc{i} = ikinPickAndPlace(X{i},L);

    fprintf('Recorded pose %d\n', i);
end

%% Save Data

% Save the variables H e2b, q exp, X, and q calc to the file Lab05_Data.mat
save('Lab05_Data_New.mat','H_e2b','q_exp','X','q_calc');

%% Compare calculated joint configuration with ground truth of UR3e

for i = 1:8
    % Calculate error using normalized angles (0-2pi)
    error{i} = mod(q_calc{1, i}, 2*pi) - mod(q_exp{1, i}.', 2*pi);
end

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

% *Add* (aka append) the variable X t to the file Lab05 Data.mat
save('Lab05_Data_New.mat','X_t','-append');

% Find the figure handle for the user drawing
fig = findobj('Type','figure','Name','drawOnTarget');

% Save the user drawing
saveas(fig(end),'Lab05_UserDrawing.png','png');

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

ur.MoveType = 'LinearTask';
for i = 1:size(X_b,2)
    X = [X_b(1:3,i); phi]; % task configuration
    q = ikinPickAndPlace(X, L); % calculate ikin
    ur.Joints = q; % move UR3e manipulator
end

ur.MoveType = 'LinearJoint';

Xf = [Xf_b_above; phi]; % task configuration for last point
q = ikinPickAndPlace(Xf, L); % calculate ikin for last point
ur.Joints = q; % move UR3e manipulator