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

%% Save Data

save('Lab6_Data.mat',...
    'p_0Tob','p_1Tob','p_2Tob','H_tTob');

%% Testing Alignment

p_0Tot = [ 0; 0; 0]; % Point 0 relative to Frame t
p_1Tot = [160; 0; 0]; % Point 1 relative to Frame t
p_2Tot = [ 0; 100; 0]; % Point 2 relative to Frame t
p_3Tot = [ 60; 40; 0]; % Point 3 relative to Frame t
p_4Tot = [170; 100; 0]; % Point 4 relative to Frame t
p_iTot = [p_0Tot, p_1Tot, p_2Tot, p_3Tot, p_4Tot];

ur.Home; % Move robot home

for i = 1:3
    p_iTob = H_tTob * [p_iTot(:,i); 1]; % Define point relative to base frame
    H_eTob = Ry(pi); % Define end-effector orientation
    H_eTob(1:3,4) = p_iTob(1:3,:); % Define end-effector position
    ur.Pose = Tz(10)*H_eTob; % Move above the point
    ur.Pose = H_eTob; % Move onto the point
    ur.Pose = Tz(10)*H_eTob; % Move above the point
    ur.Home; % Move robot home
end

%% Repetitive Task

p_3Tob = H_tTob * [p_iTot(:,4); 1]; % Define point 3 relative to base frame
p_4Tob = H_tTob * [p_iTot(:,5); 1]; % Define point 4 relative to base frame
H_eTob = Ry(pi); % Define end-effector orientation

for i = 1:20
    ur.Home; % Move robot home

    % Hit point 3
    H_eTob(1:3,4) = p_3Tob(1:3,:); % Define end-effector position for point 3
    ur.Pose = Tz(10)*H_eTob; % Move above point 3
    ur.Pose = H_eTob; % Move onto the point
    ur.Pose = Tz(10)*H_eTob; % Move above point 3

    ur.Home; % Move robot home

    % Hit point 4
    H_eTob(1:3,4) = p_4Tob(1:3,:); % Define end-effector position for point 4
    ur.Pose = Tz(10)*H_eTob; % Move above point 4
    ur.Pose = H_eTob; % Move onto the point
    ur.Pose = Tz(10)*H_eTob; % Move above point 4
end

ur.Home; % Move robot home