% Connect to Create3

crt = Create3_HW('florida', 59);

%% Test LED Colors

% Create a 1x3 vector [R, G, B]
red = [255, 0, 0];
blue = [0, 0, 180];
grey = [128, 128, 128];
gold = [255, 215, 0];
white = [255, 255, 255];

% Repeat this row 6 times to make a 6x3 matrix
color = repmat(blue, 6, 1);

% Send command
crt.setLEDCmd(color);

%% Shuttle Script

orange = [255, 127, 0];
white = [255, 255, 255];
blue = [0, 0, 255];
orange_color = repmat(orange, 6, 1);
white_color = repmat(white, 6, 1);
blue_color = repmat(blue, 6, 1);

load("shuttle_waypoints.mat")

% Initialize Empty Arrays to be filled with data for plotting
time = []; % time array
pos_dat = []; % position of vehicle over time
vel_dat = []; % velocity of vehicle over time
eul_dat = []; % Euler angles of vehicle over time
ang_vel_dat = []; % angular velocity of vehicle over 
ctrl_data = []; % robot control commands
des_pos_dat = [];
des_psi_dat = [];

% Define Controller Gains and algorithm constants
numWypts = size(p_des,2); % number of waypoints
wp_num = 2; % waypoint index
wp_rad = 0.05; % waypoint radius
Kp_yaw = 3; % heading gain
Kp_speed = 1; % forward speed gain
yaw_thresh = deg2rad(2);

% Turn LEDs off and reset pose
off = repmat([0, 0, 0], 6, 1);
crt.setLEDCmd(off);
crt.setLEDCmd(white_color);
zeroPose(crt);

run_t = inf; % run time
elapsedTime = 0;
startTime = tic; 
watiTimeToc = 0;

while elapsedTime < run_t
    elapsedTime = toc(startTime); % update elapsedTime

    [pose, vel] = crt.getOdomPose();

    pos = [pose(1); pose(2); pose(3)];
    eul = [pose(4); pose(5); pose(6)];
    vel_g = [vel(1); vel(2); vel(3)];
    ang_vel = [vel(4); vel(5); vel(6)];

    % If the waypoint is NaN, turn off LEDs
    if isnan(p_des(1, wp_num)) && isnan(p_des(2, wp_num))
        crt.setLEDCmd(off);
        wp_num = wp_num + 1;
        continue;
    end

    % Set Desired Position (waypoint location at time t)
    x_des = p_des(1, wp_num);
    y_des = p_des(2, wp_num);

    %-------------------- Waypoint Control ------------------

    % Calculate the waypoint control algorithm
    x = pos(1);
    y = pos(2);
    x_error = x_des - x;
    y_error = y_des - y;
    psi_des = atan2(y_error, x_error); % calculated desired heading
    dist2wp = sqrt(x_error^2 + y_error^2);
    u_des = Kp_speed*dist2wp; % proportional forward speed control

    % -------------------------------------------------------
    % -----------------Speed Control -----------------------

    psi = eul(3);
    yaw_error = psi_des - psi;
    r_des = Kp_yaw * wrapToPi(yaw_error); % proportional heading control

    x_des_prev = p_des(1, wp_num-1);
    y_des_prev = p_des(2, wp_num-1);
    x_error_prev = x_des_prev - x;
    y_error_prev = y_des_prev - y;
    dist2wp_prev = sqrt(x_error_prev^2 + y_error_prev^2);

    % Don't move forward until facing right direction
    if abs(yaw_error) > yaw_thresh && dist2wp_prev < wp_rad
        u_des = 0;
    end

    % Iterate waypoints after reaching waypoint radius and turn on LEDs
    if dist2wp < wp_rad
        wp_num = wp_num + 1;
        if wp_num > 3 && wp_num < 14
            crt.setLEDCmd(orange_color);
        end    
        if wp_num > 14
            crt.setLEDCmd(blue_color);
        end
        if wp_num == 14 || wp_num == 18
            crt.setLEDCmd(off);
        end
    end

    % Turn LEDs off and break after reaching final waypoint
    if wp_num > numWypts
        crt.setLEDCmd(off);
        break
    end

    % Bound forward speed and turn rate
    u = max(0,min(u_des,0.306));
    r = max(-1.57,min(r_des,1.57));

    % v. Set the servo and esc command signals must be integers
    crt.setVelCmd(u, r);    

    % vii. Append data from this time step to arrays
    % add data from this time step to arrays
    time = [time elapsedTime];
    des_pos_dat = [des_pos_dat [x_des;y_des]];
    des_psi_dat = [des_psi_dat psi_des];
    pos_dat = [pos_dat pos];
    vel_dat = [vel_dat vel_g];
    eul_dat = [eul_dat eul];
    ctrl_data = [ctrl_data [u;r]];
    
    pause(0.01); % iterate at 100 Hz
end

crt.setVelCmd(0, 0); % stop vehicle
crt.setLEDCmd(off); % turn LEDs off

%% Go Navy Script

load("navy.mat") % load waypoints

% Initialize Empty Arrays to be filled with data for plotting
time = []; % time array
pos_dat = []; % position of vehicle over time
vel_dat = []; % velocity of vehicle over time
eul_dat = []; % Euler angles of vehicle over time
ang_vel_dat = []; % angular velocity of vehicle over 
ctrl_data = []; % robot control commands
des_pos_dat = []; % desired position of vehicle over time
des_psi_dat = []; % desired yaw of vehicle over time

% Define Controller Gains and algorithm constants
numWypts = size(p_des,2); % number of waypoints
wp_num = 2; % waypoint index
wp_rad = 0.05; % waypoint radius
Kp_yaw = 3; % heading gain
Kp_speed = 1; % forward speed gain
yaw_thresh = deg2rad(2); % yaw angle threshold

% RBG colors
blue = [0, 0, 255];
gold = [255, 215, 0];

% Create LED color matricies
gold_color = repmat(gold, 6, 1);
blue_color = repmat(blue, 6, 1);

% Turn LEDs off and reset pose
off = repmat([0, 0, 0], 6, 1);
crt.setLEDCmd(off);
zeroPose(crt);

% Create timer
run_t = inf;
elapsedTime = 0;
startTime = tic; 

while elapsedTime < run_t
    elapsedTime = toc(startTime); % update elapsedTime

    [pose, vel] = crt.getOdomPose(); % get pose

    % Parse pose data
    pos = [pose(1); pose(2); pose(3)];
    eul = [pose(4); pose(5); pose(6)];

    % Set Desired Position (waypoint location at time t)
    x_des = p_des(1, wp_num);
    y_des = p_des(2, wp_num);

    %-------------------- Waypoint Control ------------------

    % Calculate the waypoint control algorithm
    x = pos(1);
    y = pos(2);
    x_error = x_des - x;
    y_error = y_des - y;
    psi_des = atan2(y_error, x_error); % calculated desired heading
    dist2wp = sqrt(x_error^2 + y_error^2);
    u_des = Kp_speed*dist2wp; % proportional forward speed control

    % -------------------------------------------------------
    % -----------------Speed Control -----------------------

    psi = eul(3); % get yaw of vehicle
    yaw_error = psi_des - psi; % calculate yaw error
    r_des = Kp_yaw * wrapToPi(yaw_error); % proportional heading control

    % Calculate previous waypoint
    x_des_prev = p_des(1, wp_num-1);
    y_des_prev = p_des(2, wp_num-1);
    x_error_prev = x_des_prev - x;
    y_error_prev = y_des_prev - y;
    dist2wp_prev = sqrt(x_error_prev^2 + y_error_prev^2);

    % Don't move forward until facing right direction (if close to previous
    % waypoint)
    if abs(yaw_error) > yaw_thresh && dist2wp_prev < wp_rad
        u_des = 0;
    end

    % Iterate waypoints after reaching waypoint radius
    if dist2wp < wp_rad
        wp_num = wp_num + 1;
    end

    % Turn LEDs off and break after reaching final waypoint
    if wp_num > numWypts
        crt.setLEDCmd(off);
        break
    end

    % Bound forward speed and turn rate
    u = max(0,min(u_des,0.306));
    r = max(-1.57,min(r_des,1.57));

    % Set the servo and esc command signals must be integers
    crt.setVelCmd(u, r);    

    color = p_des(3, wp_num); % get color

    % Set color based off number in waypoint list
    if color == 0
        crt.setLEDCmd(off);
    elseif color == 1
        crt.setLEDCmd(blue_color);
    elseif color == 2
        crt.setLEDCmd(gold_color);
    end

    % Append data from this time step to arrays add data from this time step to arrays
    time = [time elapsedTime];
    des_pos_dat = [des_pos_dat [x_des;y_des]];
    des_psi_dat = [des_psi_dat psi_des];
    pos_dat = [pos_dat pos];
    eul_dat = [eul_dat eul];
    ctrl_data = [ctrl_data [u;r]];
    
    pause(0.01); % iterate at 100 Hz
end

crt.setVelCmd(0, 0); % stop vehicle
crt.setLEDCmd(off); % turn LEDs off

%% Plot results

% load("navy_data.mat")
load("parachute_data.mat")

low_des_pos_dat = des_pos_dat(1:2, :) - wp_rad;
high_des_pos_dat = des_pos_dat(1:2, :) + wp_rad;
low_des_psi_dat = des_psi_dat - yaw_thresh;
high_des_psi_dat = des_psi_dat + yaw_thresh;

% Plot X vs. Y with waypoints
figure(1)
plot(pos_dat(1, :), pos_dat(2, :))
hold on
scatter(p_des(1, :), p_des(2, :), '*');
hold off
axis equal
title("X vs. Y Vehicle Trajectory")
xlabel("X Position (m)")
ylabel("Y Position (m)")
legend("Vehicle Trajectory", "Waypoints")

% Plot x position vs. desired x position
figure(2)
plot(time, pos_dat(1, :))
hold on
plot(time, des_pos_dat(1, :))
plot(time, low_des_pos_dat(1, :), 'Color', '#EDB120')
plot(time, high_des_pos_dat(1, :), 'Color', '#EDB120')
hold off
title("X Position vs. Desired X Position")
xlabel("Time (s)")
ylabel("X Position (m)")
legend("Vehicle Position", "Desired Position",  "+/- 0.05m")

% Plot y position vs. desired y position
figure(3)
plot(time, pos_dat(2, :))
hold on
plot(time, des_pos_dat(2, :))
plot(time, low_des_pos_dat(2, :), 'Color', '#EDB120')
plot(time, high_des_pos_dat(2, :), 'Color', '#EDB120')
hold off
title("Y Position vs. Desired Y Position")
xlabel("Time (s)")
ylabel("Y Position (m)")
legend("Vehicle Position", "Desired Position", "+/- 0.05m")

% Plot heading vs desired heading
figure(4)
plot(time, eul_dat(3, :))
hold on
plot(time, des_psi_dat)
plot(time, low_des_psi_dat, 'Color', '#EDB120')
plot(time, high_des_psi_dat, 'Color', '#EDB120')
hold off
title("Vehicle Heading vs. Desired Heading")
xlabel("Time (s)")
ylabel("Heading (rad)")
legend("Vehicle Heading", "Desired Heading", "+/- 2deg")

%% Steady State Error Calculation

% Extract desied and actual data
desired_X = des_pos_dat(1, :);
actual_X = pos_dat(1, :);
desired_Y = des_pos_dat(2, :);
actual_Y = pos_dat(2, :);
desired_yaw = des_psi_dat;
actual_yaw = eul_dat(3, :);

% Find the indices where the waypoint switches 
switch_indices_X = find(abs(diff(desired_X)) > 0);
switch_indices_Y = find(abs(diff(desired_Y)) > 0);
switch_indices_yaw = find(abs(diff(desired_yaw)) > 0.1); % use a 0.1 rad threshold due to noisy des_psi_dat

% Add the last point to evaluate
eval_points_X = [switch_indices_X, length(desired_X)];
eval_points_Y = [switch_indices_Y, length(desired_Y)];
eval_points_yaw = [switch_indices_yaw(2:end), length(desired_yaw)]; % remove first due to initial pose zeroing

% Calculate error at evaluation points
error_X = abs(actual_X(eval_points_X) - desired_X(eval_points_X));
error_Y = abs(actual_Y(eval_points_Y) - desired_Y(eval_points_Y));
error_yaw = abs(actual_yaw(eval_points_yaw) - desired_yaw(eval_points_yaw));

% Average the error
avg_error_X = mean(error_X); 
avg_error_Y = mean(error_Y); 
avg_error_yaw = mean(error_yaw); 

% Print results
fprintf('Average X-axis SS error: %.4f meters\n', avg_error_X);
fprintf('Average Y-axis SS error: %.4f meters\n', avg_error_Y);
fprintf('Average Heading SS error: %.4f radians\n', avg_error_yaw);