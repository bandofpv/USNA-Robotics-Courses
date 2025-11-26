%% Connect to CoppeliaSim

close all
clear
fprintf('Program started\n')

% Create Object for CoppeliaSim Interface
client = RemoteAPIClient();
sim = client.require('sim');

% Get Handles for Quadrotor and Waypoint
quad_h = sim.getObject('/Quadrotor');
waypoint_h = sim.getObject('/Waypoint');
%% Simulation Code

% Initialize Empty Arrays to be filled with data for plotting
time = [];
pos_dat = [];
vel_dat = [];
eul_dat = [];
err_dat = [];
ctrl_data = [];
des_vel_dat = [];
des_pos_dat = [];
des_yaw_dat = [];

% Define desired body velocites (x, y, z) and yaw rate
% des_vel_b = [1 1 1];
% des_yaw_rate = 0;

% Define List of Waypoints
wp = [5 -5 -5 5;
      5 5 -5 -5;
      1 2  1  2];
wp_rad = 0.05; % waypoint radius
wp_index = 1;

%  Known Parameters
mass = 2.2;
g = 9.81;

% Initilize integral variables
e_x_int = 0;
e_y_int = 0;
e_z_int = 0;

% Initilize position error
e_pos = inf;

% Define Controller Gains
Kp_vel_z = 15;
Ki_vel_z = 20;
Kp_vel_xy = 0.1;
Ki_vel_xy = 0;
Kp_xy = 0.4;
Kp_z = 1;
K_yaw = 3;

% Set Saturation Limits
vel_max_xy = 2.0; % max x & y velocity (m/s)
vel_max_z = 1.0; % max z velocity (m/s)

% Define variables for initial pose
pos_init = [0.0,0.0,1.0];
eul_init = deg2rad([0.0,0.0,0.0]);

% Set Initial pose of Quadcopter
sim.setObjectPosition(quad_h,pos_init)
sim.setObjectOrientation(quad_h,eul_init)

% Run a simulation in stepping mode:
sim.setStepping(true);
sim.startSimulation();

% Get Initial Simulation time
t = sim.getSimulationTime();
t_run = 60; % Run Simulation Time
while t < t_run 
    t = sim.getSimulationTime();
    dt = sim.getSimulationTimeStep();
    
    % Get the Rover states from the simulation
    pos = cell2mat(sim.getObjectPosition(quad_h))';
    quat = sim.getObjectQuaternion(quad_h)'; % Coppelia uses [x y z W] convention
    quat = [quat{4} quat{1} quat{2} quat{3}]; % Matlab uses [x y z w]
    eul = quat2eul(quat, 'XYZ')';
    [vel_g, ang_vel] = sim.getObjectVelocity(quad_h);
    vel_g = [vel_g{1} vel_g{2} vel_g{3}]';
    ang_vel = [ang_vel{1} ang_vel{2} ang_vel{3}]';
    
    % Compute Rotation Matrix from Euler Angles
    R_gb = eul2rotm(eul','XYZ')'; % eul2rotm returns matrix from body to global
    vel_b = R_gb*vel_g;  % Compute the body frame velocity vector

    % Set Desired Position Based on current WP index
    % x_des = wp(1, wp_index);
    % y_des = wp(2, wp_index);
    % z_des = wp(3, wp_index);

    % Waypoint controller testing
    x_des = 3;
    y_des = 3;
    z_des = 3;
    if norm(e_pos) < wp_rad
        break
    end

    des_pos = [x_des; y_des; z_des];

    % Update the location of the visual waypoint in the simulation
    sim.setObjectPosition(waypoint_h,[x_des,y_des,z_des]);

    %%%% Waypoint Controllers %%%%%

    % Calculate position error
    e_pos = des_pos - pos;

    % Go to next waypoint once reached
    if norm(e_pos) < wp_rad
        wp_index = wp_index + 1;
        % Stop simulation when reached last waypoint
        if wp_index > size(wp, 2)
            break
        end
    end

    % Global Frame Velocity P-Controller
    x_dot_cmd = Kp_xy*e_pos(1);
    y_dot_cmd = Kp_xy*e_pos(2);
    z_dot_cmd = Kp_z*e_pos(3);

    % Global-to-Body Rotation
    des_vel_b(1) = x_dot_cmd*cos(eul(3)) + y_dot_cmd*sin(eul(3));
    des_vel_b(2) = - x_dot_cmd* sin(eul(3)) + y_dot_cmd * cos(eul(3));
    des_vel_b(3) = z_dot_cmd;

    % Cap Velocity
    des_vel_b(1) = max(min(des_vel_b(1), vel_max_xy), -vel_max_xy);
    des_vel_b(2) = max(min(des_vel_b(2), vel_max_z), -vel_max_z);
    des_vel_b(3) = max(min(des_vel_b(3), vel_max_z), -vel_max_z);

    % Yaw/Heading Controller (Points towards waypoint)
    des_yaw = atan2(e_pos(2),e_pos(1));
    des_yaw_rate = K_yaw*wrapToPi(des_yaw - eul(3));

    %%%% Velocity Controllers %%%%%

    % Z-Velocity Controller (PI + Gravity Feed-Forward)
    e_vel_z = des_vel_b(3) - vel_g(3); % z error
    e_z_int = e_z_int + e_vel_z * dt; % z integral
    F = Kp_vel_z*e_vel_z + Ki_vel_z*e_z_int + mass*g; % z-vel controller

    % XY-Velocity Controller (PI)
    e_vel_x_b = des_vel_b(1) - vel_b(1); % x error
    e_x_int = e_x_int + e_vel_x_b * dt; % x integral

    e_vel_y_b = des_vel_b(2) - vel_b(2); % y error
    e_y_int = e_y_int + e_vel_y_b * dt; % y integral

    des_pitch = Kp_vel_xy*e_vel_x_b + Ki_vel_xy*e_x_int; % x-vel controller
    des_roll = -Kp_vel_xy*e_vel_y_b - Ki_vel_xy*e_y_int; % y-vel controller

    % Set the servo and esc command signals must be integers
    sim.setFloatSignal("des_roll", des_roll); % Radians (gets saturated in sim)
    sim.setFloatSignal("des_pitch", des_pitch); % Radians (gets saturated in sim)
    sim.setFloatSignal("des_yaw_rate", des_yaw_rate); % Radians/sec (gets saturated in sim)
    sim.setFloatSignal("des_thrust", F); % Newtons (gets saturated in sim)

    sim.step();  % triggers next simulation step

    time = [time t];
    des_vel_dat = [des_vel_dat [des_vel_b(1); des_vel_b(2); des_vel_b(3)]];
    des_pos_dat = [des_pos_dat [x_des; y_des; z_des]];
    pos_dat = [pos_dat pos];
    vel_dat = [vel_dat vel_b];
    eul_dat = [eul_dat [eul]];
    des_yaw_dat = [des_yaw_dat [des_yaw]];
    err_dat = [err_dat [e_pos]];

end
sim.stopSimulation();

%% Plot Desired and Actual Body-Frame Velocities

fig = figure;
subplot(3, 1, 1)
plot(time, des_vel_dat(1, :))
hold on
plot(time, vel_dat(1, :))
hold off
title("X Body-Frame Velocity")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
legend("Desired", "Actual")

subplot(3, 1, 2)
plot(time, des_vel_dat(2, :))
hold on
plot(time, vel_dat(2, :))
hold off
title("Y Body-Frame Velocity")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
legend("Desired", "Actual")

subplot(3, 1, 3)
plot(time, des_vel_dat(3, :))
hold on
plot(time, vel_dat(3, :))
hold off
title("Z Body-Frame Velocity")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
legend("Desired", "Actual")

%% Plot Position/Waypoint Controller Performance

fig = figure;
subplot(3, 1, 1)
plot(time, des_pos_dat(1, :))
hold on
plot(time, pos_dat(1, :))
hold off
title("X Postition")
xlabel("Time (s)")
ylabel("Postition (m)")
legend("Desired", "Actual")

subplot(3, 1, 2)
plot(time, des_pos_dat(2, :))
hold on
plot(time, pos_dat(2, :))
hold off
title("Y Postition")
xlabel("Time (s)")
ylabel("Postition (m)")
legend("Desired", "Actual")

subplot(3, 1, 3)
plot(time, des_pos_dat(3, :))
hold on
plot(time, pos_dat(3, :))
hold off
title("Z Postition")
xlabel("Time (s)")
ylabel("Postition (m)")
legend("Desired", "Actual")

fig = figure;
plot(time, err_dat(1, :))
hold on
plot(time, err_dat(2, :))
plot(time, err_dat(3, :))
title("Position Error")
xlabel("Time (s)")
ylabel("Postition (m)")
legend("X", "Y", "Z")

%% Plot Yaw/Heading Controller

fig = figure;
plot(time, des_yaw_dat)
hold on
plot(time, eul_dat(3, :))
hold off
title("Heading Controller")
xlabel("Time (s)")
ylabel("Heading (rad)")
legend("Desired", "Actual")

%% Plot Waypoint Mission

fig = figure;
plot3(pos_dat(1, :), pos_dat(2, :), pos_dat(3, :))
hold on
scatter3(wp(1, :), wp(2, :), wp(3, :))
hold off
title("Waypoint Mission")
xlabel("X Postition (m)")
ylabel("Y Postition (m)")
zlabel("Z Postition (m)")
legend("Trajectory", "Wapoints")