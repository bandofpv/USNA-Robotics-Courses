%% Section 1. Clear all variables and establish connection between MATLAB and CoppeliaSim

close all
clear

% object connection to CoppeliaSim
client = RemoteAPIClient();
sim = client.require('sim'); % establish simulation object
create_h = sim.getObject('/create'); % establish create3 object for sending commands
create_name = sim.getObjectAlias(create_h); % name of create in simulation


%% Section 2. Heading and Speed Control implementation

% a. Initialize Empty Arrays to be filled with data for plotting
time = [];
pos_dat = [];
vel_dat = [];
eul_dat = [];
ctrl_data = [];
des_psi_dat = [];

% b. Gains and constants
Kp_yaw = 0.5; % heading gain
des_yaw = pi/2; % desired heading
des_u = 0.2; % desired forward speed

% c. Set Initial pose of Rover
sim.setObjectOrientation(create_h,[0,0,0])

% d. Run a simulation in stepping mode:
sim.setStepping(true);
sim.startSimulation();

% e. Get Initial Simulation time
t = sim.getSimulationTime();
t_run = 20; % Total Simulation Run Time

% f. Iterate through time steps
while t < t_run 
    
    % i. query time
    t = sim.getSimulationTime(); % get elapsed time of CoppeliaSim simulation
    dt = sim.getSimulationTimeStep(); % get timestep of simulation
    
    % ii. Query position, orientation, velocity, and angular velocity
    pos_cell = sim.getObjectPosition(create_h)'; % get position of robot from CoppeliaSim
    pos = [pos_cell{1},pos_cell{2},pos_cell{3}]; % format position into array
    quat_cell = sim.getObjectQuaternion(create_h)'; % Coppelia uses [x y z W] convention for quaternion
    quat = [quat_cell{4};quat_cell{1};quat_cell{2};quat_cell{3}]';      %   Matlab uses [w x y z] convention
    eul = quat2eul(quat, 'XYZ')'; % convert quaternion to euler angles
    [vel_g, ang_vel] = sim.getObjectVelocity(create_h); % get velocity and angular velocity from simulation
    vel_g = [vel_g{1}, vel_g{2}, vel_g{3}]'; % format into numeric array
    ang_vel = [ang_vel{1} ang_vel{2} ang_vel{3}]'; % format into numeric array
    
    % iii. ------------------ Heading control ------------------
    
    des_yaw = (pi/6)*t;

    yaw = eul(3);
    yaw_error = des_yaw - yaw;
    r = Kp_yaw * wrapToPi(yaw_error); % proportional control
    % r = Kp_yaw * yaw_error;

    % Limit turn rate from -pi/2 to pi/2
    if r > pi/2
        r = pi/2;
    elseif r < -pi/2
        r = -pi/2;
    end
    
    % -------------------------------------------------------
    % iv. ---------------Speed Control -----------------------

    u = des_u;

    % Limit forward speed from 0 to 0.301 m/s
    if u < 0
        u = 0;
    elseif u > 0.301
        u = 0.301;
    end

    % v. Set the servo and esc command signals must be integers
    sim.setFloatSignal(sprintf("%s_u",create_name), u); % forward speed command
    sim.setFloatSignal(sprintf("%s_r",create_name), r);  % turn rate command
    
    % vi. step forward 1 time step
    sim.step();  % triggers next simulation step
    
    % vii. Append data from this time step to arrays
    % add data from this time step to arrays
    time = [time t];
    des_psi_dat = [des_psi_dat des_yaw];
    pos_dat = [pos_dat pos'];
    vel_dat = [vel_dat vel_g];
    eul_dat = [eul_dat eul];
    ctrl_data = [ctrl_data [u;r]];
end

% end simulation
sim.stopSimulation();

%% Section 3. Plot results

% Plot X vs. Y
figure(1)
plot(pos_dat(1, :), pos_dat(2, :))
axis equal
title("X vs. Y Vehicle Trajectory")
xlabel("X Position (m)")
ylabel("Y Position (m)")

% Plot Yaw Angle
figure(2)
plot(time, eul_dat(3, :));
hold on
plot(time, des_psi_dat);
hold off
title("Yaw Angle vs. Time")
xlabel("Time (s)")
ylabel("Yaw Angle (rad)")
legend("Achieved Yaw Angle", "Desired Yaw Angle")

%% Section 4. Waypoint implementation

% load waypoint file
load("Waypoints_N.mat")

% Initialize Empty Arrays to be filled with data for plotting
time = [];
pos_dat = [];
vel_dat = [];
eul_dat = [];
ang_vel_dat = [];
ctrl_data = [];
des_pos_dat = [];
des_psi_dat = [];

% Define Controller Gains and algorithm constants
wp_num = 1; % waypoing index
wp_rad = 0.1; % waypoint radius
Kp_yaw = 0.5; % heading gain
Kp_speed = 1; % forward speed gain

% Set Initial pose of Rover
sim.setObjectOrientation(create_h,[0,0,0])

% Run a simulation in stepping mode:
sim.setStepping(true);
sim.startSimulation();

% plot waypoints in CoppeliaSim for visual reference
numWypts = size(p_des,2); % number of waypoints
WpHandles = zeros(numWypts,1); % placeholder for handles of waypoint object in CoppeliaSim
colorComponent = sim.colorcomponent_ambient_diffuse; % color of waypoints
for mm = 1:numWypts % iterate over waypoints
    WpHandles(mm) = sim.createDummy(wp_rad,zeros(12,1)); % create a dummy object in CoppeliaSim
    sim.setObjectPosition(WpHandles(mm), -1, [p_des(1,mm), p_des(2,mm), 0.25]); % set the position of the waypoint object
    sim.setObjectColor(WpHandles(mm),0,colorComponent,[1 0 0]); % set the color of the waypoint object
end

% Get Initial Simulation time
t = sim.getSimulationTime();
t_run = 240; % Run Simulation Time

% iterate over time until you reach t_run seconds
while t < t_run

    % i. query time
    t = sim.getSimulationTime(); % get elapsed time of CoppeliaSim simulation
    dt = sim.getSimulationTimeStep(); % get timestep of simulation
    
    % ii. Query position, orientation, velocity, and angular velocity
    pos_cell = sim.getObjectPosition(create_h)'; % get position of robot from CoppeliaSim
    pos = [pos_cell{1},pos_cell{2},pos_cell{3}]; % format position into array
    quat_cell = sim.getObjectQuaternion(create_h)'; % Coppelia uses [x y z W] convention for quaternion
    quat = [quat_cell{4};quat_cell{1};quat_cell{2};quat_cell{3}]';      %   Matlab uses [x y z w] convention
    eul = quat2eul(quat, 'XYZ')'; % convert quaternion to euler angles
    [vel_g, ang_vel] = sim.getObjectVelocity(create_h); % get velocity and angular velocity from simulation
    vel_g = [vel_g{1}, vel_g{2}, vel_g{3}]'; % format into numeric array
    ang_vel = [ang_vel{1} ang_vel{2} ang_vel{3}]'; % format into numeric array
    
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

    % Iterate waypoints after reaching waypoing radius
    if dist2wp < wp_rad
        wp_num = wp_num + 1;
    end

    % Break after reaching final waypoint
    if wp_num > numWypts
        break
    end
    
    % -------------------------------------------------------
    % -----------------Speed Control -----------------------
    
    psi = eul(3);
    yaw_error = psi_des - psi;
    r = Kp_yaw * wrapToPi(yaw_error); % proportional heading control

    u = u_des;

    % Limit turn rate from -pi/2 to pi/2
    if r > pi/2
        r = pi/2;
    elseif r < -pi/2
        r = -pi/2;
    end

    % Limit forward speed from 0 to 0.301 m/s
    if u < 0
        u = 0;
    elseif u > 0.301
        u = 0.301;
    end

    % v. Set the servo and esc command signals must be integers
    sim.setFloatSignal(sprintf("%s_u",create_name), u); % forward speed command
    sim.setFloatSignal(sprintf("%s_r",create_name), r);  % turn rate command
    
    % vi. step forward 1 time step
    sim.step();  % triggers next simulation step
    
    % vii. Append data from this time step to arrays
    % add data from this time step to arrays
    time = [time t];
    des_pos_dat = [des_pos_dat [x_des;y_des]];
    des_psi_dat = [des_psi_dat psi_des];
    pos_dat = [pos_dat pos'];
    vel_dat = [vel_dat vel_g];
    eul_dat = [eul_dat eul];
    ctrl_data = [ctrl_data [u;r]];

end

% delete waypoint markers
for i=1:numWypts
    sim.removeObject(WpHandles(i));
end

% stop simulation
sim.stopSimulation();

%% Plot Results from simulation

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
