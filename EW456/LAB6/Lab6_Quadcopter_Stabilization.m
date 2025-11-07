close all
clear
fprintf('Program started\n')

% Create Object for CoppeliaSim Interface
client = RemoteAPIClient();
sim = client.require('sim');

% Get Handles for Quadrotor and Waypoint
quad_h = sim.getObject('/Quadrotor');
waypoint_h = sim.getObject('/Waypoint');

%%
% Initialize Empty Arrays to be filled with data for plotting
time = [];
pos_dat = [];
vel_dat = [];
eul_dat = [];
ang_vel_dat = [];
err_dat = [];
ctrl_dat = [];
des_eul_dat = [];
des_alt_dat = [];

% Quadcopter Parameter from Sim
mass = 2.2;
g = 9.81;
L = 0.16;
C = 0.1;

% Define Controller Gains
Kp_z = 20.0;
Ki_z = 3.8;
Kd_z = 9.1;
Kp_rp = 0.6;
Kd_rp = 0.2;
K_yaw_rate = 0.2;

% Set Saturation Limits
F_max = 12.3875;
rp_max = deg2rad(25); % Max Roll Pitch Angle
vel_max = 2; % Maximum Velocity 

% Define variables for initial pose
pos_init = [0,0,1];
eul_init = deg2rad([4,5,0]);
e_z_int = 0;

% Set Initial pose of Quadcopter
sim.setObjectPosition(quad_h,pos_init)
sim.setObjectOrientation(quad_h,eul_init)

% Run a simulation in stepping mode:
sim.setStepping(true);
sim.startSimulation();

% Get Initial Simulation time
t = sim.getSimulationTime();
t_run = 15; % Run Simulation Time
while t < t_run 
    t = sim.getSimulationTime();
    dt = sim.getSimulationTimeStep();
    
    % Get the Quadcopter states from the simulation
    pos = cell2mat(sim.getObjectPosition(quad_h))';
    abg = sim.getObjectOrientation(quad_h);
    R = eul2rotm([abg{1} abg{2} abg{3}],'XYZ');
    ypr = rotm2eul(R,'ZYX');
    eul = [ypr(3); ypr(2); ypr(1)];
    [vel_g, ang_vel] = sim.getObjectVelocity(quad_h);
    vel_g = [cell2mat(vel_g(1)) cell2mat(vel_g(2)), cell2mat(vel_g(3))]';
    ang_vel = [cell2mat(ang_vel(1)) cell2mat(ang_vel(2)) cell2mat(ang_vel(3))]';
    
    % Compute Rotation Matrix from Euler Angles
    R_gb = angle2dcm(eul(1),eul(2),eul(3),'ZYX'); % eul2rotm returns matrix from body to global
    vel_b = R_gb*vel_g;  % Compute the body frame velocity vector

    % =====================================================================
    % =================== STUDENT TODO: IMPLEMENT CONTROLLERS =============
    % =====================================================================
    % Define Desired States
    des_z = 3.0;        % Desired altitude (meters)
    des_roll = 0;       % Desired roll angle (radians)
    des_pitch = 0;      % Desired pitch angle (radians)
    des_yaw_rate = 0;   % Desired yaw rate (radians/sec)

    % TODO 1: Implement Altitude Controller (PD + Gravity Compensation)
    % Use: Kp_z, Kd_z, des_z, pos(3), vel_g(3), mass, g
    e_z = des_z - pos(3); % z error
    e_dot_z = 0 - vel_g(3); % z error rate
    e_z_int = e_z_int + e_z*dt; % integral of z error
    % F = Kp_z*e_z + Kd_z*e_dot_z + mass*g;
    F = Kp_z*e_z + Kd_z*e_dot_z + mass*g + Ki_z*e_z_int;

    % TODO 2: Implement Roll Controller (PD)
    % Use: Kp_rp, Kd_rp, des_roll, eul(1), ang_vel(1)
    e_roll = des_roll - eul(1); % roll error
    e_dot_roll = 0 - ang_vel(1); % roll error rate
    Mx = Kp_rp*e_roll + Kd_rp*e_dot_roll;
    
    % TODO 3: Implement Pitch Controller (PD)
    % Use: Kp_rp, Kd_rp, des_pitch, eul(2), ang_vel(2)
    e_pitch = des_pitch - eul(2); % pitch error
    e_dot_pitch = 0 - ang_vel(2); % pitch error rate
    My = Kp_rp*e_pitch + Kd_rp*e_dot_pitch; 

    % TODO 4: Implement Yaw Rate Controller (P)
    % Use: K_yaw_rate, des_yaw_rate, ang_vel(3)
    e_yaw_rate = des_yaw_rate - ang_vel(3); % yaw rate error
    Mz = K_yaw_rate*e_yaw_rate;

    % Implement mixing matrix to convert F Mx My Mz tp F1 F2 F3 F4
    mixing_matrix = inv([1 1 1 1; L L -L -L; -L L L -L; C -C C -C]);

    % Calculate forces vector
    F_cmd = mixing_matrix*[F; Mx; My; Mz]; 

    % =====================================================================
    % =================== END OF STUDENT TODO =============================
    % =====================================================================

    % Divide Command force by max force to get Duty Cycle
    m1_dc = F_cmd(1)/F_max;
    m2_dc = F_cmd(2)/F_max;
    m3_dc = F_cmd(3)/F_max;
    m4_dc = F_cmd(4)/F_max;

    % Set the servo and esc command signals must be integers
    sim.setFloatSignal("m1_dc", m1_dc); % Send Duty Cycle (0-1)
    sim.setFloatSignal("m2_dc", m2_dc); % Send Duty Cycle (0-1)
    sim.setFloatSignal("m3_dc", m3_dc); % Duty Cycle (0-1)
    sim.setFloatSignal("m4_dc", m4_dc); % Duty Cycle (0-1)
    
    sim.step();  % triggers next simulation step

    % --- Data Logging ---
    time = [time t];
    pos_dat = [pos_dat pos];
    vel_dat = [vel_dat vel_b];
    ang_vel_dat = [ang_vel_dat ang_vel];
    eul_dat = [eul_dat eul];
    des_eul_dat = [des_eul_dat [des_roll;des_pitch;des_yaw_rate]];
    des_alt_dat = [des_alt_dat des_z];
end
sim.stopSimulation();


%% --- Plotting ---

figure('Name','Attitude Control')
subplot(211)
plot(time,rad2deg(des_eul_dat(1,:)),'k--','LineWidth',2),hold on
plot(time,rad2deg(eul_dat(1,:)),'b')
legend('Desired','Actual')
title('Roll Angle')
ylabel('Angle (deg)')
xlabel('Time (s)')

subplot(212)
plot(time,rad2deg(des_eul_dat(2,:)),'k--','LineWidth',2),hold on
plot(time,rad2deg(eul_dat(2,:)),'b')
legend('Desired','Actual')
title('Pitch Angle')
ylabel('Angle (deg)')
xlabel('Time (s)')

figure('Name','Altitude Control')
plot(time,des_alt_dat(1,:),'k--','LineWidth',2),hold on
plot(time,pos_dat(3,:),'b')
legend('Desired','Actual')
title('Altitude (Z-position)')
ylabel('Altitude (m)')
xlabel('Time (s)')
