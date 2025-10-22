% ==========================================================
% Data logging script for ES456 Gimbal IMU setup
% 
%   Outputs:
%           Acceleration - udot, vdot, wdot (m/s^2)
%           Angular Rate - p, q, r (deg/sec)
%           Sample Time - time (sec)
%           
%   Uses gimbalClass class and basicImuDisplay function
% ==========================================================

% Clear old arrays
clear time udot vdot wdot p q r
% clear time mx my mz hdg

% Create data acquisition object and initialize serial port connection
portNum= 55; % ---CHANGE PORT NUMBER TO CORRESPOND WITH HOST COMPUTER---
gb = gimbalClass(portNum);

%%

StartTime = tic; % all timing info is relative to start, in sec
StopTime = 30; % how long to collect data for
if exist('fig','var') % clear the figure if it was left open
    clear fig
    pause(2)
end

accel_data = [];
gyro_data = [];
euler_data = [];

while(toc(StartTime)<StopTime)    
    
    if(gb.imu_data_flag)

        [t_imu,udot,vdot,wdot,p,q,r] = gb.getImuData(); % Raw IMU data
    
        accel_data = [accel_data;[t_imu udot vdot wdot]];
        gyro_data = [gyro_data;[t_imu p q r]];
    
        gb.imu_data_flag = false;

    end
        
   pause(0.001);
end

pause(1);
% delete(gb)
% clear gb

%% Save Data

save('new.mat','accel_data','gyro_data')

%% Flat Spin

load("flat.mat")
gyro_data(:, 1) = gyro_data(:, 1) - gyro_data(1, 1); % start time at t=0

% Plot p, q, r
plot(gyro_data(:, 1), gyro_data(:, 2:4))
title("Gyro Rates: Flat Spin")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
legend("p", "q", "r", "Location", "northeastoutside")

%% Spin at 45 degree roll

load("fourty_five.mat")
gyro_data(:, 1) = gyro_data(:, 1) - gyro_data(1, 1); % start time at t=0

% Plot p, q, r
plot(gyro_data(:, 1), gyro_data(:, 2:4))
title("Gyro Rates: Spin @ 45 deg roll")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
legend("p", "q", "r", "Location", "northeastoutside")

%% Low Pass Filter

load("random.mat")
accel_data(:, 1) = accel_data(:, 1) - accel_data(1, 1); % start time at t=0

filter_accel = [];
filter_accel(1, 1:3, 1) = accel_data(1, 2:4); % initilize with accel at t=0
filter_accel(1, 1:3, 2) = accel_data(1, 2:4); % initilize with accel at t=0
filter_accel(1, 1:3, 3) = accel_data(1, 2:4); % initilize with accel at t=0

% Set cutoff freq (in rad/s)
a = 2*pi.*[0.1; 1; 10];

for n = 1:size(a,1)
    for i = 2:size(accel_data, 1)
        dt = accel_data(i, 1) - accel_data(i-1, 1);
        if (1 - a(n)*dt) > 0
            filter_accel(i, 1:3, n) = (1 - a(n)*dt)*...
                filter_accel(i-1, 1:3, n) + a(n)*dt*accel_data(i-1, 2:4);
        end
        % filter_accel(i, 1:3, n) = (1 - a(n)*dt)*...
        %     filter_accel(i-1, 1:3, n) + a(n)*dt*accel_data(i-1, 2);
    end
end

time = accel_data(:, 1);

t = tiledlayout(3, 1,'TileSpacing','loose'); % 'compact', 'tight', 'loose', or 'none'

nexttile;
plot(time, accel_data(:, 2))
hold on
plot(time, filter_accel(:, 1, 1)) % plot 
plot(time, filter_accel(:, 1, 2))
% plot(time, filter_accel(:, 1, 3)) % Don't plot 10Hz bc dt is too large
title("$\dot{u}$", "Interpreter", "latex")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
legend("Raw", "Filtered (0.1Hz)", "Filtered (1Hz)", "Filtered (10Hz)", "Location", "northeastoutside")
hold off

nexttile;
plot(time, accel_data(:, 3))
hold on
plot(time, filter_accel(:, 2, 1))
plot(time, filter_accel(:, 2, 2))
% plot(time, filter_accel(:, 2, 3)) % Don't plot 10Hz bc dt is too large
title("$\dot{v}$", "Interpreter", "latex")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
legend("Raw", "Filtered (0.1Hz)", "Filtered (1Hz)", "Filtered (10Hz)", "Location", "northeastoutside")
hold off

nexttile;
plot(time, accel_data(:, 4))
hold on
plot(time, filter_accel(:, 3, 1))
plot(time, filter_accel(:, 3, 2))
% plot(time, filter_accel(:, 3, 3)) % Don't plot 10Hz bc dt is too large
title("$\dot{w}$", "Interpreter", "latex")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")
legend("Raw", "Filtered (0.1Hz)", "Filtered (1Hz)", "Filtered (10Hz)", "Location", "northeastoutside")
hold off

%% Estimating Attitude

StartTime = tic; % all timing info is relative to start, in sec
StopTime = 10; % how long to collect data for
if exist('fig','var') % clear the figure if it was left open
    clear fig
    pause(2)
end

accel_data = [];
gyro_data = [];
euler_data = [];

while(toc(StartTime)<StopTime)    
    
    if(gb.imu_data_flag)

        [t_imu,udot,vdot,wdot,p,q,r] = gb.getImuData(); % Raw IMU data

        % Calculate Euler Angles
        theta = asin(udot/9.81);
        theta_deg = rad2deg(theta);

        phi = asin(vdot/(-9.81*cos(theta)));
        phi_deg = rad2deg(phi);

        psi = 0;
    
        accel_data = [accel_data;[t_imu udot vdot wdot]];
        gyro_data = [gyro_data;[t_imu p q r]];
        euler_data = [euler_data;[t_imu, phi_deg, theta_deg, psi]];

        gb.imu_data_flag = false;

        fprintf('Roll(deg) = %.1f, Pitch(deg) = %.1f \n', ...
            [phi_deg, theta_deg]);

    end
        
   pause(0.001);
end

pause(1);

%% Save Data

save('new.mat','accel_data','gyro_data', 'euler_data')

%% Plot Attitude

load("atitude.mat")
euler_data(:, 1) = euler_data(:, 1) - euler_data(1, 1); % start at t=0

yline(45) % desired roll and pitch
hold on
plot(euler_data(:, 1), euler_data(:, 2:3)) % plot measured roll and pitch
yline(min(euler_data(:, 2)), 'g')
yline(max(euler_data(:, 2)), 'g')
yline(min(euler_data(:, 3)), 'g')
yline(max(euler_data(:, 3)), 'g')
hold off
ylim([35, 50])
yticks(35:1:50)
title("Estimating Roll and Pitch Angle")
xlabel("Time (s)")
ylabel("Attitude Angle (deg)")
legend("Actual Roll/Pitch", "Estimated Roll", "Estimated Pitch")

fprintf("Roll range = %.1f Pitch range = %.1f\n",...
    [max(euler_data(:, 2))-min(euler_data(:, 2)), ...
    max(euler_data(:, 3))-min(euler_data(:, 3))])
