%% Initialize Simulation

client = RemoteAPIClient();
sim = client.require('sim');

sim.setStepping(true);

%% Run Simulation

% object handlers
robot_h = sim.getObject('/robot');
right_motor_h = sim.getObject('/robot/right_motor');
left_motor_h = sim.getObject('/robot/left_motor');

% initialize variables
time = 0;
pos = [];
vel = [];

i = 0; % index
sim.startSimulation();
while true
    i = 1 + i; % increment index
    t = sim.getSimulationTime();

    if t >= 30; break; end
    fprintf('Simulation time: %.2f [s]\n', t);

    % drive in a circle
    if t < 20
        sim.setJointTargetVelocity(right_motor_h, 1);
        sim.setJointTargetVelocity(left_motor_h, 3.5);
    end

    % stop
    if t >= 20 && t < 22
        sim.setJointTargetVelocity(right_motor_h, 0);
        sim.setJointTargetVelocity(left_motor_h, 0);
    end

    % drive in a straight line
    if t >= 22
        sim.setJointTargetVelocity(right_motor_h, 2);
        sim.setJointTargetVelocity(left_motor_h, 2);
    end

    % store time, position, and velocity into array
    time(i) = t;
    pos_cell = sim.getObjectPosition(robot_h);
    pos(i,:) = cell2mat(pos_cell);
    vel_cell = sim.getObjectVelocity(robot_h);
    vel(i,:) = cell2mat(vel_cell);

    sim.step();
end

sim.stopSimulation();

%% Save Data

save("LAB2_Data.mat", 'time', 'pos', 'vel')

%% Plot Graphs

% plot x position vs. y position
figure(1)
plot(pos(:,1), pos(:,2));
title("X Position vs. Y Position")
xlabel("X Position (m)")
ylabel("Y Position (m)")
axis equal;

% plot x and y velocities vs time
figure(2)
subplot(2, 1, 1)
plot(time, pos(:, 1));
title("X Velocity vs. Time")
xlabel("Time (s)")
ylabel("X Velocity(m/s)")
subplot(2, 1, 2)
plot(time, vel(:,2));
title("Y Velocity vs. Time")
xlabel("Time (s)")
ylabel("Y Velocity(m/s)")