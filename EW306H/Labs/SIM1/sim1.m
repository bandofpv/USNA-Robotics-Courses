%% Zero-input
clear;
A = [0 1; -2 -3];
B = [0; 1];
C =[1 1];
D = 0;
u = 0;
x_init = [1; 0];

out = sim('sim1_simulink', 5);

figure;
plot(out.y.Time, out.y.Data);
hold on
plot(out.x.Time, out.x.Data(:,1), '.');
plot(out.x.Time, out.x.Data(:,2), '--');
title("Zero-input (free) Response")
xlabel("Time (sec)")
ylabel("Position (rad)")
legend("y", "x_1", "x_2")

%% Zero-state
clear;
A = [0 1; -2 -3];
B = [0; 1];
C =[1 1];
D = 0;
u = 1;
x_init = [0; 0];

out = sim('sim1_simulink', 5);

figure;
plot(out.y.Time, out.y.Data);
hold on
plot(out.x.Time, out.x.Data(:,1), '.');
plot(out.x.Time, out.x.Data(:,2), '--');
title("Zero-state (forced) Response")
xlabel("Time (sec)")
ylabel("Position (rad)")
legend("y", "x_1", "x_2")

%% Complete Respose
A = [0 1; -2 -3];
B = [0; 1];
C =[1 1];
D = 0;
u = 1;
x_init = [1; 0];

out = sim('sim1_simulink', 5);

figure;
plot(out.y.Time, out.y.Data);
hold on
plot(out.x.Time, out.x.Data(:,1), '.');
plot(out.x.Time, out.x.Data(:,2), '--');
title("Complete Response")
xlabel("Time (sec)")
ylabel("Position (rad)")
legend("y", "x_1", "x_2")

%% State-space Graphs
clear;
A = [0 1; -2 -3];
B = [0; 1];
C =[1 1];
D = 0;
u = 0;
x_1 = [-1 -1 -1 -1 -1 ...
        1 1 1 1 1 ...
        0.75 0.5 0.25 0 -0.25 -0.5 -0.75 ...
        0.75 0.5 0.25 0 -0.25 -0.5 -0.75];
x_2 = [-1 -0.5 0 0.5 1 ...
        -1 -0.5 0 0.5 1 ...
        1 1 1 1 1 1 1 ...
        -1 -1 -1 -1 -1 -1 -1];

figure;
for i = 1:length(x_1)
    x_init = [x_1(i); x_2(i)];
    out = sim('sim1_simulink', 5);
    
    plot(out.x.Data(:,1), out.x.Data(:,2));
    title("State Space Graph")
    xlabel("x_1 (rad)")
    ylabel("x_2 (rad)")
    hold on
end
hold off