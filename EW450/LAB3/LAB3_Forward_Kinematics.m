%% Estimate Link Lengths

% Initialize UR3e robot
ur = URQt('UR3e');
ur.Initialize; 

X1 = []; X2 = []; X3 = []; X4 = []; X5 = [];

%% Joint 2

% Move from start to end angles
ur.WaitOn = false;
ur.Home;
ur.WaitForMove;
ur.Joint2 = -3*pi/4;
ur.WaitForMove;
ur.Joint2 = -pi/4;

% Record end effector pose trajectory
while ur.isMoving
    H_e2o = ur.Pose;
    X2(:,end+1) = H_e2o(1:3,4);
end

% Fit to circle and calculate link length
cfit2 = fitCircle(X2);
L1 = cfit2.Center(3);

%% Joint 4

% Move from start to end angles
ur.WaitOn = false;
ur.Home;
ur.WaitForMove;
ur.Joint4 = -pi;
ur.WaitForMove;
ur.Joint4 = 0;

% Record end effector pose trajectory
while ur.isMoving
    H_e2o = ur.Pose;
    X4(:,end+1) = H_e2o(1:3,4);
end

% Fit to circle and calculate link length
cfit4 = fitCircle(X4);
L5 = cfit4.Radius;

%% Joint 5

% Move from start to end angles
ur.WaitOn = false;
ur.Home;
ur.WaitForMove;
ur.Joint5 = -pi;
ur.WaitForMove;
ur.Joint5 = pi;

% Record end effector pose trajectory
while ur.isMoving
    H_e2o = ur.Pose;
    X5(:,end+1) = H_e2o(1:3,4);
end

% Fit to circle and calculate link length
cfit5 = fitCircle(X5);
L6 = cfit5.Radius;

%% Joint 3

% Move from start to end angles
ur.WaitOn = false;
ur.Home;
ur.WaitForMove;
ur.Joint3 = -pi/2;
ur.WaitForMove;
ur.Joint3 = pi/2;

% Record end effector pose trajectory
while ur.isMoving
    H_e2o = ur.Pose;
    X3(:,end+1) = H_e2o(1:3,4);
end

% Fit to circle and calculate link length
cfit3 = fitCircle(X3);
L3 = cfit3.Radius - L5;

%% Joint 1

% Move from start to end angles
ur.WaitOn = false;
ur.Home;
ur.WaitForMove;
ur.Joint1 = -pi;
ur.WaitForMove;
ur.Joint1 = pi;

% Record end effector pose trajectory
while ur.isMoving
    H_e2o = ur.Pose;
    X1(:,end+1) = H_e2o(1:3,4);
end

% Fit to circle and calculate link length
cfit1 = fitCircle(X1);
L4 = cfit1.Radius - L6;

%% Calculate remaining link length

L2 = cfit2.Radius - L3 -L5;

%% Compare Kinematics

q_all = []; H_e2o_all= [];

% Record 20 poses in Local mode
for i = 1:20
    q_all(:,i) = ur.Joints;
    H_e2o_all{i} = ur.Pose;
    fprintf('Recorded pose %d\n', i);
    pause; 
end    

%% Save data

save('Lab3 Data.mat',...
'X1','X2','X3','X4','X5',...
'cfit1','cfit2','cfit3','cfit4','cfit5',...
'L1','L2','L3','L4','L5','L6',...
'q_all','H_e2o_all');

%% Graph comaparing fkin using published and estimated lenghts to GT

% Add link lengths to array
L_pub = [151.85, 243.55, 213.20, 131.05, 85.35, 92.10];
L_exp = [L1, L2, L3, L4, L5, L6];

fig = figure;
axs = axes('Parent',fig);
hold(axs,'on');
daspect(axs,[1 1 1]);
title(axs, 'Forward Kinematics of the UR3e');
xlabel(axs,'x (mm)');
ylabel(axs,'y (mm)');
zlabel(axs,'z (mm)');

for i = 1:20
    X_con = H_e2o_all{i}(1:3,4);
    H_e2o_pub = fkin(q_all(:,i),L_pub);
    X_pub = H_e2o_pub(1:3,4);
    H_e2o_exp = fkin(q_all(:,i),L_exp);
    X_exp = H_e2o_exp(1:3,4);
    plot3(axs,X_con(1),X_con(2),X_con(3),'rx');
    plot3(axs,X_pub(1),X_pub(2),X_pub(3),'g+');
    plot3(axs,X_exp(1),X_exp(2),X_exp(3),'bo');
end

legend(axs, {'Controller', 'Published', 'Estimated'});

%% Plot fitted cirlces

% h1 = plotCircle(cfit1);
% h2 = plotCircle(cfit2);
% h3 = plotCircle(cfit3);
% h4 = plotCircle(cfit4);
h5 = plotCircle(cfit5);

title('Joint 5 Circle');
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');