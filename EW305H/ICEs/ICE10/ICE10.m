%% PD Controller

Gp = zpk([],[0,-4,-6],1); % define the plant TF as an object
Gc = zpk(-3, [], 1); % define the compensator TF
H = zpk([], [], 1); % define the sensor TF
K = 48.7;

OLTF = Gc*Gp*H; % blocks in cascade
CLTF = feedback(K*Gc*Gp,H); % negative feedback loop: forward path K*Gc*Gp & feedback H

[y,tOut] = step(CLTF, 5); % unit step on CLTF
S = stepinfo(CLTF); % calculate step response info
T_s = S.SettlingTime;
OS = S.Overshoot;
fprintf('PD Controller:\n T_s=%f s\n OS=%f \n',T_s,OS)

figure; clf;
plot(tOut, y)
xline(S.SettlingTime, 'g')
title("Step Response: PD Controller")
xlabel("Time (sec)")
ylabel("U(t)")
legend("Response", "T_s", "Location", "southeast")

figure; clf;
rlocus(OLTF); % generate the RL for positive K
title("Root Locus: PD Controller")
sgrid; % overlay a grid in polar coordinates
axis; % adjust ranges along x and y axes

%% Lead Compensator

Gp = zpk([],[0,-4,-6],1); % define the plant TF as an object
Gc = zpk(-2.1, -47.7, 1); % define the compensator TF
H = zpk([], [], 1); % define the sensor TF
K = 2100;

OLTF = Gc*Gp*H; % blocks in cascade
CLTF = feedback(K*Gc*Gp,H); % negative feedback loop: forward path K*Gc*Gp & feedback H

[y,tOut] = step(CLTF, 5); % unit step on CLTF
S = stepinfo(CLTF); % calculate step response info
T_s = S.SettlingTime;
OS = S.Overshoot;
fprintf('Lead Compensator:\n T_s=%f s\n OS=%f \n',T_s,OS)

figure; clf;
plot(tOut, y)
xline(S.SettlingTime, 'g')
title("Step Response: Lead Controller")
xlabel("Time (sec)")
ylabel("U(t)")
legend("Response", "T_s", "Location", "southeast")

figure; clf;
rlocus(OLTF); % generate the RL for positive K
title("Root Locus: Lead Compensator")
sgrid; % overlay a grid in polar coordinates
axis; % adjust ranges along x and y axes

%% Uncompensated

Gp = zpk([],[0,-4,-6],1); % define the plant TF as an object
Gc = zpk([], [], 1); % define the compensator TF
H = zpk([], [], 1); % define the sensor TF
K = 43;

OLTF = Gc*Gp*H; % blocks in cascade
CLTF = feedback(K*Gc*Gp,H); % negative feedback loop: forward path K*Gc*Gp & feedback H

[y,tOut] = step(CLTF, 5); % unit step on CLTF
S = stepinfo(CLTF); % calculate step response info
T_s = S.SettlingTime;
OS = S.Overshoot;
fprintf('Uncompensated:\n T_s=%fs\n OS=%f \n',T_s,OS)

figure; clf;
plot(tOut, y)
xline(S.SettlingTime, 'g')
title("Step Response: Uncompensated")
xlabel("Time (sec)")
ylabel("U(t)")
legend("Response", "T_s", "Location", "southeast")

figure; clf;
rlocus(OLTF); % generate the RL for positive K
title("Root Locus: Uncompensated")
sgrid; % overlay a grid in polar coordinates
axis; % adjust ranges along x and y axes