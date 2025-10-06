%% JacobianPickAndPlace
% Input 1: in1 - 4×1 array containing the UR3e joint configuration q such
%   that q = (θ1, θ2, θ3, θ6)^T where θi terms are defined in radians.
% Input 2: in2 - 6 × 1 array containing UR3e link lengths in millimeters.
% Output 1: J - 4 × 4 array containing UR3e pick-and-place Jacobian matrix 
%   for the specified joint configuration.

syms theta1 theta2 theta3 theta6 L1 L2 L3 L4 L5 L6

% Define X ( and q symbolically
X = [-L2*cos(theta1)*cos(theta2) - L3*cos(theta1)*cos(theta2+theta3) ...
    + L4*sin(theta1) - L5*cos(theta1); ...
    -L2*sin(theta1)*cos(theta2) - L3*sin(theta1)*cos(theta2+theta3) ...
    - L4*cos(theta1) - L5*sin(theta1);
    L1 - L2*sin(theta2) - L3*sin(theta2+theta3) - L6;
    pi/2 + theta1 - theta6];
q = [theta1; theta2; theta3; theta6];

% Calculate the Jacobian matrix
J_sym = jacobian(X,q);

matlabFunction(J_sym,...
    'Vars',{[theta1; theta2; theta3; theta6],[L1; L2; L3; L4; L5; L6]}, ...
    'File','JacobianPickAndPlace', ...
    'Comments','UR3e Pick And Place Jacobian');