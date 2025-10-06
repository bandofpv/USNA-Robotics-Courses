%% q6Toq4
% Input 1: q6 - 6 × 1 array containing the 6-element joint configuration  
%   used by the UR3e manipulator. Note that 
%   q6 = (θ1, θ2, θ3, θ4, θ5, θ6)^T.
% Output 1: q4 - 4 × 1 array containing the 4-element joint configuration
%   used for our Jacobian. Note that q4 = (θ1, θ2, θ3, θ6)^T.

function q4 = q6Toq4(q6)
    theta1 = q6(1); theta2 = q6(2); theta3 = q6(3); theta6 = q6(6);
    q4 = [theta1; theta2; theta3; theta6];
end