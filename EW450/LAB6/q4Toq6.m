%% q4Toq6
% Input 1: q4 - 4 × 1 array containing the 4-element joint configuration
%   used for our Jacobian. Note that q4 = (θ1, θ2, θ3, θ6)^T.
% Output 1: q6 - 6 × 1 array containing the 6-element joint configuration  
%   used by the UR3e manipulator. Note that 
%   q6 = (θ1, θ2, θ3, θ4, θ5, θ6)^T.

function q6 = q4Toq6(q4)
    theta1 = q4(1); theta2 = q4(2); theta3 = q4(3); theta6 = q4(4);
    theta4 = -(theta2 + theta3 + pi/2);
    theta5 = -pi/2;
    q6 = [theta1; theta2; theta3; theta4; theta5; theta6];
end