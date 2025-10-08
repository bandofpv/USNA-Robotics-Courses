%% Inverse Kinematics Function
% X - 4-element array containing UR3e task configuration such that X = (x, y, z, ϕ)
%   where x, y, and z are defined in millimeters, and ϕ is defined in radians.
% L - 6-element array containing UR3e link lengths in millimeters.
% q_eu - 6-element array containing UR3e elbow-up joint configuration in radians.
% q_ed - 6-element array containing UR3e elbow-down joint configuration in radians.

function [q_eu, q_ed] = ikinPickAndPlace(X,L)
    % extract data from inputs
    x = X(1); y = X(2); z = X(3); phi = X(4);
    l_1 = L(1); l_2 = L(2); l_3 = L(3); l_4 = L(4); l_5 = L(5); l_6 = L(6);

    % fix theta_5
    theta_5 = -(pi/2);

    a_1 = -y;
    b_1 = -x;
    c_1 = sqrt(a_1^2 + b_1^2);
    alpha_1 = atan2(a_1, b_1);

    a_2 = l_4; 
    b_2 = sqrt(c_1^2 - a_2^2);
    alpha_2 = atan2(a_2, b_2);

    theta_1 = alpha_1 - alpha_2;

    c_3 = b_2 - l_5;

    a_4 = (z + l_6) - l_1;
    c_4 = sqrt(c_3^2 + a_4^2);
    alpha_4 = atan2(a_4, c_3);
    
    b_5 = l_2;
    a_5 = l_3;
    alpha_5 = acos((b_5^2 + c_4^2 - a_5^2) / (2*b_5*c_4));
    gamma_5 = acos((a_5^2 + b_5^2 - c_4^2) / (2*a_5*b_5));

    % calculate elbow-up specific joint angles
    theta_2_eu = -(alpha_4 + alpha_5);
    theta_3_eu = pi - gamma_5;
    theta_4_eu = -(theta_2_eu + theta_3_eu + (pi/2));

    % calculate elbow-down specific joint angles
    theta_2_ed = -(alpha_4 - alpha_5);
    theta_3_ed = gamma_5 - pi;
    theta_4_ed = -(theta_2_ed + theta_3_ed + (pi/2));

    theta_6 = (pi/2) + theta_1 - phi;

    % save joint angles for elbow-up and elbow-down 
    q_eu = [theta_1, theta_2_eu, theta_3_eu, theta_4_eu, theta_5, theta_6];
    q_ed = [theta_1, theta_2_ed, theta_3_ed, theta_4_ed, theta_5, theta_6];
end