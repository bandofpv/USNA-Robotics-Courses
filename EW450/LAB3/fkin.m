%% Forward Kinematics Function

% q - 6-element array containing UR3e joint angles in radians.
% L - 6-element array containing UR3e link lengths in millimeters.
% H_e2o - 4 × 4 element of SE(3) describing the pose of the UR3e 
%   end-effector frame relative to the base frame, H0
function H_e2o = fkin(q,L)
    H_one2o = Rz(pi + q(1))*Rx(pi/2)*Ty(L(1));
    H_two2one = Rz(-q(2))*Tx(L(2));
    H_three2two = Rz(-q(3))*Tx(L(3));
    H_four2three = Rz(-q(4))*Rx(pi/2)*Ty(-L(4));
    H_five2four = Rz(q(5))*Rx(-pi/2)*Ty(-L(5));
    H_six2five = Rz(pi - q(6))*Rx(pi)*Tz(L(6));
    H_e2o = H_one2o*H_two2one*H_three2two*H_four2three*H_five2four*H_six2five;
end