%% poseToTask
%Input 1: H_e2b - 4 × 4 matrix element of SE(3) representing the 
%   end-effector pose of the UR3e manipulator.
% Output 1: X - 4×1 array containing the task configuration of the UR3e 
%   manipulator such that X = (x, y, z, ϕ)^T.

function X = poseToTask(H_e2b)
    phi = atan2(H_e2b(2,1), H_e2b(1,1));
    X = [H_e2b(4,1); H_e2b(4,2); H_e2b(4,3); phi];
end