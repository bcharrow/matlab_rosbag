function [xyt] = pose2xyt(pose_msg)
% xyt = POSE2XYT(pose_msg) Convert a 3D pose struct to a 2D pose vector 
% (x, y, yaw)
xyt = zeros(3, 1);
xyt(1:2) = pose_msg.position(1:2);
quat = pose_msg.orientation;
xyt(3) = atan2(2 * (quat(3) * quat(4) + quat(1) * quat(2)), ...
               1 - 2 * (quat(2) * quat(2) + quat(3) * quat(3)));
end
