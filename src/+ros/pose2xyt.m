function [xyt] = pose2xyt(pose_msg)
% xyt = POSE2XYT(pose_msg) Convert a 3D pose struct to a 2D pose vector 
% (x, y, yaw)
xyt = zeros(3, 1);
xyt(1:2) = [pose_msg.position.x, pose_msg.position.y];
quat = pose_msg.orientation;
xyt(3) = atan2(2 * (quat.z * quat.w + quat.x * quat.y), ...
               1 - 2 * (quat.y * quat.y + quat.z * quat.z));
end
