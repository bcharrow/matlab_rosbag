clear rosbag_wrapper;
clear ros.Bag;
clear all
%% Read all messages from a bag on a set of topics
bagfile = 'example.bag';
topic1 = '/scarab1/amcl_pose';
topic2 = '/scarab1/odom';
bag = ros.Bag(bagfile);
msgs = bag.readAll({topic1, topic2});

%% Re-read msgs on topic1 and get their metadata

[msgs, meta] = bag.readAll(topic1);

%% Read messages incrementally
bag.resetView(topic1);
count = 0;
while bag.hasNext();
    [msg, meta] = bag.read();
    count = count + 1;
end

%% Get information about the bag

bag.info()

%% See definitions of messages contained within the bag
bag.definition('nav_msgs/OccupancyGrid')

% Note that geometry_msgs/Quaternion's definition is present even though it's
% not in bag.info(); the definition comes from nav_msgs/OccupancyGrid.
%
% Setting 'raw' to true shows the original message definition with comments
raw = true;
bag.definition('Quaternion', raw)

%% Convert pose messages to a matrix and plot xy trajectory
msgs = bag.readAll(topic1);
accessor = @(msg) msg.pose.pose;
converter = @ros.pose2xyt;
xytheta = ros.msgs2mat(msgs, accessor, converter);
% z-dim is time
plot3(xytheta(1, :).', xytheta(2, :).', 1:length(xytheta), '.')
