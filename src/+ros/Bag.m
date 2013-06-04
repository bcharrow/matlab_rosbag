classdef Bag
    % Bag A class for reading data from ROS bags.
    properties
        handle = -1;
        cleanup = @(x) x;
    end

    methods
        function [obj] = Bag(bag)
            [~, ~, endian] = computer;
            if endian ~= 'L'
                error(['This machine is not little endian; ' ...
                       'rosbag_wrapper() won''t work']);
            end
            obj.handle = rosbag_wrapper(uint64(0), 'construct', bag);
            h = obj.handle;
            % Tell wrapper to destroy handle when we're deleted, but don't
            % generate an error if handle no longer exists
            % (e.g., due to clearing all envirnoment variables)
            obj.cleanup = onCleanup(@() rosbag_wrapper(uint64(0), 'destruct', h, false));
        end

        function [] = resetView(obj, topics)
        % Reset which topics to read messages from
        %
        % resetView(topics) jumps to the beginning of the bagfile and changes
        % the view to be topics.  topics can be a string or a cell array of
        % strings.
            rosbag_wrapper(obj.handle, 'resetView', topics)
        end

        function [hn] = hasNext(obj)
        % Return true if there is at least one more message to read
        % [hn] = hasNext()
            hn = rosbag_wrapper(obj.handle, 'hasNext');
        end

        function [msg, meta] = readMessage(obj, flatten)
        % Read a message from the bag
        % [MSG] = readMessage() gets the next message from the bag.  By default,
        % messages are flattened.
        %
        % [MSG] = readMessage(flatten) flattens messages if flatten is true.
        %
        % [MSG, META] = readMessage(...) gets the next message and return meta
        % data associated with it.
            if nargin < 2
                flatten = true;
            else
                flatten = logical(flatten);
            end

            if nargout == 1
                msg = rosbag_wrapper(obj.handle, 'readMessage', false, flatten);
            else
                [msg, meta] = rosbag_wrapper(obj.handle, 'readMessage', true, flatten);
            end
        end

        function [msg, meta] = readAllMessages(obj, topics, flatten)
        % Read remaining messages from the bag
        % [MSG] = readAllMessages() returns all messages from the current
        % point on as a cell array.
        %
        % [MSG, META] = readAllMessages(...) returns meta data for each message
        % in a cell array.
        %
        % [...] = readAllMessages(topics) resets the view to 'topics' and then
        % reads all messages.
            if nargin > 1
                obj.resetView(topics)
            end

            if nargin < 3
                flatten = true;
            else
                flatten = logical(flatten);
            end


            if nargout == 1
                msg = rosbag_wrapper(obj.handle, 'readAllMessages', false, ...
                                     flatten);
            else
                [msg, meta] = rosbag_wrapper(obj.handle, 'readAllMessages', ...
                                             true, flatten);
            end
        end

        function [info] = info(obj)
        % Get information about the bag's contents (similar to 'rosbag info')
        % [INFO] = info()
            info = rosbag_wrapper(obj.handle, 'info');
        end

        function [defn] = definition(obj, msg_type)
        % Get a string describing fields in a message (similar to 'rosmsg show')
        %
        % [msg_def] = definition(msg_type) where msg_type is a ROS message type
        % (e.g., 'geometry_msgs/Vector3').  Also works if msg_type is
        % unqualified (e.g., 'Vector3').
            defn = rosbag_wrapper(obj.handle, 'definition', msg_type);
        end

        function [raw_defn] = rawDefinition(obj, msg_type)
        % Get a message's raw definition (similar to 'rosmsg show --raw')
        %
        % [msg_def] = rawDefinition(msg_type) where msg_type is a ROS message
        % type (e.g., 'geometry_msgs/Vector3').  Also works if msg_type is
        % unqualified (e.g., 'Vector3').
            raw_defn = rosbag_wrapper(obj.handle, 'rawDefinition', msg_type);
        end
    end
end
