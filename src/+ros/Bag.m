classdef Bag
    % Bag A class for reading data from ROS bags.
    properties
        handle = -1;
        path = '';
        cleanup = @(x) x;
    end

    methods
        function [obj] = Bag(path)
            [~, ~, endian] = computer;
            if endian ~= 'L'
                error(['This machine is not little endian; ' ...
                       'rosbag_wrapper() won''t work']);
            end
            obj.path = path;
            obj.handle = rosbag_wrapper(uint64(0), 'construct', obj.path);
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

        function [msg, meta] = read(obj, flatten)
        % Get the next message
        % [MSG] = read() returns the next message from the bag as a struct.  By
        % default, messages are flattened if possible.
        %
        % [MSG] = read(flatten) flattens messages if flatten is true.
        %
        % [MSG, META] = read(...) gets the next message and return meta
        % data associated with it.
        %
        % A message can be flattened if all its members are fixed size
        % primitives (i.e., all ROS primitives except strings).  Instead of
        % being returned as a struct, a flattened field is returned as an
        % M-By-N matrix of doubles where M is the number of fields in the
        % message and N is the length of the array.  For example, a flattened
        % geometry_msgs/Vector3[5] would be returned as a 3-by-5 matrix.
            if nargin < 2
                flatten = true;
            else
                flatten = logical(flatten);
            end

            if nargout == 1
                msg = rosbag_wrapper(obj.handle, 'read', false, flatten);
            else
                [msg, meta] = rosbag_wrapper(obj.handle, 'read', true, flatten);
            end
        end

        function [msg, meta] = readAll(obj, topics, flatten)
        % Read remaining messages
        % [MSG] = readAll() returns all messages from the current point on
        % as a cell array.  By default, messages are flattened.
        %
        % [MSG, META] = readAll(...) returns meta data for each message in a
        % cell array.
        %
        % [...] = readAll(topics) resets the view to 'topics' and then reads
        % all messages.
            if nargin > 1
                obj.resetView(topics)
            end

            if nargin < 3
                flatten = true;
            else
                flatten = logical(flatten);
            end


            if nargout == 1
                msg = rosbag_wrapper(obj.handle, 'readAll', false, flatten);
            else
                [msg, meta] = rosbag_wrapper(obj.handle, 'readAll', true, ...
                                             flatten);
            end
        end

        function [info] = info(obj)
        % Get information about the bag's contents (similar to 'rosbag info')
        % [INFO] = info()
            info = rosbag_wrapper(obj.handle, 'info');
        end

        function [defn] = definition(obj, msg_type, raw)
        % Get a string describing fields in a message
        % [MSG_DEF] = definition(msg_type, ...) where msg_type is a ROS message
        % type (e.g., 'geometry_msgs/Vector3').  Also works if msg_type is
        % unqualified (e.g., 'Vector3').
        %
        % [MSG_DEF] = definition(msg_type, raw) returns a definition similar to
        % 'rosmsg show' if raw is false and returns output similar to
        % 'rosmsg show --raw' otherwise.  By default raw is false.
            if nargin < 3
                raw = false;
            else
                raw = logical(raw);
            end
            defn = rosbag_wrapper(obj.handle, 'definition', msg_type, raw);
        end

        function [out] = disp(obj)
            fprintf('ros.Bag(''%s'')\n\n', obj.path);
        end

        function [type] = topicType(obj, topic)
        % Get a string listing the message type of topic or topics
        % [type] = topics(topic)
            type = rosbag_wrapper(obj.handle, 'topicType', topic);
        end

        function [topic, types] = topics(obj, regex)
        % Get a cell array listing all topics in bag and their types.
        % Optionally, use regular expression regex to select topics.
        % [topic, types] = topics(regex)
            if nargin < 2
                regex = '.*';
            end
            topic = rosbag_wrapper(obj.handle, 'topics', regex);
            if nargout > 1
                types = rosbag_wrapper(obj.handle, 'topicType', topic);
            end
        end

    end
end
