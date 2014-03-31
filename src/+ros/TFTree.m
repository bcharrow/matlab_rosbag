classdef TFTree < ros.HiddenHandle
    %TFTree Dealing with TF information from a bag
    properties
        bag = -1;  % Bag associated with this tree.
        topic = ''; % Topic tf messages are read from.
        time_begin = -1; % Timestamp of first valid transformation (seconds since epoch)
        time_end = -1; % Timestamp of last valid transformation (seconds since epoch)
    end

    properties(Hidden=true)
        handle = -1;
        cleanup = @(x) x;
    end

    methods
        function [obj] = TFTree(varargin)
            % Construct a tree and optionally build it.
            %
            % Accepts arguments to TFTree.build()
            obj.handle = rosbag_wrapper(uint64(0), 'construct', 'TFWrapper');
            h = obj.handle;
            obj.cleanup = onCleanup(@() rosbag_wrapper(uint64(0), 'destruct', h, false));
            if ~isempty(varargin);
                obj.build(varargin{:});
            end
        end

        function [] = disp(obj)
            fprintf('ros.TFTree()\n');
            if isempty(obj) || isempty(obj.topic)
                fprintf('  Not built yet\n\n');
            else
                fprintf('  Built with ros.Bag(''%s'') on %s\n', ...
                    obj.bag.path, obj.topic);
                if (obj.time_end == 1e-9)
                    fprintf('  No transforms found\n\n');
                else
                    ts = @(unix_sec) datestr(unix_sec/86400 + datenum(1970,1,1));
                    fprintf('  Transforms from %s to %s\n\n', ...
                        ts(obj.time_begin), ts(obj.time_end));
                end
            end
        end

        function [obj] = build(obj, bag, start_time, end_time, tf_topic)
        % Create the TF tree using a bag
        %
        % build(bag) builds the tree using all messages in the bag on the
        % /tf topic.
        %
        % build(bag, start_time, end_time) only uses messages whose
        % timestamp are in between start_time and end_time.
        %
        % build(bag, [], [], tf_topic) builds the tree using messages on
        % tf_topic instead of /tf
            obj.bag = bag;
            if nargin < 3 || isempty(start_time)
                start_time = 0.0;
            end
            if nargin < 4 || isempty(end_time)
                end_time = double(intmax());
            end
            if nargin < 5 || isempty(tf_topic)
                tf_topic = '/tf';
            end
            obj.topic = tf_topic;
            [obj.time_begin, obj.time_end] = rosbag_wrapper(obj.handle, ...
                'build', bag.handle, start_time, end_time, tf_topic);
            obj.time_begin = obj.time_begin.time;
            obj.time_end = obj.time_end.time;
        end

        function [frames] = allFrames(obj)
        % Get a string describing all TF frames
        %
        % Must call build() first
            frames = rosbag_wrapper(obj.handle, 'allFrames');
        end

        function [transforms] = lookup(obj, target_frame, source_frame, times, just2d)
        % Get the transformation(s) between two frames at various points in time
        %
        % [transforms] = lookup(target_frame, source_frame, times)
        % returns a struct array whose ith element is the 3D transform which
        % projects data in source_frame to target_frame at times(i).
        %
        % [transforms] = lookup(target_frame, source_frame, times, true)
        % returns a matrix of transforms; columns are 2D transformations (x, y, yaw)
        %
        % Must call build() first
        if nargin < 5
            just2d = false;
        else
            just2d = logical(just2d);
        end
        transforms = rosbag_wrapper(obj.handle, 'lookup', target_frame, ...
            source_frame, times, just2d);
        end
    end
end
