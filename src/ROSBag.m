classdef ROSBag < handle
    properties
        handle = -1;
        cleanup = @(x) x;
    end

    methods
        function [obj] = ROSBag(bag)
            obj.handle = rosbag_wrapper(uint64(0), 'construct', bag);
            h = obj.handle;
            % Tell wrapper to destroy handle when we're deleted, but don't
            % generate an error if handle no longer exists
            % (e.g., do to clearing all envirnoment variables)

            obj.cleanup = onCleanup(@() rosbag_wrapper(uint64(0), 'destruct', h, false));
        end

        function [] = resetView(obj, topics)
            rosbag_wrapper(obj.handle, 'resetView', topics)
        end

        function [hn] = hasNext(obj)
            hn = rosbag_wrapper(obj.handle, 'hasNext');
        end

        function [msg, meta] = readMessage(obj)
            if nargout == 1
                msg = rosbag_wrapper(obj.handle, 'readMessage', false);
            else
                [msg, meta] = rosbag_wrapper(obj.handle, 'readMessage', true);
            end
        end
    end
end
