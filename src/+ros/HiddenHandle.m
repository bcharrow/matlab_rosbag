classdef HiddenHandle < handle
    %HiddenHandle A superclass to hide documentation of unused functions

    % See http://www.mathworks.com/matlabcentral/answers/47751
    methods (Hidden)
        function addlistener(varargin)
            addlistener@handle(varargin{:})
        end

        function notify(varargin)
            notify@handle(varargin{:})
        end

        function [] = delete(varargin)
            delete@handle(varargin{:});
        end

        function [out] = findobj(varargin)
            out = findobj@handle(varargin{:});
        end

        function [out] = lt(varargin)
            out = lt@handle(varargin{:});
        end

        function [out] = le(varargin)
            out = le@handle(varargin{:});
        end

        function [out] = ne(varargin)
            out = ne@handle(varargin{:});
        end

        function [out] = gt(varargin)
            out = gt@handle(varargin{:});
        end

        function [out] = ge(varargin)
            out = ge@handle(varargin{:});
        end

        function [out] = eq(varargin)
            out = eq@handle(varargin{:});
        end

        function [out] = findprop(varargin)
            out = findprop@handle(varargin{:});
        end
    end
end
