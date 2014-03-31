function [mat] = msgs2mat(msgs, accessor, converter)
% MSGS2MAT Convert ROS messages to a matrix
% MAT = MSGS2MAT(msgs, accessor) Returns an M-by-N matrix where M is the number
% of fields in each message and N is the length of msgs.  accessor is a
% function which takes a message and returns a vector or a struct whose entries
% are numeric.  If a struct is returned, it is converted to a vector by stacking
% its fields in order.  By default, accessor is the identity function.
%
% MAT = MSG2MAT(msgs, accessor, converter)  Apply the function converter to each
% struct from accessor.  converter should take the output of accessor and
% return a vector.
if nargin < 2 || isempty(accessor)
    accessor = @(x) x;
end
if nargin < 3
    converter = @default_converter;
end
columnify = @(x) x(:);
mat = cellfun(@(x) columnify(converter(accessor(x))), msgs, ...
              'UniformOutput', false);
mat = [mat{:}];
end

function [vector] = default_converter(element)
if isstruct(element)
    vector = cell2mat(struct2cell(element));
elseif isnumeric(element)
    vector = element;
else
    error('unknown');
end
end
