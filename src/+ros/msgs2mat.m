function [mat] = msgs2mat(msgs, accessor, converter)
% MSGS2MAT Convert ROS messages to a matrix 
% MAT = MSGS2MAT(msgs, accessor) Returns an M-by-N matrix where M is the number
% of fields in each message and N is the length of msgs.  accessor is a 
% function which takes a message and returns a struct whose entries are numeric.
% Each struct is converted to a column vector by stacking its fields in order.
%
% MAT = MSG2MAT(msgs, accessor, converter)  Apply the function converter to each
% struct from accessor.  converter should take a struct as input and return a
% vector.
if nargin < 3
    converter = @(msg) cell2mat(struct2cell(msg));
end
columnify = @(x) x(:);
mat = cellfun(@(x) columnify(converter(accessor(x))), msgs, ...
              'UniformOutput', false);
mat = [mat{:}];
end
