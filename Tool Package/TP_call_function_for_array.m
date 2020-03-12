%calls function function_handle( ) with a few scalar-vector inputs for
%every line in input arrays
%
%EXAMPLE
%
%q = rand(1000, 5)
%v = rand(1000, 5)
%vC = TP_call_function_for_array(@get_vC, q, v)
%
%
%EXAMPLE
%
%x1 = [1; 2; 3]
%x2 = [12; 22; 32]
%x3 = [22; 33; 44]
%func = @(x, y, z) x+y+z
%vC = TP_call_function_for_array(func, x1, x2, x3)
%
function output = TP_call_function_for_array(function_handle, varargin)

n = length(varargin);
m = size(varargin{1}, 1);
inp = cell(n, 1);

for i = 1:m
    
    for j = 1:n
        inp{j} = varargin{j}(i, :);
    end
    y = function_handle(inp{:});
    
    if i == 1
        output = zeros(m, length(y));
    end
    
    output(i, :) = y;
end
end