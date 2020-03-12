%calls function function_handle( ) with a few scalar-vector inputs for
%every line in input arrays
%
%EXAMPLE
%
%q = rand(1000, 5)
%v = rand(1000, 5)
%vC = TP_call_function_for_array(@get_C, q, v)
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