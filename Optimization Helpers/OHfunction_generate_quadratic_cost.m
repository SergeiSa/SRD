%this function helps to generate quadratic cost functions.
%Helpful when there are many variables and cost is an additive function
%with individual components involving only a few of the variables; 
%Example: var - x(1:100), cost - x(10)^2+x(25)^2;
%The function is basically used to avoid the pain of padding the right
%amount of zeros to the H matrix in  x*H*x
%
%inputs:
%'Cost' = x*H*x + c*x, the left hand side of the constraint presented as a vector (symbolic expression); 
%'var' = x, the constrained variables (symbolic expression); 
%'Numeric' - if true, the output is numeric
%'FileName_H' - file name where a function generated from the symbolic expression for H will be stored 
%'generated_function_vars' - variables for the generated functions
function [H, c] = OHfunction_generate_quadratic_cost(varargin)
Parser = inputParser;
Parser.FunctionName = 'OHfunction_generate_quadratic_cost';
Parser.addOptional('Cost', []);
Parser.addOptional('var', []); 
Parser.addOptional('Numeric', true); 
Parser.addOptional('FileName_H', []);
Parser.addOptional('FileName_c', []);
Parser.addOptional('generated_function_vars', []);
Parser.parse(varargin{:});

expression = Parser.Results.Cost;
var = Parser.Results.var;
assume(var, 'real');

H = jacobian(jacobian(expression, var), var);
H = simplify(H);

remainder = expression - 0.5*(var' * H * var);
remainder = simplify(remainder);

c = jacobian(remainder, var);
c = simplify(c);

if Parser.Results.Numeric
    H = double(H);
    c = double(c);
end
    
if (~isempty(Parser.Results.FileName_H)) && (~isempty(Parser.Results.generated_function_vars))
    matlabFunction(H, 'File', Parser.Results.FileName_H, 'Vars', {Parser.Results.generated_function_vars});
end
if (~isempty(Parser.Results.FileName_c)) && (~isempty(Parser.Results.generated_function_vars))
    matlabFunction(c, 'File', Parser.Results.FileName_c, 'Vars', {Parser.Results.generated_function_vars});
end

end