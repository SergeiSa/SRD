%this function helps to generate linear constraints - equality or
%inequality kinds. Helpful when there are many variables and individual
%constrainst involve only a few; 
%Example: var - x(1:100), constraint - x(10)+x(25)=5;
%The function is basically used to avoid the pain of padding the right
%amount of zeros to the A matrix in  A*x = b or A*x <= b
%
%inputs:
%'LHS' = A*x, the left hand side of the constraint presented as a vector (symbolic expression); 
%'RHS' = b, the right hand side of the constraint (symbolic expression); 
%>> alternatively, 'LHS' = A*x - b, 'RHS' = 0;
%'var' = x, the constrained variables (symbolic expression); 
%'Numeric' - if true, the output is numeric
%'FileName_A' - file name where a function generated from the symbolic expression for A will be stored 
%'FileName_b' - file name where a function generated from the symbolic expression for b will be stored 
%'generated_function_vars' - variables for the generated functions
function [A, b] = OHfunction_generate_linear_constraints(varargin)
Parser = inputParser;
Parser.FunctionName = 'OHfunction_generate_linear_constraints';
Parser.addOptional('LHS', []); 
Parser.addOptional('RHS', []); 
Parser.addOptional('var', []); 
Parser.addOptional('Numeric', true); 
Parser.addOptional('FileName_A', []);
Parser.addOptional('FileName_b', []);
Parser.addOptional('generated_function_vars', []);
Parser.parse(varargin{:});

if isempty(Parser.Results.RHS)
    expression = Parser.Results.LHS;
else
    expression = Parser.Results.LHS - Parser.Results.RHS;
end
var = Parser.Results.var;
assume(var, 'real');

Matrix = jacobian(expression, var);
remainder = -(expression - Matrix*var);
remainder = simplify(remainder);

if Parser.Results.Numeric
    A = double(Matrix);
    b = double(remainder);
else
    A = Matrix;
    b = remainder;
end
    
if (~isempty(Parser.Results.FileName_A)) && (~isempty(Parser.Results.generated_function_vars))
    matlabFunction(A, 'File', Parser.Results.FileName_A, 'Vars', {Parser.Results.generated_function_vars});
end
if (~isempty(Parser.Results.FileName_b)) && (~isempty(Parser.Results.generated_function_vars))
    matlabFunction(b, 'File', Parser.Results.FileName_b, 'Vars', {Parser.Results.generated_function_vars});
end

end