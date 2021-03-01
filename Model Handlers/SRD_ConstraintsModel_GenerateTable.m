function [N_table, G_table] = SRD_ConstraintsModel_GenerateTable(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_LinearModel_GenerateTable';
Parser.addOptional('Handler_Constraints_Model', []);
Parser.addOptional('x_table', []);
Parser.addOptional('new_dimentions', []);

Parser.parse(varargin{:});


Count = size(Parser.Results.x_table, 2);
n = size(Parser.Results.x_table, 1);
k = Parser.Results.Handler_Constraints_Model.dof_Constraint;

if ~isempty(Parser.Results.new_dimentions)
    nn = Parser.Results.new_dimentions;
else
    nn = n - k;
end
    

N_table = zeros(n, nn, Count);
G_table = zeros(2*k, n, Count);

for i = 1:Count
    
    x = Parser.Results.x_table(:, i);
    q = x(1:(n/2));
    v = x((n/2 + 1):end);
    
    F = Parser.Results.Handler_Constraints_Model.get_Jacobian(q);
    dFdq = Parser.Results.Handler_Constraints_Model.get_Jacobian_derivative(q, v);
    
    G = [zeros(k, n/2), F; F, dFdq];
    N = null(G);
    
    G_table(:, :, i) = G;
    N_table(:, :, i) = N;
end

end