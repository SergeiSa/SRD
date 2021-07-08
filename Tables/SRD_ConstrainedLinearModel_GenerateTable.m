function [N_table, G_table, An_table, Bn_table, cn_table, xn_table, dxn_table] = SRD_ConstrainedLinearModel_GenerateTable(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_LinearModel_GenerateTable';
Parser.addOptional('Handler_Constraints_Model', []);
Parser.addOptional('A_table', []);
Parser.addOptional('B_table', []);
Parser.addOptional('c_table', []);
Parser.addOptional('x_table', []);
Parser.addOptional('dx_table', []);
Parser.addOptional('new_dimentions', []);

Parser.parse(varargin{:});


Count = size(Parser.Results.A_table, 3);
n = size(Parser.Results.A_table, 2);
m = size(Parser.Results.B_table, 2);
k = Parser.Results.Handler_Constraints_Model.dof_Constraint;

if ~isempty(Parser.Results.new_dimentions)
    nn = Parser.Results.new_dimentions;
else
    nn = n - k;
end
    

N_table = zeros(n, nn, Count);
G_table = zeros(2*k, n, Count);
An_table = zeros(nn, nn, Count);
Bn_table = zeros(nn, m, Count);
cn_table = zeros(nn, Count);
xn_table = zeros(nn, Count);
dxn_table = zeros(nn, Count);

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
    
    An_table(:, :, i) = N' * Parser.Results.A_table(:, :, i) * N;
    Bn_table(:, :, i) = N' * Parser.Results.B_table(:, :, i);
    cn_table(:, i)    = N' * Parser.Results.c_table(:, i);
    
    xn_table(:, i) = N' * Parser.Results.x_table(:, i);
    dxn_table(:, i) = N' * Parser.Results.dx_table(:, i);
    
end

end