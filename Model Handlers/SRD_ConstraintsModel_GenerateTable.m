function [N_table, G_table, F_table] = SRD_ConstraintsModel_GenerateTable(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_ConstraintsModel_GenerateTable';
Parser.addOptional('Handler_Constraints_Model', []);
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('x_table', []);
Parser.addOptional('new_dimentions', []);

Parser.parse(varargin{:});


Count = size(Parser.Results.x_table, 2);
n = size(Parser.Results.x_table, 1);
k = Parser.Results.Handler_Constraints_Model.dof_Constraint;

if ~isempty(Parser.Results.new_dimentions)
    nn = Parser.Results.new_dimentions;
else
    nn = n -2*k;
end
    

N_table = zeros(n, nn, Count);
G_table = zeros(2*k, n, Count);
F_table = zeros(n, k, Count);

for i = 1:Count
    
    x = Parser.Results.x_table(:, i);
    q = x(1:(n/2));
    v = x((n/2 + 1):end);
    
    iH = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix_inverse(q);
    
    F = Parser.Results.Handler_Constraints_Model.get_Jacobian(q);
    dFdq = Parser.Results.Handler_Constraints_Model.get_Jacobian_derivative(q, v);
    
    G = [zeros(k, n/2), F; F, dFdq];
    
    
    N = null(G);
    
%     if isempty(N)
%         N_table(:, :, i)= zeros(n, nn, 1);
%     else 
    N_table(:, :, i) = N;
%     end
    
    G_table(:, :, i) = G;
    F_table(:, :, i) = [zeros(n/2, k); iH * F'];
end

end