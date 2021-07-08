function [A_table, B_table, g_table, error] = SRD_LinearModel_GenerateTable_v2(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_LinearModel_GenerateTable_v2';
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_dynamics_Linearized_Model', []);
Parser.addOptional('x_table', []);
Parser.addOptional('u_table', []);
Parser.addOptional('dx_table', []);

Parser.parse(varargin{:});

Count = size(Parser.Results.x_table, 2);

dof = Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
n = 2 * dof;
m = Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_control;

A_table = zeros(n, n, Count);
B_table = zeros(n, m, Count);
g_table = zeros(n, Count);

if ~isempty(Parser.Results.dx_table)
    error = zeros(n, Count);
end

for i = 1:Count
    
    x = Parser.Results.x_table(:, i);
    q = x(1:dof);
    v = x((1+dof):end);
    u = Parser.Results.u_table(:, i);
    
    iH = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix_inverse(q);
    T = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_control_map(q);
    c = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_bias_vector(q, v);
    
    f0 = [v; iH*(T*u - c)];
    
    A = Parser.Results.Handler_dynamics_Linearized_Model.get_A(q, v, u, iH);
    B = Parser.Results.Handler_dynamics_Linearized_Model.get_B(q, u, iH);
    g = f0 - A * x - B * u;
    
    A_table(:, :, i) = A;
    B_table(:, :, i) = B;
    g_table(:, i) = g;
    
    if ~isempty(Parser.Results.dx_table)
        error(:, i) = Parser.Results.dx_table(:, i) - A * x - B * u - g;
    end
end

end