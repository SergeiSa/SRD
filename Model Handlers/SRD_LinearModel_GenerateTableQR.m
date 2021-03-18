function [A_table, B_table, c_table, x_table, u_table, dx_table] = SRD_LinearModel_GenerateTableQR(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_LinearModel_GenerateTable';
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_dynamics_Linearized_Model', []);
Parser.addOptional('Handler_IK_Solution', []);
Parser.addOptional('Handler_Constraints_Model', []);
Parser.addOptional('TimeTable', 0:0.01:1);

Parser.parse(varargin{:});

Count = length(Parser.Results.TimeTable);
n = 2 * Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
m = Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_control;

A_table = zeros(n, n, Count);
B_table = zeros(n, m, Count);
c_table = zeros(n, Count);
x_table = zeros(n, Count);
u_table = zeros(m, Count);
dx_table = zeros(n, Count);

for i = 1:Count
    
    t = Parser.Results.TimeTable(i);
    
    w = Parser.Results.Handler_IK_Solution.get_position_velocity_acceleration(t);
    q = w(:, 1);
    v = w(:, 2);
    a = w(:, 3);
    
    H = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(q);
    iH = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix_inverse(q);
    T = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_control_map(q);
    c = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_bais_vector(q, v);
    
    F = Parser.Results.Handler_Constraints_Model.get_Jacobian(q);
    k = Parser.Results.Handler_Constraints_Model.dof_Constraint;

    F = orth(F');
    F = F';
% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % %

    [Q, R] = qr(F');
    
    R_c = R(1:k, :);

    I = eye(m);
    I_c = I(1:k, :);
    I_u = I((k+1):end, :);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % %
    u_FF = pinv(I_u * Q' * T) * I_u * Q' * (H*a + c);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % %

    u = u_FF;
    
    
    u_table(:, i) = u;

    A_table(:, :, i) = Parser.Results.Handler_dynamics_Linearized_Model.get_A(q, v, u, iH);
    B_table(:, :, i) = Parser.Results.Handler_dynamics_Linearized_Model.get_B(q, u, iH);
        
    x_table(:, i) = [q; v];
    
    dx_table(:, i) = [v; a];
    
    c_table(:, i) = dx_table(:, i) - A_table(:, :, i) * x_table(:, i) - B_table(:, :, i) * u_table(:, i);

end

end