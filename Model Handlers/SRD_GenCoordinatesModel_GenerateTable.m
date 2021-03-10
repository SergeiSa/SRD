function [H_table, iH_table, T_table, c_table, q_table, v_table, a_table, u_table] = SRD_GenCoordinatesModel_GenerateTable(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_GenCoordinatesModel_GenerateTable';
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_IK_Solution', []);
Parser.addOptional('TimeTable', 0:0.01:1);

Parser.parse(varargin{:});

Count = length(Parser.Results.TimeTable);
n = Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
m = Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_control;

H_table = zeros(n, n, Count);
iH_table = zeros(n, n, Count);
T_table = zeros(n, m, Count);
c_table = zeros(n, Count);
q_table = zeros(n, Count);
v_table = zeros(n, Count);
a_table = zeros(n, Count);
u_table = zeros(m, Count);

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
    
    u = pinv(T)* (H*a + c);
    
    H_table(:, :, i) = H;
    iH_table(:, :, i) = iH;
    T_table(:, :, i) = T;
    c_table(:, i) = c;
    
        
    q_table(:, i) = q;
    v_table(:, i) = v;
    a_table(:, i) = a;
    u_table(:, i) = u;
end

end