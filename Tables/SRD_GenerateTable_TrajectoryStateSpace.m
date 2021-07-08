function [x_table, u_table, dx_table, error] = SRD_GenerateTable_TrajectoryStateSpace(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_LinearModel_GenerateTable';
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_dynamics_Linearized_Model', []);
Parser.addOptional('Handler_IK_Solution', []);
Parser.addOptional('TimeTable', 0:0.01:1);

Parser.parse(varargin{:});

Count = length(Parser.Results.TimeTable);
dof = Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
n = 2 * dof;
m = Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_control;


x_table = zeros(n, Count);
u_table = zeros(m, Count);
error   = zeros(dof, Count);
dx_table = zeros(n, Count);

for i = 1:Count
    
    t = Parser.Results.TimeTable(i);
    
    w = Parser.Results.Handler_IK_Solution.get_position_velocity_acceleration(t);
    q = w(:, 1);
    v = w(:, 2);
    a = w(:, 3);
    
    H = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(q);
    T = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_control_map(q);
    c = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_bias_vector(q, v);
    
    u = pinv(T)* (H*a + c);
    
    error(:, i) = H*a + c - T*u;
    
    x_table(:, i) = [q; v];
    dx_table(:, i) = [v; a];
    u_table(:, i) = u;
end

end