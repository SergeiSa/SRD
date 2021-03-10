function K_table = SRD_CTC_GenerateTable(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_CTC_GenerateTable';
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_IK_Solution', []);
Parser.addOptional('Kp', []);
Parser.addOptional('Kd', []);
Parser.addOptional('TimeTable', 0:0.01:1);

Parser.parse(varargin{:});

Count = length(Parser.Results.TimeTable);
n = 2 * Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
m = Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_control;

K_table = zeros(m, n, Count);

for i = 1:Count
    
    t = Parser.Results.TimeTable(i);
    
    w = Parser.Results.Handler_IK_Solution.get_position_velocity_acceleration(t);
    q = w(:, 1);
    
    H = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(q);
    T = Parser.Results.Handler_dynamics_generalized_coordinates_model.get_control_map(q);
    
    K_table(:, :, i) = pinv(T) * H * [Parser.Results.Kp, Parser.Results.Kd];
    
end

end