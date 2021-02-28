function [At_table, Bt_table, x_table, u_table, dx_table] = SRD_LinearModelTransverse_GenerateTable(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_LinearModel_GenerateTable';
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_dynamics_Linearized_Model', []);
Parser.addOptional('Handler_IK_Solution', []);
Parser.addOptional('TimeTable', 0:0.01:1);

Parser.parse(varargin{:});


[A_table, B_table, ~, x_table, u_table, dx_table] = ...
    SRD_LinearModel_GenerateTable('Handler_dynamics_generalized_coordinates_model', Parser.Results.Handler_dynamics_generalized_coordinates_model, ...
    'Handler_dynamics_Linearized_Model', Parser.Results.Handler_dynamics_Linearized_Model, ...
    'Handler_IK_Solution', Parser.Results.Handler_IK_Solution, ...
    'TimeTable', Parser.Results.TimeTable);

TL = tpTransverseLinearization(A_table, B_table, x_table, u_table, dx_table, ...
    eye(2*Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot));

[At_table, Bt_table] = TL.compute_transverse_linearization();

end