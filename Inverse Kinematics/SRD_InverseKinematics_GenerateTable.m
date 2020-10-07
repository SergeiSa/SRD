function IK_Table = SRD_InverseKinematics_GenerateTable(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_InverseKinematics_GenerateTable';
Parser.addOptional('Handler_IK_Model', []);
Parser.addOptional('Handler_IK_task', []);
Parser.addOptional('InitialGuess', []);
Parser.addOptional('method', @SRD_InversePositionProblemSolver_lsqnonlin);
Parser.addOptional('opts', optimoptions(@lsqnonlin, ...
    'SpecifyObjectiveGradient', true, ...
    'Display', 'none', ...
     'Algorithm', 'levenberg-marquardt'));

Parser.addOptional('TimeTable', []);

Parser.parse(varargin{:});

Count = length(Parser.Results.TimeTable);
dof = Parser.Results.Handler_IK_Model.dof_robot;

IK_Table = zeros(Count, dof);
q0 = Parser.Results.InitialGuess;

for i = 1:Count
    
    if rem( i, floor(Count / 100)) == 0
        disp(['calculating ', num2str( floor(100 * i / Count) ), ' %']);
    end
    
    t = Parser.Results.TimeTable(i);
    TaskValue = Parser.Results.Handler_IK_task.get_Task(t);
    q = SRD_InversePositionProblemSolver_lsqnonlin(Parser.Results.Handler_IK_Model.get_Task_handle, ...
        Parser.Results.Handler_IK_Model.get_Jacobian_handle, TaskValue, q0, ...
        Parser.Results.opts);
    
    IK_Table(i, :) = q;
    q0 = q;
end

end