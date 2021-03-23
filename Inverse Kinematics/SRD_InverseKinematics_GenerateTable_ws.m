function IK_Table = SRD_InverseKinematics_GenerateTable_ws(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_InverseKinematics_GenerateTable';
Parser.addOptional('Handler_IK_Model', []);
Parser.addOptional('Handler_IK_task', []);
Parser.addOptional('InitialGuess', []);
Parser.addOptional('method', @SRD_InversePositionProblemSolver_quadprog_ws);
Parser.addOptional('opts', optimoptions(@quadprog, 'Algorithm', 'interior-point-convex', 'Display', 'off'));

Parser.addOptional('TimeTable', []);

Parser.parse(varargin{:});

Count = length(Parser.Results.TimeTable);
dof = Parser.Results.Handler_IK_Model.dof_robot;

IK_Table = zeros(Count, dof);

ws0 = optimwarmstart(Parser.Results.InitialGuess, Parser.Results.opts);

for i = 1:Count
    
    if rem( i, floor(Count / 100)) == 0
        disp(['calculating ', num2str( floor(100 * i / Count) ), ' %']);
    end
    
    t = Parser.Results.TimeTable(i);
    
    TaskValue = Parser.Results.Handler_IK_task.get_Task(t);
    
    ws = Parser.Results.method(...
        Parser.Results.Handler_IK_Model.get_Task_handle, ...
        Parser.Results.Handler_IK_Model.get_Jacobian_handle, ...
        TaskValue, ...
        ws0, ...
        Parser.Results.opts);
    
    IK_Table(i, :) = ws.X;
    ws0 = ws;
end

end

