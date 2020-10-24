function Handler_IK_Solution = SRD_get_handler__IK_solution__interp1(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__IK_solution__interp1';
Parser.addOptional('Handler_IK_Model_name', []);
Parser.addOptional('Handler_IK_task_name', []);
Parser.addOptional('IK_Table', []);
Parser.addOptional('TimeTable', []);
Parser.addOptional('method', 'linear'); %linear', 'nearest', 'next', 'previous', 'pchip', 'cubic', 'v5cubic', 'makima', or 'spline'

Parser.parse(varargin{:});

method = Parser.Results.method;
Handler_IK_Model_name = Parser.Results.Handler_IK_Model_name;
Handler_IK_task_name = Parser.Results.Handler_IK_task_name;

Handler_IK_Solution = SRDHandler_IK_Solution;

%the reason we do serialization prep here is that Handler_IK_Model can
%contain casadi objects, and we can abstract it out using SRD_get and
%Handler_IK_Model's Sserialization prep functions; Then we might as well do
%teh same for Handler_IK_task. This way we don't keep their copies in the
%save files
Handler_IK_Solution.SerializationPrepNeeded = true;
Handler_IK_Solution.PostSerializationPrepFunction = @PostSerializationPrepFunction;
Handler_IK_Solution.PreSerializationPrepFunction = @PreSerializationPrepFunction;

    function PostSerializationPrepFunction(Handler_IK_Solution)
        Handler_IK_Solution.State.Handler_IK_Model = SRD_get(Handler_IK_Model_name);
        Handler_IK_Solution.State.Handler_IK_task  = SRD_get(Handler_IK_task_name);
        
        Handler_IK_Solution.dof_robot = Handler_IK_Solution.State.Handler_IK_Model.dof_robot;
        
        Handler_IK_Solution.TimeStart      = max(Handler_IK_Solution.State.Handler_IK_Model.TimeStart, ...
                                                 Handler_IK_Solution.State.Handler_IK_task.TimeStart);
        Handler_IK_Solution.TimeExpiration = min(Handler_IK_Solution.State.Handler_IK_Model.TimeExpiration, ...
                                                 Handler_IK_Solution.State.Handler_IK_task.TimeExpiration);
    end
    function PreSerializationPrepFunction(Handler_IK_Solution)
        Handler_IK_Solution.State.Handler_IK_Model = [];
        Handler_IK_Solution.State.Handler_IK_task  = [];
    end

Handler_IK_Solution.State.TimeTable = Parser.Results.TimeTable;
Handler_IK_Solution.State.IK_Table = Parser.Results.IK_Table;

Handler_IK_Solution.get_position = @(t) get_position(t, Handler_IK_Solution, method);
Handler_IK_Solution.get_position_velocity_acceleration = ...
    @(t) get_position_velocity_acceleration(t, Handler_IK_Solution, method);


    function q = get_position(t, Handler_IK_Solution, method)
        q = interp1(Handler_IK_Solution.State.TimeTable, ...
            Handler_IK_Solution.State.IK_Table, ...
            t, ...
            method);
        q = reshape(q, [], 1);
    end
    function squized = get_position_velocity_acceleration(t, Handler_IK_Solution, method)
        q = get_position(t, Handler_IK_Solution, method);
        
        task_v = Handler_IK_Solution.State.Handler_IK_task.get_Task_derivative(t);
        task_a = Handler_IK_Solution.State.Handler_IK_task.get_Task_second_derivative(t);
        
        J = Handler_IK_Solution.State.Handler_IK_Model.get_Jacobian(q);
        v = pinv(J)*task_v;
        
        dJ = Handler_IK_Solution.State.Handler_IK_Model.get_Jacobian_derivative(q, v);
        
        %res = [v a]
        res = pinv([J,  zeros(size(J)); 
                    dJ, J]) * [task_v; task_a];
                    
        v = res(1:Handler_IK_Solution.dof_robot); 
        a = res((Handler_IK_Solution.dof_robot + 1):end);
        
        squized = [q, v, a];
    end
        
end