function [IK_Table,dq] = SRD_InverseKinematics_GenerateTable_ode(varargin)

%Generate IK table base on Ode integration on dq
% Available Methode :
%           SRD_InversePositionProblemSolver_Ode_Pinv
%           SRD_InversePositionProblemSolver_Ode_Dynamics


Parser = inputParser;
Parser.FunctionName = 'SRD_InverseKinematics_GenerateTable_ode';
Parser.addOptional('Task_params', []);
Parser.addOptional('Wieght', []);
Parser.addOptional('Handler_IK_Model', []);
Parser.addOptional('InitialGuess', []);
Parser.addOptional('TimeTable', []);
Parser.addOptional('method', @SRD_InversePositionProblemSolver_Ode_Dynamics);


Parser.parse(varargin{:});

dof_robot = Parser.Results.Handler_IK_Model.dof_robot;
q0 = Parser.Results.InitialGuess;
J = Parser.Results.Handler_IK_Model.get_Jacobian_handle;
tspan = Parser.Results.TimeTable;


[t,IK_Table] = ode45(@(t,q0)sys_ode(t,q0,Parser,J),tspan,q0);

dq = zeros(length(tspan),dof_robot);
for i = 1:length(tspan)
    dq(i,:) = sys_ode(tspan(i),IK_Table(i,:),Parser,J);
end

% a= sys_ode(0,q0,Parser,J)

    function dq = sys_ode(t,q0,Parser,J)
        K = Parser.Results.Wieght(1);
        U = Parser.Results.Wieght(2);
        desired_task = Parser.Results.Task_params(:,1);
        obs_pose = Parser.Results.Task_params(:,2);
        
        tasks =  Parser.Results.Handler_IK_Model.get_Task(q0);
        v_attraction = K*(desired_task-tasks);
        v_repulsion = U*potential_fiel(Parser,q0,obs_pose,1);
        dof = Parser.Results.Handler_IK_Model.dof_robot;

        
        x_dot = v_attraction+v_repulsion;
        dq = SRD_InversePositionProblemSolver_Ode_Dynamics(q0,x_dot,J,dof);


    function F = potential_fiel(Parser,q,obs_pose,tolerant)
        diff =  Parser.Results.Handler_IK_Model.get_Task(q)-obs_pose;
        distace = norm(diff);
        if distace<1
            d = 10*distace;
        else 
            d = distace;
        end
        F = (1/d - 1/tolerant)*(diff./d^3);
    end

end

end