function IK_Table = SRD_InverseKinematics_GenerateTable_ode(varargin)

%Generate IK table base on Ode integration on dq
% Available Methode :
%           SRD_InversePositionProblemSolver_Ode_Pinv
%           SRD_InversePositionProblemSolver_Ode_Dynamics


Parser = inputParser;
Parser.FunctionName = 'SRD_InverseKinematics_GenerateTable_ode';
Parser.addOptional('Handler_IK_Model', []);
Parser.addOptional('Handler_dynamics', []);
Parser.addOptional('Task_velocity', []);
Parser.addOptional('InitialGuess', []);
Parser.addOptional('TimeTable', []);
Parser.addOptional('method', @SRD_InversePositionProblemSolver_Ode_Pinv);


Parser.parse(varargin{:});

q0 = Parser.Results.InitialGuess;
J = Parser.Results.Handler_IK_Model.get_Jacobian_handle;
tspan = Parser.Results.TimeTable;
dx = Parser.Results.Task_velocity;
H = Parser.Results.Handler_dynamics.get_joint_space_inertia_matrix;

[t,IK_Table] = ode45(@(t,q0)Parser.Results.method(t,q0,dx,J,H),tspan,q0);


end