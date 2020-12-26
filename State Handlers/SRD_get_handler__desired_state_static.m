function Handler_Desired_State = SRD_get_handler__desired_state_static(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__desired_state_static';
Parser.addOptional('static_q', []);
Parser.addOptional('dof_robot', []);
Parser.parse(varargin{:});

Handler_Desired_State = SRDHandler_Desired_State();

Handler_Desired_State.dof_robot = Parser.Results.dof_robot;

Handler_Desired_State.Update = @() Update(Handler_Desired_State, ...
    Parser.Results.static_q, ...
    Parser.Results.dof_robot);

        
    function Update(Handler_State, static_q, dof_robot)
        
        Handler_State.q = reshape(static_q, [], 1);
        Handler_State.v = zeros(dof_robot, 1);
        Handler_State.a = zeros(dof_robot, 1);
    end

end