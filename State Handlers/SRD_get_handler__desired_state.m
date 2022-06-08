function Handler_Desired_State = SRD_get_handler__desired_state(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__desired_state';
Parser.addOptional('Handler_ControlInput', []);
Parser.addOptional('Handler_Time', []);%you can pass simulation handler here
Parser.parse(varargin{:});

Handler_Desired_State = SRDHandler_Desired_State();

Handler_Desired_State.dof_configuration_space_robot = Parser.Results.Handler_ControlInput.dof_configuration_space_robot;

Handler_Desired_State.Update = @() Update(Handler_Desired_State, ...
    Parser.Results.Handler_Time, ...
    Parser.Results.Handler_ControlInput);

        
    function Update(Handler_State, Handler_Time, Handler_ControlInput)
        t = Handler_Time.CurrentTime;
        
        squished = Handler_ControlInput.get_position_velocity_acceleration(t);
        Handler_State.q = squished(:, 1);
        Handler_State.v = squished(:, 2);
        Handler_State.a = squished(:, 3);
    end

end