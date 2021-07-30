%Logger that saves control gains; for example, if control law is given by u
%= -K*x, it will store K matrices on every iteration. Works with LQR, CLQR,
%etc.
function Handler_control_Logger = SRD_get_handler__ControlGains_Logger(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__ControlGains_Logger';
Parser.addOptional('Controller', []);
Parser.addOptional('dof_control', []);
Parser.addOptional('dof_state', []);
Parser.addOptional('Handler_Time', []);

Parser.parse(varargin{:});

Handler_control_Logger = SRDHandler_Logger;

Handler_control_Logger.Log.K = ...
    NaN(Parser.Results.dof_control, ...
        Parser.Results.dof_state, ...
        length(Parser.Results.Handler_Time.TimeLog));

Handler_control_Logger.Update = @() Update(...
    Handler_control_Logger, ...
    Parser.Results.Controller, ...
    Parser.Results.Handler_Time);

    function Update(Handler_control_Logger_vanilla, Controller, Handler_Time)
        i = Handler_Time.CurrentIndex;
        
        Handler_control_Logger_vanilla.Log.K(:, :, i) = Controller.control_law.K;
    end

end