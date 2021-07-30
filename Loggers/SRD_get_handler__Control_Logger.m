function Handler_control_Logger = SRD_get_handler__Control_Logger(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__Control_Logger';
Parser.addOptional('Controller', []);
Parser.addOptional('dof_control', []);
Parser.addOptional('Handler_Time', []);

Parser.parse(varargin{:});

Handler_control_Logger = SRDHandler_Logger;

Handler_control_Logger.Log.u = ...
    NaN(length(Parser.Results.Handler_Time.TimeLog), Parser.Results.dof_control);

Handler_control_Logger.Update = @() Update(...
    Handler_control_Logger, ...
    Parser.Results.Controller, ...
    Parser.Results.Handler_Time);

    function Update(Handler_control_Logger_vanilla, Controller, Handler_Time)
        i = Handler_Time.CurrentIndex;
        
        Handler_control_Logger_vanilla.Log.u(i, :) = reshape(Controller.u, 1, []);
    end

end