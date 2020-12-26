function Handler_control_Logger = SRD_get_handler__Control_Logger(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__State_Logger__vanilla';
Parser.addOptional('Controller', []);
Parser.addOptional('dof_control', []);
Parser.addOptional('Handler_Simulation', []);

Parser.parse(varargin{:});

Handler_control_Logger = SRDHandler_Logger;

Handler_control_Logger.Log.u = ...
    NaN(length(Parser.Results.Handler_Simulation.TimeLog), Parser.Results.dof_control);

Handler_control_Logger.Update = @() Update(...
    Handler_control_Logger, ...
    Parser.Results.Controller, ...
    Parser.Results.Handler_Simulation);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_control_Logger.SerializationPrepNeeded = true;
Handler_control_Logger.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_State_Logger_vanilla; create a new one on the fly instead')
    end


    function Update(Handler_control_Logger_vanilla, Controller, Handler_Simulation)
        i = Handler_Simulation.CurrentIndex;
        
        Handler_control_Logger_vanilla.Log.u(i, :) = reshape(Controller.u, 1, []);
    end

end