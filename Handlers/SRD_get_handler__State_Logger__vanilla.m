function Handler_State_Logger_vanilla = SRD_get_handler__State_Logger__vanilla(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__State_Logger__vanilla';
Parser.addOptional('Handler_State', []);
Parser.addOptional('Handler_Simulation', []);

Parser.parse(varargin{:});

Handler_State_Logger_vanilla = SRDHandler_Logger;

Handler_State_Logger_vanilla.Log.q = ...
    NaN(length(Parser.Results.Handler_Simulation.TimeLog), Parser.Results.Handler_State.dof_robot);
Handler_State_Logger_vanilla.Log.v = ...
    NaN(length(Parser.Results.Handler_Simulation.TimeLog), Parser.Results.Handler_State.dof_robot);
Handler_State_Logger_vanilla.Log.a = ...
    NaN(length(Parser.Results.Handler_Simulation.TimeLog), Parser.Results.Handler_State.dof_robot);


Handler_State_Logger_vanilla.Update = @() Update(...
    Handler_State_Logger_vanilla, ...
    Parser.Results.Handler_State, ...
    Parser.Results.Handler_Simulation);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_State_Logger_vanilla.SerializationPrepNeeded = true;
Handler_State_Logger_vanilla.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_State_Logger_vanilla; create a new one on the fly instead')
    end


    function Update(Handler_State_Logger_vanilla, Handler_State, Handler_Simulation)
        
        i = Handler_Simulation.CurrentIndex;
        
        Handler_State_Logger_vanilla.Log.q(i, :) = reshape(Handler_State.q, 1, []);
        Handler_State_Logger_vanilla.Log.v(i, :) = reshape(Handler_State.v, 1, []);
        Handler_State_Logger_vanilla.Log.a(i, :) = reshape(Handler_State.a, 1, []);
           
    end

end