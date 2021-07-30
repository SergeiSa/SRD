function Handler_State_Logger_vanilla = SRD_get_handler__State_Logger__vanilla(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__State_Logger__vanilla';
Parser.addOptional('Handler_State', []);
Parser.addOptional('Handler_Time', []);
Parser.addOptional('ToLogAcceleration', true);

Parser.parse(varargin{:});

Handler_State_Logger_vanilla = SRDHandler_Logger;

Handler_State_Logger_vanilla.Log.q = ...
    NaN(length(Parser.Results.Handler_Time.TimeLog), Parser.Results.Handler_State.dof_robot);
Handler_State_Logger_vanilla.Log.v = ...
    NaN(length(Parser.Results.Handler_Time.TimeLog), Parser.Results.Handler_State.dof_robot);
if Parser.Results.ToLogAcceleration
Handler_State_Logger_vanilla.Log.a = ...
    NaN(length(Parser.Results.Handler_Time.TimeLog), Parser.Results.Handler_State.dof_robot);
end


Handler_State_Logger_vanilla.Update = @() Update(...
    Handler_State_Logger_vanilla, ...
    Parser.Results.Handler_State, ...
    Parser.Results.Handler_Time, ...
    Parser.Results.ToLogAcceleration);


    function Update(Handler_State_Logger_vanilla, Handler_State, Handler_Time, ToLogAcceleration)
        
        i = Handler_Time.CurrentIndex;
        
        Handler_State_Logger_vanilla.Log.q(i, :) = reshape(Handler_State.q, 1, []);
        Handler_State_Logger_vanilla.Log.v(i, :) = reshape(Handler_State.v, 1, []);
        
        if ToLogAcceleration
            Handler_State_Logger_vanilla.Log.a(i, :) = reshape(Handler_State.a, 1, []);
        end
           
    end

end