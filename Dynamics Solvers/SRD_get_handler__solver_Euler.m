function Handler_solver_Euler = SRD_get_handler__solver_Euler(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__solver_Euler';
Parser.addOptional('Handler_StateSpace', []);
Parser.addOptional('Handler_Controller', []);
Parser.addOptional('Handler_FirstOrderSystem', []);
Parser.addOptional('Handler_Time', []);

Parser.parse(varargin{:});

Handler_solver_Euler = SRDHandler_Solver;

Handler_solver_Euler.Update = @() Update(...
    Parser.Results.Handler_StateSpace, ...
    Parser.Results.Handler_Controller, ...
    Parser.Results.Handler_FirstOrderSystem, ...
    Parser.Results.Handler_Time);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_solver_Euler.SerializationPrepNeeded = true;
Handler_solver_Euler.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_solver_Euler; create a new one on the fly instead')
    end


    function Update(Handler_StateSpace, Handler_Controller, ...
            Handler_FirstOrderSystem, Handler_Time)
        
        dt = Handler_Time.TimeLog(Handler_Time.CurrentIndex + 1) - Handler_Time.TimeLog(Handler_Time.CurrentIndex);
        
        x = Handler_StateSpace.x;
        u = Handler_Controller.u;
        
        dx = Handler_FirstOrderSystem.get_dx(x, u);
        x = x + dt * dx;
        
        Handler_StateSpace.x = x;
    end

end