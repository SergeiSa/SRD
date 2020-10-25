function Handler_StateConverter_GenCoord2StateSpace = SRD_get_handler__StateConverter_GenCoord2StateSpace(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__StateConverter_GenCoord2StateSpace';
Parser.addOptional('Handler_State', []);
Parser.parse(varargin{:});

Handler_StateConverter_GenCoord2StateSpace = SRDHandler_StateConverter_GenCoord2StateSpace();

Handler_StateConverter_GenCoord2StateSpace.dof_robot_StateSpace = 2*Parser.Results.Handler_State.dof_robot;

Handler_StateConverter_GenCoord2StateSpace.Update = @() Update(Handler_StateConverter_GenCoord2StateSpace, ...
    Parser.Results.Handler_State);

        
    function Update(Handler_StateConverter_GenCoord2StateSpace, Handler_State)
        Handler_StateConverter_GenCoord2StateSpace.x = ...
            [Handler_State.q; Handler_State.v];
        Handler_StateConverter_GenCoord2StateSpace.dx = ...
            [Handler_State.v; Handler_State.a];
    end

end