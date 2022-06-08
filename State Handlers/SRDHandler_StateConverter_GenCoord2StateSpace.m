classdef SRDHandler_StateConverter_GenCoord2StateSpace < SRDHandler
    properties
        x;
        dx;
        
        dof_state_space_robot;
        dof_configuration_space_robot;
        
        Update;
    end
    methods
        function obj = SRDHandler_StateConverter_GenCoord2StateSpace(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_StateConverter_GenCoord2StateSpace';
            Parser.addOptional('Handler_State', []);
            Parser.parse(varargin{:});
            
            obj.dof_state_space_robot         = 2*Parser.Results.Handler_State.dof_configuration_space_robot;
            obj.dof_configuration_space_robot =   Parser.Results.Handler_State.dof_configuration_space_robot;
            
            obj.Update = @() Update(obj, ...
                Parser.Results.Handler_State);
            
            function Update(obj, Handler_State)
                obj.x = [Handler_State.q; Handler_State.v];
                obj.dx = [Handler_State.v; Handler_State.a];
            end
        end
        
        function squized = get_x_dx(obj, ~)            
            squized = [obj.x, obj.dx];
        end
    end
end
   