classdef SRDHandler_StateConverter_StateSpace2GenCoord < SRDHandler
    properties
        q;
        v;
        a;
        
        dof_robot;
        dof_robot_StateSpace;
        
        Update;
    end
    methods
        function obj = SRDHandler_StateConverter_StateSpace2GenCoord(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_StateConverter_StateSpace2GenCoord';
            Parser.addOptional('Handler_StateSpace', []);
            Parser.parse(varargin{:});
            
            obj.dof_robot = Parser.Results.Handler_StateSpace.dof_StateSpace / 2;
            obj.dof_robot_StateSpace = Parser.Results.Handler_StateSpace.dof_StateSpace;
            
            obj.Update = @() Update(obj, ...
                Parser.Results.Handler_StateSpace);
            
            function Update(obj, Handler_StateSpace)
                obj.q = Handler_StateSpace.x(1:obj.dof_robot);
                obj.v = Handler_StateSpace.x ((obj.dof_robot + 1):end);
                obj.a = Handler_StateSpace.dx((obj.dof_robot + 1):end);
            end
        end
        
        function squized = get_position_velocity_acceleration(obj, ~)            
            squized = [obj.q, obj.v, obj.a];
        end
    end
end
   