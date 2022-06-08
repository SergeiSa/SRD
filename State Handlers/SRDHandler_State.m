classdef SRDHandler_State < SRDHandler
    properties
        q;
        v;
        a;
        Other;
        
        dof_configuration_space_robot;
    end
    methods
        function obj = SRDHandler_State(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRD_get_handler__state';
            Parser.addOptional('InitialPosition', []);
            Parser.addOptional('InitialVelocity', []);
            Parser.parse(varargin{:});
            
            obj.q = reshape(Parser.Results.InitialPosition, [], 1);
            obj.v = reshape(Parser.Results.InitialVelocity, [], 1);
            obj.a = NaN(size(obj.v));
            
            obj.dof_configuration_space_robot = length(Parser.Results.InitialPosition);
        end

        function squized = get_position_velocity_acceleration(obj, ~)            
            squized = [obj.q, obj.v, obj.a];
        end
    end
end
   