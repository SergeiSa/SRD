classdef SRDHandler_StateSpace < SRDHandler
    properties
        x;
        dx;
        
        dof_state_space_robot;
    end
    methods
        function obj = SRDHandler_StateSpace(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_StateSpace';
            Parser.addOptional('InitialState', []);
            Parser.parse(varargin{:});
            
            obj.x = reshape(Parser.Results.InitialState, [], 1);
            obj.dx = zeros(size(obj.x));
            
            obj.dof_state_space_robot = length(obj.x);
        end
    end
end
   