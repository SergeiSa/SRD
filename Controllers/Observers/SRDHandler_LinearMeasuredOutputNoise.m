%for system with state x, th linear measured output is
%y = C*x + v
%v - measurement noise
classdef SRDHandler_LinearMeasuredOutputNoise < SRDHandler
    properties
        
        y; %y = C*x + v
        v; %sensor noise
        
        C;
        dof_MeasuredOutput;
        
        Handler_State_StateSpace;
        
        type;          %'randn', 'rand'
        scale;         % v = scale * rand 
    end
    methods
        function obj = SRDHandler_LinearMeasuredOutputNoise(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_StateSpace';
            Parser.addOptional('Handler_State_StateSpace', []);
            Parser.addOptional('C', []);
            Parser.addOptional('type', 'randn'); %
            Parser.addOptional('scale', 1); %
            Parser.parse(varargin{:});
            
            obj.C = Parser.Results.C;
            obj.Handler_State_StateSpace = Parser.Results.Handler_State_StateSpace;
            
            obj.dof_MeasuredOutput = size(obj.C, 1);
            
            obj.type  = Parser.Results.type;
            obj.scale = Parser.Results.scale;
        end
        
        function Update(obj)
            
            switch obj.type
                case 'randn'
                    obj.v = obj.scale * randn(obj.dof_MeasuredOutput, 1);
                case 'rand'
                    obj.v = obj.scale * (rand(obj.dof_MeasuredOutput, 1) - 2*ones(obj.dof_MeasuredOutput, 1));
            end
            
            obj.y = obj.C * obj.Handler_State_StateSpace.x + obj.v;
        end
    end
end
   































