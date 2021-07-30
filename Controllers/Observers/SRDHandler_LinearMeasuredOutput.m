%for system with state x, th linear measured output is
%y = C*x
classdef SRDHandler_LinearMeasuredOutput < SRDHandler
    properties
        
        y;
        
        C;
        dof_MeasuredOutput;
        
        Handler_State_StateSpace;
    end
    methods
        function obj = SRDHandler_LinearMeasuredOutput(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_StateSpace';
            Parser.addOptional('Handler_State_StateSpace', []);
            Parser.addOptional('C', []);
            Parser.parse(varargin{:});
            
            obj.C = Parser.Results.C;
            obj.Handler_State_StateSpace = Parser.Results.Handler_State_StateSpace;
            
            obj.dof_MeasuredOutput = size(obj.C, 1);
        end
        
        function Update(obj)
            obj.y = obj.C * obj.Handler_State_StateSpace.x;
        end
    end
end
   