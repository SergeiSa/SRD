classdef SRDHandler_Time < SRDHandler
    properties
        TimeLog = [];
        CurrentTime = 0;
        CurrentIndex = 1;
        
        dt;
    end
    methods
        function obj = SRDHandler_Time(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_Time';
            Parser.addOptional('TimeLog', []);
            Parser.parse(varargin{:});
            
            obj.TimeLog = reshape(Parser.Results.TimeLog, [], 1);
            obj.dt = obj.TimeLog(2) - obj.TimeLog(1);
        end
        function Update(obj)
            obj.CurrentIndex = obj.CurrentIndex + 1;
            obj.CurrentTime = obj.TimeLog(obj.CurrentIndex);
            
            if obj.CurrentIndex < length(obj.TimeLog)
                obj.dt = obj.TimeLog(obj.CurrentIndex + 1) - obj.TimeLog(obj.CurrentIndex);
            end
        end
    end
end