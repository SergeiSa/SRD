classdef SRDHandler_Time < SRDHandler
    properties
        TimeLog = [];
        CurrentTime = 0;
        CurrentIndex = 1;
    end
    methods
        function obj = SRDHandler_Time(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_Time';
            Parser.addOptional('TimeLog', []);
            Parser.parse(varargin{:});
            
            obj.TimeLog = reshape(Parser.Results.TimeLog, [], 1);
        end
        function Update(obj)
            obj.CurrentIndex = obj.CurrentIndex + 1;
            obj.CurrentTime = obj.TimeLog(obj.CurrentIndex);
        end
    end
end