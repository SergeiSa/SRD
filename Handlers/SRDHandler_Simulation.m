classdef SRDHandler_Simulation < SRDHandler
    properties
        TimeLog = [];
        CurrentTime = [];
        CurrentIndex = [];
        
        
        PreprocessingHandlersArray = {};
        ControllerArray = {};
        SolverArray = {};
        LoggerArray = {};
    end
    methods
        function Simulate(obj)
            
            for i = 1:(length(obj.TimeLog) - 1)
                
                obj.CurrentTime = obj.TimeLog(i);
                obj.CurrentIndex = i;
                
                for j = 1:length(obj.PreprocessingHandlersArray)
                    obj.PreprocessingHandlersArray{j}.Update();
                end
                for j = 1:length(obj.ControllerArray)
                    obj.ControllerArray{j}.Update();
                end
                for j = 1:length(obj.SolverArray)
                    obj.SolverArray{j}.Update();
                end
                for j = 1:length(obj.LoggerArray)
                    obj.LoggerArray{j}.Update();
                end
                
            end
            
        end
    end
end
   