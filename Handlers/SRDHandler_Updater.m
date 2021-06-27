classdef SRDHandler_Updater < SRDHandler
    properties
        TimeLog = [];
        CurrentTime = [];
        CurrentIndex = [];
        
        
        ToUpdate = {};
    end
    methods
        
        function obj = SRDHandler_Updater(ToUpdate)
            obj.ToUpdate = ToUpdate;
        end
        
        function Update(obj)
            for i = 1:length(obj.ToUpdate)
                obj.ToUpdate{i}.Update();
            end
        end
    end
end
   