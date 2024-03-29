%class that Loggers are instances of or inherit from. Used to automatically
%create scecific logs during the Simulation/Update of the systems elements
classdef SRDHandler_Logger < SRDHandler
    properties
        
        %this function needs to be called every time the logs are to be
        %written (typically on every iteration of Simulation/Update)
        Update;
        
        %the data to be logged is stored here. Structure of this field can
        %change.
        Log;
    end
    methods
        function obj = SRDHandler_Logger()
            obj.SerializationPrepNeeded = true;
            obj.PreSerializationPrepFunction = @PreSerializationPrepFunction;
            function PreSerializationPrepFunction(~)
                error('do not attempt to save Handler_State_Logger_vanilla; create a new one on the fly instead')
            end
        end
 
    end
end
   