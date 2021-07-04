classdef SRDHandler_Controller < SRDHandler
    properties
        u;
        
        Update;
        
        SerializationPrepNeeded = true;
    end
    methods
        function PreSerializationPrepFunction(~)
            error('do not attempt to save Handler_ComputedTorqueController; create a new one on the fly instead')
        end
        
    end
end
   