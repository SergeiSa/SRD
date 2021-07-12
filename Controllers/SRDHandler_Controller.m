classdef SRDHandler_Controller < SRDHandler
    properties
        u;
        
        Update;
    end
    methods
        
        function obj = SRDHandler_Controller()
            obj.SerializationPrepNeeded = true;
            obj.PreSerializationPrepFunction = @obj.PreSerializationPrepFunction_Controller;
        end
        
        function PreSerializationPrepFunction_Controller(~)
            error('do not attempt to save Handler_ComputedTorqueController; create a new one on the fly instead')
        end
        
    end
end
   