%This class is supposed to be used to creat controllers (their either
%inherit from it, are instances)
classdef SRDHandler_Controller < SRDHandler
    properties
        %here the control input on the current step of simulation/update is
        %stored, it is supposed to be used in application 
        u; 
        
        %number of elements in u
        dof_control;
        
        %This function (same as other Update functions) should be called to
        %make teh controller process the availible information and generate
        %new value of u
        Update;
        
        %This is a FYI storage for the controller; here, information about
        %the details how u was computed can be stored. Can remain empty
        %too.
        control_law;
        
        %if false, less data will be stored in .control_law
        ToLog = true;
    end
    methods
        
        function obj = SRDHandler_Controller()
            obj.SerializationPrepNeeded = true;
            obj.PreSerializationPrepFunction = @obj.PreSerializationPrepFunction_Controller;
        end
        
        function PreSerializationPrepFunction_Controller(~)
            error('do not attempt to save Controller Handler; create a new one on the fly instead')
        end
        
    end
end
   