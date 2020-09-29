classdef SRDHandler < handle
    properties
        TimeStart = -Inf;
        TimeExpiration = Inf;
        
        State = [];
        
        SerializationPrepNeeded = false;
        PreSerializationPrepFunction = [];
        PostSerializationPrepFunction = [];
    end
    methods
        
    end
end
   