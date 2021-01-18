%SRD joint class - used to create SRD joint objects, which are used to
%connect SRD links
%
%See documentation for PivotX (SRD_get_Joint_PivotX)
%
classdef SRD_Joint < handle
    properties
    
    Name = [];                  %The name of the joint
    
    Type = [];                  %The type of the joint
    
    Update = [];                %function handle, to be assigned
    
    ParentLink = [];            %to which link the joint is connected
    ChildLink = [];             %which joints is connected to the joint
    
    ActionUpdate = [];          %function handle, needs to be defined
    
    UsedGeneralizedCoordinates = [];
    UsedControlInputs = [];
    
    DefaultJointOrientation = [];
    
    end
    methods
                
    end
end