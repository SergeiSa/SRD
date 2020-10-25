%This class implements methods for working with chains of links defined by
%LinkWithJointClass
classdef SRD_Chain < handle
    properties
        LinkArray;
        %the array of links the chain consists of.
        
        JointArray;
        
        dof;
        %number of generalized coordinates used; the property is used by subclasses
        
        control_dof;
    end
    methods
        %This is the class constructor. It requires LinkArray - an array of links (objects of
        %LinkWithJointClass) - as an input;
        function obj = SRD_Chain(LinkArray)
            obj.LinkArray = reshape(LinkArray, [], 1);
            obj.Sort();
            
            for i = 1:size(obj.LinkArray, 1)
                if ~isempty(obj.LinkArray(i).Joint)
                    obj.JointArray = [obj.JointArray; obj.LinkArray(i).Joint];
                end
            end
            
            %Find the number of degrees of freedom for the robot
            GenCoordinates = [];
            for i = 1:size(obj.LinkArray, 1)
                if ~isempty(obj.LinkArray(i).Joint)
                    GenCoordinates = [GenCoordinates; reshape(obj.LinkArray(i).Joint.UsedGeneralizedCoordinates, [], 1)];
                end
            end
            obj.dof = size(GenCoordinates, 1);
            
            
            %Same for control inputs
            ControlInputs = [];
            for i = 1:size(obj.LinkArray, 1)
                if ~isempty(obj.LinkArray(i).Joint)
                    ControlInputs = [ControlInputs; reshape(obj.LinkArray(i).Joint.UsedControlInputs, [], 1)];
                end
            end
            obj.control_dof = size(ControlInputs, 1);
            
        end
        
        %This function updates the chain in the correct order.
        %Note that each link should be initialized with
        %SetUsedGenCoordinates method of LinkWithJointClass to use Update
        function Update(obj, q)
            for i = 1:size(obj.LinkArray, 1)
                obj.LinkArray(i).Update(q);
            end
        end
    end
    
    methods (Access = private)
        %this function sorts the LinkArray according to Order property of
        %the elements;
        function Sort(obj)
            %below is a sorting algorithm
            i = 1;
            while i < size(obj.LinkArray, 1) - 1
                if obj.LinkArray(i).Order > obj.LinkArray(i + 1).Order
                    buffer = obj.LinkArray(i + 1);
                    obj.LinkArray(i + 1) = obj.LinkArray(i);
                    obj.LinkArray(i) = buffer;
                    i = 1;
                end
                i = i + 1;
            end
        end
    end
end