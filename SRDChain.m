%This class implements methods for working with chains of links defined by
%LinkWithJointClass
%last update 03.06.16
classdef SRDChain < handle
    properties
        LinkArray;           
        %the array of links the chain consists of.
        
        nob;                 
        %the number of bodies the mechanism has. Ground doesn't count.
        
        dof;                 
        %number of generalized coordinates used; the property is used by subclasses
    end
    methods
        %This is the class constructor. It requires LinkArray - an array of links (objects of 
        %LinkWithJointClass) - as an input;
        function obj = SRDChain(LinkArray)
            obj.LinkArray = reshape(LinkArray, [], 1);
             
            %here we try to find max value of UsedGenCoordinates property
            %of all links, that should tell us the number of generalized
            %coordinates used, which we then write to obj.dof
            GC = [];
            for i = 1:size(obj.LinkArray, 1)
                GC = [GC; obj.LinkArray(i).UsedGenCoordinates];
            end
            obj.dof = size(GC, 1);
           
            obj.Check();
            obj.Sort();
        end
        
        %This function updates the chain in the correct order.
        %Note that each link should be initialized with
        %SetUsedGenCoordinates method of LinkWithJointClass to use Update
        function Update(obj, q)
            for i = 1:size(obj.LinkArray, 1)
                if obj.LinkArray(i).Order > 0
                   obj.LinkArray(i).JointUpdateGC(q);
                end
                   obj.LinkArray(i).Update();
            end
        end
        
    end
    methods (Access = private)
        
        %this function sorts the LinkArray according to Order property of
        %the elements;
        %it also finds how many moving bodies there are in the array and
        %writes the result in the nob property
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
            
            %below is a calculation of how many bodies there are in the
            %chain
            n = 0;
            for i = 1:size(obj.LinkArray, 1)
                if obj.LinkArray(i).Order > 0
                   n = n + 1;
                end
            end
            obj.nob = n;     
        end      
        
        %this function checks whether or not all joints of the mechanism
        %are compatible with the chosen mechanism type
        function Check(~)
            %warning('This method has not been implemented yet'); 
        end
         
    end
    methods (Access = public)
        %this function calculates mechanism's total mass
        function TotalMass = GetTotalMass(obj)
            TotalMass = 0;
            for i = 1:size(obj.LinkArray, 1)
                TotalMass = TotalMass + obj.LinkArray(i).Mass;
            end
        end
        
        %This function retrieves the index of the link named Name in
        %.LinkArray
        function Index = RetreaveLinkIndexInLinkArray(obj, Name)
            Index = -1;
            for i = 1:length(obj.LinkArray)
                ith_link_name = obj.LinkArray(i).Name;
                if strcmp(ith_link_name, Name)
                    Index = i;
                end
            end
        end
        
        %This function retrieves the link named Name in .LinkArray
        function Link = RetreaveLinkInLinkArray(obj, Name)
            Link = [];
            for i = 1:length(obj.LinkArray)
                ith_link_name = obj.LinkArray(i).Name;
                if strcmp(ith_link_name, Name)
                    Link = obj.LinkArray(i);
                end
            end
        end
        
    end
end