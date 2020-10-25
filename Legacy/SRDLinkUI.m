classdef SRDLinkUI < handle
    properties
        
        Name;
        %link's name
        
        %%%%%%%%%%%%%%%%%%%%%
        %input geometry
        
        Base = [0; 0; 0];
        %base point of the link in the local frame
        
        CenterOfMass = [];
        %center of mass of the link in the local frame
        
        Follower = {};
        %array of follower points of the link in the local frame
        
        %%%%%%%%%%%%%%%%%%%%%
        %input parameters
        
        Mass = [];
        %links' mass
        
        Inertia = [];
        %links' inertia
        
        %%%%%%%%%%%%%%%%%%%%%
        %animation parameters 
        
        
        %%%%%%%%%%%%%%%%%%%%%
        %calculated parameters 
        
        FollowerCalculated;
        %array of follower points of the link in the local frame
        
    end
    methods
        function Draw(obj)
            Count = length(obj.Follower);
            Points = zeros(Count, 3);
            
            for i = 1:Count
                Points(i, :) = obj.Follower{i};
            end
            
            draw
        end
    end
end