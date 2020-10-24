%This class represents a link in a chain mechanism. It implements basic
%forward kinematics
classdef SRD_Link < handle
    properties
    
    Name = [];                  %The name of the link
    
    Update = [];                %function handle, to be assigned
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                            
    %geometrical properties    
    RelativeBase = [];          %Position of the Base in the local RF
    RelativeCoM = [];           %Position of the Center of Mass in the local RF 
    RelativeOrientation = [];   %Orientation of the local RF relative to the FR of the parent link
    RelativeFollower = [];      %Position of the Follower in the local RF
                                %there can be more than one follower, in that case
                                %each gets a column in RelativeFollower matrix
    
    AbsoluteBase = [];          %Position of the Base in the ground RF
    AbsoluteCoM = [];           %Position of the Center of Mass in the ground RF
    AbsoluteOrientation = [];   %Orientation of the local RF relative to the ground RF
    AbsoluteFollower = [];      %Position of the Follower in the ground RF 
                                %(also see description of RelativeFollower)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                            
    %inertial properties
    Mass = [];                  %the mass of the link
    Inertia = [];               %the inertia of the link                            

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                            
    %Settings
    UseAbsoluteCoordinates = false; 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                            
    %visual properties                                
    StlPath = [];
    Mesh = struct('Faces', [], 'Vertices', []);
    Color = [];

    %if true the Update function will not change the .AbsoluteOrientation
    %property
    %if false .AbsoluteOrientation = ParentLink.AbsoluteOrientation * .RelativeOrientation
        
    ParentLink = [];            %handle of the parent link
    ParentFollowerNumber = [];  %handle of the parent link
    
    Order = [];                 %order of the link, used to decide the order of link updates
                                %if order == 0 then it defines the link as ground, 
                                %if order > 0 - as a normal link; ground
                                %has no parent link
     
    %%%%%%%%%%%%%%%%%%%
    % for symbolic/AD derivations
    Jacobian_CenterOfMass = [];
    Jacobian_AngularVelocity = [];
    
    AbsoluteOrientation_derivative = [];
    AngularVelocity = [];
    %%%%%%%%%%%%%%%%%%%
    
    %dynamically added additional parameters
    calculated = [];
    
    end
    methods
                
    end

end