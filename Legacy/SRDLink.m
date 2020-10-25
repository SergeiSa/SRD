%This class represents a link in a chain mechanism. It implements basic
%forward kinematics
%last update 03.06.16
classdef SRDLink < handle
    properties
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

    StlPath = "";
    Mesh = struct('Faces',[],'Vertices',[]);
    Color = [];

    UseAbsoluteCoordinates = false; 
    %if true the Update function will not change the .AbsoluteOrientation
    %property
    %if false .AbsoluteOrientation = ParentLink.AbsoluteOrientation * .RelativeOrientation
        
    FileName = [];              %name of the file that stores geometry of the link    
    
    ParentLink = [];            %handle of the parent link
    ParentFollowerNumber = [];  %handle of the parent link
    
    Order = [];                 %order of the link, used to decide the order of link updates
                                %if order == 0 then it defines the link as ground, 
                                %if order > 0 - as a normal link; ground
                                %has no parent link
                                
    Mass = [];                  %the mass of the link
    Inertia = [];               %the inertia of the link
                                
    Name = [];                  %The name of the link
    
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
        
        %Class constructor, requires Order, FileName and ParentLink and (descriptions are above)
        %if Order == 0 you don't need to pass ParentLink
        %function obj = SRDLink(Order, FileName, ParentLink, ParentFollowerNumber)
        function obj = SRDLink(varargin)
            %this if-else implements backwards compatibility with the
            %original SRD robot generation design
            if (nargin == 4) && isnumeric(varargin{1})
                obj.Order = varargin{1};
                obj.FileName = varargin{2};
                
                if obj.Order > 0
                    obj.ParentLink = varargin{3};
                    obj.ParentFollowerNumber = varargin{4};
                end
                
            else
                Parser = inputParser;
                Parser.FunctionName = 'SRDLink';
                Parser.addOptional('Order', []);
                Parser.addOptional('FileName', []);
                Parser.addOptional('LinkParametersStructure', []);
                Parser.addOptional('ParentLink', []);
                Parser.addOptional('ParentFollowerNumber', []);
                Parser.addOptional('JointType', []); %child parameter
                Parser.parse(varargin{:});
                
                %below are checks that all the parameters have been passed
                if isempty(Parser.Results.Order)
                    error('Pass Order parameter when creating a Link')
                else
                    obj.Order = Parser.Results.Order;
                end
                if ~xor(isempty(Parser.Results.FileName), isempty(Parser.Results.LinkParametersStructure))
                    error('Pass FileName or LinkParametersStructure when creating a Link')
                end
                
                if obj.Order > 0
                    if isempty(Parser.Results.ParentLink)
                        error('Pass ParentLink parameter when creating a Link')
                    else
                        obj.ParentLink = Parser.Results.ParentLink;
                    end
                    if isempty(Parser.Results.ParentFollowerNumber)
                        error('Pass ParentFollowerNumber parameter when creating a Link')
                    else
                        obj.ParentFollowerNumber = Parser.Results.ParentFollowerNumber;
                    end
                end
                
                obj.FileName = Parser.Results.FileName;              
            end
            
            
            if isempty(obj.FileName)
                Structure = Parser.Results.LinkParametersStructure;
            else
                %loading a structure from the file, and using it to assign the
                %properties of the class
                Structure = load(obj.FileName);
            end

            obj.RelativeBase = Structure.RelativeBase;
            obj.RelativeFollower = Structure.RelativeFollower; 
            obj.RelativeCoM = Structure.RelativeCoM;
            
            obj.Mass = Structure.Mass;
            obj.Inertia = Structure.Inertia;
            
            obj.Name = Structure.Name;
        end
        
        %This function updates the absolute properties of the link. Should
        %be called only when the parent link has already been updated.
        %Note that RelativeOrientation property needs to be set before
        %one can use this method
        function Update(obj)
            if obj.Order > 0
                %Link's AbsoluteBase is the same as parent's AbsoluteFollower
                obj.AbsoluteBase = obj.ParentLink.AbsoluteFollower(:, obj.ParentFollowerNumber);
                if ~obj.UseAbsoluteCoordinates
                    obj.AbsoluteOrientation = obj.ParentLink.AbsoluteOrientation * obj.RelativeOrientation;
                end
                
                rBaseToFollower = obj.RelativeFollower - repmat(obj.RelativeBase, 1, size(obj.RelativeFollower, 2));
                rBaseToCoM = obj.RelativeCoM - obj.RelativeBase;

                obj.AbsoluteFollower = repmat(obj.AbsoluteBase, 1, size(obj.RelativeFollower, 2)) + obj.AbsoluteOrientation*rBaseToFollower;
                obj.AbsoluteCoM = obj.AbsoluteBase + obj.AbsoluteOrientation*rBaseToCoM;     
            else
                %for the ground link (obj.Order == 0) there is no
                %difference between Relative and Absolute
                obj.AbsoluteBase = obj.RelativeBase;          
                obj.AbsoluteFollower = obj.RelativeFollower;      
                obj.AbsoluteCoM = obj.RelativeCoM; 
                obj.AbsoluteOrientation = obj.RelativeOrientation;  
            end                   
        end
        
        
    end
    
    methods (Access = private)     
    end
end