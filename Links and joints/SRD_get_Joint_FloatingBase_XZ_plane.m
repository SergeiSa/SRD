%See documentation for PivotX (SRD_get_Joint_PivotX)
%
%This link directly assigns Link.AbsoluteBase via gen.coordinates, but
%allows to set Link.AbsoluteOrientation as a pure function of the 3
%angles (if the link's parent is Groud or another link with
%constant orientation) or a function of 3 angles and other gen.
%coordinates, if the parent had a non-constant orientation
%
function Joint = SRD_get_Joint_FloatingBase_XZ_plane(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_Joint_FloatingBase_XZ_plane';
Parser.addOptional('Name', []);

Parser.addOptional('ChildLink', []);
Parser.addOptional('ParentLink', []);
Parser.addOptional('ParentFollowerNumber', []);

Parser.addOptional('UsedGeneralizedCoordinates', []);
Parser.addOptional('UsedControlInputs', []);

Parser.addOptional('DefaultJointOrientation', []);
Parser.parse(varargin{:});

Joint = SRD_Joint;

Joint.Type = 'FloatingBase_XZ_plane';

Joint.Name            = Parser.Results.Name;
Joint.ChildLink       = Parser.Results.ChildLink;
Joint.ParentLink      = Parser.Results.ParentLink;

Joint.ChildLink.ParentLink            = Joint.ParentLink;
Joint.ChildLink.ParentFollowerNumber  = Parser.Results.ParentFollowerNumber;
Joint.ChildLink.Joint                 = Joint;

Joint.UsedGeneralizedCoordinates      = Parser.Results.UsedGeneralizedCoordinates;
Joint.UsedControlInputs               = Parser.Results.UsedControlInputs;

Joint.DefaultJointOrientation         = Parser.Results.DefaultJointOrientation;


Joint.ChildLink.Update = @(Input) Update(Joint.ChildLink, Input);

Joint.ActionUpdate     = @(Input) ActionUpdate(Joint, Input);

    function Update(Link, Input)
        input_vec = Input(abs(Link.Joint.UsedGeneralizedCoordinates));
        
        q = diag(sign(Link.Joint.UsedGeneralizedCoordinates)) * ...
            input_vec(:);
        
        
        Link.RelativeOrientation = SRD_RotationMatrix3D_y(q(3));
        
        Link.AbsoluteBase = [q(1); 0; q(2)];
        Link.AbsoluteOrientation = Link.ParentLink.AbsoluteOrientation * Link.RelativeOrientation;
        
        rBaseToFollower = Link.RelativeFollower - repmat(Link.RelativeBase, 1, size(Link.RelativeFollower, 2));
        rBaseToCoM      = Link.RelativeCoM - Link.RelativeBase;
        
        Link.AbsoluteFollower = repmat(Link.AbsoluteBase, 1, size(Link.RelativeFollower, 2)) + Link.AbsoluteOrientation*rBaseToFollower;
        Link.AbsoluteCoM = Link.AbsoluteBase + Link.AbsoluteOrientation*rBaseToCoM;
        
    end

    function generalized_force = ActionUpdate(Joint, Input)
        
    end

end

