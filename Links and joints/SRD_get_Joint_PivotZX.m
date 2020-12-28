function Joint = SRD_get_Joint_PivotZX(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_Joint_PivotZX';
Parser.addOptional('Name', []);

Parser.addOptional('ChildLink', []);
Parser.addOptional('ParentLink', []);
Parser.addOptional('ParentFollowerNumber', []);

Parser.addOptional('UsedGeneralizedCoordinates', []);
Parser.addOptional('UsedControlInputs', []);

Parser.addOptional('DefaultJointOrientation', []);
Parser.parse(varargin{:});

Joint = SRD_Joint;

Joint.Type = 'PivotZX';

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

Joint.ActionUpdate     = @(Input) Update(Joint, Input);

    function Update(Link, Input)

        q = Input(Link.Joint.UsedGeneralizedCoordinates);
        
        Tz = SRD_RotationMatrix3D_z(q(1));
        Tx = SRD_RotationMatrix3D_x(q(2));

        Link.RelativeOrientation = Link.Joint.DefaultJointOrientation*Tx*Tz;
        
        SRD_ForwardKinematics_JointUpdate_RelativeOrientationType(Link);
    end

    function generalized_force = ActionUpdate(Joint, Input)
        warning('Action update is not defined')
    end

end
