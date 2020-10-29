function Joint = SRD_get_Joint_Fixed(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_Joint_Fixed';
Parser.addOptional('Name', []);

Parser.addOptional('ChildLink', []);
Parser.addOptional('ParentLink', []);
Parser.addOptional('ParentFollowerNumber', []);

Parser.addOptional('UsedGeneralizedCoordinates', []);
Parser.addOptional('UsedControlInputs', []);

Parser.addOptional('DefaultJointOrientation', []);
Parser.parse(varargin{:});

Joint = SRD_Joint;

Joint.Type = 'Fixed';

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

    function Update(Link, ~)
        
        Link.RelativeOrientation =  Link.Joint.DefaultJointOrientation;
        
        SRD_ForwardKinematics_JointUpdate_RelativeOrientationType(Link);
    end

    function generalized_force = ActionUpdate(~, ~)
        generalized_force = [];
    end

end


