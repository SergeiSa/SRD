%See documentation for PivotX (SRD_get_Joint_PivotX)
function Joint = SRD_get_Joint_PivotZ(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_Joint_PivotZ';
Parser.addOptional('Name', []);

Parser.addOptional('ChildLink', []);
Parser.addOptional('ParentLink', []);
Parser.addOptional('ParentFollowerNumber', []);

Parser.addOptional('UsedGeneralizedCoordinates', []);
Parser.addOptional('UsedControlInputs', []);

Parser.addOptional('DefaultJointOrientation', []);
Parser.parse(varargin{:});

Joint = SRD_Joint;

Joint.Type = 'PivotZ';

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
        q = SRD_get_UsedGeneralizedCoordinates(Link, Input);
        
        Link.RelativeOrientation =  Link.Joint.DefaultJointOrientation * SRD_RotationMatrix3D_z(q);
        
        SRD_ForwardKinematics_JointUpdate_RelativeOrientationType(Link);
    end

    function generalized_force = ActionUpdate(Joint, Input)
        u = Input(Joint.UsedControlInputs);
        
        Child_T = Joint.ChildLink.AbsoluteOrientation;
        
        %this is because torque_child is expressed in the relative
        %coordinates, while jacobians are in the absolute ones
        torque_child  = Child_T * [0; 0; u];
        torque_parent = Child_T * [0; 0; -u];
        
        %get angular velocity jacobians
        Child_Jw = Joint.ChildLink.Jacobian_AngularVelocity;
        Parent_Jw = Joint.ParentLink.Jacobian_AngularVelocity;
        
        %find generalized forces generated by the actuator
        gen_force_child  = Child_Jw'  * torque_child;
        gen_force_parent = Parent_Jw' * torque_parent;
        
        generalized_force = gen_force_child + gen_force_parent;
    end

end


