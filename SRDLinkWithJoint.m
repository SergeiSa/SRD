%Class represents a link with a joint.
%this class inherits from LinkClass that represents a link in a chain mechanism
%LinkWithJointClass adds a joint to the base of each link, allowing for 
%more automation in forward kinematics
%last update 03.06.16
classdef SRDLinkWithJoint < SRDLink
    properties
        
        JointType = []; %This property defines what joint connects this link to the previous one
        
        UsedGenCoordinates = []; %This property defines which elements of the vector of 
                                 %generalised coordinates will be used as inputs
                                 %to the functions that implement joints.
                                 %Should be a column vector of numbers    
                                 
        PrizmaticJointZeroPosition = [];
        %this is only needed for prizmatic joints, it defines where the
        %parent follower for the link should be when the corresponding gen
        %coordinate is zero.
        PivotZeroOrientation = eye(3);
        %this is optional for pivit joints, for more seemless connection
        %with URDF-style robot description. Lets you add a defaught
        %orientation offset for the link. Same can be achieved by the
        %choice of the base/follower/CoM vectors.
        
        StaticOrientationMatrix = eye(3);
        %this matrix SO(3) defines orientation of the body if
        %'prismatic_XYZ' joint type is used
        
    end
    
    properties (Access = protected)          
        
        ValidJointTypes = {'none'; ... %Those are valid values of JointType property
                           'FloatingBase_6dof'; ...
                           'FloatingBase_6dof_ZYX'; ...
                           'abs_spherical'; ...
                           'prismatic_XYZ'; ...
                           'planarX'; ...
                           'planarY';...
                           'planarZ';...
                           'abs_planarX'; ...
                           'abs_planarY';...
                           'abs_planarZ';...
                           'pivotX'; ...
                           'pivotY'; ...
                           'pivotZ';...
                           'abs_pivotX'; ...
                           'abs_pivotY'; ...
                           'abs_pivotZ'; ...
                           'pivotXY'; ...
                           'pivotYZ'; ...
                           'pivotZX'; ...
                           'prismaticX'; ...
                           'prismaticY'; ...
                           'prismaticZ'; ...
                           'fixed'};
    end
    
    properties (Access = private) 
        Math = MathClass; %This is an object of MathClass that implements some useful math operations     
    end
    
    methods
        %this is the class constructor, in addition to the parameters needed
        %for the class constructor of LinkClass it requires input value JointType, 
        %which can only take values determined by ValidJointTypes property
        function obj = SRDLinkWithJoint(varargin)
            if (nargin == 5) && isnumeric(varargin{2})
                arg = {varargin{2}, varargin{3}, varargin{4}, varargin{5}};
                JointType = varargin{1};
            else
                arg = varargin;
                
                Parser = inputParser;
                Parser.FunctionName = 'SRDLinkWithJoint';
                Parser.addOptional('JointType', []);
                Parser.addOptional('Order', []);     %parent parameter
                Parser.addOptional('FileName', []);  %parent parameter
                Parser.addOptional('LinkParametersStructure', []); %parent parameter
                Parser.addOptional('ParentLink', []); %parent parameter
                Parser.addOptional('ParentFollowerNumber', []); %parent parameter
                Parser.parse(varargin{:});
                JointType = Parser.Results.JointType;
            end
            obj@SRDLink(arg{:});
            
            switch JointType
                case obj.ValidJointTypes
                    obj.JointType = JointType;
                otherwise
                    warning('Invalid joint type'); 
            end
            
            %if the JointType is absosute then write that into the property
            switch JointType
                case {'abs_spherical', 'abs_pivotX', 'abs_pivotY', 'abs_pivotZ'}
                    obj.UseAbsoluteCoordinates = true;
                case {'prismaticX', 'prismaticY', 'prismaticZ',...
                      'planarX', 'planarY', 'planarZ'}
                    obj.PrizmaticJointZeroPosition = obj.ParentLink.RelativeFollower(:, obj.ParentFollowerNumber);
                case {'prismatic_XYZ',...
                      'abs_planarX', 'abs_planarY', 'abs_planarZ'}
                    obj.UseAbsoluteCoordinates = true;
                    obj.PrizmaticJointZeroPosition = obj.ParentLink.RelativeFollower(:, obj.ParentFollowerNumber);
            end
            
        end
        
        %this function checks what dimentions are required by the chosen 
        %joint type
        function ProperSize = GetJointInputsRequirements(obj)
            switch obj.JointType
                case {'none', 'fixed'}
                    ProperSize = 0;
                case {'FloatingBase_6dof', 'FloatingBase_6dof_ZYX'}
                    ProperSize = 6;
                case {'abs_spherical', 'prismatic_XYZ', ...
                      'planarX', 'planarY', 'planarZ', ...
                      'abs_planarX', 'abs_planarY', 'abs_planarZ'}
                    ProperSize = 3;
                case {'pivotXY', 'pivotYZ', 'pivotZX'}
                    ProperSize = 2;
                case {'pivotX', 'pivotY', 'pivotZ', 'abs_pivotX', 'abs_pivotY', 'abs_pivotZ', ...
                        'prismaticX', 'prismaticY', 'prismaticZ'}
                    ProperSize = 1;
                otherwise
                    warning('Invalid joint type');
            end            
        end
        
        %this function sets the value for UsedGenCoordinates property,
        %checking for it is to match the dimentions required by the joint
        %using method GetJointInputsRequirements
        %function calls JointUpdatePrivate method to do the work
        function obj = SetUsedGenCoordinates(obj, UsedGenCoordinates)    
            ProperSize = obj.GetJointInputsRequirements();
            
            if length(UsedGenCoordinates) == ProperSize
                obj.UsedGenCoordinates = reshape(UsedGenCoordinates, [], 1);
            else
                warning('This joint type requires input of different dimentions');
            end
        end
        
        %This functions updates the values that are dependant on the
        %joint's state. Caller needs to take care of providing correct
        %inputs, as determined by GetJointInputsRequirements
        %function calls JointUpdatePrivate method to do the work
        function JointUpdate(obj, Input)
            ProperSize = obj.GetJointInputsRequirements();
            
            if size(Input, 1) == ProperSize
                obj.JointUpdatePrivate(Input);
            else
                warning('This joint type requires input of different dimentions');
            end
        end
        
        %This functions updates the values that are dependant on the
        %joint's state. It takes full vector of generalised coordinates as an input        
        function JointUpdateGC(obj, q)
            Input = obj.GetInputFrom_q(q);
            obj.JointUpdatePrivate(Input);
        end        

        
    end
    
    methods (Access = private)
        %This functions updates the values that are dependant on the
        %joint's state. It calls the correct method for the chosen joint
        %type
        function JointUpdatePrivate(obj, Input)
            switch obj.JointType
                case 'none'
                    warning('Attempt to update a 0 dof joint');
                case 'FloatingBase_6dof'
                    obj.FloatingBase_6dof(Input);
                case 'FloatingBase_6dof_ZYX'
                    obj.FloatingBase_6dof_ZYX(Input);
                case 'abs_spherical'
                    obj.abs_spherical(Input);
                case 'prismatic_XYZ'
                    obj.prismatic_XYZ(Input);
                case 'planarX'
                    obj.planarX(Input, false); %absolute = false
                case 'planarY'
                    obj.planarY(Input, false); %absolute = false
                case 'planarZ'
                    obj.planarZ(Input, false); %absolute = false
                case 'abs_planarX'
                    obj.planarX(Input, true); %absolute = true
                case 'abs_planarY'
                    obj.planarY(Input, true); %absolute = true
                case 'abs_planarZ'
                    obj.planarZ(Input, true); %absolute = true
                case 'pivotX'
                    obj.pivotX(Input);
                case 'pivotY'
                    obj.pivotY(Input);
                case 'pivotZ'
                    obj.pivotZ(Input);
                case 'abs_pivotX'
                    obj.abs_pivotX(Input);
                case 'abs_pivotY'
                    obj.abs_pivotY(Input);
                case 'abs_pivotZ'
                    obj.abs_pivotZ(Input);
                case 'pivotXY'
                    obj.pivotXY(Input);
                case 'pivotYZ'
                    obj.pivotYZ(Input);
                case 'pivotZX'
                    obj.pivotZX(Input);
                case 'prismaticX'
                    obj.prismaticX(Input);
                case 'prismaticY'
                    obj.prismaticY(Input);
                case 'prismaticZ'
                    obj.prismaticZ(Input);
                case 'fixed'
                    obj.fixed(Input);
                otherwise
                    warning('Invalid joint type');
            end              
        end
        
        %This function prepares Input in the form that works for the
        %functions that implement joints, taking the proper elements from the
        %vector of generalized coordinates q.
        %It uses UsedGenCoordinates property to choose which elements to
        %take
        function Input = GetInputFrom_q(obj, q)
            
            n = length(obj.UsedGenCoordinates);
            
            %the class can be used for both symbolic and numeric
            %computations
            switch class(q)
                case 'sym'
                    Input = sym(zeros(n, 1));
                case 'double'
                    Input = zeros(n, 1);
                case 'casadi.SX'
                    import casadi.*
                    Input = SX.zeros(n, 1);
                otherwise
                    error('invalid type of q')
            end
            
            for i = 1:n
                Input(i) = q(abs(obj.UsedGenCoordinates(i))) * sign(obj.UsedGenCoordinates(i));
            end
        end
        
        %this function implements pivotX joint - a pivot which axis of rotation 
        %is alined with X axis (in body frame)
        function pivotX(obj, Input)
            obj.RelativeOrientation =  obj.PivotZeroOrientation*obj.Math.RotationMatrix3D_x(Input);
        end
        
        %this function implements pivotY joint - a pivot which axis of rotation 
        %is alined with Y axis (in body frame)
        function pivotY(obj, Input)
            obj.RelativeOrientation =  obj.PivotZeroOrientation*obj.Math.RotationMatrix3D_y(Input);
        end   
        
        %this function implements pivotZ joint - a pivot which axis of rotation 
        %is alined with Z axis (in body frame) 
        function pivotZ(obj, Input)
            obj.RelativeOrientation = obj.PivotZeroOrientation*obj.Math.RotationMatrix3D_z(Input);
        end  
        
        %this function implements pivotX joint - a pivot which axis of rotation 
        %is alined with X axis (in world frame)
        function abs_pivotX(obj, Input)
            obj.AbsoluteOrientation = obj.PivotZeroOrientation*obj.Math.RotationMatrix3D_x(Input);
        end
        
        %this function implements pivotY joint - a pivot which axis of rotation 
        %is alined with Y axis (in world frame)
        function abs_pivotY(obj, Input)
            obj.AbsoluteOrientation =  obj.PivotZeroOrientation*obj.Math.RotationMatrix3D_y(Input);
        end
        
        %this function implements pivotZ joint - a pivot which axis of rotation 
        %is alined with Z axis (in world frame)
        function abs_pivotZ(obj, Input)
            obj.AbsoluteOrientation = obj.PivotZeroOrientation*obj.Math.RotationMatrix3D_z(Input);
        end
        
        
        %this function implements FloatingBase_6dof joint - a free body floating in 3D space        
        function FloatingBase_6dof(obj, Input)
            Tx = obj.Math.RotationMatrix3D_x(Input(1));
            Ty = obj.Math.RotationMatrix3D_y(Input(2));
            Tz = obj.Math.RotationMatrix3D_z(Input(3));
            
            obj.RelativeOrientation = Tz*Ty*Tx;
            
            switch class(Input)
                case 'sym'
                    obj.ParentLink.RelativeFollower = sym(obj.ParentLink.RelativeFollower);
                case 'double'
                    %obj.ParentLink.RelativeFollower = obj.ParentLink.RelativeFollower;
                case 'casadi.SX'
                    import casadi.*
                    obj.ParentLink.RelativeFollower = SX(obj.ParentLink.RelativeFollower);
                otherwise
                    error('invalid type of Input')
            end
            
            obj.ParentLink.RelativeFollower(:, obj.ParentFollowerNumber) = [Input(4); Input(5); Input(6)];
            obj.ParentLink.Update();
        end
        
        %this function implements FloatingBase_6dof joint - a free body floating in 3D space        
        function FloatingBase_6dof_ZYX(obj, Input)
            Tx = obj.Math.RotationMatrix3D_x(Input(1));
            Ty = obj.Math.RotationMatrix3D_y(Input(2));
            Tz = obj.Math.RotationMatrix3D_z(Input(3));
            
            obj.RelativeOrientation = Tx*Ty*Tz;
            obj.ParentLink.RelativeFollower(:, obj.ParentFollowerNumber) = [Input(4); Input(5); Input(6)];
            obj.ParentLink.Update();
        end        
        
        %this function implements FloatingBase_6dof joint - a free body floating in 3D space
        %The orientation is absolute, doesn't depend on the prevous joints
        function abs_spherical(obj, Input)
            Tx = obj.Math.RotationMatrix3D_x(Input(1));
            Ty = obj.Math.RotationMatrix3D_y(Input(2));
            Tz = obj.Math.RotationMatrix3D_z(Input(3));
            
            obj.AbsoluteOrientation = Tz*Ty*Tx;
        end
        
        %this function implements prismatic_XYZ joint - a free body floating in 3D space without rotation        
        function prismatic_XYZ(obj, Input)
            obj.AbsoluteOrientation = obj.StaticOrientationMatrix;
            obj.ParentLink.RelativeFollower(:, obj.ParentFollowerNumber) = [Input(1); Input(2); Input(3)];
            obj.ParentLink.Update();
        end        
        
        
        %this function implements planarX joint - a free body floating in YOZ space           
        function planarX(obj, Input, absolute)
            if absolute
                obj.AbsoluteOrientation = obj.Math.RotationMatrix3D_x(Input(1));
            else
                obj.RelativeOrientation = obj.Math.RotationMatrix3D_x(Input(1));
            end
            
            obj.ParentLink.RelativeFollower(2, obj.ParentFollowerNumber) = obj.PrizmaticJointZeroPosition(2) + Input(2);
            obj.ParentLink.RelativeFollower(3, obj.ParentFollowerNumber) = obj.PrizmaticJointZeroPosition(3) + Input(3);
            obj.ParentLink.Update();
        end
        
        %this function implements planarY joint - a free body floating in ZOX space           
        function planarY(obj, Input, absolute)
            if absolute
                obj.AbsoluteOrientation = obj.Math.RotationMatrix3D_y(Input(1));
            else
                obj.RelativeOrientation = obj.Math.RotationMatrix3D_y(Input(1));
            end
            
            obj.ParentLink.RelativeFollower(1, obj.ParentFollowerNumber) = obj.PrizmaticJointZeroPosition(1) + Input(2);
            obj.ParentLink.RelativeFollower(3, obj.ParentFollowerNumber) = obj.PrizmaticJointZeroPosition(3) + Input(3);
            
            obj.ParentLink.Update();
        end
        
        %this function implements planarY joint - a free body floating in XOY space           
        function planarZ(obj, Input, absolute)
            if absolute
                obj.AbsoluteOrientation = obj.Math.RotationMatrix3D_z(Input(1));
            else
                obj.RelativeOrientation = obj.Math.RotationMatrix3D_z(Input(1));
            end
            
            obj.ParentLink.RelativeFollower(1, obj.ParentFollowerNumber) = obj.PrizmaticJointZeroPosition(1) + Input(2);
            obj.ParentLink.RelativeFollower(2, obj.ParentFollowerNumber) = obj.PrizmaticJointZeroPosition(2) + Input(3);
            
            obj.ParentLink.Update();
        end
        
        %this function implements joint that gives rotation around both X and Y axes 
        function pivotXY(obj, Input)
            Tx = obj.Math.RotationMatrix3D_x(Input(1));
            Ty = obj.Math.RotationMatrix3D_y(Input(2));
            
            obj.RelativeOrientation = obj.PivotZeroOrientation*Ty*Tx ;
        end
        
        %this function implements joint that gives rotation around both Y and Z axes 
        function pivotYZ(obj, Input)
            Ty = obj.Math.RotationMatrix3D_y(Input(1));
            Tz = obj.Math.RotationMatrix3D_z(Input(2));
            
            obj.RelativeOrientation = obj.PivotZeroOrientation*Tz*Ty;
        end
        
        %this function implements joint that gives rotation around both Z and X axes 
        function pivotZX(obj, Input)
            Tz = obj.Math.RotationMatrix3D_z(Input(1));
            Tx = obj.Math.RotationMatrix3D_x(Input(2));
            
            obj.RelativeOrientation = obj.PivotZeroOrientation*Tx*Tz;
        end
        
        %prizmatic joint along X axis
        function prismaticX(obj, Input)
            if isempty(obj.RelativeOrientation)
                obj.RelativeOrientation = eye(3);
            end
            obj.ParentLink.RelativeFollower(1, obj.ParentFollowerNumber) = obj.PrizmaticJointZeroPosition(1) + Input;
            obj.ParentLink.Update();
        end
        
        %prizmatic joint along Y axis
        function prismaticY(obj, Input)
            if isempty(obj.RelativeOrientation)
                obj.RelativeOrientation = eye(3);
            end
            obj.ParentLink.RelativeFollower(2, obj.ParentFollowerNumber) = obj.PrizmaticJointZeroPosition(2) + Input;
            obj.ParentLink.Update();
        end
        
        %prizmatic joint along Z axis
        function prismaticZ(obj, Input)
            if isempty(obj.RelativeOrientation)
                obj.RelativeOrientation = eye(3);
            end
            obj.ParentLink.RelativeFollower(3, obj.ParentFollowerNumber) = obj.PrizmaticJointZeroPosition(3) + Input;
            obj.ParentLink.Update();
        end
        
        %prizmatic joint along Z axis
        function fixed(obj, ~)
            if isempty(obj.RelativeOrientation)
                obj.RelativeOrientation = eye(3);
            end
            
            obj.ParentLink.Update();
        end
        
        
        
    end    
    
end
    