%This class generates equations for kinematics and dynamics
%last update 1.10.16
classdef SRDMechanicalEquations < SRDChain
    properties
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Structures and arrays storing symbolic expressions
        
        GeometryArray;  % a cell array of structures S:
        % S.rC - the position of the center of mass (CoM) of the link
        % S.T - orientation of the link
        % S.J - jacobian of the link's CoM; J = d(rC)/dq
        %
        % S.Link - a reference to the link this elements is connected to;
        % The array should have obj.nob elements;
        
        KinematicsArray;    % a cell array of structures S:
        % S.vC - the velocity of the center of mass (CoM) of the link
        % S.aC - the acceleration of the center of mass (CoM) of the link
        % S.J1 - jacobian J1 of the link's CoM; J1 = d(vC)/dv = J
        % S.J2 - jacobian J2 of the link's CoM; J2 = d(vC)/dq
        %
        % S.Link - a reference to the link this elements is connected to;
        % The array should have obj.nob elements;
        
        AngularVelocityArray;   % a cell array of structures S:
        % S.w - Angular Velocity of the link
        % S.Jw - jacobian, a linear map between the gen. velocities and
        % the angular velocity: Jw = d(w)/dv, w = Jw*v;
        %
        % S.Link - a reference to the link this elements is connected to;
        % The array should have obj.nob elements;
        
        MotorActionStructure = [];
        % a structure with fileds:
        % .ActionArray - an array identical to .ExternalForcesArray, read
        % comment below. ActionArray is parametrized with obj.u        
        % .ControlActionsList - a cell array of strings, the strings
        % explain meaning of each available control action; you can use it
        % as a reference
        MotorActionStructure_user_defined = false;
        %if true, the system will not update it unless specifically
        %requested to;
        
        ExternalForcesArray = []; % an array each element of which 
        % is a structure with fields:
        % .Name - the name of the force/torque
        % .Link - the link the force/torque acts on
        % .Type - is either 'force' or 'torque'
        % .ApplicationPoint is a 3 by 1 vector of Cartesian coordinates of
        % the application point of the force; described in local
        % coordinates of the link; not needed for torques.
        % .ExpressedIn - is either 'absolute coordinates' or 
        % 'relative coordinates'
        % .Parameterized - defines how the force/torque is parametrised
        %An example of how to use it:
            % ExternalForcesArray{1}.Name = 'test force';
            % ExternalForcesArray{1}.Link = LinkName1;
            % ExternalForcesArray{1}.Type = 'force';
            % ExternalForcesArray{1}.ApplicationPoint = [0; 0; 0.1];
            % ExternalForcesArray{1}.ExpressedIn = 'absolute coordinates';
            % ExternalForcesArray{1}.Parameterized = [obj.FT(1); 0; 0];
            % 
            % ExternalForcesArray{2}.Name = 'test torque';
            % ExternalForcesArray{2}.Link = LinkName2;
            % ExternalForcesArray{2}.Type = 'torque';
            % ExternalForcesArray{2}.ApplicationPoint = [];
            % ExternalForcesArray{2}.ExpressedIn = 'relative coordinates'; 
            % ExternalForcesArray{2}.Parameterized = [0; obj.FT(3); obj.FT(2)];
            %
            % obj.ExternalForcesArray = ExternalForcesArray;
            
        ExternalAndMotorForcesArray = [];
        %combination of ExternalForcesArray and
        %MotorActionStructure.ActionArray, it is the result of the work of 
        %UpdateExternalAndMotorForcesArray() method. It also contains
        %additional fieds:
        %.LinearMap - a linear map to the space of gen forces.
        %.Jacobian - the jacobian associated with that force.

        ForwardDynamicsStructure;
        % a structure that has following fields:
        % .JSIM - the joint space inertia matrix of the mechanism
        % .KineticEnergy - the kinetic energy of the mechanism
        % .GenInertialForces - generalised inertial forces of the mechanism.
        % .GenGravitationalForces - generalised gravinational forces
        % acting on the mechanism
        % .GenDisspativeForces - generalised dissipative forces
        % acting on the mechanism
        % .ExternalAndMotorForces - generalised external forces
        % .ControlActionsToGenMotorTorquesMap - map that maps control 
        % actions to generalized forses due to torques
        % .ExternalForcesParametersToGenExternalForcesMap - map that maps  
        % external forces/torques parameters to generalized external forses
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Model parameters 
        
        gravitational_constant = [0; 0; -9.8];
        %This is gravitational constant, the first element of the cell
        %array is the value for 3D case, the second - for the planar case
        
        dissipation_coefficients;
        %This is a vector which elements are coefficients of linear
        %dissipative forces
        %By default is a column vector of ones.
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Symbolic variables, used for derivations 
        
        q; %the vector of generalized coordinates;
        v; %the vector of generalized velocities;
        a; %the vector of generalized accelerations;
        M; %a matrix of torques acting on the bodies
        %There is an assumption that only one torque is acting on each
        %body. To know which column of the matrix is assinged to which body
        %look at the Name in properties in GeometryArray, the order is
        %preserved.
        u; %the vector of control actions (scalar values of torques that
        %are being produced by the motors)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Code generation parameters
        
        ToOptimizeFunctions = true;
        %if true every generated function will be optimized;
        
        ToDeepSimplify = false;
        %This property determines whether or not some of the intermidiate
        %results in the symbolic computations should be simplified
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% External classes
        
        Math
        %object of MathClass class; drovides useful functions;
        
        
        NoExternalForcesorTorques = true;
        %If this property is true methods UpdateExternalForcesArray(),
        %GetGenExternalForces() and
        %GetExternalForcesParametersToGenExternalForcesMap() will be
        %skipped when building the dynamics eqs.
        
        Casadi = false;
        
    end
    properties (SetObservable)
        
        UseParallelizedSimplification = false;
        %If true, the programm will simplify the elements of symbolic 
        %vector expressions in parallel and it will report the progress
        
        NumberOfWorkers = 9;
        %Defines the number of MATLAB workers that will be used in parallel
        %computing
    end
    methods
        % class constructor, it sets dof property,
        % initializes properties q, v, a, M,
        % and calls the superclass constructor.
        % it also assignes a valuse to dissipation_coefficients
        % property
        function obj = SRDMechanicalEquations(LinkArray, Casadi)
            obj = obj@SRDChain(LinkArray);
            
            obj.Casadi = Casadi;
            
            if obj.Casadi
                import casadi.*
                
                obj.q = SX.sym('q', [obj.dof, 1]);
                obj.v = SX.sym('v', [obj.dof, 1]);
                obj.a = SX.sym('a', [obj.dof, 1]);
                obj.M = SX.sym('M', [3, obj.nob]);
            else
                obj.q = sym('q', [obj.dof, 1]);
                obj.v = sym('v', [obj.dof, 1]);
                obj.a = sym('a', [obj.dof, 1]);
                obj.M = sym('M', [3, obj.nob]);
                obj.SetAssumptions();
            end
            
            obj.dissipation_coefficients = ones(obj.dof, 1);
            obj.Math = MathClass(obj.NumberOfWorkers);
            
            addlistener(obj, 'UseParallelizedSimplification', 'PostSet', @obj.handleParallelizationEvents);
            addlistener(obj, 'NumberOfWorkers',               'PostSet', @obj.handleParallelizationEvents);
        end
        
        
        % This function find a cell array of structures, see
        % description of GeometryArray property; Some of the field of
        % the elements of GeometryArray won't be filled up;
        function GA = PrepareGeometryArray(obj, ~)
            % Here we deal with the problem that RelativeFollower field
            % is initialised as a numeric array, and then the code will
            % attempt to modify it column by column; so to avoid Matlab
            % trying to convert symbolic to numerical we need to make
            % sure all matrices are converted to symbolic beforehand
            for i = 1:size(obj.LinkArray, 1)
                switch class(obj.q)
                    case 'sym'
                        obj.LinkArray(i).RelativeFollower = sym(obj.LinkArray(i).RelativeFollower);
                    case 'double'
                        %obj.LinkArray(i).RelativeFollower = obj.LinkArray(i).RelativeFollower;
                    case 'casadi.SX'
                        %obj.LinkArray(i).RelativeFollower = obj.LinkArray(i).RelativeFollower;
                    otherwise
                        error('invalid type of q')
                end
            end
            
            obj.Update(obj.q);
            obj.Math.UseParallel = obj.UseParallelizedSimplification;
            
            %v = dq/dt - generalized velocities
            %vC = d(rC)/dt - velocity of the CoM
            %J = d(rC)/dq - jacobian matrix
            %vC = J*v
            index = 0;
            for i = 1:size(obj.LinkArray, 1)
                if obj.LinkArray(i).Order > 0
                    index = index + 1;
                    %here we fill an element of GeometryArray
                    
                    
                    GeometryStructure.rC = obj.LinkArray(i).AbsoluteCoM;
                    GeometryStructure.T = obj.LinkArray(i).AbsoluteOrientation;
                    
                    if isa(obj.q, 'sym')
                        GeometryStructure.rC = obj.Math.simplify(GeometryStructure.rC, 'rC');
                        GeometryStructure.T  = obj.Math.simplify(GeometryStructure.T,  'T');
                    end
                    
                    GeometryStructure.J = jacobian(GeometryStructure.rC, obj.q);
                    if isa(obj.q, 'sym')
                        GeometryStructure.J = obj.Math.simplify(GeometryStructure.J, 'J');
                    end
                    
                    GeometryStructure.Link = obj.LinkArray(i);
                    
                    GA{index} = GeometryStructure;
                    
                    disp(['Finished calculating geometry for ''', GeometryStructure.Link.Name, ...
                        ''' link, ', int2str(obj.nob - index), ' remaining']);
                    
                end
            end % obj.nob should be equal to size(GeometryArray, 1)
            disp('* Finished calculating geometry'); disp(' ');
        end            
        
        % This function fills in the rest of the fields of the elements
        % of OuputArray (calculating velocity, and jacobians
        % that are needed for working with acceleration)
        % You need to first use UpdateGeometryArray() method
        % if ToSimplify == true, the method will be simplifying
        % everything, which makes the calculations much longer.
        function OuputArray = PrepareKinematicsArray(obj, ToSimplify)
            
            if nargin < 2
                ToSimplify = false; %the default value of ToSimplify
            end
            
            %v = dq/dt - generalized velocities
            %a = dv/dt - generalized acceleration
            %vC = d(rC)/dt - velocity of the CoM
            %aC = d(vC)/dt - acceleration of the CoM
            %J = d(rC)/dq - jacobian matrix
            %vC = J*v
            %J1 = d(vC)/dv = J
            %J2 = d(vC)/dq
            %aC = J1*a + J2*v
            for i = 1:obj.nob
                GeometryStructure = obj.GeometryArray{i};
                
                KinematicsStructure.Link = GeometryStructure.Link;
                
                KinematicsStructure.vC = GeometryStructure.J*obj.v;                                    
                disp('Finished calculating the velocity of the CoM of an ellement');
                KinematicsStructure.J1 = GeometryStructure.J;                                          
                disp('Finished calculating Jacobian J1 for CoM of an ellement');
                KinematicsStructure.J2 = jacobian(KinematicsStructure.vC, obj.q);                      
                disp('Finished calculating Jacobian J2 for CoM of an ellement');
                KinematicsStructure.aC = KinematicsStructure.J1*obj.a + KinematicsStructure.J2*obj.v;  
                disp('Finished calculating the acceleration of the CoM of an ellement');
                
                if ToSimplify
                    KinematicsStructure.vC = obj.Math.simplify(KinematicsStructure.vC, 'vC'); disp('Finished simplifying the velocity of the CoM of an ellement');
                    KinematicsStructure.aC = obj.Math.simplify(KinematicsStructure.aC, 'aC'); disp('Finished simplifying the acceleration of the CoM of an ellement');
                    KinematicsStructure.J2 = obj.Math.simplify(KinematicsStructure.J2, 'J2'); disp('Finished simplifying the Jacobian J2 of the CoM of an ellement');
                end
                
                OuputArray{i} = KinematicsStructure;
                disp(['Finished calculating basic kinematics for ''', KinematicsStructure.Link.Name, ...
                    ''' link, ', int2str(obj.nob - i), ' remaining']);
            end
            disp('* Finished calculating basic kinematics'); disp(' ');  
        end
        

        
        % This method finds an angular velocity of a link with matrix
        % of directional cosines T
        % if ToSimplify == true, the method will be simplifying
        % everything, which makes the calculations much longer.
        function w = GetAngularVelocity(obj, T, ToSimplify)
            
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false; %the default value of ToSimplify
            end
            
            %here we find the first derivative of T
            %we do it column by column
            switch class(obj.q)
                case 'sym'
                    dT = obj.Math.MatrixDerivative(T, obj.q, obj.v);
                    if ToSimplify
                        disp('Started simplifying derivative of the rotation matrix for a link');
                        dT = obj.Math.simplify(dT, 'derivative of a rotation matrix');
                    end
                case 'casadi.SX'
                    dT = jacobian(T, obj.q) * obj.v;
                    dT = reshape(dT, [3, 3]);
                otherwise
                    error('invalid type of q')
            end

            % this is the so-called Poisson formula, that defines the
            % relations between the matrix of directional cosines and
            % the angular velocity in a skew-simmetric form (angular
            % velocity tensor)
            % 
            % There are two eq. of interest for angular velocity
            % tensor; 
            % first: (0)W = dT*T' where T is the matrix of directional
            % cosines for a local frame (basis), dT is its derivative, 
            % (0)W denotes W expressed in the world frame;
            % second: (l)W = T'*dT, (1)W denotes W expressed in the local
            % frame. The following equality also holds:
            % (0)W = T*(l)W*T';
            %
            % For finding kinetic energy we need (l)W, because the tensor
            % of inertia will be expressed in the local frame.
            
            Omega = T'*dT;
            
            %The following gives us angular velocity w, expressed in the 
            %local frame, see description of the angular velocity tensor.
            w = [-Omega(2, 3); Omega(1, 3); -Omega(1, 2)];
            
            if ToSimplify
                w = obj.Math.simplify(w, 'angular velocity');
            end
        end
        
        % This function finds the angular velocities of all links and
        % writes them into OutputArray - a cell array which elemets are
        % structures S:
        % S.w - Angular Velocity of the link
        % S.Jw - jacobian J1 of the link's CoM; Jw = d(w)/dv
        function OutputArray = PrepareAngularVelocityArray(obj, ToSimplify)
            
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false; %the default value of ToSimplify
            end
            
            OutputArray = cell(obj.nob, 1);
            
            for i = 1:obj.nob
                GeometryStructure = obj.GeometryArray{i};
                w = obj.GetAngularVelocity(GeometryStructure.T, ToSimplify);
                Jw = jacobian(w, obj.v);
                
                if ToSimplify
                    Jw = obj.Math.simplify(Jw);
                end
                
                AngularVelocityStructure.Jw = Jw;
                AngularVelocityStructure.w = w;
                AngularVelocityStructure.Link = GeometryStructure.Link;
                
                OutputArray{i} = AngularVelocityStructure;
                
                disp(['Finished calculating Angular Velocity for ''', AngularVelocityStructure.Link.Name, ...
                    ''' link, ', int2str(obj.nob - i), ' remaining']);
            end
            disp('* Finished calculating Angular Velocities'); disp(' ');
        end
        
        %This function provides a joint space inertia matrix (JSIM)
        %(or generalized inertia matrix)
        %which is the matrix A in manipulator equation:
        %H*ddq + C - G - F = Q;
        %where ddq = d^2(q)/dt^2;
        function H = GetGenInertiaMatrix(obj, ToSimplify)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false; %the default value of ToSimplify
            end
            timerVal = tic;
            
            switch class(obj.q)
                case 'sym'
                    H = sym(zeros(obj.dof, obj.dof));
                case 'casadi.SX'
                    H = zeros(obj.dof, obj.dof);
                otherwise
                    error('invalid type of q')
            end
            
            for i = 1:obj.nob
                GeometryStructure = obj.GeometryArray{i};
                AngularVelocityStructure = obj.AngularVelocityArray{i};
                
                H = H + GeometryStructure.J' * GeometryStructure.Link.Mass * GeometryStructure.J + ...
                    AngularVelocityStructure.Jw' * GeometryStructure.Link.Inertia * AngularVelocityStructure.Jw;
                
                disp(['Finished calculating joint space inertia matrix (JSIM) for ''', GeometryStructure.Link.Name, ...
                    ''' link, ', int2str(obj.nob - i), ' remaining']);
            end
            
            if ToSimplify
                disp('Started simplifying joint space inertia matrix (JSIM) of the mechanism');
                H = obj.Math.simplify(H, 'JSIM');
            end
            elapsedTime = toc(timerVal); disp(['Joint space inertia matrix (JSIM) took ', num2str(elapsedTime), ' seconds to compute']);
        end
        
        %This function provides kinetic energy of the system
        function T = GetKineticEnergy(obj, ToSimplify)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false; %the default value of ToSimplify
            end
            
            A = obj.ForwardDynamicsStructure.JSIM;
            
            T = 0.5*obj.v'*A*obj.v;
            
            if ToSimplify
                disp('Started simplifying kinetic energy of the mechanism');
                T = obj.Math.simplify(T, 'kinetic energy');
            end
        end
        
        %This function provides a vector of generalized inertial forces
        %(Coriolis and normal)
        %It is the vector C in manipulator equation:
        %A*ddq + C - G - F = Q;
        %where ddq = d^2(q)/dt^2;
        function [C, dJSIM] = GetGenInertialForces(obj, ToSimplify)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false; %the default value of ToSimplify
            end
            
            switch class(obj.q)
                case 'sym'
                    dJSIM = obj.Math.MatrixDerivative(obj.ForwardDynamicsStructure.JSIM, obj.q, obj.v);
                    if ToSimplify
                        disp('Started simplifying dJSIM');
                        dJSIM = obj.Math.simplify(dJSIM, 'dJSIM');
                    end
                case 'casadi.SX'
                    dJSIM = jacobian(obj.ForwardDynamicsStructure.JSIM, obj.q) * obj.v;
                    dJSIM = reshape(dJSIM, size(obj.ForwardDynamicsStructure.JSIM));
                otherwise
                    error('invalid type of q')
            end
            
            C = 0.5*dJSIM*obj.v;
            
            %Another way to do this:
            %p = A*obj.v;
            %C = 0.5*jacobian(p, obj.q) * obj.v;
            
            if ToSimplify
                disp('Started simplifying generalized inertial forces');
                C = obj.Math.simplify(C, 'generalized inertial forces');
            end
        end
        
        %This function provides a vector of generalized gravitational forces
        %It is the vector G in manipulator equation:
        %A*ddq + C - G - F = Q;
        %where ddq = d^2(q)/dt^2;
        function G = GetGenGravitationalForces(obj, ToSimplify)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false; %the default value of ToSimplify
            end
                
            switch class(obj.q)
                case 'sym'
                    G = sym(zeros(obj.dof, 1));
                case 'casadi.SX'
                    G = zeros(obj.dof, 1);
                otherwise
                    error('invalid type of q')
            end
            
            for i = 1:obj.nob
                GeometryStructure = obj.GeometryArray{i};
                  
                G = G + GeometryStructure.Link.Mass * GeometryStructure.J' * obj.gravitational_constant;
                
                disp(['Finished calculating generalized gravitational forces for ''', GeometryStructure.Link.Name, ...
                    ''' link, ', int2str(obj.nob - i), ' remaining']);
            end
            
            if ToSimplify
                disp('Started simplifying generalized gravitational forces');
                G = obj.Math.simplify(G, 'generalized gravitational forces');
            end
        end
        
        %This function provides a vector of simple generalized disspative forces
        %of the form Q = -v*mu
        %It is the vector F in manipulator equation:
        %A*ddq + C - G - F = Q;
        %where ddq = d^2(q)/dt^2;
        function F = GetSimpleGenDisspativeForces(obj)
            F = -diag(obj.dissipation_coefficients)*obj.v;
        end
        
        % REWRITE THIS COMMENT
        %This function generates an array MotorTorquesStructure
        %the .MotorTorquesArray array is filled using the following logic:
        %every 1 dof joint adds two torques to the array, one to the link
        %that owns the joint and one, of the opposite sign, to its parent
        %higher dof joints do not have torque assosiated with them. If
        %needed this can be changed in a subclass.
        function UpdateMotorActionStructure(obj)
            
            switch class(obj.q)
                case 'sym'
                    U = sym('u', [2*obj.dof, 1]); assume(U, 'real');
                case 'casadi.SX'
                    import casadi.*
                    U = SX.sym('u', [2*obj.dof, 1]);
                otherwise
                    error('invalid type of q')
            end
            
            
            Uindex = 0;
            ForcesIndex = 0;
            ActionArray = [];
            ControlActionsList = [];
            
            for i = 1:max(size(obj.GeometryArray))
                %This switch is needed to set apart the torques, the forces
                %and unactivated joints
                switch obj.GeometryArray{i}.Link.JointType
                    case {'pivotX', 'pivotY', 'pivotZ', 'pivotXY', 'pivotYZ', 'pivotZX', 'abs_spherical', 'abs_pivotX', 'abs_pivotY', 'abs_pivotZ'}
                        Type = 'torque';
                        
                        Force.Name = ['generated torque for the joint of the link ', obj.GeometryArray{i}.Link.Name, ...
                            ' that acts on ', obj.GeometryArray{i}.Link.Name]; 
                        Force.Link = obj.GeometryArray{i}.Link;
                        Force.Type = Type;
                        Force.ApplicationPoint = [];
                        
                        %Each case of this switch does the following:
                        %makes the correct parametrization of the motor 
                        %action (force or torque), and writes a log about
                        %it in ControlActionsList.
                        switch obj.GeometryArray{i}.Link.JointType
                            case 'pivotX'
                                Uindex = Uindex + 1;                               
                                Force.Parameterized = [U(Uindex); 0; 0];
                                Force.ExpressedIn = 'relative coordinates';
                                
                                ControlActionsList{Uindex} = ['Control action u #', num2str(Uindex), ' corresponds to ', ...
                                obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name]; 
                            case 'pivotY'
                                Uindex = Uindex + 1;
                                Force.Parameterized = [0; U(Uindex); 0];
                                Force.ExpressedIn = 'relative coordinates';
                                
                                ControlActionsList{Uindex} = ['Control action u #', num2str(Uindex), ' corresponds to ', ...
                                obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name]; 
                            case 'pivotZ'
                                Uindex = Uindex + 1;
                                Force.Parameterized = [0; 0; U(Uindex)];
                                Force.ExpressedIn = 'relative coordinates';
                                
                                ControlActionsList{Uindex} = ['Control action u #', num2str(Uindex), ' corresponds to ', ...
                                obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name]; 
                            case 'pivotXY'
                                Uindex = Uindex + 1;
                                Force.Parameterized = [U(Uindex); U(Uindex + 1); 0];
                                Force.ExpressedIn = 'relative coordinates';
                                
                                ControlActionsList{Uindex} = ['Control actions ', num2str(Uindex), ' and ', char(U(Uindex + 1)), ...
                                ' correspond to ', obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name, ', X and Y rotations respectively']; 
                                Uindex = Uindex + 1;
                            case 'pivotYZ'
                                Uindex = Uindex + 1;
                                Force.Parameterized = [0; U(Uindex); U(Uindex + 1)];
                                Force.ExpressedIn = 'relative coordinates';
                                
                                ControlActionsList{Uindex} = ['Control actions ', num2str(Uindex), ' and ', char(U(Uindex + 1)), ...
                                ' correspond to ', obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name, ', Y and Z rotations respectively'];
                                Uindex = Uindex + 1;
                            case 'pivotZX'
                                Uindex = Uindex + 1;
                                Force.Parameterized = [U(Uindex); 0; U(Uindex + 1)];
                                Force.ExpressedIn = 'relative coordinates';
                                
                                ControlActionsList{Uindex} = ['Control actions ', num2str(Uindex), ' and ', char(U(Uindex + 1)), ...
                                ' correspond to ', obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name, ', X and Z rotations respectively'];
                                Uindex = Uindex + 1;
                            case 'abs_pivotX'    
                                Uindex = Uindex + 1;
                                Force.Parameterized = [U(Uindex); 0; 0];
                                Force.ExpressedIn = 'absolute coordinates';
                                
                                ControlActionsList{Uindex} = ['Control action u #', num2str(Uindex), ' corresponds to ', ...
                                obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name];
                            
                            case 'abs_pivotY'    
                                Uindex = Uindex + 1;
                                Force.Parameterized = [0; U(Uindex); 0];
                                Force.ExpressedIn = 'absolute coordinates';
                                
                                ControlActionsList{Uindex} = ['Control action u #', num2str(Uindex), ' corresponds to ', ...
                                obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name]; 
                            
                            case 'abs_pivotZ'    
                                Uindex = Uindex + 1;
                                Force.Parameterized = [0; 0; U(Uindex)];
                                Force.ExpressedIn = 'absolute coordinates';
                                
                                ControlActionsList{Uindex} = ['Control action u #', num2str(Uindex), ' corresponds to ', ...
                                obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name]; 
                            
                            case 'abs_spherical'
                                Uindex = Uindex + 1;
                                Force.Parameterized = [U(Uindex); U(Uindex + 1); U(Uindex + 2)];
                                Force.ExpressedIn = 'absolute coordinates';
                                
                                ControlActionsList{Uindex} = ['Control action ', num2str(Uindex), ...
                                ' corresponds to ', obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name, ', X component'];
                                ControlActionsList{Uindex + 1} = ['Control action ', num2str(Uindex + 1), ...
                                ' corresponds to ', obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name, ', Y component'];
                                ControlActionsList{Uindex + 2} = ['Control action ', num2str(Uindex + 2), ...
                                ' corresponds to ', obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name, ', Z component'];
                                Uindex = Uindex + 2;
                        end

                    case {'prismaticX', 'prismaticY', 'prismaticZ'}    
                        Type = 'force';
                        
                        Force.Name = ['generated force for the joint of the link ', obj.GeometryArray{i}.Link.Name, ...
                            ' that acts on ', obj.GeometryArray{i}.Link.Name]; 
                        Force.Link = obj.GeometryArray{i}.Link;
                        Force.Type = Type;
                        Force.ApplicationPoint = Force.Link.ParentLink.RelativeFollower(:, Force.Link.ParentFollowerNumber); 
                        switch obj.GeometryArray{i}.Link.JointType
                            case 'prismaticX'
                                Uindex = Uindex + 1;                               
                                Force.Parameterized = [U(Uindex); 0; 0];
                                Force.ExpressedIn = 'relative coordinates';
                                
                                ControlActionsList{Uindex} = ['Control action u #', num2str(Uindex), ' corresponds to ', ...
                                obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name];      
                            case 'prismaticY'
                                Uindex = Uindex + 1;                               
                                Force.Parameterized = [0; U(Uindex); 0];
                                Force.ExpressedIn = 'relative coordinates';
                                
                                ControlActionsList{Uindex} = ['Control action u #', num2str(Uindex), ' corresponds to ', ...
                                obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name];   
                            case 'prismaticZ'
                                Uindex = Uindex + 1;                               
                                Force.Parameterized = [0; 0; U(Uindex)];
                                Force.ExpressedIn = 'relative coordinates';
                                
                                ControlActionsList{Uindex} = ['Control action u #', num2str(Uindex), ' corresponds to ', ...
                                obj.GeometryArray{i}.Link.JointType, ' that connects ', obj.GeometryArray{i}.Link.Name, ...
                                ' and ', obj.GeometryArray{i}.Link.ParentLink.Name];                      
                        end
                       
                    case {'none', 'fixed', 'FloatingBase_6dof', 'planarX', 'planarY', 'planarZ', 'prismatic_XYZ'}
                        Type = 'none';
                    otherwise
                        warning(['Invalid joint type applied to link  ''', obj.GeometryArray{i}.Link.Name, ...
                    ', type with name ', obj.GeometryArray{i}.Link.JointType, ' does not exist']);
                end

                %applying opposite force/torque to the parent link
                switch obj.GeometryArray{i}.Link.JointType
                    case {'prismaticX', 'prismaticY', 'prismaticZ', 'pivotX', 'pivotY', 'pivotZ', 'pivotXY', 'pivotYZ', ...
                            'pivotZX', 'abs_spherical', 'abs_pivotX', 'abs_pivotY', 'abs_pivotZ'}  
                        if Force.Link.ParentLink.Order ~= 0
                            Force2.Name = ['generated force for the joint of the link ', obj.GeometryArray{i}.Link.Name, ...
                                ' that acts on ', obj.GeometryArray{i}.Link.ParentLink.Name];
                            Force2.Link = obj.GeometryArray{i}.Link.ParentLink;
                            Force2.Type = Type;
                            Force2.ApplicationPoint = Force.ApplicationPoint;
                            Force2.ExpressedIn = Force.ExpressedIn;
                            Force2.Parameterized = -Force.Parameterized;
                        else
                            Force2 = [];
                        end  
                    otherwise
                        Force2 = [];
                end
                
                if ~strcmp(Type, 'none')
                    ForcesIndex = ForcesIndex + 1;
                    ActionArray{ForcesIndex} = Force;
                    if ~isempty(Force2)
                        ForcesIndex = ForcesIndex + 1;
                        ActionArray{ForcesIndex} = Force2;
                    end
                end
            end
            
            %we make sure the obj.u has correct size
            switch class(obj.q)
                case 'sym'
                    obj.u = sym('u', [Uindex, 1]); assume(obj.u, 'real');
                case 'casadi.SX'
                    obj.u = U(1:Uindex, :);
                otherwise
                    error('invalid type of q')
            end
            
            %save Uindex value to a file
            Control_dof = Uindex; save('datafile_settings_Control_dof', 'Control_dof');
            
            obj.MotorActionStructure.ActionArray = ActionArray;
            obj.MotorActionStructure.ControlActionsList = ControlActionsList; 
            disp('* Finished updating motor action structure'); disp(' ');  
        end
        
        %This function prints obj.MotorTorquesStructure.ControlActionsList
        function PrintControlActionCorrespondanceList(obj)
            n = max(size(obj.MotorActionStructure.ControlActionsList));
            for i = 1:n
                disp(obj.MotorActionStructure.ControlActionsList{i});
            end 
        end
        
        
        %Use this function if you have external forces/torques acting on
        %the mechanism.
        %    
        %This function reads from the field .ExternalForcesArray of the
        %class. If the field is empty the function will ignore it, if it is
        %not, the function will calculate maps (the linear map that maps the
        %force/torque to the generalised forces) and jacobians for the
        %external forces.
        %
        %The function also reads from .MotorActionStructure.ActionArray,
        %and if it exists and is not empty, the function will calculate
        %maps and jacobians for motor actions too.
        function UpdateExternalAndMotorForcesArray(obj, ToSimplify)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false; %the default value of ToSimplify
            end
            
            if ~isempty(obj.ExternalForcesArray)
                obj.NoExternalForcesorTorques = false;
                ExternalForces = obj.ExternalForcesArray;
            else
                ExternalForces = [];
            end
            
            if ~isempty(obj.MotorActionStructure.ActionArray)
                MotorActions = obj.MotorActionStructure.ActionArray;
            else
                MotorActions = [];
            end
            
            ForcesArray = [obj.ExternalForcesArray, MotorActions];
            
            N = max(size(ForcesArray));
            for i = 1:N
                
                T = ForcesArray{i}.Link.AbsoluteOrientation;
                
                %Depending on what we have (force or torque) the method we
                %use to calculate the jacobian and linear map are
                %different.
                switch ForcesArray{i}.Type
                    case 'force'
                        AP = ForcesArray{i}.ApplicationPoint;
                        R = T*AP + ForcesArray{i}.Link.AbsoluteBase;
                        %R is the radius vector defining the point of
                        %application of the force, in abs coordinates
                        
                        J = jacobian(R, obj.q);
                        if ToSimplify
                            J = simplify(J);
                        end
                    case 'torque'
                        index = obj.RetrieveLinkIndexInArray(ForcesArray{i}.Link.Name, 'AngularVelocityArray');
                        J = obj.AngularVelocityArray{index}.Jw;
                    otherwise
                        warning(['Invalid external force/torque type named ', ForcesArray{i}.Name,...
                            ', applied to ', ForcesArray{i}.Link.Name, '. Choose ''force'' or ''torque''']);
                        J = [];
                end
                
                if strcmp(ForcesArray{i}.ExpressedIn, 'relative coordinates')
                    Map = J'*T;
                else
                    Map = J';
                end
                                 
                if ToSimplify
                    Map = simplify(Map);
                end
                
                ForcesArray{i}.LinearMap = Map;
                ForcesArray{i}.Jacobian = J;
                
                obj.ExternalAndMotorForcesArray = ForcesArray;
                
                disp(['Finished calculating generalized forces due to external forces/torques for ''', ForcesArray{i}.Name, ...
                    ''', ', int2str(N - i), ' remaining']);
            end
            
            disp('* Finished calculating generalized forces due to external forces/torques'); disp(' ');
        end
        
        %This function calculates the generalized forces due to external
        %forces and torques, using .ExternalForcesArray property. Method
        %UpdateExternalForcesArray() needs to have been called before
        %GetGenExternalForces()
        function Q = GetGenExternalAndMotorForces(obj, ToSimplify)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false; %the default value of ToSimplify
            end
            
            switch class(obj.q)
                case 'sym'
                    Q = sym(zeros(obj.dof, 1));
                case 'casadi.SX'
                    Q = zeros(obj.dof, 1);
                otherwise
                    error('invalid type of q')
            end
            
            N = max(size(obj.ExternalAndMotorForcesArray));
            for i = 1:N
                Q = Q + obj.ExternalAndMotorForcesArray{i}.LinearMap * obj.ExternalAndMotorForcesArray{i}.Parameterized;
            end
            
            if ToSimplify
                disp('Started simplifying generalized external and motor forces');
                Q = obj.Math.simplify(Q, 'generalized external and motor forces');
            end            
        end
           

        %This function finds linear map (linear transformation, a matrix) -
        %B that maps control actions u to generalized forses due to motor 
        %torques/forces Q, which in equation form is:
        %Q = Qm + Qe, Qm = B*u
        %H*ddq + C - G - F = B*u + Qe;
        %B = dQ/du
        function B = GetControlActionsToGenMotorTorquesMap(obj, ToSimplify)
            if (nargin < 2) || (~isa(obj.q, 'sym'))
                ToSimplify = false; %the default value of ToSimplify
            end
            
            Q = obj.ForwardDynamicsStructure.ExternalAndMotorForces;
            B = jacobian(Q, obj.u);
            
            if ToSimplify
                disp('Started simplifying linear map that maps control actions to generalized forses due to torques');
                B = simplify(B);
            end
        end               
        

        %This function retrieves the index of the link named Name in
        %obj.GeometryArray array
        function Index = RetrieveLinkIndexInArray(obj, Name, Type)
            Index = -1;
            
            for i = 1:obj.nob
                switch Type
                    case 'GeometryArray'
                        ith_link_name = obj.GeometryArray{i}.Link.Name;
                    case 'KinematicsArray'
                        ith_link_name = obj.KinematicsArray{i}.Link.Name;
                    case 'AngularVelocityArray'
                        ith_link_name = obj.AngularVelocityArray{i}.Link.Name;
                    otherwise
                        warning(['Wrong array type, array with name ', Type,...
                            ' does not exist. Choose GeometryArray, KinematicsArray or AngularVelocityArray']);
                end
                if strcmp(ith_link_name, Name)
                    Index = i;
                end
            end
        end          
        
        %This function sets assumptions on symbolic variables;
        function SetAssumptions(obj)
            assume(obj.q, 'real');
            assume(obj.v, 'real');
            assume(obj.a, 'real');
            assume(obj.M, 'real');
        end
    end
    methods
        %This is a callback function for a listener for .NumberOfWorkers
        %and .UseParallelizedSimplification properties
        function handleParallelizationEvents(obj, ~, ~)
            obj.Math = MathClass(obj.NumberOfWorkers);
            obj.Math.UseParallel = obj.UseParallelizedSimplification;
        end
    end
end


function dx = Dynamics(x, AA, theta)
A = tdot3(AA, theta);
dx = A*x;
end

function C = tdot3(A, b)
C = zeros(size(A, 1), size(A, 2));
for i = 1:size(A, 3)
    C = C + A(:, :, i)*b(i);
end
end