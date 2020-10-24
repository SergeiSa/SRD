%This class generates equations for kinematics and dynamics
%last update 1.10.16
classdef SRDMechanicalEquations_Numeric < SRDChain
    properties
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Structures and arrays storing symbolic expressions
        
        GeometryArray;  
        
        ForwardDynamicsStructure;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Other
        
        delta_JacobianArray_handle;
        
        parameters_for_forward_dynamics;
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

        u; %the vector of control actions (scalar values of torques that
        %are being produced by the motors)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% External classes
        
        Math
        %object of MathClass class; drovides useful functions;
        
        %%%%%
        %%%%%% info
        
        Control_dof = [];
        
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
        function obj = SRDMechanicalEquations_Numeric(LinkArray)
            obj = obj@SRDChain(LinkArray);
            
            obj.q = sym('q', [obj.dof, 1]); 
            obj.v = sym('v', [obj.dof, 1]); 
            obj.a = sym('a', [obj.dof, 1]); 
%             obj.M = sym('M', [3, obj.nob]); 
            obj.SetAssumptions();
            
            obj.dissipation_coefficients = ones(obj.dof, 1);
            obj.Math = MathClass(obj.NumberOfWorkers);
            
            addlistener(obj, 'UseParallelizedSimplification', 'PostSet', @obj.handleParallelizationEvents);
            addlistener(obj, 'NumberOfWorkers',               'PostSet', @obj.handleParallelizationEvents);
        end
        
        
        % This function find a cell array of structures, see
        % description of GeometryArray property; Some of the field of
        % the elements of GeometryArray won't be filled up;
        function PrepareGeometryArray(obj, ~)
            % Here we deal with the problem that RelativeFollower field
            % is initialised as a numeric array, and then the code will
            % attempt to modify it column by column; so to avoid Matlab
            % trying to convert symbolic to numerical we need to make
            % sure all matrices are converted to symbolic beforehand
            for i = 1:size(obj.LinkArray, 1)
                obj.LinkArray(i).RelativeFollower = sym(obj.LinkArray(i).RelativeFollower);
            end
            
            %update link chain for given position, in this case - for the 
            %given symbolic variable
            obj.Update(obj.q);
            obj.Math.UseParallel = obj.UseParallelizedSimplification;
            
            index = 0;
            for i = 1:size(obj.LinkArray, 1)
                if obj.LinkArray(i).Order > 0
                    index = index + 1;
                    %here we fill an element of GeometryArray
                    
                    %%%%%%%%%%%%
                    %Forward kinematics
                    
                    %delta_ substript means that it is a relative jacobian; 
                    %to get absolute jacobian J_{i} you use something like: 
                    %J_{i} = J_{i-1} + J_delta
                    
                    %%%%%%%%%%%%
                    %Linear velocity jacobians
                    calculated.delta_rC = obj.Math.simplify(obj.LinkArray(i).AbsoluteCoM - obj.LinkArray(i).ParentLink.AbsoluteCoM, 'delta_rC');
                    calculated.delta_Jacobian = obj.Math.simplify(jacobian(calculated.delta_rC, obj.q), 'delta_J');
                    %%%%%%%%%%%%
                    %%%%%%%%%%%%
                    %Linear velocity jacobians - derivatives
                    calculated.delta_Jacobian___derivative = obj.Math.MatrixDerivative(calculated.delta_Jacobian, obj.q, obj.v);
                    %%%%%%%%%%%%
                    
                    %%%%%%%%%%%%
                    %Angular velocity jacobians
                    calculated.dT = obj.Math.MatrixDerivative(obj.LinkArray(i).AbsoluteOrientation, obj.q, obj.v);
                    calculated.dT = obj.Math.simplify(calculated.dT, 'derivative of a rotation matrix');
                    
                    Omega = obj.Math.simplify(obj.LinkArray(i).AbsoluteOrientation' * calculated.dT, 'Omega');
                    calculated.AngularVelocity = [-Omega(2, 3); Omega(1, 3); -Omega(1, 2)];
                    
                    calculated.delta_AngularVelocity = obj.Math.simplify(calculated.AngularVelocity - obj.LinkArray(i).ParentLink.calculated.AngularVelocity, 'delta_AngularVelocity');
                    
                    calculated.delta_AngularVelocityJacobian = jacobian(calculated.delta_AngularVelocity, obj.v);
                    calculated.delta_AngularVelocityJacobian = obj.Math.simplify(calculated.delta_AngularVelocityJacobian, 'delta_AngularVelocityJacobian');
                    %%%%%%%%%%%%
                    %%%%%%%%%%%%
                    %Angular velocity jacobians - derivatives
                    calculated.delta_AngularVelocityJacobian___derivative = obj.Math.MatrixDerivative(calculated.delta_AngularVelocityJacobian, obj.q, obj.v);
                    %%%%%%%%%%%%
                    
                    
                    
                    %store things in memory
                    obj.LinkArray(i).calculated = calculated;
                    
                    GeometryStructure.Link = obj.LinkArray(i);
                    
                    obj.GeometryArray{index} = GeometryStructure;
                    
                    disp(['Finished calculating geometry for ''', GeometryStructure.Link.Name, ...
                        ''' link, ', int2str(obj.nob - index), ' remaining']);
                    
                else
                    calculated.Jacobian_value = zeros(3, obj.dof);
                    calculated.AngularVelocity = zeros(3, 1);
                    calculated.AngularVelocityJacobian_value = zeros(3, obj.dof);
                    
                    obj.LinkArray(i).calculated = calculated;
                end
            end % obj.nob should be equal to size(GeometryArray, 1)
            
            %%%%%%%%%%%%
            %symbolic functions generation
            
            %Here we assemble all jacobians into one bulk - CombinedJacobianArray; 
            %we do it so Matlab can optimize trigonometric function calculations
            delta_JacobianArray = [];
            delta_AngularVelocityJacobianArray = [];
            delta_Jacobian___derivative___Array = [];
            delta_AngularVelocityJacobian___derivative___Array = [];
            for i = 1:obj.nob
                delta_JacobianArray = [delta_JacobianArray; 
                                       obj.GeometryArray{i}.Link.calculated.delta_Jacobian];
                                   
                delta_AngularVelocityJacobianArray = [delta_AngularVelocityJacobianArray; 
                                                      obj.GeometryArray{i}.Link.calculated.delta_AngularVelocityJacobian];
                
                delta_Jacobian___derivative___Array = [delta_Jacobian___derivative___Array; 
                                                    obj.GeometryArray{i}.Link.calculated.delta_Jacobian___derivative];
                                                
                delta_AngularVelocityJacobian___derivative___Array = [delta_AngularVelocityJacobian___derivative___Array; 
                                                                   obj.GeometryArray{i}.Link.calculated.delta_AngularVelocityJacobian___derivative];
            end
            CombinedJacobianArray = [delta_JacobianArray; 
                                     delta_AngularVelocityJacobianArray; 
                                     delta_Jacobian___derivative___Array; 
                                     delta_AngularVelocityJacobian___derivative___Array];
            
            disp('Generating a function for CombinedJacobianArray...');
            obj.delta_JacobianArray_handle = matlabFunction(CombinedJacobianArray, 'File', ...
                        'g_relative_JacobianArray', 'Vars', {obj.q, obj.v}, 'Optimize', true);            
            
                    
            %parameter arrays
            parameters.ChildParentMatrix = zeros(1, obj.nob);
            parameters.Mass = zeros(1, obj.nob);
            parameters.Inertia = zeros(3, 3, obj.nob);
            parameters.Jacobian_value = zeros(3, obj.dof, obj.nob+1); %+1 is because the 1st block is the ground
            parameters.AngularVelocityJacobian_value = zeros(3, obj.dof, obj.nob+1);
            
            for i = 1:obj.nob
                parameters.Mass(i) = obj.GeometryArray{i}.Link.Mass;
                parameters.Inertia(:, :, i) = obj.GeometryArray{i}.Link.Inertia;
                
                ParentName = obj.GeometryArray{i}.Link.ParentLink.Name;
                for j = 1:obj.nob
                    if strcmp(obj.GeometryArray{j}.Link.Name, ParentName)
                        parameters.ChildParentMatrix(i) = j;
                    end
                end
            end       
            
            obj.parameters_for_forward_dynamics = parameters;
            
            disp('* Finished calculating geometry'); disp(' ');
        end            
        
        %This function provides a joint space inertia matrix (JSIM)
        %(or generalized inertia matrix)
        %which is the matrix A in manipulator equation:
        %H*ddq + C - G - F = Q;
        %where ddq = d^2(q)/dt^2;
        function Output = GetDynamics(obj, q, v)
            H  = zeros(obj.dof, obj.dof);
            dH = zeros(obj.dof, obj.dof);
            G  = zeros(obj.dof, 1);
            
            delta_JacobianArray_value     = obj.delta_JacobianArray_handle(q, v);
            
            ChildParentMatrix             = obj.parameters_for_forward_dynamics.ChildParentMatrix;
            J_array                       = obj.parameters_for_forward_dynamics.Jacobian_value;
            Jw_array                      = obj.parameters_for_forward_dynamics.AngularVelocityJacobian_value;
            dJ_array                      = obj.parameters_for_forward_dynamics.Jacobian_value;
            dJw_array                     = obj.parameters_for_forward_dynamics.AngularVelocityJacobian_value;
            Mass                          = obj.parameters_for_forward_dynamics.Mass;
            Inertia                       = obj.parameters_for_forward_dynamics.Inertia;
            
            g = obj.gravitational_constant;
            
            for i = 1:obj.nob
                parent_index = ChildParentMatrix(i);
                
                index = 3*(i - 1) + 1;
                delta_Jacobian_value = delta_JacobianArray_value(index:(index+2), :);
                J_array(:, :, i+1) = J_array(:, :, parent_index+1) + delta_Jacobian_value;
                
                index = index + 3*obj.nob;
                delta_AngularVelocityJacobian_value = delta_JacobianArray_value(index:(index+2), :);
                Jw_array(:, :, i+1) = Jw_array(:, :, parent_index+1) + delta_AngularVelocityJacobian_value;
                
                index = index + 3*obj.nob;
                delta_Jacobian___derivative___value = delta_JacobianArray_value(index:(index+2), :);
                dJ_array(:, :, i+1) = dJ_array(:, :, parent_index+1) + delta_Jacobian___derivative___value;
                
                index = index + 3*obj.nob;
                delta_AngularVelocityJacobian___derivative___value = delta_JacobianArray_value(index:(index+2), :);
                dJw_array(:, :, i+1) = dJw_array(:, :, parent_index+1) + delta_AngularVelocityJacobian___derivative___value;
                
                J = J_array(:, :, i+1);
                Jw = Jw_array(:, :, i+1);
                dJ = dJ_array(:, :, i+1);
                dJw = dJw_array(:, :, i+1);
                
                I = Inertia(:, :, i);
                m = Mass(i);
                
                H = H + J'  * m * J + ...
                        Jw' * I * Jw;
                    
                dH = dH + dJ'  * m * J  + ...
                          J'   * m * dJ + ...
                          dJw' * I * Jw + ...
                          Jw'  * I * dJw;
                      
                G = G + m * J' * g;
                
                Dissipation = -diag(obj.dissipation_coefficients) * v;
            end
            
            Output.H = H;
            Output.dH = dH;
            Output.G = G;
            Output.Dissipation = Dissipation;
        end       

        
        function GetCheatControlMap(obj)
            obj.Control_dof = 0; index = 0;
            B = [];
           
            for i = 1:max(size(obj.GeometryArray))
                
                %check if th ejoint is actuated
                switch obj.GeometryArray{i}.Link.JointType
                    case {'pivotX', 'pivotY', 'pivotZ', ...
                            'pivotXY', 'pivotYZ', 'pivotZX', ...
                            'abs_spherical', 'abs_pivotX', 'abs_pivotY', 'abs_pivotZ', ...
                            'prismaticX', 'prismaticY', 'prismaticZ'}
                        Actuated = true;
                    otherwise
                        Actuated = false;
                end
                
                %add ones to teh B matrix if it is actuated
                for j = 1:length(obj.GeometryArray{i}.Link.UsedGenCoordinates)
                    
                    index = index + 1;
                    
                    if Actuated
                        obj.Control_dof = obj.Control_dof + 1;
                        B = [B, zeros(obj.dof, 1)];
                       %B(index, obj.Control_dof) = 1;
                        B(obj.GeometryArray{i}.Link.UsedGenCoordinates(j), obj.Control_dof) = 1;
                    end
                end
                
            end
            
            B = sym(B);
            
            matlabFunction(B, 'File', ...
                'g_dynamics_ControlMap', 'Vars', {obj.q}, 'Optimize', true);
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
%             assume(obj.M, 'real');
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