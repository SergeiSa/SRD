%This class generates equations for control
%last update 09.10.17
classdef SRDControlEquations < SRDMechanicalEquationsManager
    properties
        
        %%%%%%%% Linearization %%%%%%%%
        
        ForwardDynamicsStructureSS;
        % a structure that has same fields are .ForwardDynamicsStructure
        % It contains the same expressions as .ForwardDynamicsStructure,
        % but re-expressed in state-space form (in phase coordinates), 
        % s = [q; v])
        
        LinearizedDynamics;
        % is a structure with the following fields:
        % .SSIM, .RHS_A, .RHS_B, .RHS_c
        % For details see description of UpdateLinearizedDynamics method
        
        LinearizationType = 'Proper';
        % If 'Proper':
        % From eq of the form: H*ddq + dH*dq = Q(q, u) we go to eq.:
        % SSIM*dx = rA*x + rB*u + rc;
        % where x = [q; v] = [q; dq]; SSIM = [I  0]
        %                                    [dH H]
        % rA = dQ/dx;  rB = dQ/du;  rc = f - rA*x - rB*u;
        %
        % If 'Naive':
        % From eq of the form: H*ddq = f(q, dq, u) we go to eq.:
        % SSIM*dx = rA*x + rB*u + rc;
        % where x = [q; v] = [q; dq]; SSIM = [I  0]
        %                                    [0  H]
        % rA = df/dx;  rB = df/du;  rc = f - rA*x - rB*u;
        
        %%%%%%%% Lagrange Multiplier Method %%%%%%%%
                
        LagrangeMultiplierEq;
        %This is a a structure tha contains Lagrange eq. with Multipliers.
        %in the form LHSm*g = RHSv;
        %it has fields: .JointSpace and .StateSpace (unused yet); Each of 
        %those has their own fields:
        %
        %.RHS - contains rthewright hand side of the eq.
        %.LHS_matrix - contains the matrix on the left hand side
        %.Constraint - constraints used in LagrangeMultiplier eq.
        %.ConstraintJacobian - constraint jacobian
        %.RHS_jacobian - right hand side jacobian
        %
        % See description of BuildLagrangeMultiplierEq_JS()
        
        %%%%%%%% Symbolic variables, used for derivations %%%%%%%%
        s; %the vector of state-space coordinates, s = [q; v];
        
    end 
    methods
        
        % class constructor
        % initializes properties s and calls the superclass constructor.
        function obj = SRDControlEquations(LinkArray, Casadi)
            obj = obj@SRDMechanicalEquationsManager(LinkArray, Casadi);
            
            obj.s = sym('s', [2*obj.dof, 1]); assume(obj.s, 'real');
        end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% Linearization code - begin %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
        
        %This function generates a .ForwardDynamicsStructureSS structure
        %from .ForwardDynamicsStructure by substituting .s for .q and .v
        function UpdateStateSpaceForwardDynamics(obj, OnlyForLinearization)
            timerVal = tic;
            disp('Started convertion to state space');
            
            if nargin < 2
                OnlyForLinearization = false; %default value for OnlyForLinearization
            end
            
            %This part is to check what fileds in ForwardDynamicsStructure
            %exist, and store all the fileds into variables
            if isfield(obj.ForwardDynamicsStructure, 'JSIM')
                JSIM = obj.ForwardDynamicsStructure.JSIM;
            else
                JSIM = []; 
                disp('JSIM was not computed prior to convertion to state space');
            end
      
            switch obj.LinearizationType
                case 'Naive'
                    if isfield(obj.ForwardDynamicsStructure, 'RightHandSide_ofManipulatorEq')
                        RightHandSide_ofManipulatorEq = obj.ForwardDynamicsStructure.RightHandSide_ofManipulatorEq;
                    else
                        RightHandSide_ofManipulatorEq = [];
                        disp('Right Hand Side of Manipulator Equasions was not computed prior to convertion to state space');
                    end
                case 'Proper'
                    if isfield(obj.ForwardDynamicsStructure, 'dJSIM')
                        dJSIM = obj.ForwardDynamicsStructure.dJSIM;
                    else
                        dJSIM = [];
                        disp('JSIM was not computed prior to convertion to state space');
                    end
                    
                    if isfield(obj.ForwardDynamicsStructure, 'GenGravitationalForces')
                        GenGravitationalForces = obj.ForwardDynamicsStructure.GenGravitationalForces;
                    else
                        GenGravitationalForces = [];
                        disp('Generalized Gravitational Forces were not computed prior to convertion to state space');
                    end
                    
                    if isfield(obj.ForwardDynamicsStructure, 'GenDisspativeForces')
                        GenDisspativeForces = obj.ForwardDynamicsStructure.GenDisspativeForces;
                    else
                        GenDisspativeForces = [];
                        disp('Generalized Disspative Forces were not computed prior to convertion to state space');
                    end
                    
                    if isfield(obj.ForwardDynamicsStructure, 'ExternalAndMotorForces')
                        ExternalAndMotorForces = obj.ForwardDynamicsStructure.ExternalAndMotorForces;
                    else
                        ExternalAndMotorForces = [];
                        disp('External And Motor Forces were not computed prior to convertion to state space');
                    end
                    obj.ForwardDynamicsStructureSS.ExternalAndMotorForces = ExternalAndMotorForces;
                otherwise
                    warning('Incorrect Version of linearizer algorithm requested; use ''Naive'' or ''Proper''');
            end
            
            if ~OnlyForLinearization
                
                if isfield(obj.ForwardDynamicsStructure, 'KineticEnergy')
                    KineticEnergy = obj.ForwardDynamicsStructure.KineticEnergy;
                else
                    KineticEnergy = [];
                    disp('Kinetic Energy was not computed prior to convertion to state space');
                end
                
                if isfield(obj.ForwardDynamicsStructure, 'GenInertialForces')
                    GenInertialForces = obj.ForwardDynamicsStructure.GenInertialForces;
                else
                    GenInertialForces = [];
                    disp('Generalized Inertial Forces were not computed prior to convertion to state space');
                end
                
                if isfield(obj.ForwardDynamicsStructure, 'GenMotorTorques')
                    GenMotorTorques = obj.ForwardDynamicsStructure.GenMotorTorques;
                else
                    GenMotorTorques = [];
                    disp('Generalized Motor Torques were not computed prior to convertion to state space');
                end
                
                if isfield(obj.ForwardDynamicsStructure, 'ControlActionsToGenMotorTorquesMap')
                    ControlActionsToGenMotorTorquesMap = obj.ForwardDynamicsStructure.ControlActionsToGenMotorTorquesMap;
                else
                    ControlActionsToGenMotorTorquesMap = [];
                    disp('Control Actions To Generalized Motor Torques Map was not computed prior to convertion to state space');
                end
                
                if ~obj.NoExternalForcesorTorques
                    if isfield(obj.ForwardDynamicsStructure, 'GenExternalForces')
                        GenExternalForces = obj.ForwardDynamicsStructure.GenExternalForces;
                    else
                        GenExternalForces = []; disp('Generalized External Forces were not computed prior to convertion to state space');
                    end
                    if isfield(obj.ForwardDynamicsStructure, 'ExternalForcesParametersToGenExternalForcesMap')
                        ExternalForcesParametersToGenExternalForcesMap = obj.ForwardDynamicsStructure.ExternalForcesParametersToGenExternalForcesMap;
                    else
                        ExternalForcesParametersToGenExternalForcesMap = [];
                        disp('External Forces Parameters To Generalized External Forces Map were not computed prior to convertion to state space');
                    end
                end
            end
            
            %This part is substitution          
            for i = 1:obj.dof
                JSIM = subs(JSIM, obj.q(i), obj.s(i));
                dJSIM = subs(dJSIM, obj.q(i), obj.s(i));
       
                
                switch obj.LinearizationType
                    case 'Naive'
                        RightHandSide_ofManipulatorEq = subs(RightHandSide_ofManipulatorEq, obj.q(i), obj.s(i));
                        RightHandSide_ofManipulatorEq = subs(RightHandSide_ofManipulatorEq, obj.v(i), obj.s(i + obj.dof));
                    case 'Proper'
                        dJSIM = subs(dJSIM, obj.q(i), obj.s(i)); %seems redundant
                        dJSIM = subs(dJSIM, obj.v(i), obj.s(i + obj.dof));
                        
                        GenGravitationalForces = subs(GenGravitationalForces, obj.q(i), obj.s(i));
                        
                        GenDisspativeForces = subs(GenDisspativeForces, obj.q(i), obj.s(i));
                        GenDisspativeForces = subs(GenDisspativeForces, obj.v(i), obj.s(i + obj.dof));
                        
                        ExternalAndMotorForces = subs(ExternalAndMotorForces, obj.q(i), obj.s(i));
                        ExternalAndMotorForces = subs(ExternalAndMotorForces, obj.v(i), obj.s(i + obj.dof));
                    otherwise
                        warning('Incorrect Version of linearizer algorithm requested; use ''Naive'' or ''Proper''');
                end

                if ~OnlyForLinearization
                    
                    KineticEnergy = subs(KineticEnergy, obj.q(i), obj.s(i));
                    KineticEnergy = subs(KineticEnergy, obj.v(i), obj.s(i + obj.dof));
                    
                    GenInertialForces = subs(GenInertialForces, obj.q(i), obj.s(i));
                    GenInertialForces = subs(GenInertialForces, obj.v(i), obj.s(i + obj.dof));
                    
                    GenMotorTorques = subs(GenMotorTorques, obj.q(i), obj.s(i));
                    
                    ControlActionsToGenMotorTorquesMap = subs(ControlActionsToGenMotorTorquesMap, obj.q(i), obj.s(i));
                    
                    if ~obj.NoExternalForcesorTorques
                        GenExternalForces = subs(GenExternalForces, obj.q(i), obj.s(i));
                        
                        ExternalForcesParametersToGenExternalForcesMap = subs(ExternalForcesParametersToGenExternalForcesMap, obj.q(i), obj.s(i));
                    end
                end
            end
            
            obj.ForwardDynamicsStructureSS.JSIM = JSIM;
            switch obj.LinearizationType
                case 'Naive'
                    obj.ForwardDynamicsStructureSS.RightHandSide_ofManipulatorEq = RightHandSide_ofManipulatorEq;
                case 'Proper'
                    obj.ForwardDynamicsStructureSS.dJSIM = dJSIM;
                    obj.ForwardDynamicsStructureSS.GenGravitationalForces = GenGravitationalForces;
                    obj.ForwardDynamicsStructureSS.GenDisspativeForces = GenDisspativeForces;
                    obj.ForwardDynamicsStructureSS.ExternalAndMotorForces = ExternalAndMotorForces;
                otherwise
                    warning('Incorrect Version of linearizer algorithm requested; use ''Naive'' or ''Proper''');
            end
            
            if ~OnlyForLinearization
                obj.ForwardDynamicsStructureSS.KineticEnergy = KineticEnergy;
                obj.ForwardDynamicsStructureSS.GenInertialForces = GenInertialForces;
                obj.ForwardDynamicsStructureSS.GenMotorTorques = GenMotorTorques;
                obj.ForwardDynamicsStructureSS.ControlActionsToGenMotorTorquesMap = ControlActionsToGenMotorTorquesMap;
                if ~obj.NoExternalForcesorTorques
                    obj.ForwardDynamicsStructureSS.GenExternalForces = GenExternalForces;
                    obj.ForwardDynamicsStructureSS.ExternalForcesParametersToGenExternalForcesMap = ExternalForcesParametersToGenExternalForcesMap;
                end
            end
            
            disp('* Finished convertion to state space'); disp(' ');
            toc(timerVal);
        end
        
        % For dynamics in the form:
        % SSIM*dx/dt = f(x, u)         (1)
        % this function produces a state-space linearization as follows:
        %
        % SSIM*dx/dt = rA*x + rB*u + rc, where
        % SSIM = [I  0
        %         0  JSIM], 
        % JSIM - joint-space inertia matrix from manipulator eq.
        % I - identity matrix of proper dimentions;
        %
        % rA = df/dx;  rB = df/du;  rc = f - rA*x - rB*u;
        %
        % In practice we don't have eq. (1), but we use manipulator eq.
        % to produce them.
        %
        % Eq. 1 is derived from manipulator eq. re-expressed in state-space
        % coordinates, produced by UpdateStateSpaceForwardDynamics method
        function UpdateLinearizedDynamics(obj, ToSimplify)
            timerVal = tic;
            disp('Started linearization: H*dx/dt = rA*x + rB*u + rc');
            if nargin < 2
                ToSimplify = false; %default value for ToSimplify
            end
            
            % generalized velocities expressed via state space coordinates
            SSv = obj.s((obj.dof +1):(2*obj.dof));            
            
            %State-space inertial matrix
            switch obj.LinearizationType
                case 'Naive'
                    SSIM = [eye(obj.dof), zeros(obj.dof, obj.dof);
                            zeros(obj.dof, obj.dof), obj.ForwardDynamicsStructureSS.JSIM];
                    
                    %State-space right hand side
                    SSRHS = [SSv; obj.ForwardDynamicsStructureSS.RightHandSide_ofManipulatorEq];
                case 'Proper'
                    SSIM = [eye(obj.dof),                         zeros(obj.dof, obj.dof);
                            obj.ForwardDynamicsStructureSS.dJSIM, obj.ForwardDynamicsStructureSS.JSIM];
                        
                    %State-space right hand side
                    ManipulatorEqQ = obj.ForwardDynamicsStructureSS.ExternalAndMotorForces + ...
                        obj.ForwardDynamicsStructureSS.GenGravitationalForces + ...
                        obj.ForwardDynamicsStructureSS.GenDisspativeForces;
                    SSRHS = [SSv; ManipulatorEqQ];
                      
                otherwise
                    warning('Incorrect Version of linearizer algorithm requested; use ''Naive'' or ''Proper''');
            end
                
            if ToSimplify
                disp('Started simplifying state-space inertia matrix');
                SSIM = obj.Math.simplify(SSIM, 'SSIM');
            end
            
            disp('Started calculating right-hand-side rA linearization matrix');
            RHS_A = jacobian(SSRHS, obj.s);
            if ToSimplify
                disp('Started simplifying right-hand-side rA linearization matrix');
                RHS_A = obj.Math.simplify(RHS_A, 'right-hand-side rA linearization matrix');
            end
            
            disp('Started calculating right-hand-side rB linearization matrix');
            RHS_B = jacobian(SSRHS, obj.u);
            if ToSimplify
                disp('Started simplifying right-hand-side rB linearization matrix');
                RHS_B = obj.Math.simplify(RHS_B, 'right-hand-side rB linearization matrix');
            end
            
            disp('Started calculating right-hand-side rc linearization vector');
            RHS_c = SSRHS - RHS_A*obj.s - RHS_B*obj.u;
            if ToSimplify
                disp('Started simplifying right-hand-side rc linearization matrix');
                RHS_c = obj.Math.simplify(RHS_c, 'right-hand-side rc linearization matrix');
            end
            
            obj.LinearizedDynamics.SSIM = SSIM;
            obj.LinearizedDynamics.RHS_A = RHS_A;
            obj.LinearizedDynamics.RHS_B = RHS_B;
            obj.LinearizedDynamics.RHS_c = RHS_c;
            
            disp('* Finished linearization: H*dx/dt = rA*x + rB*u + rc'); disp(' ');
            toc(timerVal);
        end
        
        %This function generates matlab functions for linearization,
        %before using it, call UpdateLinearizedDynamics method
        function GenerateLinearizationFunctions(obj)
            timerVal = tic;
            disp('Started generating linearization functions');
            
            matlabFunction(obj.LinearizedDynamics.SSIM, 'File', ...
                'g_dynamics_Linearization_SSIM', 'Vars', {obj.s}, 'Optimize', obj.ToOptimizeFunctions);
            
            matlabFunction(obj.LinearizedDynamics.RHS_A, 'File', ...
                'g_dynamics_Linearization_RHS_A', 'Vars', {obj.s, obj.u}, 'Optimize', obj.ToOptimizeFunctions);
            
            matlabFunction(obj.LinearizedDynamics.RHS_B, 'File', ...
                'g_dynamics_Linearization_RHS_B', 'Vars', {obj.s}, 'Optimize', obj.ToOptimizeFunctions);
            
            matlabFunction(obj.LinearizedDynamics.RHS_c, 'File', ...
                'g_dynamics_Linearization_RHS_c', 'Vars', {obj.s, obj.u}, 'Optimize', obj.ToOptimizeFunctions); 
                %!!! Normally there should only be RHSinputs2 inputs (no u)
            
            disp('* Finished generating linearization functions'); disp(' ');
            toc(timerVal);
        end
        
        %This function calls three methods that prepare all that is needed
        %for the GetLinearization method to work
        function DoLinearization(obj, ToSimplify)
            obj.UpdateStateSpaceForwardDynamics(true);
            obj.UpdateLinearizedDynamics(ToSimplify);
            obj.GenerateLinearizationFunctions();
        end

    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% Linearization code - end %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Lagrange multipliers method code - begin %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
   

        %This function builds equations of motion for the case when
        %additional constrains are applied on top of generalized
        %coordinates. The equations are built using Lagrange multiplier 
        %method.
        %Produces DAE with second order derivatives.
        %
        % The initial eq. has the following form:
        %
        % JSIM*ddq = b, where ddq = d^2(q)/dt^2
        %
        % Constraints are defined as the following function: f(q) = 0;
        % We define the following Jacobian matrix:
        % F = df/dq
        % Differentiating the constraint eq. twice we get:
        % F*ddq + phi = 0, where phi = d(F*v)/dq * v,  v = dq/dt
        %
        % Then Lagrange eq. with multipliers for the mechanical system will
        % take the following form:
        %
        % JSIM*ddq = b + F'*l,
        % F*ddq + phi = 0;
        % where F' means F transposed, l - unknown multipliers (reaction
        % forces).
        %
        % We introduce a vector g = [ddq; l], a vector RHSv = [b; -phi],
        % a matrix LHSm = [JSIM, -F';
        %                  F,     0];
        % 
        % LHSm stands for left hand side matrix;
        % RHSv stands for right hand side vector;
        % Then the Lagrange eq. with multipliers can be written as:
        %
        % LHSm*g = RHSv;
        %
        % This functions calculates matrix LHSm and vector RHSv;
        
        function BuildLagrangeMultiplierEq_JS(obj, Constraint, ToSimplify)
            timerVal = tic;
            disp('Started generating Lagrange Multiplier Equations - joint space form');
            
            F = jacobian(Constraint, obj.q);
            F = simplify(F);
            
            dfdt = F*obj.v;
            if ToSimplify
                dfdt = simplify(dfdt);
            end
            
            SecondJacobian = jacobian(dfdt, obj.q);
            SecondJacobian = simplify(SecondJacobian);
            
            phi = SecondJacobian*obj.v;
            phi = simplify(phi);
            
            LHSm = [obj.ForwardDynamicsStructure.JSIM, (-F');
                    F,                                 zeros(size(F, 1), size(F, 1))];
             
            if ToSimplify
                disp('Started simplifying left hand side of the Lagrange Multiplier Equations');
                LHSm = obj.Math.simplify(LHSm, 'LHSm');
            end
            
            RHSv = [obj.ForwardDynamicsStructure.RightHandSide_ofManipulatorEq; -phi];
            
            obj.LagrangeMultiplierEq.JointSpace.LHS_matrix = LHSm;
            obj.LagrangeMultiplierEq.JointSpace.RHS = RHSv; 
            
            obj.LagrangeMultiplierEq.JointSpace.SecondJacobian = SecondJacobian;
            
            obj.LagrangeMultiplierEq.JointSpace.Constraint = Constraint;
            obj.LagrangeMultiplierEq.JointSpace.ConstraintJacobian = F;
            
            disp('Started generating functions for right hand side vector of the Lagrange Multiplier Equations');
            matlabFunction(RHSv, 'File', 'g_dynamics_LagrangeMultiplier_RHSv', 'Vars', {obj.q, obj.v, obj.u}, 'Optimize', obj.ToOptimizeFunctions);
            
            disp('Started generating functions for left hand side matrix of the Lagrange Multiplier Equations');
            matlabFunction(LHSm, 'File', 'g_dynamics_LagrangeMultiplier_LHSm', 'Vars', {obj.q}, 'Optimize', obj.ToOptimizeFunctions);
            
            matlabFunction(Constraint, 'File', 'g_dynamics_LagrangeMultiplier_Constraint', 'Vars', {obj.q}, 'Optimize', obj.ToOptimizeFunctions);
            matlabFunction(F, 'File', 'g_dynamics_LagrangeMultiplier_ConstraintJacobian', 'Vars', {obj.q}, 'Optimize', obj.ToOptimizeFunctions);
            matlabFunction(SecondJacobian, 'File', 'g_dynamics_LagrangeMultiplier_ConstraintSecondJacobian', 'Vars', {obj.q, obj.v}, 'Optimize', obj.ToOptimizeFunctions);
            
            disp('* Finished generating functions for the Lagrange Multiplier Equations in joint space form'); disp(' ');
            toc(timerVal);
        end


    end
end