%This class provides robot model
classdef SRDModelHandler < handle
    properties
        %%%%%%%%%%%
        %switches
        
        model_type_code = 1;
        %options: 0 - 'not configured', 
        %         1 - 'exact'
        %         2 - 'OrtegaSpong'
        %         3 - 'numeric'
        
        %%%%%%%%%%%
        %function handles
        
        
        MechanicalEquations_Numeric = [];
        
        %%%%%%%%%%%
        %state
        
        theta = []; %Ortega-Spong model parameters - actual value
        estimated_theta = []; %Ortega-Spong model parameters - estimate
        
        numeric_functions_updated_time = 0;
        numeric_functions_value = [];
        
        g_dynamics_JSIM = [];
        g_dynamics_RHS = [];
        g_dynamics_ControlMap = [];
        g_dynamics_LagrangeMultiplier_ConstraintJacobian = [];
        ForcesForComputedTorqueController = [];
        
        Casadi = false;
    end
    methods
        
        function obj = SRDModelHandler(Casadi)
            if nargin < 1
                Casadi = false;
            end
            obj.Casadi = Casadi;
            
            if Casadi
                import casadi.*
                
                g_dynamics_JSIM_C = external('g_dynamics_JSIM', './g_dynamics.so');
                g_dynamics_RHS_C = external('g_dynamics_RHS', './g_dynamics.so');
                g_dynamics_ControlMap_C = external('g_dynamics_ControlMap', './g_dynamics.so');
                %this is a function handle used in computed torque
                %controller:
                ForcesForComputedTorqueController_C = external('g_control_ForcesForComputedTorqueController', './g_dynamics.so');
                warning('obj.g_dynamics_LagrangeMultiplier_ConstraintJacobian not initialized')
                %obj.g_dynamics_LagrangeMultiplier_ConstraintJacobian = external('g_dynamics_LagrangeMultiplier_ConstraintJacobian', './g_dynamics.so');
                
                obj.g_dynamics_JSIM = @(q) full(evalf(g_dynamics_JSIM_C(q)));
                obj.g_dynamics_RHS = @(q, v, u) full(evalf(g_dynamics_RHS_C(q, v, u)));
                obj.g_dynamics_ControlMap = @(q) full(evalf(g_dynamics_ControlMap_C(q)));
                obj.ForcesForComputedTorqueController = @(q, v) full(evalf(ForcesForComputedTorqueController_C(q, v)));
                
            else
                obj.g_dynamics_JSIM = @g_dynamics_JSIM;
                obj.g_dynamics_RHS = @g_dynamics_RHS;
                obj.g_dynamics_ControlMap = @g_dynamics_ControlMap;
                obj.g_dynamics_LagrangeMultiplier_ConstraintJacobian = @g_dynamics_LagrangeMultiplier_ConstraintJacobian;
                obj.ForcesForComputedTorqueController = @g_control_ForcesForComputedTorqueController;
                %this is a function handle used in computed torque controller
            end
        end
        
        function Setup(obj, MechanicalEquations_Numeric)
            %check if g_dynamics_JSIM and other dynamics functions require
            %theta as input
            
            if nargin < 2
                MechanicalEquations_Numeric = [];
            end
            obj.MechanicalEquations_Numeric = MechanicalEquations_Numeric;
            
            if isempty(MechanicalEquations_Numeric)
                if nargin(@g_dynamics_JSIM) == 1
                    obj.model_type_code = 1;
                else
                    obj.model_type_code = 2;
                end
            else
                obj.model_type_code = 3;
            end
            
            if exist('g_control_ForcesForComputedTorqueController', 'file') == 2
                obj.ForcesForComputedTorqueController = @g_control_ForcesForComputedTorqueController;
            end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%
        % dynamics functions
        %%%%%%%%%%%%%%%%%%%%%%
        
        function Dynamics = get_Dynamics(obj, q, v)
            if ~isempty(obj.MechanicalEquations_Numeric)
                Dynamics = obj.MechanicalEquations_Numeric.GetDynamics(q, v);
                
                Dynamics.c = 0.5*Dynamics.dH*v - Dynamics.G - Dynamics.Dissipation;
            else
                Dynamics.H = obj.g_dynamics_JSIM(q);
                Dynamics.c = obj.ForcesForComputedTorqueController(q, v);
            end
        end
   
        %this function provides actual value of JSIM (joint space inertia
        %matrix), only to be used by solvers
        function JSIM = get_actual_JSIM(obj, q)
            if obj.model_type_code == 1
                JSIM = obj.g_dynamics_JSIM(q);
            else
                JSIM = obj.g_dynamics_JSIM(q, obj.theta);
            end
        end
        %this function provides estimated value of JSIM (joint space inertia
        %matrix)
        function JSIM = get_estimated_JSIM(obj, q)
            if obj.model_type_code == 1
                JSIM = obj.g_dynamics_JSIM(q);
            else
                JSIM = obj.g_dynamics_JSIM(q, obj.estimated_theta);
            end
        end        
        
        %this function provides actual value of RHS (right hand side of the
        %dynamics eq.), only to be used by solvers
        function RHS = get_actual_RHS(obj, q, v, u)
            if obj.model_type_code == 1
                RHS = obj.g_dynamics_RHS(q, v, u);
            else
                RHS = obj.g_dynamics_RHS(q, v, u, obj.theta);
            end
        end
        %this function provides estimated value of RHS (right hand side of the
        %dynamics eq.)
        function RHS = get_estimated_RHS(obj, q, v, u)
            if obj.model_type_code == 1
                RHS = obj.g_dynamics_RHS(q, v, u);
            else
                RHS = obj.g_dynamics_RHS(q, v, u, obj.estimated_theta);
            end
        end          
   
        %this function provides actual value of ControlMap, only to be used
        %by solvers 
        function ControlMap = get_actual_ControlMap(obj, q)
            switch obj.model_type_code
                case {1, 3}
                    ControlMap = obj.g_dynamics_ControlMap(q);
                case 2
                    ControlMap = obj.g_dynamics_ControlMap(q, obj.theta);
            end
        end
        %this function provides estimated value of ControlMap
        function ControlMap = get_estimated_ControlMap(obj, q)
            switch obj.model_type_code
                case {1, 3}
                    ControlMap = obj.g_dynamics_ControlMap(q);
                case 2
                    ControlMap = obj.g_dynamics_ControlMap(q, obj.estimated_theta);
            end
        end         
        
        %this function provides estimated value of ForcesForComputedTorqueController
        function Control = get_estimated_ForcesForComputedTorqueController(obj, q, v)
            if obj.model_type_code == 1
                Control = obj.ForcesForComputedTorqueController(q, v);
            else
                Control = obj.ForcesForComputedTorqueController(q, v, obj.estimated_theta);
            end
        end   
        
        
        %this function provides actual value of RHS (right hand side of the
        %dynamics eq.), only to be used by solvers
        function ConstraintJacobian = get_actual_ConstraintJacobian(obj, q)
            if obj.model_type_code == 1
                ConstraintJacobian = obj.g_dynamics_LagrangeMultiplier_ConstraintJacobian(q);
            else
                ConstraintJacobian = obj.g_dynamics_LagrangeMultiplier_ConstraintJacobian(q, obj.theta);
            end
        end
        %this function provides estimated value of RHS (right hand side of the
        %dynamics eq.)
        function ConstraintJacobian = get_estimated_ConstraintJacobian(obj, q)
            if obj.model_type_code == 1
                ConstraintJacobian = obj.g_dynamics_LagrangeMultiplier_ConstraintJacobian(q);
            else
                ConstraintJacobian = obj.g_dynamics_LagrangeMultiplier_ConstraintJacobian(q, obj.estimated_theta);
            end
        end  
        
    end
    
end