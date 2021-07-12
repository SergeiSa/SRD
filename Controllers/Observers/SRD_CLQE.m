classdef SRD_CLQE < SRDHandler_Controller
    properties
        Solution;
    end
    methods
        function obj = SRD_CLQE(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRD_CLQE';
            Parser.addOptional('Handler_Time', []);
            Parser.addOptional('Handler_ControlInput_StateSpace', []);
            Parser.addOptional('Handler_State', []);
            Parser.addOptional('Handler_InverseDynamics', []);
            Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
            Parser.addOptional('Handler_dynamics_Linearized_Model', []);
            Parser.addOptional('Handler_Constraints_Model', []);
            Parser.addOptional('C', []);
            Parser.addOptional('ControllerSettings', []);
            Parser.addOptional('ObserverSettings', []);
            Parser.addOptional('tol', []);
            
            Parser.parse(varargin{:});
            
            obj.Update = @() obj.MyUpdate(...
                Parser.Results.Handler_Time, ...
                Parser.Results.Handler_ControlInput_StateSpace, ...
                Parser.Results.Handler_InverseDynamics, ...
                Parser.Results.Handler_dynamics_generalized_coordinates_model, ...
                Parser.Results.Handler_dynamics_Linearized_Model, ...
                Parser.Results.Handler_Constraints_Model, ...
                Parser.Results.C, ...
                Parser.Results.ControllerSettings, ...
                Parser.Results.ObserverSettings, ...
                Parser.Results.tol);
            
        end
        
        
        function MyUpdate(...
                obj, ...
                Handler_Time, ...
                Handler_ControlInput_StateSpace, ...
                Handler_InverseDynamics, ...
                Handler_dynamics_generalized_coordinates_model, ...
                Handler_dynamics_Linearized_Model, ...
                Handler_Constraints_Model, ...
                C, ...
                ControllerSettings, ...
                ObserverSettings, ...
                tol)
                   
            
            t = Handler_Time.CurrentTime;
            
            dof = Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
            k = Handler_Constraints_Model.dof_Constraint;
            
            desired = Handler_ControlInput_StateSpace.get_x_dx(t);
            desired_x =  desired(:, 1);
            desired_dx = desired(:, 2);
            desired_q = desired_x(1:dof);
            desired_v = desired_x((dof + 1):end);
            
            desired_u = Handler_InverseDynamics.u;
            
            %f0 = Handler_dynamics_generalized_coordinates_model.get_FirstOrderSystem_qv(desired_x, desired_u);
            H = Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(desired_q);
            c = Handler_dynamics_generalized_coordinates_model.get_bias_vector(desired_q, desired_v);
            T = Handler_dynamics_generalized_coordinates_model.get_control_map(desired_q);
            n = Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
                      
            F  = Handler_Constraints_Model.get_Jacobian(desired_q);
            dF = Handler_Constraints_Model.get_Jacobian_derivative(desired_q, desired_v);
            G = [zeros(k, dof), F; F, dF];
            k = Handler_Constraints_Model.dof_Constraint;
            
            M = [H, -F';
                 F, zeros(k, k)];
            iM = pinv(M);
            Ma = iM(1:n, :);
            
            a0 = Ma*[(T*desired_u - c); -dF*desired_v];
            g = [desired_v; a0]; 
            
            A = Handler_dynamics_Linearized_Model.get_A();
            B = Handler_dynamics_Linearized_Model.get_B();
            %g = f0 - A * desired_x - B * desired_u;
            
            System = struct('A', A, 'B', B, 'C', C, 'G', G, 'g', g, 'tol', tol, ...
                'ControllerSettings', ControllerSettings, 'ObserverSettings', ObserverSettings, ...
                'x_desired', desired_x, 'dx_desired', desired_dx);
            SaveCopmutations = 1;
            obj.Solution = LTI_CLQE(System, SaveCopmutations);
        end
        
    end
end
