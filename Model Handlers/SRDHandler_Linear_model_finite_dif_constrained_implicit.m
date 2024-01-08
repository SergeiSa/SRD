classdef SRDHandler_Linear_model_finite_dif_constrained_implicit < SRDHandler
    properties
        dof_state_space_robot;
        dof_configuration_space_robot;
        dof_control;
        dof_Constraint;
        
        last_update_q;
        last_update_v;
        last_update_u;
        
        Handler_dynamics_generalized_coordinates_model;
        Handler_dynamics_Linearized_Model;
        Handler_Constraints_Model;
        
        Handler_State;
        Handler_Controller;
        
        finite_dif_step_x;
        finite_dif_step_u;

        LinearizationType;
        TemporalType;
        
        % x = [q, v]
        %dx/dt = A*x + B*u + c
        A;
        B;
        
        tol = 10^(-9);
    end
    methods
        
        function obj = SRDHandler_Linear_model_finite_dif_constrained_implicit(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_Linear_model_finite_dif_constrained_implicit';
            Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
            Parser.addOptional('Handler_Constraints_Model', []);
            Parser.addOptional('Handler_State', []);
            Parser.addOptional('Handler_Controller', []);
            Parser.addOptional('finite_dif_step_x', 0.0001);
            Parser.addOptional('finite_dif_step_u', 0.0001);
            Parser.parse(varargin{:});
            
            obj.Handler_dynamics_generalized_coordinates_model = Parser.Results.Handler_dynamics_generalized_coordinates_model;
            obj.Handler_Constraints_Model                      = Parser.Results.Handler_Constraints_Model;
            obj.Handler_State                                  = Parser.Results.Handler_State;
            obj.Handler_Controller                             = Parser.Results.Handler_Controller;
            
            if ~strcmp(obj.Handler_dynamics_generalized_coordinates_model.type, "function caller")
                warning("Handler_dynamics_generalized_coordinates_model appears to be of a wrong type")
            end
            
            obj.LinearizationType             = "Finite-difference constrained";
            obj.TemporalType                  = "ContiniousTime";
            obj.dof_control                   = obj.Handler_dynamics_generalized_coordinates_model.dof_control;
            obj.dof_configuration_space_robot = obj.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
            obj.dof_state_space_robot         = 2 * obj.dof_configuration_space_robot;
            obj.dof_Constraint                = obj.Handler_Constraints_Model.dof_Constraint;
            
            
            obj.finite_dif_step_x                = Parser.Results.finite_dif_step_x;
            obj.finite_dif_step_u                = Parser.Results.finite_dif_step_u;

            
            if isscalar(obj.finite_dif_step_x) 
                obj.finite_dif_step_x = eye(obj.dof_state_space_robot) * obj.finite_dif_step_x;
            end
            if isscalar(obj.finite_dif_step_u) 
                obj.finite_dif_step_u = eye(obj.dof_control) * obj.finite_dif_step_u;
            end


            %implementing serialization for arbitrary cell arrays of handlers seems to
            %be more pain than it is worth
            obj.SerializationPrepNeeded = true;
            obj.PreSerializationPrepFunction = @PreSerializationPrepFunction;
            function PreSerializationPrepFunction(~)
                error('do not attempt to save this function; create a new one on the fly instead')
            end
        end
        
        function Update(obj, ~)
            dof = obj.dof_configuration_space_robot;
            dof_ctrl = obj.dof_control;
            k = obj.Handler_Constraints_Model.dof_Constraint;
            n = obj.dof_state_space_robot;
            
            squized = obj.Handler_State.get_position_velocity_acceleration();
            q = squized(:, 1);
            v = squized(:, 2);
            u = obj.Handler_Controller.u;
            
            x = [q; v];
            a = obj.get_acceleration(x, u);
            
            
            J  = obj.Handler_Constraints_Model.get_Jacobian(q);
            dJ = obj.Handler_Constraints_Model.get_Jacobian_derivative(q, v);
            G = [J,  zeros(k, dof); 
                 dJ, J];
            
            N = null(G);
            P = N*N';
            
            delta_X = P * obj.finite_dif_step_x;
            delta_U =     obj.finite_dif_step_u;
            
            a_array = zeros(n, n);
            for i = 1:n
                xi = x + delta_X(:, i);
                ai = obj.get_acceleration(xi, u);
                a_array(:, i) = ai - a;
            end
            obj.A = a_array * pinv(delta_X, obj.tol);
            
            
            b_array = zeros(n, dof_ctrl);  
            for j = 1:dof_ctrl
                uj = u + delta_U(:, j);
                aj = obj.get_acceleration(x, uj);
                b_array(:, j) = aj - a;
            end
            obj.B = b_array * pinv(delta_U, obj.tol);

            obj.last_update_q = q;
            obj.last_update_v = v;
            obj.last_update_u = u;
        end
        
        
        function [a] = get_acceleration(obj, x, u)
            k = obj.Handler_Constraints_Model.dof_Constraint;
            n = obj.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
            %m = obj.Handler_dynamics_generalized_coordinates_model.dof_control;
            
            q = x(1:n);
            v = x((n+1):end);
            
            H = obj.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(q);
            c = obj.Handler_dynamics_generalized_coordinates_model.get_bias_vector(q, v);
            T = obj.Handler_dynamics_generalized_coordinates_model.get_control_map(q);
            J = obj.Handler_Constraints_Model.get_Jacobian(q);
            dJ = obj.Handler_Constraints_Model.get_Jacobian_derivative(q, v);
            
            M = [H, -J'; J, zeros(k, k)];
            vec = pinv(M) * [(T*u - c); -dJ*v];
            a = [v; vec(1:n)];
            
        end
        
        
        
        function A = get_A(obj, ~, ~, ~, ~)
            A = obj.A;
        end
        function B = get_B(obj, ~, ~, ~)
            B = obj.B;
        end
    end
end