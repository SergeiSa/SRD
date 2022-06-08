classdef SRDHandler_Constraints < SRDHandler
    properties
        
        Handler_dynamics_generalized_coordinates_model;
        
        get_Constraint;
        get_Jacobian;
        get_Jacobian_derivative;
        
        dof_configuration_space_robot;
        dof_Constraint;
    end
    methods
        
        function [G, F, dF] = get_linear_constraint_on_dx(obj, x)
            q = x(1:obj.dof_configuration_space_robot);
            v = x(1+obj.dof_configuration_space_robot:end);
            
            F  = obj.get_Jacobian(q);
            dF = obj.get_Jacobian_derivative(q, v);
            
            G = [F, zeros(size(F)); dF, F];
        end
        
        
        
        function FirstOrderSystem_qv_handle = get_FirstOrderSystem_qv_handle(obj)
            FirstOrderSystem_qv_handle = @(x, u) obj.get_FirstOrderSystem_qv(x, u);
        end
        function dx = get_FirstOrderSystem_qv(obj, x, u)
            q = x(1:obj.dof_configuration_space_robot);
            v = x((obj.dof_configuration_space_robot+1):end);
            
            F  = obj.get_Jacobian(q);
            dF = obj.get_Jacobian_derivative(q, v);
            
            H = obj.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(q);
            c  = obj.Handler_dynamics_generalized_coordinates_model.get_bias_vector(q, v);
            T  = obj.Handler_dynamics_generalized_coordinates_model.get_control_map(q);
            
            M = [H, -F'; F, zeros(size(F, 1))];
            vec = [T*u - c; -dF*v];
            
            a = pinv(M)*vec;
            
            dx = [v; a(1:obj.dof_configuration_space_robot)];
        end
        
    end
end