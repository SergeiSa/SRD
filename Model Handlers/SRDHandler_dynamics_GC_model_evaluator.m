classdef SRDHandler_dynamics_GC_model_evaluator < SRDHandler
    properties
        dof_configuration_space_robot;
        dof_control;
        
        UsePinv;
        
        last_update_q;
        last_update_v;
        Handler_dynamics_generalized_coordinates_model;
        Handler_State;
        
        %H*ddq + c = T*u
        joint_space_inertia_matrix;
        joint_space_inertia_matrix_inverse;
        bais_vector;
        control_map;
    end
    methods
        
        function Update(obj, ~)
            squized = obj.Handler_State.get_position_velocity_acceleration();
            q = squized(:, 1);
            v = squized(:, 2);
            
            obj.joint_space_inertia_matrix = obj.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(q);
            obj.bais_vector                = obj.Handler_dynamics_generalized_coordinates_model.get_bais_vector(q, v);
            obj.control_map                = obj.Handler_dynamics_generalized_coordinates_model.get_control_map(q);
            
            if obj.UsePinv
                obj.joint_space_inertia_matrix_inverse = pinv(obj.joint_space_inertia_matrix);
            else
                obj.joint_space_inertia_matrix_inverse = obj.joint_space_inertia_matrix \ eye(size(obj.joint_space_inertia_matrix));
            end
            
            obj.last_update_q = q;
            obj.last_update_v = v;
        end
        
        function H = get_joint_space_inertia_matrix(obj, ~)
            H = obj.joint_space_inertia_matrix;
        end
        function c = get_bais_vector(obj, ~, ~)
            c = obj.bais_vector;
        end
        function T = get_control_map(obj, ~)
            T = obj.control_map;
        end
        function iH = get_joint_space_inertia_matrix_inverse(obj, ~)
            iH = obj.joint_space_inertia_matrix_inverse;
        end
    end
end