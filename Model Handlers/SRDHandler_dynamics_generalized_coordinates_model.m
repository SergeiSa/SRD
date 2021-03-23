classdef SRDHandler_dynamics_generalized_coordinates_model < SRDHandler
    properties
        %H*ddq + c = T*u
        get_joint_space_inertia_matrix; %H
        get_bias_vector; %c
        get_control_map; %T
        
        dof_configuration_space_robot;
        dof_control;
        
        UsePinv;
    end
    methods
        
        function iH = get_joint_space_inertia_matrix_inverse(obj, q)
            H = obj.get_joint_space_inertia_matrix(q);
            
            if obj.UsePinv
                iH = pinv(H);
            else
                iH = H \ eye(size(H));
            end
        end
    end
end