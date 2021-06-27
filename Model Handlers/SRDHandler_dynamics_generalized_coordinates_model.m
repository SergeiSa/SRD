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
        
        function dx = get_FirstOrderSystem_qv(obj, x, u)
            q = x(1:obj.dof_configuration_space_robot);
            v = x((obj.dof_configuration_space_robot+1):end);
            
            iH = obj.get_joint_space_inertia_matrix_inverse(q);
            c = obj.get_bias_vector(q, v);
            T = obj.get_control_map(q);
            
            dx = [v; 
                  iH * (T*u - c)];
        end
    end
end