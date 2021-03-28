classdef SRDHandler_dynamics_HCg_model < SRDHandler
    properties
        %H*ddq + C*dq + g = T*u
        get_H; %H
        get_C; %c
        get_g; %g
        get_T; %T
        
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