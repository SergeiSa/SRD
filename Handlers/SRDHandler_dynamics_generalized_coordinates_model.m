classdef SRDHandler_dynamics_generalized_coordinates_model < SRDHandler
    properties
        %H*ddq + c = T*u
        get_joint_space_inertia_matrix; %H
        get_bais_vector; %c
        get_control_map; %T
        
        dof_configuration_space_robot;
        dof_control;
    end
    methods

    end
end