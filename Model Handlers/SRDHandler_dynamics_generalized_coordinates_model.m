%A handler for dynamics in the form:
%H*ddq + c = T*u
%
classdef SRDHandler_dynamics_generalized_coordinates_model < SRDHandler
    properties
        get_joint_space_inertia_matrix; %H
        get_bias_vector; %c
        get_control_map; %T
        
        dof_configuration_space_robot; %number of degrees of freedom
        dof_control; %number of control inputs
        
        UsePinv; %if true - use pinv for matrix inversion, else use \
        
        type = "function caller"; 
        %can be useful to distinguish between handlers that actually call functions to re-evaluate the model
        %and handlers that only recall most resent model values from memory
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
        
        %In some cases (for simulation with ode45, for inverse dynamics,
        %etc, it is good to have system in the normal form - as a system of
        %first order ODE.
        %
        %x = [q;v], where q - gen. coordinates (position), v - gen.
        %velocities
        %u - control input
        %
        function FirstOrderSystem_qv_handle = get_FirstOrderSystem_qv_handle(obj)
            FirstOrderSystem_qv_handle = @(x, u) obj.get_FirstOrderSystem_qv(x, u);
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