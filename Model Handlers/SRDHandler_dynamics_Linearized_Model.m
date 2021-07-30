classdef SRDHandler_dynamics_Linearized_Model < SRDHandler
    properties
        get_A;
        get_B;
        get_c;
        
        dof_configuration_space_robot;
        dof_state_space_robot;
        dof_control;
        
        LinearizationType;
    end
    methods
        function handle = get_A_handle(obj)
            handle = @(q, v, u, iH) obj.get_A(q, v, u, iH);
        end
        function handle = get_B_handle(obj)
            handle = @(q, v,    iH) obj.get_B(q, v,    iH);
        end
        function handle = get_c_handle(obj)
            handle = @(q, v, u, iH) obj.get_c(q, v, u, iH);
        end
    end
end
   