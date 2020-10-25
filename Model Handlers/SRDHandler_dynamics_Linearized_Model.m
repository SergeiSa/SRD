classdef SRDHandler_dynamics_Linearized_Model < SRDHandler
    properties
        get_A;
        get_B;
        get_c;
        
        dof_configuration_space_robot;
        dof_state_space_robot;
        dof_control;
    end
    methods
        function handle = get_A_handle(obj)
            handle = @(q) obj.get_A(q);
        end
        function handle = get_B_handle(obj)
            handle = @(q) obj.get_B(q);
        end
        function handle = get_c_handle(obj)
            handle = @(q, v) obj.get_c(q, v);
        end
    end
end
   