classdef SRDHandler_IK_Model < SRDHandler
    properties
        get_Task;
        get_Jacobian;
        get_Jacobian_derivative;
        
        dof_configuration_space_robot;
        dof_Task;
    end
    methods
        function handle = get_Task_handle(obj)
            handle = @(q) obj.get_Task(q);
        end
        function handle = get_Jacobian_handle(obj)
            handle = @(q) obj.get_Jacobian(q);
        end
        function handle = get_Jacobian_derivative_handle(obj)
            handle = @(q, v) obj.get_Jacobian_derivative(q, v);
        end
    end
end
   