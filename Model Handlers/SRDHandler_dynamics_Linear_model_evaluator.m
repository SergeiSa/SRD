classdef SRDHandler_dynamics_Linear_model_evaluator < SRDHandler
    properties
        dof_robot_StateSpace;
        dof_control;
        
        UsePinv;
        
        last_update_q;
        last_update_v;
        last_update_u;
        
        Handler_dynamics_generalized_coordinates_model;
        Handler_dynamics_Linearized_Model;
        
        Handler_State;
        Handler_Controller;
        
        %dx/dt = A*x + B*u + c
        A;
        B;
        c;
        
        ToEvaluate_c;
    end
    methods
        
        function Update(obj, ~)
            squized = obj.Handler_State.get_position_velocity_acceleration();
            q = squized(:, 1);
            v = squized(:, 2);
            

            u = obj.Handler_Controller.u;
            
            %H*ddq + c = T*u
            iH = obj.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix_inverse(q);
            
            obj.A = obj.Handler_dynamics_Linearized_Model.get_A(q, v, u, iH);
            obj.B = obj.Handler_dynamics_Linearized_Model.get_B(q, v,    iH);
            
            if obj.ToEvaluate_c
                obj.c = obj.Handler_dynamics_Linearized_Model.get_c(q, v, u, iH);
            end
            
            obj.last_update_q = q;
            obj.last_update_v = v;
            obj.last_update_u = u;
        end
        
        function A = get_A(obj, ~, ~, ~, ~)
            A = obj.A;
        end
        function B = get_B(obj, ~, ~, ~)
            B = obj.B;
        end
        function c = get_c(obj, ~, ~, ~, ~)
            c = obj.c;
        end
    end
end