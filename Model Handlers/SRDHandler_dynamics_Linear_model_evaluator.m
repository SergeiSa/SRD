classdef SRDHandler_dynamics_Linear_model_evaluator < SRDHandler
    properties
        dof_state_space_robot;
        dof_configuration_space_robot;
        dof_control;
        
        UsePinv;
        
        last_update_q;
        last_update_v;
        last_update_u;
        
        Handler_dynamics_generalized_coordinates_model;
        Handler_dynamics_Linearized_Model;
        
        Handler_State;
        Handler_Controller;
        
        LinearizationType;
        TemporalType;
        
        %dx/dt = A*x + B*u + c
        A;
        B;
    end
    methods
        
        function obj = SRDHandler_dynamics_Linear_model_evaluator(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRD_get_handler__dynamics_GC_model_evaluator';
            Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
            Parser.addOptional('Handler_dynamics_Linearized_Model', []);
            Parser.addOptional('Handler_State', []);
            Parser.addOptional('Handler_Controller', []);
            Parser.parse(varargin{:});
            
            obj.Handler_dynamics_generalized_coordinates_model = Parser.Results.Handler_dynamics_generalized_coordinates_model;
            obj.Handler_dynamics_Linearized_Model              = Parser.Results.Handler_dynamics_Linearized_Model;
            obj.Handler_State                                  = Parser.Results.Handler_State;
            obj.Handler_Controller                             = Parser.Results.Handler_Controller;
            
            
            obj.LinearizationType             = obj.Handler_dynamics_Linearized_Model.LinearizationType;
            obj.TemporalType                  = obj.Handler_dynamics_Linearized_Model.TemporalType;
            obj.dof_control                   = obj.Handler_dynamics_generalized_coordinates_model.dof_control;
            obj.dof_configuration_space_robot = obj.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
            obj.dof_state_space_robot         = 2 * obj.dof_configuration_space_robot;
            
            
            %implementing serialization for arbitrary cell arrays of handlers seems to
            %be more pain than it is worth
            obj.SerializationPrepNeeded = true;
            obj.PreSerializationPrepFunction = @PreSerializationPrepFunction;
            function PreSerializationPrepFunction(~)
                error('do not attempt to save Handler_dynamics_Linear_model_evaluator; create a new one on the fly instead')
            end
        end
        
        function Update(obj, ~)
            squized = obj.Handler_State.get_position_velocity_acceleration();
            q = squized(:, 1);
            v = squized(:, 2);
            
            u = obj.Handler_Controller.u;
            
            %H*ddq + c = T*u
            iH = obj.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix_inverse(q);
            
            obj.A = obj.Handler_dynamics_Linearized_Model.get_A(q, v, u, iH);
            obj.B = obj.Handler_dynamics_Linearized_Model.get_B(q, v,    iH);
            
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
    end
end