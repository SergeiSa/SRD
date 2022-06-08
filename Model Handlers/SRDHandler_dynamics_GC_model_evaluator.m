classdef SRDHandler_dynamics_GC_model_evaluator < SRDHandler
    properties
        dof_configuration_space_robot;
        dof_control;
        
        UsePinv;
        
        last_update_q;
        last_update_v;
        
        Handler_dynamics_generalized_coordinates_model;
        Handler_State;
        
        %H*ddq + c = T*u
        joint_space_inertia_matrix;
        joint_space_inertia_matrix_inverse;
        bias_vector;
        control_map;
    end
    methods
        
        function obj = SRDHandler_dynamics_GC_model_evaluator(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_dynamics_GC_model_evaluator';
            Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
            Parser.addOptional('Handler_State', []);
            Parser.addOptional('UsePinv', true);
            Parser.parse(varargin{:});
            
            obj.Handler_dynamics_generalized_coordinates_model = Parser.Results.Handler_dynamics_generalized_coordinates_model;
            obj.Handler_State                                  = Parser.Results.Handler_State;
            
            obj.dof_configuration_space_robot = Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
            obj.dof_control                   = Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_control;
            
            obj.UsePinv = Parser.Results.UsePinv;
            
            %implementing serialization for arbitrary cell arrays of handlers seems to
            %be more pain than it is worth
            obj.SerializationPrepNeeded = true;
            obj.PreSerializationPrepFunction = @PreSerializationPrepFunction;
            function PreSerializationPrepFunction(~)
                error('do not attempt to save Handler_dynamics_GC_model_evaluator; create a new one on the fly instead')
            end
            
        end
        
        function Update(obj, ~)
            squized = obj.Handler_State.get_position_velocity_acceleration();
            q = squized(:, 1);
            v = squized(:, 2);
            
            obj.joint_space_inertia_matrix = obj.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(q);
            obj.bias_vector                = obj.Handler_dynamics_generalized_coordinates_model.get_bias_vector(q, v);
            obj.control_map                = obj.Handler_dynamics_generalized_coordinates_model.get_control_map(q);
            
            if obj.UsePinv
                obj.joint_space_inertia_matrix_inverse = pinv(obj.joint_space_inertia_matrix);
            else
                obj.joint_space_inertia_matrix_inverse = obj.joint_space_inertia_matrix \ eye(size(obj.joint_space_inertia_matrix));
            end
            
            obj.last_update_q = q;
            obj.last_update_v = v;
        end
        
        function H = get_joint_space_inertia_matrix(obj, ~)
            H = obj.joint_space_inertia_matrix;
        end
        function c = get_bias_vector(obj, ~, ~)
            c = obj.bias_vector;
        end
        function T = get_control_map(obj, ~)
            T = obj.control_map;
        end
        function iH = get_joint_space_inertia_matrix_inverse(obj, ~)
            iH = obj.joint_space_inertia_matrix_inverse;
        end
        
        
        function FirstOrderSystem_qv_handle = get_FirstOrderSystem_qv_handle(obj)
            FirstOrderSystem_qv_handle = @(x, u) obj.get_FirstOrderSystem_qv(x, u);
        end
        function dx = get_FirstOrderSystem_qv(obj, x, u)
            iH = obj.joint_space_inertia_matrix_inverse;
            c = obj.bias_vector;
            T = obj.control_map;
            
            v = x((1+obj.dof_configuration_space_robot):end);
            
            dx = [v; 
                  iH * (T*u - c)];
        end
    end
end