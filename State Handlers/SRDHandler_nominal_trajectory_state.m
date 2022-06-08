classdef SRDHandler_nominal_trajectory_state < SRDHandler
    properties
        
        Handler_Time;
        Handler_ControlInput_qva;
        Handler_InverseDynamics;
        function_Original_Model;
        
        dof_state_space_robot;
        dof_configuration_space_robot;
        dof_control;
        
        x;
        dx
        u;
        f;
        nominal_trajectory_error;
        
        ToEvaluateOriginalFunction;
    end
    methods
        function obj = SRDHandler_nominal_trajectory_state(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_nominal_trajectory_state';
            Parser.addOptional('Handler_Time', []);
            Parser.addOptional('Handler_ControlInput_qva',   []);
            Parser.addOptional('Handler_InverseDynamics',    []);
            Parser.addOptional('function_Original_Model',    []);
            Parser.addOptional('ToEvaluateOriginalFunction', true);
            Parser.parse(varargin{:});
            
            obj.Handler_Time               = Parser.Results.Handler_Time;
            obj.Handler_ControlInput_qva   = Parser.Results.Handler_ControlInput_qva;
            obj.Handler_InverseDynamics    = Parser.Results.Handler_InverseDynamics;
            obj.function_Original_Model    = Parser.Results.function_Original_Model;
            obj.ToEvaluateOriginalFunction = Parser.Results.ToEvaluateOriginalFunction;
            
            obj.dof_configuration_space_robot =     obj.Handler_ControlInput_qva.dof_configuration_space_robot;
            obj.dof_state_space_robot         = 2 * obj.Handler_ControlInput_qva.dof_configuration_space_robot;
            obj.dof_control                   =     obj.Handler_InverseDynamics.dof_control;
            
        end
        
        function Update(obj)
            t = obj.Handler_Time.CurrentTime;
            
            squished = obj.Handler_ControlInput_qva.get_position_velocity_acceleration(t);
            
            obj.x  = [squished(:, 1); squished(:, 2)];
            obj.dx = [squished(:, 2); squished(:, 3)];
            obj.u  = obj.Handler_InverseDynamics.u;
            
            if obj.ToEvaluateOriginalFunction
                obj.f  = obj.function_Original_Model(obj.x, obj.u);
                obj.nominal_trajectory_error  = obj.f - obj.dx;
            end
        end
        
        
    end
end
