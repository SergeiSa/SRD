function Handler_dynamics_Linear_model_evaluator = SRD_get_handler__dynamics_Linear_model_evaluator(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__dynamics_GC_model_evaluator';
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_dynamics_Linearized_Model', []);
Parser.addOptional('Handler_State', []);
Parser.addOptional('Handler_Controller', []);
Parser.addOptional('ToEvaluate_c', true);
Parser.parse(varargin{:});

Handler_dynamics_Linear_model_evaluator = SRDHandler_dynamics_Linear_model_evaluator();
Handler_dynamics_Linear_model_evaluator.Handler_dynamics_generalized_coordinates_model = Parser.Results.Handler_dynamics_generalized_coordinates_model;
Handler_dynamics_Linear_model_evaluator.Handler_dynamics_Linearized_Model              = Parser.Results.Handler_dynamics_Linearized_Model;
Handler_dynamics_Linear_model_evaluator.Handler_State                                  = Parser.Results.Handler_State;
Handler_dynamics_Linear_model_evaluator.Handler_Controller                             = Parser.Results.Handler_Controller;

 
Handler_dynamics_Linear_model_evaluator.dof_robot_StateSpace   = 2 * Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
Handler_dynamics_Linear_model_evaluator.dof_control            = Parser.Results.Handler_dynamics_generalized_coordinates_model.dof_control;

Handler_dynamics_Linear_model_evaluator.ToEvaluate_c           = Parser.Results.ToEvaluate_c;


%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_dynamics_Linear_model_evaluator.SerializationPrepNeeded = true;
Handler_dynamics_Linear_model_evaluator.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_dynamics_Linear_model_evaluator; create a new one on the fly instead')
    end

end