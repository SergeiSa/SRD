function Handler_NoInputController = SRD_get_handler__NoInputController(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__NoInputController';
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);

Parser.parse(varargin{:});

Handler_NoInputController = SRDHandler_Controller;

Handler_NoInputController.Update = @() Update(...
    Handler_NoInputController, ...
    Parser.Results.Handler_dynamics_generalized_coordinates_model);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_NoInputController.SerializationPrepNeeded = true;
Handler_NoInputController.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_NoInputController; create a new one on the fly instead')
    end


    function Update(Handler_NoInputController, ...
            Handler_dynamics_generalized_coordinates_model)
        
        Handler_NoInputController.u = zeros(Handler_dynamics_generalized_coordinates_model.dof_control, 1);
        
    end

end