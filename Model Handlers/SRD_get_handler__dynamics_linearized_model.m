function Handler_dynamics_Linearized_Model = SRD_get_handler__dynamics_linearized_model(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__dynamics_linearized_model';
Parser.addOptional('description', []);
Parser.parse(varargin{:});

description = Parser.Results.description;

if isfield(description, 'Casadi_cfile_name')
    Casadi = true;
else
    Casadi = false;
end

Handler_dynamics_Linearized_Model = SRDHandler_dynamics_Linearized_Model();
Handler_dynamics_Linearized_Model.State.description = description;

Handler_dynamics_Linearized_Model.dof_configuration_space_robot = description.dof_configuration_space_robot;
Handler_dynamics_Linearized_Model.dof_state_space_robot         = description.dof_state_space_robot;
Handler_dynamics_Linearized_Model.dof_control                   = description.dof_control;
    
if Casadi
    Handler_dynamics_Linearized_Model.SerializationPrepNeeded = true;
    
    Handler_dynamics_Linearized_Model.PostSerializationPrepFunction = @PostSerializationPrepFunction;
    Handler_dynamics_Linearized_Model.PreSerializationPrepFunction  = @PreSerializationPrepFunction;

else
    Handler_dynamics_Linearized_Model.get_A = str2func(Handler_dynamics_Linearized_Model.State.description.FunctionName_A);
    Handler_dynamics_Linearized_Model.get_B = str2func(Handler_dynamics_Linearized_Model.State.description.FunctionName_B);
    Handler_dynamics_Linearized_Model.get_c = str2func(Handler_dynamics_Linearized_Model.State.description.FunctionName_c);
end


    function PostSerializationPrepFunction(Handler_dynamics_Linearized_Model)
        import casadi.*
        
        so_function_name = [Handler_dynamics_Linearized_Model.State.description.Path, Handler_dynamics_Linearized_Model.State.description.Casadi_cfile_name, '.so'];
        
        external_A = external(Handler_dynamics_Linearized_Model.State.description.FunctionName_A, so_function_name);
        external_B = external(Handler_dynamics_Linearized_Model.State.description.FunctionName_B, so_function_name);
        external_c = external(Handler_dynamics_Linearized_Model.State.description.FunctionName_c, so_function_name);
               
        Handler_dynamics_Linearized_Model.get_A = @(q, v, u, iH) full(evalf(external_A(q, v, u, iH)));
        Handler_dynamics_Linearized_Model.get_B = @(q, v,    iH) full(evalf(external_B(q, v,    iH)));
        Handler_dynamics_Linearized_Model.get_c = @(q, v, u, iH) full(evalf(external_c(q, v, u, iH)));
  
    end
    function PreSerializationPrepFunction(Handler_dynamics_Linearized_Model)
        
        Handler_dynamics_Linearized_Model.get_A = [];
        Handler_dynamics_Linearized_Model.get_B = [];
        Handler_dynamics_Linearized_Model.get_c = [];
        
    end

end