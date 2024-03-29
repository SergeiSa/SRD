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
Handler_dynamics_Linearized_Model.LinearizationType = description.LinearizationType;

Handler_dynamics_Linearized_Model.dof_configuration_space_robot = description.dof_configuration_space_robot;
Handler_dynamics_Linearized_Model.dof_state_space_robot         = description.dof_state_space_robot;
Handler_dynamics_Linearized_Model.dof_control                   = description.dof_control;
    
Handler_dynamics_Linearized_Model.SerializationPrepNeeded = true;
Handler_dynamics_Linearized_Model.PreSerializationPrepFunction  = @PreSerializationPrepFunction;

if Casadi
    Handler_dynamics_Linearized_Model.PostSerializationPrepFunction = @PostSerializationPrepFunction_casadi; 
else
    Handler_dynamics_Linearized_Model.PostSerializationPrepFunction = @PostSerializationPrepFunction_m; 
end


    function PostSerializationPrepFunction_m(Handler_dynamics_Linearized_Model)
    if ~isempty(Handler_dynamics_Linearized_Model.State.description.Path)
        current_dir = pwd;
        cd(Handler_dynamics_Linearized_Model.State.description.Path);
    end
    
    Handler_dynamics_Linearized_Model.get_A = str2func(Handler_dynamics_Linearized_Model.State.description.FunctionName_A);
    Handler_dynamics_Linearized_Model.get_B = str2func(Handler_dynamics_Linearized_Model.State.description.FunctionName_B);
    
    if ~isempty(Handler_dynamics_Linearized_Model.State.description.Path)
        cd(current_dir);
    end
  
    end
    function PostSerializationPrepFunction_casadi(Handler_dynamics_Linearized_Model)
        import casadi.*
        
        so_function_name = [Handler_dynamics_Linearized_Model.State.description.Path, Handler_dynamics_Linearized_Model.State.description.Casadi_cfile_name, '.so'];
        
        external_A = external(Handler_dynamics_Linearized_Model.State.description.FunctionName_A, so_function_name);
        external_B = external(Handler_dynamics_Linearized_Model.State.description.FunctionName_B, so_function_name);
               
        Handler_dynamics_Linearized_Model.get_A = @(q, v, u, iH) full(evalf(external_A(q, v, u, iH)));
        Handler_dynamics_Linearized_Model.get_B = @(q, v,    iH) full(evalf(external_B(q, v,    iH)));
  
    end
    function PreSerializationPrepFunction(Handler_dynamics_Linearized_Model)
        
        Handler_dynamics_Linearized_Model.get_A = [];
        Handler_dynamics_Linearized_Model.get_B = [];
        
    end

end