function Handler_dynamics_generalized_coordinates_model = SRD_get_handler__dynamics_generalized_coordinates_model(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__dynamics_generalized_coordinates_model';
Parser.addOptional('description', []);
Parser.parse(varargin{:});

description = Parser.Results.description;

if isfield(description, 'Casadi_cfile_name')
    Casadi = true;
else
    Casadi = false;
end

Handler_dynamics_generalized_coordinates_model = SRDHandler_dynamics_generalized_coordinates_model();
Handler_dynamics_generalized_coordinates_model.State.description = description;

Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot = description.dof_configuration_space_robot;
Handler_dynamics_generalized_coordinates_model.dof_control                   = description.dof_control;
    
if Casadi
    Handler_dynamics_generalized_coordinates_model.SerializationPrepNeeded = true;
    
    Handler_dynamics_generalized_coordinates_model.PostSerializationPrepFunction = @PostSerializationPrepFunction;
    Handler_dynamics_generalized_coordinates_model.PreSerializationPrepFunction  = @PreSerializationPrepFunction;

else
    if ~isempty(description.Path)
        current_dir = pwd;
        cd(description.Path);
    end
    
    %H*ddq + c = T*u
    Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix = str2func(description.FunctionName_H);
    Handler_dynamics_generalized_coordinates_model.get_bais_vector = str2func(description.FunctionName_c);
    Handler_dynamics_generalized_coordinates_model.get_control_map = str2func(description.FunctionName_T);
    
    if ~isempty(description.Path)
        cd(current_dir);
    end
end

        
    function PostSerializationPrepFunction(Handler_dynamics_generalized_coordinates_model)
        import casadi.*
        
        so_function_name = [Handler_dynamics_generalized_coordinates_model.State.description.Path, ...
            Handler_dynamics_generalized_coordinates_model.State.description.Casadi_cfile_name, '.so'];
        
        external_H = external(Handler_dynamics_generalized_coordinates_model.State.description.FunctionName_H, so_function_name);
        external_c = external(Handler_dynamics_generalized_coordinates_model.State.description.FunctionName_c, so_function_name);
        external_T = external(Handler_dynamics_generalized_coordinates_model.State.description.FunctionName_T, so_function_name);
        
        %H*ddq + c = T*u       
        Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix = @(q)    full(evalf(external_H(q)));
        Handler_dynamics_generalized_coordinates_model.get_bais_vector                = @(q, v) full(evalf(external_c(q, v)));
        Handler_dynamics_generalized_coordinates_model.get_control_map                = @(q)    full(evalf(external_T(q)));
  
    end
    function PreSerializationPrepFunction(Handler_dynamics_Linearized_Model)
        
        Handler_dynamics_Linearized_Model.get_joint_space_inertia_matrix = [];
        Handler_dynamics_Linearized_Model.get_bais_vector = [];
        Handler_dynamics_Linearized_Model.get_control_map = [];
        
    end

end