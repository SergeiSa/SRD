function Handler_dynamics_HCg_model = SRD_get_handler__dynamics_HCg_model(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__dynamics_HCg_model';
Parser.addOptional('description', []);
Parser.addOptional('UsePinv', false);
Parser.parse(varargin{:});


if isfield(Parser.Results.description, 'Casadi_cfile_name')
    Casadi = true;
else
    Casadi = false;
end

Handler_dynamics_HCg_model = SRDHandler_dynamics_HCg_model();
Handler_dynamics_HCg_model.State.description = Parser.Results.description;

Handler_dynamics_HCg_model.dof_configuration_space_robot = Parser.Results.description.dof_configuration_space_robot;
Handler_dynamics_HCg_model.dof_control                   = Parser.Results.description.dof_control;
    


Handler_dynamics_HCg_model.SerializationPrepNeeded = true;
Handler_dynamics_HCg_model.PreSerializationPrepFunction  = @PreSerializationPrepFunction;
if Casadi
    Handler_dynamics_HCg_model.PostSerializationPrepFunction = @PostSerializationPrepFunction_casadi;
else
    Handler_dynamics_HCg_model.PostSerializationPrepFunction = @PostSerializationPrepFunction_m;
end


if Casadi
    Handler_dynamics_HCg_model.SerializationPrepNeeded = true;
    
    Handler_dynamics_HCg_model.PostSerializationPrepFunction = @PostSerializationPrepFunction;
    Handler_dynamics_HCg_model.PreSerializationPrepFunction  = @PreSerializationPrepFunction;

else

end

    function PostSerializationPrepFunction_m(Handler_dynamics_HCg_model)
        
        description = Handler_dynamics_HCg_model.State.description;
        if ~isempty(description.Path)
            current_dir = pwd;
            cd(description.Path);
        end
        
        %H*ddq + C*dq + g = T*u
        Handler_dynamics_HCg_model.get_H = str2func(description.FunctionName_H);
        Handler_dynamics_HCg_model.get_C = str2func(description.FunctionName_C);
        Handler_dynamics_HCg_model.get_g = str2func(description.FunctionName_g);
        Handler_dynamics_HCg_model.get_T = str2func(description.FunctionName_T);
        
        if ~isempty(description.Path)
            cd(current_dir);
        end
        
    end        
    function PostSerializationPrepFunction_casadi(Handler_dynamics_HCg_model)
        import casadi.*
        
        so_function_name = [Handler_dynamics_HCg_model.State.description.Path, ...
            Handler_dynamics_HCg_model.State.description.Casadi_cfile_name, '.so'];
        
        external_H = external(Handler_dynamics_HCg_model.State.description.FunctionName_H, so_function_name);
        external_C = external(Handler_dynamics_HCg_model.State.description.FunctionName_C, so_function_name);
        external_g = external(Handler_dynamics_HCg_model.State.description.FunctionName_g, so_function_name);
        external_T = external(Handler_dynamics_HCg_model.State.description.FunctionName_T, so_function_name);
        
        %H*ddq + c = T*u       
        Handler_dynamics_HCg_model.get_H = @(q)    full(evalf(external_H(q)));
        Handler_dynamics_HCg_model.get_C = @(q, v) full(evalf(external_C(q, v)));
        Handler_dynamics_HCg_model.get_g = @(q)    full(evalf(external_g(q)));
        Handler_dynamics_HCg_model.get_T = @(q)    full(evalf(external_T(q)));
  
    end
    function PreSerializationPrepFunction(Handler_dynamics_HCg_model)
        Handler_dynamics_HCg_model.get_H = [];
        Handler_dynamics_HCg_model.get_C = [];
        Handler_dynamics_HCg_model.get_g = [];
        Handler_dynamics_HCg_model.get_T = [];
    end

end