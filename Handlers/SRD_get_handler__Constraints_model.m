function Handler_Constraints = SRD_get_handler__Constraints_model(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__Constraints_model';
Parser.addOptional('description', []);
Parser.addOptional('dof_robot', []);
Parser.addOptional('dof_Constraint', []);
Parser.parse(varargin{:});

description = Parser.Results.description;

if isfield(description, 'Casadi_cfile_name')
    Casadi = true;
else
    Casadi = false;
end

Handler_Constraints = SRDHandler_Constraints();
Handler_Constraints.State.description = description;

Handler_Constraints.dof_robot = description;

    
if Casadi
    Handler_Constraints.SerializationPrepNeeded = true;
    
    Handler_Constraints.PostSerializationPrepFunction = @PostSerializationPrepFunction;
    Handler_Constraints.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    
else
    Handler_Constraints.get_Constraint          = str2func(Handler_Constraints.State.description.FunctionName_Task);
    Handler_Constraints.get_Jacobian            = str2func(Handler_Constraints.State.description.FunctionName_TaskJacobian);
    Handler_Constraints.get_Jacobian_derivative = str2func(Handler_Constraints.State.description.FunctionName_TaskJacobian_derivative);
end



    function PostSerializationPrepFunction(Handler_Constraints)
        
        import casadi.*
        
        so_function_name = [Path, Handler_Constraints.State.description.Casadi_cfile_name, '.so'];
        
        external_Task                    = external(Handler_Constraints.State.description.FunctionName_Task,                so_function_name);
        external_TaskJacobian            = external(Handler_Constraints.State.description.Function_TaskJacobian,            so_function_name);
        external_TaskJacobian_derivative = external(Handler_Constraints.State.description.Function_TaskJacobian_derivative, so_function_name);
        
        
        Handler_Constraints.get_Constraint          = @(q) full(evalf(external_Task(q)));
        Handler_Constraints.get_Jacobian            = @(q) full(evalf(external_TaskJacobian(q)));
        Handler_Constraints.get_Jacobian_derivative = @(q, v) full(evalf(external_TaskJacobian_derivative(q, v)));
        
    end
    function PreSerializationPrepFunction(Handler_Constraints)
        
        Handler_Constraints.get_Constraint          = [];
        Handler_Constraints.get_Jacobian            = [];
        Handler_Constraints.get_Jacobian_derivative = [];
        
    end

end