function Handler_IK_Model = SRD_get_handler__IK_model(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__IK_model';
Parser.addOptional('description', []);
Parser.addOptional('dof_robot', []);
Parser.addOptional('dof_Task', []);
Parser.parse(varargin{:});

description = Parser.Results.description;

if isfield(description, 'Casadi_cfile_name')
    Casadi = true;
else
    Casadi = false;
end

Handler_IK_Model = SRDHandler_IK_Model();
Handler_IK_Model.State.description = description;

Handler_IK_Model.dof_robot = Parser.Results.dof_robot;
Handler_IK_Model.dof_Task = Parser.Results.dof_Task;
    
if Casadi
    Handler_IK_Model.SerializationPrepNeeded = true;
    
    Handler_IK_Model.PostSerializationPrepFunction = @PostSerializationPrepFunction;
    Handler_IK_Model.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    
else
    Handler_IK_Model.get_Task                = str2func(Handler_IK_Model.State.description.FunctionName_Task);
    Handler_IK_Model.get_Jacobian            = str2func(Handler_IK_Model.State.description.FunctionName_TaskJacobian);
    Handler_IK_Model.get_Jacobian_derivative = str2func(Handler_IK_Model.State.description.FunctionName_TaskJacobian_derivative);
end



    function PostSerializationPrepFunction(Handler_IK_Model)
        
        import casadi.*
        
        so_function_name = [Handler_IK_Model.State.description.Path, Handler_IK_Model.State.description.Casadi_cfile_name, '.so'];
        
        external_Task                    = external(Handler_IK_Model.State.description.FunctionName_Task,                    so_function_name);
        external_TaskJacobian            = external(Handler_IK_Model.State.description.FunctionName_TaskJacobian,            so_function_name);
        external_TaskJacobian_derivative = external(Handler_IK_Model.State.description.FunctionName_TaskJacobian_derivative, so_function_name);
        
        
        Handler_IK_Model.get_Task                = @(q) full(evalf(external_Task(q)));
        Handler_IK_Model.get_Jacobian            = @(q) full(evalf(external_TaskJacobian(q)));
        Handler_IK_Model.get_Jacobian_derivative = @(q, v) full(evalf(external_TaskJacobian_derivative(q, v)));
        
    end
    function PreSerializationPrepFunction(Handler_IK_Model)
        
        Handler_IK_Model.get_Task                = [];
        Handler_IK_Model.get_Jacobian            = [];
        Handler_IK_Model.get_Jacobian_derivative = [];
        
    end

end