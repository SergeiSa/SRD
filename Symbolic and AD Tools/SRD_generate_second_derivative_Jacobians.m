function description = SRD_generate_second_derivative_Jacobians(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_generate_second_derivative_Jacobians';
Parser.addOptional('SymbolicEngine', []);
Parser.addOptional('Task', []);

Parser.addOptional('Symbolic_ToSimplify', true);
Parser.addOptional('Symbolic_UseParallelizedSimplification', false);
Parser.addOptional('Symbolic_ToOptimizeFunctions', true);

Parser.addOptional('Casadi_cfile_name', 'g_abstract');
Parser.addOptional('FunctionName_Task', 'g_abstract_Task');
Parser.addOptional('FunctionName_TaskJacobian', 'g_abstract_TaskJacobian');
Parser.addOptional('FunctionName_TaskJacobian_derivative', 'g_abstract_TaskJacobian_derivative');

Parser.addOptional('Path', []);


Parser.parse(varargin{:});

if isempty(Parser.Results.SymbolicEngine)
    error('Please provide SymbolicEngine')
else
    SymbolicEngine = Parser.Results.SymbolicEngine;
end

if ~isempty(Parser.Results.Path)
    if ~exist(Parser.Results.Path, 'dir')
        mkdir(Parser.Results.Path)
    end
end

dof_task = length(Parser.Results.Task);

if SymbolicEngine.Casadi
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Casadi
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    TaskJacobian = jacobian(Parser.Results.Task, SymbolicEngine.q);
    TaskJacobian_derivative = jacobian(TaskJacobian(:), SymbolicEngine.q) * SymbolicEngine.v;
    TaskJacobian_derivative = reshape(TaskJacobian_derivative, size(TaskJacobian));
    
    generate_functions_function_Casadi(Parser.Results.Task, TaskJacobian, TaskJacobian_derivative, Parser);
    
    description.Casadi_cfile_name = Parser.Results.Casadi_cfile_name;
else
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Matlab symbolic
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    tic;
    Task = Parser.Results.Task;
    if Parser.Results.Symbolic_ToSimplify
        Task = simplify(Task);
    end
    
    disp(['Started generating ', Parser.Results.FunctionName_Task, ' task jacobian']);
    TaskJacobian = jacobian(Task, SymbolicEngine.q);
    
    if Parser.Results.Symbolic_ToSimplify
        disp(['Started simplifying ', Parser.Results.FunctionName_Task, ' task jacobian']);
        if Parser.Results.Symbolic_UseParallelizedSimplification
            Math = MathClass;
            TaskJacobian = Math.ParallelizedSimplification(TaskJacobian, 'J');
        else
            TaskJacobian = simplify(TaskJacobian);
        end
        disp(['* Finished simplifying ', Parser.Results.FunctionName_Task, ' task jacobian']);
    end
    
    TaskJacobian_derivative = jacobian(TaskJacobian(:), SymbolicEngine.q) * SymbolicEngine.v;
    TaskJacobian_derivative = reshape(TaskJacobian_derivative, size(TaskJacobian));
    
    if Parser.Results.Symbolic_ToSimplify
        disp(['Started simplifying derivative of ', Parser.Results.FunctionName_Task, ' task jacobian']);
        if Parser.Results.Symbolic_UseParallelizedSimplification
            Math = MathClass;
            TaskJacobian_derivative = Math.ParallelizedSimplification(TaskJacobian_derivative, 'dJ');
        else
            TaskJacobian_derivative = simplify(TaskJacobian_derivative);
        end
        disp(['* Finished simplifying derivative of ', Parser.Results.FunctionName_Task, ' task jacobian']);
    end
    
    generate_functions_function_symbolic(Task, TaskJacobian, TaskJacobian_derivative, Parser);
    toc
end

description.Path  = Parser.Results.Path;
description.FunctionName_Task                     = Parser.Results.FunctionName_Task;
description.FunctionName_TaskJacobian             = Parser.Results.FunctionName_TaskJacobian;
description.FunctionName_TaskJacobian_derivative  = Parser.Results.FunctionName_TaskJacobian_derivative;
description.dof_task                              = dof_task;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function generate_functions_function_Casadi(Task, TaskJacobian, TaskJacobian_derivative, Parser)
        import casadi.*
        
        %generate functions
        disp(['Starting writing function for the ', Parser.Results.FunctionName_Task]);
        g_Task = Function(Parser.Results.FunctionName_Task, ...
            {Parser.Results.SymbolicEngine.q}, {Task}, {'q'}, {'Task'});
        
        disp(['Starting writing function for the ', Parser.Results.FunctionName_Task, ' task jacobian']);
        g_TaskJacobian = Function(Parser.Results.FunctionName_TaskJacobian, ...
            {Parser.Results.SymbolicEngine.q}, {TaskJacobian}, {'q'}, {'TaskJacobian'});
        
        disp(['Starting writing function for the derivative of ', Parser.Results.FunctionName_Task, ' task jacobian']);
        g_TaskJacobian_derivative = Function(Parser.Results.FunctionName_TaskJacobian_derivative, ...
            {Parser.Results.SymbolicEngine.q, Parser.Results.SymbolicEngine.v}, {TaskJacobian_derivative}, {'q', 'v'}, {'TaskJacobian_derivative'});
        
        
        if ~isempty(Parser.Results.Path)
            current_dir = pwd;
            cd(Parser.Results.Path);
        end
        
        c_function_name = [Parser.Results.Casadi_cfile_name, '.c'];
        so_function_name = [Parser.Results.Casadi_cfile_name, '.so'];
        
        CG = CodeGenerator(c_function_name);
        CG.add(g_Task);
        CG.add(g_TaskJacobian);
        CG.add(g_TaskJacobian_derivative);
        CG.generate();
        
        command = 'gcc -fPIC -shared ';
        command = [command, c_function_name];
        command = [command, ' -o '];
        command = [command, so_function_name];
        
        disp(' ');
        disp('Command to be executed:');
        disp(command);
        
        system(command);
        %!gcc -fPIC -shared g_InverseKinematics.c -o g_InverseKinematics.so
        
        if ~isempty(Parser.Results.Path)
            cd(current_dir);
        end
        
    end


    function generate_functions_function_symbolic(Task, TaskJacobian, TaskJacobian_derivative, Parser)
        
        FileName_Task                    = [Parser.Results.Path, Parser.Results.FunctionName_Task];
        FileName_TaskJacobian            = [Parser.Results.Path, Parser.Results.FunctionName_TaskJacobian];
        FileName_TaskJacobian_derivative = [Parser.Results.Path, Parser.Results.FunctionName_TaskJacobian_derivative];
        
        disp(['Starting writing function ', FileName_Task]);
        matlabFunction(Task, 'File', FileName_Task, ...
            'Vars', {Parser.Results.SymbolicEngine.q}, 'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp(['Starting writing function ', FileName_TaskJacobian]);
        matlabFunction(TaskJacobian, 'File', FileName_TaskJacobian, ...
            'Vars', {Parser.Results.SymbolicEngine.q}, 'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp(['Starting writing function ', FileName_TaskJacobian_derivative]);
        matlabFunction(TaskJacobian_derivative, 'File', FileName_TaskJacobian_derivative, ...
            'Vars', {Parser.Results.SymbolicEngine.q, Parser.Results.SymbolicEngine.v}, 'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp('* Finished generating functions'); disp(' ')
    end

end