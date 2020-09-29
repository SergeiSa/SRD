function output = SRD_IK_generate_inverse_kinematics_functions(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_IK_generate_inverse_kinematics_functions';
Parser.addOptional('SymbolicEngine', []);
Parser.addOptional('Task', []);

Parser.addOptional('Symbolic_ToSimplify', true);
Parser.addOptional('Symbolic_UseParallelizedSimplification', false);
Parser.addOptional('Symbolic_ToOptimizeFunctions', true);

Parser.addOptional('Casadi_cfile_name', 'g_InverseKinematics');
Parser.addOptional('FunctionName_Task', 'g_InverseKinematics_Task');
Parser.addOptional('Function_TaskJacobian', 'g_InverseKinematics_TaskJacobian');
Parser.addOptional('Function_TaskJacobian_derivative', 'g_InverseKinematics_TaskJacobian_derivative');

Parser.addOptional('Path', []);

Parser.parse(varargin{:});

if isempty(Parser.Results.SymbolicEngine)
    error('Please provide SymbolicEngine')
else
    SymbolicEngine = Parser.Results.SymbolicEngine;
end

SRD_generate_second_derivative_Jacobians('SymbolicEngine', Parser.Results.SymbolicEngine, ...
    'Task', Parser.Results.Task, ...
    'Symbolic_ToSimplify', Parser.Results.Symbolic_ToSimplify, ...
    'Symbolic_UseParallelizedSimplification', Parser.Results.Symbolic_UseParallelizedSimplification, ...
    'Symbolic_ToOptimizeFunctions', Parser.Results.Symbolic_ToOptimizeFunctions, ...
    'Casadi_cfile_name', Parser.Results.Casadi_cfile_name, ...
    'FunctionName_Task', Parser.Results.FunctionName_Task, ...
    'Function_TaskJacobian', Parser.Results.Function_TaskJacobian, ...
    'Function_TaskJacobian_derivative', Parser.Results.Function_TaskJacobian_derivative, ...
    'Path', Parser.Results.Path);

% if ~isempty(Parser.Results.Path)
%     if ~exist(Parser.Results.Path, 'dir')
%         mkdir(Parser.Results.Path)
%     end
% end
% 
% 
% if SymbolicEngine.Casadi
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Casadi
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     
%     TaskJacobian = jacobian(Parser.Results.Task, SymbolicEngine.q);
%     TaskJacobian_derivative = jacobian(TaskJacobian(:), SymbolicEngine.q) * SymbolicEngine.v;
%     TaskJacobian_derivative = reshape(TaskJacobian_derivative, size(TaskJacobian));
%     
%     handles = generate_IK_function_Casadi(Parser.Results.Task, TaskJacobian, TaskJacobian_derivative, Parser);
%     
% else
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Matlab symbolic
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     
%     tic;
%     Task = Parser.Results.Task;
%     if Parser.Results.Symbolic_ToSimplify
%         Task = simplify(Task);
%     end
%     
%     disp('Started generating inverse kinematics task jacobian');
%     TaskJacobian = jacobian(Task, q);
%     
%     if Parser.Results.Symbolic_ToSimplify
%         disp('Started simplifying inverse kinematics task jacobian');
%         if Parser.Results.Symbolic_UseParallelizedSimplification
%             Math = MathClass;
%             TaskJacobian = Math.ParallelizedSimplification(TaskJacobian, 'J');
%         else
%             TaskJacobian = simplify(TaskJacobian);
%         end
%         disp('* Finished simplifying inverse kinematics task jacobian');
%     end
%     
%     TaskJacobian_derivative = jacobian(TaskJacobian(:), SymbolicEngine.q) * SymbolicEngine.v;
%     TaskJacobian_derivative = reshape(TaskJacobian_derivative, size(TaskJacobian));
%     
%     if Parser.Results.Symbolic_ToSimplify
%         disp('Started simplifying derivative of the inverse kinematics task jacobian');
%         if Parser.Results.Symbolic_UseParallelizedSimplification
%             Math = MathClass;
%             TaskJacobian_derivative = Math.ParallelizedSimplification(TaskJacobian_derivative, 'dJ');
%         else
%             TaskJacobian_derivative = simplify(TaskJacobian_derivative);
%         end
%         disp('* Finished simplifying derivative of the inverse kinematics task jacobian');
%     end
%     
%     handles = generate_IK_function_symbolic(Task, TaskJacobian, TaskJacobian_derivative, Parser);
%     toc
% end
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % function generation
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%     function handles = generate_IK_function_Casadi(Task, TaskJacobian, TaskJacobian_derivative, Parser)
%         import casadi.*
%         
%         %generate functions
%         disp('Starting writing function for the inverse kinematics task');
%         g_InverseKinematics_Task = Function(Parser.Results.FunctionName_Task, ...
%             {Parser.Results.SymbolicEngine.q}, {Task}, {'q'}, {'Task'});
%         
%         disp('Starting writing function for the inverse kinematics task jacobian');
%         g_InverseKinematics_TaskJacobian = Function(Parser.Results.Function_TaskJacobian, ...
%             {Parser.Results.SymbolicEngine.q}, {TaskJacobian}, {'q'}, {'TaskJacobian'});
%         
%         disp('Starting writing function for the derivative of the inverse kinematics task jacobian');
%         g_InverseKinematics_TaskJacobian_derivative = Function(Parser.Results.Function_TaskJacobian_derivative, ...
%             {Parser.Results.SymbolicEngine.q, Parser.Results.SymbolicEngine.v}, {TaskJacobian_derivative}, {'q', 'v'}, {'TaskJacobian_derivative'});
%         
%         
%         
%         if ~isempty(Parser.Results.Path)
%             current_dir = pwd;
%             cd(Parser.Results.Path);
%         end
%         
%         c_function_name = [Parser.Results.Casadi_cfile_name, '.c'];
%         so_function_name = [Parser.Results.Casadi_cfile_name, '.so'];
%         
%         CG = CodeGenerator(c_function_name);
%         CG.add(g_InverseKinematics_Task);
%         CG.add(g_InverseKinematics_TaskJacobian);
%         CG.add(g_InverseKinematics_TaskJacobian_derivative);
%         CG.generate();
%         
%         command = 'gcc -fPIC -shared ';
%         command = [command, c_function_name];
%         command = [command, ' -o '];
%         command = [command, so_function_name];
%         
%         disp(' ');
%         disp('Command to be executed:');
%         disp(command);
%         
%         system(command);
%         %!gcc -fPIC -shared g_InverseKinematics.c -o g_InverseKinematics.so
%         
% 
%         external_Task = external(Parser.Results.FunctionName_Task, ['./', so_function_name]);
%         external_TaskJacobian = external(Parser.Results.Function_TaskJacobian, so_function_name);
%         external_TaskJacobian_derivative = external(Parser.Results.Function_TaskJacobian_derivative, so_function_name);
%         
%         handles.Task = @(q) full(evalf(external_Task(q)));
%         handles.TaskJacobian = @(q) full(evalf(external_TaskJacobian(q)));
%         handles.TaskJacobian_derivative = @(q) full(evalf(external_TaskJacobian_derivative(q)));
%         
%         if ~isempty(Parser.Results.Path)
%             cd(current_dir);
%         end
%         
%     end
% 
% 
%     function handles = generate_IK_function_symbolic(Task, TaskJacobian, TaskJacobian_derivative, Parser)
%         
%         FileName_Task = [Parser.Results.FunctionName_Task '.mat'];
%         FileName_TaskJacobian = [Parser.Results.Function_TaskJacobian '.mat'];
%         FileName_TaskJacobian_derivative = [Parser.Results.Function_TaskJacobian_derivative '.mat'];
%         
%         disp('Starting writing function for the inverse kinematics task');
%         handles.Task = matlabFunction(Task, 'File', FileName_Task, ...
%             'Vars', {Parser.Results.SymbolicEngine.q}, 'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
%         
%         disp('Starting writing function for the inverse kinematics task jacobian');
%         handles.TaskJacobian = matlabFunction(TaskJacobian, 'File', FileName_TaskJacobian, ...
%             'Vars', {Parser.Results.SymbolicEngine.q}, 'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
%         
%         disp('Starting writing function for the derivative of the inverse kinematics task jacobian');
%         handles.TaskJacobian_derivative = matlabFunction(TaskJacobian_derivative, 'File', FileName_TaskJacobian_derivative, ...
%             'Vars', {Parser.Results.SymbolicEngine.q, Parser.Results.SymbolicEngine.v}, 'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
%         
%         disp('* Finished generating inverse kinematics functions'); disp(' ')
%     end

end