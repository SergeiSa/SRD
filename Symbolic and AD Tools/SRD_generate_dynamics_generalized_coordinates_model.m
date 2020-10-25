function description = SRD_generate_dynamics_generalized_coordinates_model(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_generate_dynamics_generalized_coordinates_model';
Parser.addOptional('SymbolicEngine', []);

Parser.addOptional('Symbolic_ToOptimizeFunctions', true);

Parser.addOptional('Casadi_cfile_name', 'g_dynamics_generalized_coordinates');

%H*ddq + c = T*u
Parser.addOptional('H', []);
Parser.addOptional('c', []);
Parser.addOptional('T', []);

Parser.addOptional('FunctionName_H', 'g_dynamics_H');
Parser.addOptional('FunctionName_c', 'g_dynamics_c');
Parser.addOptional('FunctionName_T', 'g_dynamics_T');

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


disp('* Generation of dynamics functions started');

if SymbolicEngine.Casadi
    generate_functions_Casadi(Parser);
    description.Casadi_cfile_name = Parser.Results.Casadi_cfile_name;
else
    tic
    generate_functions_symbolic(Parser);
    toc
end    
            
description.Path  = Parser.Results.Path;
description.FunctionName_H  = Parser.Results.FunctionName_H;
description.FunctionName_c  = Parser.Results.FunctionName_c;
description.FunctionName_T  = Parser.Results.FunctionName_T;

description.dof_configuration_space_robot = SymbolicEngine.dof;
description.dof_control = length(SymbolicEngine.u);

disp('* Generation of dynamics functions finished');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function generate_functions_Casadi(Parser)
        import casadi.*
        
        H = Parser.Results.H;
        c = Parser.Results.c;
        T = Parser.Results.T;
        
        %generate functions
        disp(['Starting writing function for the ', Parser.Results.FunctionName_H]);
        g_dynamics_H = Function(Parser.Results.FunctionName_H, ...
            {Parser.Results.SymbolicEngine.q}, ...
            {H}, {'q'}, {'H'});
        
        disp(['Starting writing function for the ', Parser.Results.FunctionName_c]);
        g_dynamics_c = Function(Parser.Results.FunctionName_c, ...
            {Parser.Results.SymbolicEngine.q, ...
             Parser.Results.SymbolicEngine.v}, ...
            {c}, {'q', 'v'}, {'c'});
        
        disp(['Starting writing function for the ', Parser.Results.FunctionName_T]);
        g_dynamics_T = Function(Parser.Results.FunctionName_T, ...
            {Parser.Results.SymbolicEngine.q}, ...
            {T}, {'q'}, {'T'});
        
        if ~isempty(Parser.Results.Path)
            current_dir = pwd;
            cd(Parser.Results.Path);
        end
        
        c_function_name = [Parser.Results.Casadi_cfile_name, '.c'];
        so_function_name = [Parser.Results.Casadi_cfile_name, '.so'];
        
        CG = CodeGenerator(c_function_name);
        CG.add(g_dynamics_H);
        CG.add(g_dynamics_c);
        CG.add(g_dynamics_T);
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


    function generate_functions_symbolic(Parser)
        
        H = Parser.Results.H;
        c = Parser.Results.c;
        T = Parser.Results.T;

        FileName_H = [Parser.Results.Path, Parser.Results.FunctionName_H];
        FileName_c = [Parser.Results.Path, Parser.Results.FunctionName_c];
        FileName_T = [Parser.Results.Path, Parser.Results.FunctionName_T];
        
        disp(['Starting writing function ', FileName_H]);
        matlabFunction(H, 'File', FileName_H, ...
            'Vars', {Parser.Results.SymbolicEngine.q}, ...
            'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp(['Starting writing function ', FileName_c]);
        matlabFunction(c, 'File', FileName_c, ...
            'Vars', {Parser.Results.SymbolicEngine.q, ...
                     Parser.Results.SymbolicEngine.v}, ...
            'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp(['Starting writing function ', FileName_T]);
        matlabFunction(T, 'File', FileName_T, ...
            'Vars', {Parser.Results.SymbolicEngine.q}, ...
                     'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp('* Finished generating functions'); disp(' ')
    end

end