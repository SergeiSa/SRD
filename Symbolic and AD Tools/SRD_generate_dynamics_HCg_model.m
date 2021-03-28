function description = SRD_generate_dynamics_HCg_model(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_generate_dynamics_HCg_model';
Parser.addOptional('SymbolicEngine', []);

Parser.addOptional('Symbolic_ToOptimizeFunctions', true);

Parser.addOptional('Casadi_cfile_name', 'g_dynamics_HCg');

%H*ddq + C*dq + g = T*u
Parser.addOptional('H', []);
Parser.addOptional('C', []);
Parser.addOptional('g', []);
Parser.addOptional('T', []);

Parser.addOptional('FunctionName_H', 'g_dynamics_H');
Parser.addOptional('FunctionName_C', 'g_dynamics_C');
Parser.addOptional('FunctionName_g', 'g_dynamics_g');
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
description.FunctionName_C  = Parser.Results.FunctionName_C;
description.FunctionName_g  = Parser.Results.FunctionName_g;
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
        C = Parser.Results.C;
        g = Parser.Results.g;
        T = Parser.Results.T;
        
        %generate functions
        disp(['Starting writing function for the ', Parser.Results.FunctionName_H]);
        g_dynamics_H = Function(Parser.Results.FunctionName_H, ...
            {Parser.Results.SymbolicEngine.q}, ...
            {H}, {'q'}, {'H'});
        
        disp(['Starting writing function for the ', Parser.Results.FunctionName_C]);
        g_dynamics_C = Function(Parser.Results.FunctionName_C, ...
            {Parser.Results.SymbolicEngine.q, ...
             Parser.Results.SymbolicEngine.v}, ...
            {C}, {'q', 'v'}, {'C'});
        
        disp(['Starting writing function for the ', Parser.Results.FunctionName_g]);
        g_dynamics_g = Function(Parser.Results.FunctionName_g, ...
            {Parser.Results.SymbolicEngine.q}, ...
            {g}, {'q'}, {'g'});
        
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
        CG.add(g_dynamics_C);
        CG.add(g_dynamics_g);
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
        C = Parser.Results.C;
        g = Parser.Results.g;
        T = Parser.Results.T;

        FileName_H = [Parser.Results.Path, Parser.Results.FunctionName_H];
        FileName_C = [Parser.Results.Path, Parser.Results.FunctionName_C];
        FileName_g = [Parser.Results.Path, Parser.Results.FunctionName_g];
        FileName_T = [Parser.Results.Path, Parser.Results.FunctionName_T];
        
        disp(['Starting writing function ', FileName_H]);
        matlabFunction(H, 'File', FileName_H, ...
            'Vars', {Parser.Results.SymbolicEngine.q}, ...
            'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp(['Starting writing function ', FileName_C]);
        matlabFunction(C, 'File', FileName_C, ...
            'Vars', {Parser.Results.SymbolicEngine.q, ...
                     Parser.Results.SymbolicEngine.v}, ...
            'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp(['Starting writing function ', FileName_g]);
        matlabFunction(g, 'File', FileName_g, ...
            'Vars', {Parser.Results.SymbolicEngine.q}, ...
                     'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp(['Starting writing function ', FileName_T]);
        matlabFunction(T, 'File', FileName_T, ...
            'Vars', {Parser.Results.SymbolicEngine.q}, ...
                     'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp('* Finished generating functions'); disp(' ')
    end

end