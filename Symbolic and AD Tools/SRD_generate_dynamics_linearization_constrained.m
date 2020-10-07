function description = SRD_generate_dynamics_linearization_constrained(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_generate_dynamics_linearization';
Parser.addOptional('SymbolicEngine', []);

% Parser.addOptional('Symbolic_UseParallelizedSimplification', false);
Parser.addOptional('Symbolic_ToOptimizeFunctions', true);

Parser.addOptional('Casadi_cfile_name', 'g_dynamics_linearization');

Parser.addOptional('FunctionName_A', 'g_linearization_A');
Parser.addOptional('FunctionName_B', 'g_linearization_B');
Parser.addOptional('FunctionName_F', 'g_linearization_F');
Parser.addOptional('FunctionName_c', 'g_linearization_c');

Parser.addOptional('ConstraintJacobian', []);
Parser.addOptional('ConstraintVariables', []);

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

constraint_dof = size(Parser.Results.ConstraintJacobian, 2);


disp('* Linearization started');

% H*ddq + c = T*u;        ddq = dv/dt; v = dq/dt;
% x = [q; v]
%
%
% f= ddq = inv(H) * (T*u - c)
%
% f= ddq = inv(H) * (T*u - c)
%
% dx/dt = A*x+B*u+lc
%
% A = [0      I]
%     [df/dq  df/dv  ]
%
% B = [0           ]
%     [inv(H)*T    ]
%
% lc = [0                             ]
%      [inv(H)*c - df/dq*q -  df/dv*v ]
%
% df / dq = d(inv(H))/dq * (T*u - c) + d(T*u - c)/dq
% df / dq = inv(H) * dH/dq * inv(H) * (T*u - c) + d(T*u - c)/dq
%
% df / dv = inv(H)* d(T*u - c)/dv

H = SymbolicEngine.ForwardDynamicsStructure.JSIM;
c = SymbolicEngine.ForwardDynamicsStructure.ForcesForComputedTorqueController;
T = SymbolicEngine.ForwardDynamicsStructure.ControlActionsToGenMotorTorquesMap;

q = SymbolicEngine.q;
v = SymbolicEngine.v;    
u = SymbolicEngine.u;    

n = SymbolicEngine.dof;
m = length(SymbolicEngine.u);

if SymbolicEngine.Casadi
    iH = add_iH_variable(n);
    
    if isempty(Parser.Results.ConstraintVariables)
        lambda = add_lambda_variable(constraint_dof);
    else
        lambda = reshape(Parser.Results.ConstraintVariables, [], 1);
    end
    
    %%%%%%% ^^^^^^^^^^^^^
    
    TCq = jacobian(T*u+c, q);
    TCv = jacobian(T*u+c, v);
    
    dfdq = -iH*reshape(jacobian(H(:), q)*(iH*(T*u+c)), n, n) + ...
        TCq;
    
    dfdv = iH * TCv;
    
    A = [zeros(n, n), eye(n);
        dfdq,        dfdv];
    
    B = [zeros(n, m);
        iH*T];
    
    linear_c = [zeros(n, 1);
        iH*c - dfdq*q - dfdv*v];
    
    generate_functions_Casadi(A, B, linear_c, iH, Parser);
    description.Casadi_cfile_name = Parser.Results.Casadi_cfile_name;
else
    tic
    iH = sym('iH', [n, n]);
    
    if isempty(Parser.Results.ConstraintVariables)
        lambda = sym('lambda', [constraint_dof, 1]);
    else
        lambda = reshape(Parser.Results.ConstraintVariables, [], 1);
    end

    TCq = jacobian(T*u+c, q);
    disp('Simplifying TCq');
    TCq = simplify(TCq);
        
    TCv = jacobian(T*u+c, v);
    disp('Simplifying TCv');
    TCv = simplify(TCv);
    
    
    dfdq = -iH*reshape(jacobian(H(:), q)*(iH*(T*u+c)), n, n) + ...
        TCq;
    disp('Simplifying dfdq');
    dfdq = simplify(dfdq);
    
    dfdv = iH * TCv;
    disp('Simplifying dfdv');
    dfdv = simplify(dfdv);
    
    A = [zeros(n, n), eye(n);
         dfdq,        dfdv];
     
    B = [zeros(n, m); 
         iH*T];
     
    linear_c = [zeros(n, 1); 
                iH*c - dfdq*q - dfdv*v];
            
    disp('Simplifying A');
    A = simplify(A); 
    disp('Simplifying B');
    B = simplify(B); 
    disp('Simplifying linear_c');
    linear_c = simplify(linear_c); 
    
    generate_functions_symbolic(A, B, linear_c, iH, Parser);
    toc
end    
            
description.Path  = Parser.Results.Path;
description.FunctionName_A  = Parser.Results.FunctionName_A;
description.FunctionName_B  = Parser.Results.FunctionName_B;
description.FunctionName_c  = Parser.Results.FunctionName_c;

description.dof_configuration_space_robot = n;
description.dof_state_space_robot = 2*n;
description.dof_control = m;

disp('* Linearization finished');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function iH = add_iH_variable(dof)
        import casadi.*
        iH = SX.sym('iH', [dof, dof]);        
    end

   
    function lambda = add_lambda_variable(constraint_dof)
        import casadi.*
        lambda = SX.sym('lambda', [constraint_dof, 1]);        
    end
    
    
    function generate_functions_Casadi(A, B, linear_c, iH, Parser)
        import casadi.*
        
        %generate functions
        disp(['Starting writing function for the ', Parser.Results.FunctionName_A]);
        g_linearization_A = Function(Parser.Results.FunctionName_A, ...
            {Parser.Results.SymbolicEngine.q, ...
             Parser.Results.SymbolicEngine.v, ...
             Parser.Results.SymbolicEngine.u, ...
             iH}, ...
            {A}, {'q', 'v', 'u', 'iH'}, {'A'});
        
        disp(['Starting writing function for the ', Parser.Results.FunctionName_B]);
        g_linearization_B = Function(Parser.Results.FunctionName_B, ...
            {Parser.Results.SymbolicEngine.q, ...
             Parser.Results.SymbolicEngine.v, ...
             iH}, ...
            {B}, {'q', 'v', 'iH'}, {'B'});
        
        disp(['Starting writing function for the ', Parser.Results.FunctionName_c]);
        g_linearization_c = Function(Parser.Results.FunctionName_c, ...
            {Parser.Results.SymbolicEngine.q, ...
             Parser.Results.SymbolicEngine.v, ...
             Parser.Results.SymbolicEngine.u, ...
             iH}, ...
            {linear_c}, {'q', 'v', 'u', 'iH'}, {'c'});
        
        if ~isempty(Parser.Results.Path)
            current_dir = pwd;
            cd(Parser.Results.Path);
        end
        
        c_function_name = [Parser.Results.Casadi_cfile_name, '.c'];
        so_function_name = [Parser.Results.Casadi_cfile_name, '.so'];
        
        CG = CodeGenerator(c_function_name);
        CG.add(g_linearization_A);
        CG.add(g_linearization_B);
        CG.add(g_linearization_c);
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


    function generate_functions_symbolic(A, B, linear_c, iH, Parser)

        FileName_A = [Parser.Results.Path, Parser.Results.FunctionName_A];
        FileName_B = [Parser.Results.Path, Parser.Results.FunctionName_B];
        FileName_c = [Parser.Results.Path, Parser.Results.FunctionName_c];
        
        disp(['Starting writing function ', FileName_A]);
        matlabFunction(A, 'File', FileName_A, ...
            'Vars', {Parser.Results.SymbolicEngine.q, ...
                     Parser.Results.SymbolicEngine.v, ...
                     Parser.Results.SymbolicEngine.u, ...
                     iH}, ...
            'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp(['Starting writing function ', FileName_B]);
        matlabFunction(B, 'File', FileName_B, ...
            'Vars', {Parser.Results.SymbolicEngine.q, ...
                     Parser.Results.SymbolicEngine.v, ...
                     iH}, ...
            'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp(['Starting writing function ', FileName_c]);
        matlabFunction(linear_c, 'File', FileName_c, ...
            'Vars', {Parser.Results.SymbolicEngine.q, ...
                     Parser.Results.SymbolicEngine.v, ...
                     Parser.Results.SymbolicEngine.u, ...
                     iH}, ...
                     'Optimize', Parser.Results.Symbolic_ToOptimizeFunctions);
        
        disp('* Finished generating functions'); disp(' ')
    end

end