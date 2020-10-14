function G = SRD_dynamics_derive_GeneralizedGravitationalForces(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_dynamics_derive_GeneralizedGravitationalForces';
Parser.addOptional('SymbolicEngine', []);
Parser.addOptional('GravitationalConstant', [0; 0; -9.8]);
% Parser.addOptional('Symbolic_UseParallelizedSimplification', false);

Parser.parse(varargin{:});

if isempty(Parser.Results.SymbolicEngine)
    error('Please provide SymbolicEngine')
else
    SymbolicEngine = Parser.Results.SymbolicEngine;
end

disp('* Derivation of Generalized Gravitational Forces started');

% H*ddq + C*dq + g = T*u;        ddq = dv/dt; v = dq/dt;
%
% g = sum(  J'*m*g  )


if SymbolicEngine.Casadi
    G = zeros(SymbolicEngine.dof, 1);
else
    G = sym(zeros(SymbolicEngine.dof, 1));
end

for i = 1:length(SymbolicEngine.LinkArray)
    
    G = G + SymbolicEngine.LinkArray(i).Jacobian_CenterOfMass' * ...
            SymbolicEngine.LinkArray(i).Mass * ...
            Parser.Results.GravitationalConstant;
end

if ~SymbolicEngine.Casadi
    disp('Started simplifying generalized gravitational forces');
    G = simplify(G);
end

disp('* Derivation of Generalized Gravitational Forces finished');
    
end