function H = SRD_dynamics_derive_JSIM(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_dynamics_derive_JSIM';
Parser.addOptional('SymbolicEngine', []);
% Parser.addOptional('Symbolic_UseParallelizedSimplification', false);

Parser.parse(varargin{:});

if isempty(Parser.Results.SymbolicEngine)
    error('Please provide SymbolicEngine')
else
    SymbolicEngine = Parser.Results.SymbolicEngine;
end

disp('* Derivation of JSIM started');

% H*ddq + c = T*u;        ddq = dv/dt; v = dq/dt;
%
% H = sum(  J'*m*J +Jw'*I*Jw  )


if SymbolicEngine.Casadi
    H = zeros(SymbolicEngine.dof, SymbolicEngine.dof);
else
    H = sym(zeros(SymbolicEngine.dof, SymbolicEngine.dof));
end

for i = 1:length(SymbolicEngine.LinkArray)
    
    H = H + SymbolicEngine.LinkArray(i).Jacobian_CenterOfMass' * ...
            SymbolicEngine.LinkArray(i).Mass * ...
            SymbolicEngine.LinkArray(i).Jacobian_CenterOfMass ... 
            + ...
            SymbolicEngine.LinkArray(i).Jacobian_AngularVelocity' * ...
            SymbolicEngine.LinkArray(i).Inertia * ...
            SymbolicEngine.LinkArray(i).Jacobian_AngularVelocity;
end

if ~SymbolicEngine.Casadi
    disp('Started simplifying joint space inertia matrix (JSIM) of the mechanism');
    H = simplify(H);
end


disp('* Derivation of JSIM finished');
    
end