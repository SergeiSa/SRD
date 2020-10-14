function [in, dJSIM] = SRD_dynamics_derive_GeneralizedInertialForces_via_dJSIM(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_dynamics_derive_GeneralizedInertialForces_via_dJSIM';
Parser.addOptional('SymbolicEngine', []);
Parser.addOptional('JointSpaceInertiaMatrix', []);
% Parser.addOptional('Symbolic_UseParallelizedSimplification', false);

Parser.parse(varargin{:});

if isempty(Parser.Results.SymbolicEngine)
    error('Please provide SymbolicEngine')
else
    SymbolicEngine = Parser.Results.SymbolicEngine;
end

if isempty(Parser.Results.JointSpaceInertiaMatrix)
    error('Please provide JointSpaceInertiaMatrix')
end

disp('* Derivation of Generalized Inertial Forces started');

% H*ddq + c + g = T*u;        ddq = dv/dt; v = dq/dt;
%
% c = 0.5 * dH/dt * v

if SymbolicEngine.Casadi
    %dJSIM = jacobian(  reshape(Parser.Results.JointSpaceInertiaMatrix, [], 1), SymbolicEngine.q) * SymbolicEngine.v;
    dJSIM = jacobian(  Parser.Results.JointSpaceInertiaMatrix(:), SymbolicEngine.q) * SymbolicEngine.v;
    dJSIM = reshape(dJSIM, size(Parser.Results.JointSpaceInertiaMatrix));
    
    in = 0.5 * dJSIM * SymbolicEngine.v;
    
    
else
    dJSIM = jacobian(  reshape(Parser.Results.JointSpaceInertiaMatrix, [], 1), SymbolicEngine.q) * SymbolicEngine.v;
    dJSIM = reshape(dJSIM, size(Parser.Results.JointSpaceInertiaMatrix));
    dJSIM = simplify(dJSIM);
    
    in = 0.5 * dJSIM * SymbolicEngine.v;
    in = simplify(in);
end

disp('* Derivation of Generalized Inertial Forces finished');
    
end