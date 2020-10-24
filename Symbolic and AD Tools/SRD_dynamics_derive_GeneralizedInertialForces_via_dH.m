function [in, dH] = SRD_dynamics_derive_GeneralizedInertialForces_via_dH(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_dynamics_derive_GeneralizedInertialForces_via_dH';
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

H = Parser.Results.JointSpaceInertiaMatrix;
q = SymbolicEngine.q;
v = SymbolicEngine.v;

dH = jacobian( reshape(H, [], 1), q)*v;
dH = reshape(dH, length(q), length(q));

if SymbolicEngine.Casadi
    
    KineticEnergy = 0.5*v' * H * v;
    in = reshape(dH*v, [], 1) - reshape(jacobian(KineticEnergy, q), [], 1);
    
    %Alternative method - kinetic energy expression is not needed, but is slower
    %in = reshape(dH*v, [], 1) - 0.5*( jacobian( reshape(H, [], 1), q)'*kron(v, v) );
else
    dH = simplify(dH);
    
    KineticEnergy = 0.5*v' * H * v;
    in = reshape(dH*v, [], 1) - reshape(jacobian(KineticEnergy, q), [], 1);
    
    %Alternative method - kinetic energy expression is not needed, but is slower
    %in = reshape(dH*v, [], 1) - 0.5*( jacobian( reshape(H, [], 1), q)'*kron(v, v) );
    
    in = simplify(in);
end

disp('* Derivation of Generalized Inertial Forces finished');

end