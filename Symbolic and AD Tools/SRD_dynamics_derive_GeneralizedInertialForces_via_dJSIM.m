% Derives symbolic expression for a generalized inertial forces - in
%
% SymbolicEngine - an object of SRDSymbolicEngine class, with fields: 
%             .dof (number of gen. coordinates of the system)
%             .Casadi (true/false - flag, to use Casadi or not)
%             .LinkArray - array of SRD links 
%
% H - joint space inertial matrix, can be found by calling
% SRD_dynamics_derive_JSIM()
%
% in - generalized inertia
% dJSIM - time derivative of H: dH/dt
%
%It requires that SRD_generate_second_derivative_Jacobians() had been run
%before, to prepare SymbolicEngine.LinkArray.
%
% NOTES:
%
%SymbolicEngine contains symbolic variables and might be difficut to
%serialize (e.g. if Casadi is used). Same with in.
%
% The function runs faster than Christoffel symbol method, but does not
% return C matrix in the H*ddq + C*dq + g = u form of the dynamics equations 
%
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