% Derives symbolic expression for a generalized gravitational forces - G
%
% SymbolicEngine - an object of SRDSymbolicEngine class, with fields: 
%             .dof (number of gen. coordinates of the system)
%             .Casadi (true/false - flag, to use Casadi or not)
%             .LinkArray - array of SRD links 
%
% GravitationalConstant - 3x1 vector g, such that F = m*g (where m is the mass of
% the link) is the correct gravity force acting on the link in absolute
% (world) coordinates.
%
%It requires that SRD_generate_second_derivative_Jacobians() had been run
%before, to prepare SymbolicEngine.LinkArray.
%
% NOTES:
%
%SymbolicEngine contains symbolic variables and might be difficut to
%serialized (e.g. if Casadi is used). Same with H.
%
% The function uses comparatively slow algorithm based on adding Jacobians
% for each link (see the code, it is straight-forward), but it is designed
% to be run once and then generate an optimized callable (.m file for
% MATLAB, or a .so for Casadi)
%
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