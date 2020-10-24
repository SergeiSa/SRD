function in = SRD_dynamics_derive_GeneralizedInertialForces_via_Christoffel(varargin)
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

H = Parser.Results.JointSpaceInertiaMatrix;
q = SymbolicEngine.q;
v = SymbolicEngine.v;
n = size(H, 1);

if SymbolicEngine.Casadi
    
    in = zeros(n, 1);
    
    for i = 1:n
        for j = 1:n
            for k = 1:n
                G = 0.5 * jacobian( H(i, j), q(k) ) + ...
                    0.5 * jacobian( H(i, k), q(j) ) - ...
                    0.5 * jacobian( H(k, j), q(i) );
                
                in(i) = in(i) + G * v(j) * v(k);
            end
        end
    end
    
    
else
    
    in = sym(zeros(n, 1));
    
    for i = 1:n
        for j = 1:n
            for k = 1:n
                G = 0.5 * jacobian( H(i, j), q(k) ) + ...
                    0.5 * jacobian( H(i, k), q(j) ) - ...
                    0.5 * jacobian( H(k, j), q(i) );
                
                G = simplify(G);
                in(i) = in(i) + G * v(j) * v(k);
            end
        end
    end
end

disp('* Derivation of Generalized Inertial Forces finished');
    
end