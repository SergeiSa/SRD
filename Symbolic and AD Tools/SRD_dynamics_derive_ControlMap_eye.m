function T = SRD_dynamics_derive_ControlMap_eye(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_dynamics_derive_GeneralizedDissipativeForces_uniform';
Parser.addOptional('SymbolicEngine', []);

Parser.parse(varargin{:});

if isempty(Parser.Results.SymbolicEngine)
    error('Please provide SymbolicEngine')
else
    SymbolicEngine = Parser.Results.SymbolicEngine;
end

% H*ddq + C*dq + g + d = T*u;        ddq = dv/dt; v = dq/dt;
%
% T = I - a special but typical case

if SymbolicEngine.Casadi
    T = eye(SymbolicEngine.dof);
else
    T = sym(eye(SymbolicEngine.dof));
end

end