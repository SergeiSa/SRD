function d = SRD_dynamics_derive_GeneralizedDissipativeForces_uniform(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_dynamics_derive_GeneralizedDissipativeForces_uniform';
Parser.addOptional('SymbolicEngine', []);
Parser.addOptional('UniformCoefficient', 1);

Parser.parse(varargin{:});

if isempty(Parser.Results.SymbolicEngine)
    error('Please provide SymbolicEngine')
else
    SymbolicEngine = Parser.Results.SymbolicEngine;
end

% H*ddq + C*dq + g + d = T*u;        ddq = dv/dt; v = dq/dt;
%
% d = -k*v - dissipative forces

d = -Parser.Results.UniformCoefficient * SymbolicEngine.v;

end