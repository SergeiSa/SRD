function CoM = SRD_get_CoM_ForLinkArray(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_CoM_ForLinkArray';
Parser.addOptional('SymbolicEngine', []);


Parser.parse(varargin{:});

if isempty(Parser.Results.SymbolicEngine)
    error('Please provide SymbolicEngine')
else
    SymbolicEngine = Parser.Results.SymbolicEngine;
end

CoM = zeros(3, 1);
Mass = 0;

for i = 1:length(SymbolicEngine.LinkArray)
    
    CoM = SymbolicEngine.LinkArray(i).AbsoluteCoM * SymbolicEngine.LinkArray(i).Mass ...
        + CoM;
    
    Mass = SymbolicEngine.LinkArray(i).Mass + Mass;
end

CoM = CoM / Mass;

end