%This is an abstraction, the last child of the class hierarchy, it should
%be used to create SymbolicEngine
classdef SRDSymbolicEngine < SRDSymbolicEncoding
    properties
    end
    methods
        % class constructor
        function obj = SRDSymbolicEngine(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDSymbolicEngine';
            Parser.addOptional('LinkArray', []);
            Parser.addOptional('Casadi', false);
            Parser.parse(varargin{:});

            obj = obj@SRDSymbolicEncoding(Parser.Results.LinkArray, Parser.Results.Casadi);
        end
    end
end