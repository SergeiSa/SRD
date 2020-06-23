%This is an abstrunction, the last child of the class hierarchy, it should
%be used to create SymbolicEngine
%last update 14.12.17
classdef SRDSymbolicEngine < SRDControlEquations
    properties
    end
    methods
        % class constructor
        function obj = SRDSymbolicEngine(LinkArray, Casadi)
            if nargin < 2
                Casadi = false;
            end
            
            obj = obj@SRDControlEquations(LinkArray, Casadi);
        end
    end
end