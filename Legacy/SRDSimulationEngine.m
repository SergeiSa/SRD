%This is an abstrunction, teh last child of the class hierarchy, it should
%be used to create SimulationEngine
%last update 14.12.17
classdef SRDSimulationEngine < SRDEstimation
    properties
    end
    methods
        % class constructor
        function obj = SRDSimulationEngine(LinkArray)
            obj = obj@SRDEstimation(LinkArray);
        end
    end
end