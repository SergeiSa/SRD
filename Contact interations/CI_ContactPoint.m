%this class represents a contact point, i.e. a unilateral mechanical
%constraint
classdef CI_ContactPoint < handle
    properties
        
        Name;
        %name of the contact point
        
        Position;
        %position of the contact point
        
        Normal;
        %normal to the contact surace at the contact point
        
        Tangent;
        %tangent to the contact surace at the contact point
        
        friction_coefficient;
        %friction coefficient at the contact point
        
        %%%%%%%%%%%%%%%%%%%%%%
        %%% additional parameters
        
        max_normal_force;
        %max value of the normal force at the contact point
        
        
    end
    methods
        %class constructor
        function obj = CI_ContactPoint(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'CI_ContactPoint.CI_ContactPoint';
            Parser.addOptional('Name', []);
            Parser.addOptional('Position', []);
            Parser.addOptional('Normal', []);
            Parser.addOptional('Tangent', []);
            Parser.addOptional('friction_coefficient', []);
            Parser.addOptional('max_normal_force', Inf);
            Parser.parse(varargin{:});
            
            obj.Name = Parser.Results.Name;
            obj.Position = Parser.Results.Position;
            obj.Normal = Parser.Results.Normal;
            obj.Tangent = Parser.Results.Tangent;
            obj.friction_coefficient = Parser.Results.friction_coefficient;
            
            obj.max_normal_force = Parser.Results.max_normal_force;
            
        end
            
    end
end




