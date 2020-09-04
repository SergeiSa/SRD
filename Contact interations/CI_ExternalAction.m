%this class represents an external action, i.e. a force or a torque
classdef CI_ExternalAction < handle
    properties
        
        Name;
        %name of the external action
        
        Position;
        %position of the external action
        
        Type;
        %type of the external action - 'force' or 'torque'
        
        Value;
        %value of the external action
        
    end
    methods
        %class constructor
        function obj = CI_ExternalAction(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'CI_ExternalAction.CI_ExternalAction';
            Parser.addOptional('Name', []);
            Parser.addOptional('Position', []);
            Parser.addOptional('Type', 'force');
            Parser.addOptional('Value', []);
            Parser.parse(varargin{:});
            
            switch Parser.Results.Type
                case {'force', 'torque'}
                otherwise
                    warning('incorrect ExternalAction type!')
            end
            
            obj.Name = Parser.Results.Name;
            obj.Position = Parser.Results.Position;
            obj.Type = Parser.Results.Type;
            obj.Value = Parser.Results.Value;
        end
            
    end
end




