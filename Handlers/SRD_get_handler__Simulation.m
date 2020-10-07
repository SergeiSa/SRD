function Handler_Simulation = SRD_get_handler__Simulation(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__Simulation';
Parser.addOptional('TimeLog', []);
% Parser.addOptional('ControllerArray', []);
% Parser.addOptional('SolverArray', []);
% Parser.addOptional('LoggerArray', []);

Parser.parse(varargin{:});

Handler_Simulation = SRDHandler_Simulation;

Handler_Simulation.TimeLog = reshape(Parser.Results.TimeLog, [], 1);
Handler_Simulation.CurrentTime = Handler_Simulation.TimeLog(1);

% Handler_Simulation.ControllerArray = Parser.Results.ControllerArray;
% Handler_Simulation.SolverArray = Parser.Results.SolverArray;
% Handler_Simulation.LoggerArray = Parser.Results.LoggerArray;


%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_Simulation.SerializationPrepNeeded = true;
Handler_Simulation.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_Simulation; create a new one on the fly instead')
    end

end