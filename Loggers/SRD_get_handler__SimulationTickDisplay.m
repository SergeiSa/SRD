function Handler_SimulationTickDisplay = SRD_get_handler__SimulationTickDisplay(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__SimulationTickDisplay';
Parser.addOptional('Handler_Simulation', []);
Parser.addOptional('Custom_message_1', 'Simulation is currently at the step ');
Parser.addOptional('Custom_message_2', ' out of ');
Parser.addOptional('DisplayOneTickIn', 1);


Parser.parse(varargin{:});

Handler_SimulationTickDisplay = SRDHandler_Logger;

Handler_SimulationTickDisplay.Update = @() Update(...
    Parser.Results.Custom_message_1, ...
    Parser.Results.Custom_message_2, ...
    Parser.Results.DisplayOneTickIn, ...
    Parser.Results.Handler_Simulation);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_SimulationTickDisplay.SerializationPrepNeeded = true;
Handler_SimulationTickDisplay.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save SRD_get_handler__SimulationTickDisplay; create a new one on the fly instead')
    end


    function Update(Custom_message_1, Custom_message_2, DisplayOneTickIn, Handler_Simulation)
        if rem(Handler_Simulation.CurrentIndex, DisplayOneTickIn) == 0
            disp([Custom_message_1, num2str(Handler_Simulation.CurrentIndex), Custom_message_2, num2str(length(Handler_Simulation.TimeLog))]);
        end
    end

end
   