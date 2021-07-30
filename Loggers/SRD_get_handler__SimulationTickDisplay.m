function Handler_SimulationTickDisplay = SRD_get_handler__SimulationTickDisplay(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__SimulationTickDisplay';
Parser.addOptional('Handler_Time', []);
Parser.addOptional('Custom_message_1', 'Simulation is currently at the step ');
Parser.addOptional('Custom_message_2', ' out of ');
Parser.addOptional('DisplayOneTickIn', 1);

Parser.parse(varargin{:});

Handler_SimulationTickDisplay = SRDHandler_Logger;

Handler_SimulationTickDisplay.Update = @() Update(...
    Parser.Results.Custom_message_1, ...
    Parser.Results.Custom_message_2, ...
    Parser.Results.DisplayOneTickIn, ...
    Parser.Results.Handler_Time);

    function Update(Custom_message_1, Custom_message_2, DisplayOneTickIn, Handler_Time)
        if rem(Handler_Time.CurrentIndex, DisplayOneTickIn) == 0
            disp([Custom_message_1, num2str(Handler_Time.CurrentIndex), Custom_message_2, num2str(length(Handler_Time.TimeLog))]);
        end
    end

end
   