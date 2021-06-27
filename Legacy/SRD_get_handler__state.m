function Handler_State = SRD_get_handler__state(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__state';
Parser.addOptional('InitialPosition', []);
Parser.addOptional('InitialVelocity', []);
Parser.parse(varargin{:});


Handler_State = SRDHandler_State();
Handler_State.q = reshape(Parser.Results.InitialPosition, [], 1);
Handler_State.v = reshape(Parser.Results.InitialVelocity, [], 1);
Handler_State.a = NaN(size(Handler_State.v));

Handler_State.dof_robot = length(Parser.Results.InitialPosition);

end