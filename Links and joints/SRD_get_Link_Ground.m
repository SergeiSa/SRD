function Link = SRD_get_Link_Ground(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_Link';
Parser.addOptional('Order', 0);
Parser.addOptional('Name', 'Ground');
Parser.addOptional('RelativeBase', [0;0;0]);
Parser.addOptional('RelativeFollower', [0;0;0]);
Parser.addOptional('RelativeCoM', [0;0;0]);
Parser.addOptional('RelativeOrientation', eye(3));
Parser.addOptional('Mass', 0);
Parser.addOptional('Inertia', eye(3));
Parser.parse(varargin{:});

Link = SRD_Link;
Link.Order            = Parser.Results.Order;
Link.Name             = Parser.Results.Name;

Link.RelativeBase        = Parser.Results.RelativeBase;
Link.RelativeFollower    = Parser.Results.RelativeFollower;
Link.RelativeCoM         = Parser.Results.RelativeCoM;
Link.RelativeOrientation = Parser.Results.RelativeOrientation;

Link.Mass             = Parser.Results.Mass;
Link.Inertia          = Parser.Results.Inertia;


Link.AbsoluteBase        = Parser.Results.RelativeBase;
Link.AbsoluteFollower    = Parser.Results.RelativeFollower;
Link.AbsoluteCoM         = Parser.Results.RelativeCoM;
Link.AbsoluteOrientation = Parser.Results.RelativeOrientation;

Link.Update = @(~) [];
Link.Joint = [];

end