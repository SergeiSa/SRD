function Link = SRD_get_Link(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_Link';
Parser.addOptional('Order', []);
Parser.addOptional('Name', []);
Parser.addOptional('RelativeBase', []);
Parser.addOptional('RelativeFollower', []);
Parser.addOptional('RelativeCoM', []);
Parser.addOptional('Mass', []);
Parser.addOptional('Inertia', []);

Parser.addOptional('ToDisplay', true);
Parser.addOptional('Color', 'r');
Parser.addOptional('StlPath', []);

% Parser.addOptional('ParentLink', []);
% Parser.addOptional('ParentFollowerNumber', []);
% Parser.addOptional('Joint', []);
Parser.parse(varargin{:});

Link = SRD_Link;
Link.Order            = Parser.Results.Order;
Link.Name             = Parser.Results.Name;

Link.RelativeBase     = Parser.Results.RelativeBase;
Link.RelativeFollower = Parser.Results.RelativeFollower;
Link.RelativeCoM      = Parser.Results.RelativeCoM;
Link.Mass             = Parser.Results.Mass;
Link.Inertia          = Parser.Results.Inertia;

% Link.ParentLink            = Parser.Results.ParentLink;
% Link.ParentFollowerNumber  = Parser.Results.ParentFollowerNumber;

% Link.Update           = @Link.Joint.Update;

end