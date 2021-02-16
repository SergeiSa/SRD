%Function for getting an SRD link structure. 
%Link contains all information about the geometry, physical properties,
%visual properties.
%
% Order - number of the link in the robot chain, helps SRD update links 
% in the correct order (TO DO: automate the process). Parents should have
% lower Order than children
%
% Name - unique ID of the link
% 
% RelativeBase - position of the "base" of the link (3x1 vector). Base is the point on
% the link, by which it is attached to its parent
%
% RelativeFollower - positions of the link's "followers" (3xn array, 
% where n is the number of followers). Followers are points, to which the
% children of the link can be attached.
% - followers can be used to automatically display the links (when STL is
% not provided)
%
% RelativeCoM - position of the center of mass of the link (3x1 vector). 
%
% Mass - mass of the link.
%
% Inertia - inertia of the link.
%
% ToDisplay - (true/false) switch on or off the display of the link
%
% Color - color of the link display
%
% StlPath - path to the STL file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% NOTES.
%
% You can use symbolic variables for vectors and physical parameters, and
% then use them to derive equations, etc.
%
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