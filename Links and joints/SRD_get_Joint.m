function Joint = SRD_get_Joint_PivotX(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_Joint_PivotX';
Parser.addOptional('Name', []);
Parser.addOptional('Type', []);
Parser.addOptional('ChildLink', []);
Parser.addOptional('ParentLink', []);
Parser.addOptional('ParentFollowerNumber', []);

Parser.addOptional('UsedGeneralizedCoordinates', []);
Parser.addOptional('UsedControlInputs', []);

Parser.addOptional('DefaultJointOrientation', []);
Parser.parse(varargin{:});

Type = Parser.Results.Type;

Name            = Parser.Results.Name;
ChildLink       = Parser.Results.ChildLink;
ParentLink      = Parser.Results.ParentLink;

ParentFollowerNumber  = Parser.Results.ParentFollowerNumber;


UsedGeneralizedCoordinates      = Parser.Results.UsedGeneralizedCoordinates;
UsedControlInputs               = Parser.Results.UsedControlInputs;

DefaultJointOrientation         = Parser.Results.DefaultJointOrientation;


jointConstructor = [];

if strcmp(Type,'pivotX')
    jointConstructor = @SRD_get_Joint_PivotX;
    ProperSize = 1;
end
if strcmp(Type,'pivotY')
    jointConstructor = @SRD_get_Joint_PivotY;
end
if strcmp(Type,'pivotZ')
    jointConstructor = @SRD_get_Joint_PivotZ;
end
if strcmp(Type,'pivotXY')
    jointConstructor = @SRD_get_Joint_PivotXY;
end
if strcmp(Type,'pivotYZ')
    jointConstructor = @SRD_get_Joint_PivotYZ;
end
if strcmp(Type,'pivotXZ')
    jointConstructor = @SRD_get_Joint_PivotZX;
end

if strcmp(Type,'fixed')
    jointConstructor = @SRD_get_Joint_Fixed;
end

if isempty(jointConstructor)
    error('Invalid joint type');
end


Joint =  jointConstructor(...
    'Name', Name, ...
    'ChildLink',  ChildLink, ...
    'ParentLink', ParentLink, ...
    'ParentFollowerNumber', ParentFollowerNumber, ...
    'UsedGeneralizedCoordinates', UsedGeneralizedCoordinates, ...
    'UsedControlInputs', UsedControlInputs, ...
    'DefaultJointOrientation', DefaultJointOrientation);
end

























