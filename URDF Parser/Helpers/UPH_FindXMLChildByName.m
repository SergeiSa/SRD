%   This function returns the child node of a given XML node with a flag specified
%   in TagName parameter
%
%   <ParentNode>
%    <TagName ...  /TagName>
%   </ParentNode>

function child_node = UPH_FindXMLChildByName(varargin)
    Parser = inputParser;
    Parser.FunctionName = 'UPH_FindXMLChildByName';
    Parser.addOptional('XMLNode', []);
    Parser.addOptional('TagName', []);
    Parser.parse(varargin{:});

    if isempty(Parser.Results.XMLNode) || isempty(Parser.Results.TagName)
        error('Pass XML node object and link name')
    else
        XMLNode = Parser.Results.XMLNode;
        TagName = Parser.Results.TagName;
    end

    child = XMLNode.getFirstChild();
                    
    while ~isempty(child)
        if strcmp(child.getNodeName(),TagName)
            child_node = child;
            break;
        else
            child = child.getNextSibling();
        end
    end
    if  ~exist('child_node','var')
        child_node = {};
    end
end