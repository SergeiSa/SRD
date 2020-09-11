%   This function returns absolute path of an STL mesh file for a given 
%   LinkName from URDF (UrdfFilePath) given XML node array (LinkXMLNodes) of all links in URDF 

function result = UPH_GetMeshPathFromLinkName(varargin)
    Parser = inputParser;
    Parser.FunctionName = 'UPH_GetMeshPathFromLinkName';
    Parser.addOptional('LinkXMLNodes', []);
    Parser.addOptional('UrdfFilePath', []);
    Parser.addOptional('LinkName', []);
    Parser.parse(varargin{:});
    
    if isempty(Parser.Results.UrdfFilePath) 
        error('Please pass URDF file path')
    end
    
    if isempty(Parser.Results.LinkXMLNodes) 
        error('A proper array of XML link nodes is required')
    end
    
    if isempty(Parser.Results.LinkName)
        error('Link name is required')
    end
    
    if exist(Parser.Results.UrdfFilePath,'file')
        LinkXMLNodes = Parser.Results.LinkXMLNodes;
        UrdfFilePath = Parser.Results.UrdfFilePath;
        LinkName = Parser.Results.LinkName;
    else
        error('URDF path does not exist')
    end
    
    result = '';
    path = '';
    for link_idx=0:LinkXMLNodes.getLength()-1
        link_name = LinkXMLNodes.item(link_idx).getAttribute('name');

        if strcmp(LinkName,link_name)
            link = LinkXMLNodes.item(link_idx);

            %TODO:check if node is NULL

            visual_node = UPH_FindXMLChildByName('XMLNode',link,'TagName','visual');
            geometry_node = UPH_FindXMLChildByName('XMLNode',visual_node,'TagName','geometry');
            mesh_node = UPH_FindXMLChildByName('XMLNode',geometry_node,'TagName','mesh');

            if ~isempty(mesh_node)
                path = mesh_node.getAttribute('filename');
                path = char(path(1));
                break;
            end
        end

    end
    if ~isempty(path)
        [filepath,filename,ext] = fileparts(UrdfFilePath);
        relative_path = [filepath '/' path];
        absolute_path = fullfile(pwd, relative_path);
        if exist(absolute_path,'file')
            result = absolute_path;
        end
    end
end