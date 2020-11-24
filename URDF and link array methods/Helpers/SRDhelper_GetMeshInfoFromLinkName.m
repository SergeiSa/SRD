%   This function returns absolute path of an STL mesh file and it's scale for a given 
%   LinkName from URDF (UrdfFilePath) given XML node array (LinkXMLNodes) of all links in URDF 
%
function [mesh_path,scale] = SRDhelper_GetMeshInfoFromLinkName(varargin)
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
    
    scale = [1,1,1];
    mesh_path = '';
    path = '';
    for link_idx=0:LinkXMLNodes.getLength()-1
        link_name = LinkXMLNodes.item(link_idx).getAttribute('name');

        if strcmp(LinkName,link_name)
            link = LinkXMLNodes.item(link_idx);

            %TODO:check if node is NULL

            visual_node = SRDhelper_FindXMLChildByName('XMLNode',link,'TagName','visual');
            geometry_node = SRDhelper_FindXMLChildByName('XMLNode',visual_node,'TagName','geometry');
            mesh_node = SRDhelper_FindXMLChildByName('XMLNode',geometry_node,'TagName','mesh');

            if ~isempty(mesh_node)
                path = mesh_node.getAttribute('filename');
                if mesh_node.hasAttribute('scale')
                    scale = mesh_node.getAttribute('scale');
                    scale_vals = split(scale.toCharArray');
                    scale = [str2double(scale_vals{1}),str2double(scale_vals{2}),str2double(scale_vals{3})];
                end
                path = char(path(1));
                break;
            end
        end

    end
    if ~isempty(path)
        [filepath,~,~] = fileparts(UrdfFilePath);
        relative_path = [filepath '/' path];
        absolute_path = fullfile(pwd, relative_path);
        if exist(absolute_path,'file')
            mesh_path = absolute_path;
        end
    end
end
