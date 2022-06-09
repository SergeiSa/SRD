function [inertia_matrix] = SRDhelper_GetInertiaMatrix_from_URDF(varargin)
    Parser = inputParser;
    Parser.FunctionName = 'UPH_GetMeshPathFromLinkName';
    Parser.addOptional('LinkXMLNodes', []);
    Parser.addOptional('LinkName', []);
    Parser.parse(varargin{:});
    
    inertia_matrix = zeros(3);
    
    if isempty(Parser.Results.LinkXMLNodes) 
        error('A proper array of XML link nodes is required')
    end
    
    if isempty(Parser.Results.LinkName)
        error('Link name is required')
    end
    
    LinkXMLNodes = Parser.Results.LinkXMLNodes;
    LinkName = Parser.Results.LinkName;

    
    for link_idx=0:LinkXMLNodes.getLength()-1
        link_name = LinkXMLNodes.item(link_idx).getAttribute('name');

        if strcmp(LinkName,link_name)
            link = LinkXMLNodes.item(link_idx);

            %TODO:check if node is NULL

            inertial_node = SRDhelper_FindXMLChildByName('XMLNode',link,'TagName','inertial'); %UPH_FindXMLChildByName
            if isempty(inertial_node)
                return;
            end
            
            inertia_node = SRDhelper_FindXMLChildByName('XMLNode',inertial_node,'TagName','inertia'); %UPH_FindXMLChildByName
            

            if ~isempty(inertia_node)
                ixx = str2double( inertia_node.getAttribute('ixx'));
                ixy = str2double( inertia_node.getAttribute('ixy'));
                ixz = str2double( inertia_node.getAttribute('ixz'));
                iyy = str2double( inertia_node.getAttribute('iyy'));
                iyz = str2double( inertia_node.getAttribute('iyz'));
                izz = str2double( inertia_node.getAttribute('izz'));
                
                inertia_matrix(1,1) = ixx;
                inertia_matrix(2,2) = iyy;
                inertia_matrix(3,3) = izz;

                inertia_matrix(3,2) = iyz;
                inertia_matrix(3,1) = ixz;
                inertia_matrix(2,1) = ixy;

                inertia_matrix(2,3) = iyz;
                inertia_matrix(1,3) = ixz;
                inertia_matrix(1,2) = ixy;
                break;
            end
        end

    end
end