function SRD_PrintJoints(link_array)
disp('* Links have names:')
    for link_idx = 1:length(link_array)
        LinkName       = link_array(link_idx).Name;
        
        if ~isempty(link_array(link_idx).ParentLink)
            
            ParentLinkName = link_array(link_idx).ParentLink.Name;
            JointType      = link_array(link_idx).Joint.Type;
            UsedControlInputs          = link_array(link_idx).Joint.UsedControlInputs;
            UsedGeneralizedCoordinates = link_array(link_idx).Joint.UsedGeneralizedCoordinates;
            
            disp([JointType, ' joint connects ', LinkName, ' with ', ParentLinkName, ...
                '; u: ', mat2str(reshape(UsedControlInputs, 1, [])), ...
                '; q: ', mat2str(reshape(UsedGeneralizedCoordinates, 1, []))]);
            
        end
    end
end