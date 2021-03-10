function PropertyValue = SRD_LinkGet_PropertyByName(link_array, link_name, PropertyName)
    for link_idx = 1:length(link_array)
        link = link_array(link_idx);
        if strcmp(link_name, link.Name)
            PropertyValue = link.(PropertyName);
            break
        end    
    end
end