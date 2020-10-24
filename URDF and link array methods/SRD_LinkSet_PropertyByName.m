function SRD_LinkSet_PropertyByName(link_array, link_name, PropertyName, PropertyValue)
    for link_idx = 1:length(link_array)
        link = link_array(link_idx);
        if strcmp(link_name, link.Name)
            link.(PropertyName) = PropertyValue;
            break
        end    
    end
end