function SRD_LinkSet_Color(link_array, link_name, Color)
    for link_idx = 1:length(link_array)
        link = link_array(link_idx);
        if strcmp(link_name,link.Name)
            link.Color = Color;
            break
        end    
    end
end