function SRD_LinkSet_UsedGenCoordinates(link_array, link_name, gen_coords)
    for link_idx = 1:length(link_array)
        link = link_array(link_idx);
        if strcmp(link_name,link.Name)
            link.SetUsedGenCoordinates(gen_coords);
            break
        end    
    end
end