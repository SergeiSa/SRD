function found_link = SRD_LinkGet(link_array, link_name)
    for link_idx = 1:length(link_array)
        link = link_array(link_idx);
        if strcmp(link_name,link.Name)
            found_link = link;
            break
        end    
    end
end