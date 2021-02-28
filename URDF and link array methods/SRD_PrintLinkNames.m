function SRD_PrintLinkNames(link_array)
disp('* Links have names:')
    for link_idx = 1:length(link_array)
        disp(link_array(link_idx).Name);
    end
end