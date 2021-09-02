function SRD_remove(object_name, CustomPath)

if nargin < 3
    CustomPath = 'datafile_SRD_Objects.mat';
end

if exist(CustomPath, 'file') == 2
    temp = load(CustomPath);
    SRD_container = temp.SRD_container;
else
    SRD_container = containers.Map;
end

remove(SRD_container, object_name);

save(CustomPath, 'SRD_container');

end