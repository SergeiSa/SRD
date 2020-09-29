function SRD_save(object_to_save, object_name, CustomPath)

if nargin < 3
    CustomPath = 'datafile_SRD_Objects.mat';
end

if exist(CustomPath, 'file') == 2
    temp = load(CustomPath);
    SRD_container = temp.SRD_container;
else
    SRD_container = containers.Map;
end

SRD_container(object_name) = object_to_save;

save(CustomPath, 'SRD_container');

if isa(object_to_save, 'SRDHandler')
    if object_to_save.SerializationPrepNeeded
        object_to_save.PreSerializationPrepFunction(object_to_save);
    end
end

end