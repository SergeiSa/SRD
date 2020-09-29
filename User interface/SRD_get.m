function object_to_get = SRD_get(object_name, CustomPath)

if nargin < 2
    CustomPath = 'datafile_SRD_Objects.mat';
end

if exist(CustomPath, 'file') == 2
    temp = load(CustomPath);
    SRD_container = temp.SRD_container;
else
    error(['SRD does not yet have objects stored in ', CustomPath]);
end

object_to_get = SRD_container(object_name);

if isa(object_to_get, 'SRDHandler')
    if object_to_get.SerializationPrepNeeded
        object_to_get.PostSerializationPrepFunction(object_to_get);
    end
end
end