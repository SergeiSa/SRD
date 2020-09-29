function SRD_flush(CustomPath)

if nargin < 1
    CustomPath = 'datafile_SRD_Objects.mat';
end

if exist(CustomPath, 'file') == 2
    delete(CustomPath);
end
end