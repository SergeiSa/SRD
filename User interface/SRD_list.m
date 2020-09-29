function SRD_list(CustomPath)

if nargin < 1
    CustomPath = 'datafile_SRD_Objects.mat';
end

if exist(CustomPath, 'file') == 2
    temp = load(CustomPath);
    SRD_container = temp.SRD_container;
end

keys = SRD_container.keys;

disp(' ')
disp('SRD stores objects:')
for i = 1:length(keys)
    disp(keys{i})
end
end