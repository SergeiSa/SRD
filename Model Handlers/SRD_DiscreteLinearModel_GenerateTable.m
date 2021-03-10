function [Ad_table, Bd_table, cd_table] = SRD_DiscreteLinearModel_GenerateTable(A_table, B_table, c_table, TimeTable)

Count = length(TimeTable);
n = size(A_table, 2);
m = size(B_table, 2);

Ad_table = zeros(n, n, Count);
Bd_table = zeros(n, m, Count);
cd_table = zeros(n,    Count);

for i = 1:Count
    
    index = min((i+1), Count);
    dt = TimeTable(index) - TimeTable(index-1);
    
    Ad_table(:, :, i) = eye(n) + dt*A_table(:, :, i);
    Bd_table(:, :, i) =          dt*B_table(:, :, i);
    cd_table(:, i) =          dt*c_table(:, i);
        
end

end