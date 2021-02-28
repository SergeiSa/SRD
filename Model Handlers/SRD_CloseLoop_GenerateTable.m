function [AA_table, cc_table] = SRD_CloseLoop_GenerateTable(A_table, B_table, c_table, K_table, x_table, u_table)

Count = size(A_table, 3);
n = size(A_table, 2);
% m = size(B_table, 2);

AA_table = zeros(n, n, Count);
cc_table = zeros(n, Count);

for i = 1:Count
    
    AA_table(:, :, i) = A_table(:, :, i) - B_table(:, :, i)*K_table(:, :, i);
    
    cc_table(:, i) = B_table(:, :, i) * (K_table(:, :, i) * x_table(:, i) + u_table(:, i)) + c_table(:, i);
end

end