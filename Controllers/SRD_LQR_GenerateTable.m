function K_table = SRD_LQR_GenerateTable(A_table, B_table, Q_table, R_table)

Count = size(A_table, 3);
n = size(A_table, 2);
m = size(B_table, 2);

K_table = zeros(m, n, Count);

for i = 1:Count
    
    K_table(:, :, i) = lqr(A_table(:, :, i), B_table(:, :, i), ...
                           Q_table(:, :, i), R_table(:, :, i));
end

end