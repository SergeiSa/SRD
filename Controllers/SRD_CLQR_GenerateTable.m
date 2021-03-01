function K_table = SRD_CLQR_GenerateTable(A_table, B_table, Q_table, R_table, N_table)

Count = size(A_table, 3);
n = size(A_table, 2);
m = size(B_table, 2);

K_table = zeros(m, n, Count);

for i = 1:Count
    
    N = N_table(:, :, i);
    
    K_table(:, :, i) = lqr(N' * A_table(:, :, i)*N, N' * B_table(:, :, i), ...
                           N' * Q_table(:, :, i)*N, R_table(:, :, i)) * N';
end

end