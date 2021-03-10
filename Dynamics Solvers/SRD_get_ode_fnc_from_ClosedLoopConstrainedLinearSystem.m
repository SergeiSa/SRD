function ode_fnc_handle = SRD_get_ode_fnc_from_ClosedLoopConstrainedLinearSystem...
    (AA_table, cc_table, G_table, F_table, time_table)

ode_fnc_handle = @(t, x) ode_fnc(t, x, ...
    AA_table, ...
    cc_table, ...
    G_table, ...
    F_table, ...
    time_table);


    function dx = ode_fnc(t, x, ...
            AA_table, cc_table, G_table, F_table, time_table)
        
        [~, closest_index] = max( time_table(time_table <= t) );
        
        n = size(AA_table, 1);
        k = size(F_table,  2);
        
        AA = AA_table(:, :, closest_index);
        cc = cc_table(:,    closest_index);
        G  = G_table(:,  :, closest_index);
        F  = F_table(:,  :, closest_index);
        
        M = [eye(n), -F;
             G,       zeros(2*k, k)];
        var = pinv( M ) * [(AA*x + cc); zeros(2*k, 1)];
        
        dx = var(1:n);
    end

end