function ode_fnc_handle = SRD_get_ode_fnc_from_ClosedLoopLinearSystem(AA_table, cc_table, time_table)

ode_fnc_handle = @(t, x) ode_fnc(t, x, ...
    AA_table, ...
    cc_table, ...
    time_table);


    function dx = ode_fnc(t, x, ...
            AA_table, cc_table, time_table)
        
        [~, closest_index] = max( time_table(time_table <= t) );
        
        AA = AA_table(:, :, closest_index);
        cc = cc_table(:, closest_index);
        
        dx = AA*x + cc;
    end

end