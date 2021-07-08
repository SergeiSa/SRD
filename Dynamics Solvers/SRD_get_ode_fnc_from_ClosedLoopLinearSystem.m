function ode_fnc_handle = SRD_get_ode_fnc_from_ClosedLoopLinearSystem(AA_table, cc_table, time_table, Period)

if nargin < 4
    Period = 0;
end
    

ode_fnc_handle = @(t, x) ode_fnc(t, x, ...
    AA_table, ...
    cc_table, ...
    time_table, ...
    Period);


    function dx = ode_fnc(t, x, ...
            AA_table, cc_table, time_table, Period)
        
        if Period > 0
            tau = mod(t, Period);
        else
            tau = t;
        end
        
        [~, closest_index] = max( time_table(time_table <= tau) );
        
        AA = AA_table(:, :, closest_index);
        cc = cc_table(:, closest_index);
        
        dx = AA*x + cc;
    end

end