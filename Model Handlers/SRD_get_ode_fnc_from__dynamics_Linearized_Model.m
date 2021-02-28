function ode_fnc_handle = SRD_get_ode_fnc_from__dynamics_Linearized_Model(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_get_ode_fnc_from__dynamics_Linearized_Model';
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_dynamics_Linearized_Model', []);
Parser.addOptional('K_table', []);
Parser.addOptional('x_table', []);
Parser.addOptional('u_table', []);
Parser.addOptional('time_table', []);

Parser.parse(varargin{:});


ode_fnc_handle = @(t, x) ode_fnc(t, x, ...
    Parser.Results.Handler_dynamics_generalized_coordinates_model, ...
    Parser.Results.Handler_dynamics_Linearized_Model, ...
    Parser.Results.K_table, ...
    Parser.Results.x_table, ...
    Parser.Results.u_table, ...
    Parser.Results.time_table);


    function dx = ode_fnc(t, x, ...
            Handler_dynamics_generalized_coordinates_model, ...
            Handler_dynamics_Linearized_Model, ...
            K_table, x_table, u_table, time_table)
        
        [~, closest_index] = max( time_table(time_table <= t) );
        
        n = Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
        
        K = K_table(:, :, closest_index);
        x_desired = x_table(:, closest_index);
        u_desired = u_table(:, closest_index);
        
        u = -K*(x - x_desired) + u_desired;
        
        q = x(1:n);
        v = x((n+1):n);
        
        q_desired = x_desired(1:n);
        v_desired = x_desired((n+1):(2*n));
        
        iH = Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix_inverse(q);
        
        iH_desired = Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix_inverse(q_desired);
        c_desired = Handler_dynamics_generalized_coordinates_model.get_bais_vector(q_desired, v_desired);
        T_desired = Handler_dynamics_generalized_coordinates_model.get_control_map(q_desired);
        
        a_desired = iH_desired*(T_desired*u_desired - c_desired);
        
        A =  Handler_dynamics_Linearized_Model.get_A(q, v, u, iH);     
        B =  Handler_dynamics_Linearized_Model.get_B(q, u,    iH);
        
        f0 = [v_desired; a_desired];
        dx = f0 + A * (x - x_desired) + B * (u - u_desired);
    end

end