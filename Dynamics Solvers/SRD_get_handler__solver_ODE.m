function Handler_solver_Taylor = SRD_get_handler__solver_ODE(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__solver_ImplicitTaylor';
Parser.addOptional('Handler_State', []);
Parser.addOptional('Handler_Controller', []);
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_Simulation', []);

Parser.parse(varargin{:});

Handler_solver_Taylor = SRDHandler_Solver;

Handler_solver_Taylor.Update = @() Update(...
    Parser.Results.Handler_State, ...
    Parser.Results.Handler_Controller, ...
    Parser.Results.Handler_dynamics_generalized_coordinates_model, ...
    Parser.Results.Handler_Simulation);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_solver_Taylor.SerializationPrepNeeded = true;
Handler_solver_Taylor.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save a solver; create a new one on the fly instead')
    end


    function Update(Handler_State, Handler_Controller, ...
            Handler_dynamics_generalized_coordinates_model, Handler_Simulation)
        
        n = Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
        
        dt = Handler_Simulation.TimeLog(Handler_Simulation.CurrentIndex + 1) - Handler_Simulation.TimeLog(Handler_Simulation.CurrentIndex);
        
        q0 = Handler_State.q;
        v0 = Handler_State.v;
        u = Handler_Controller.u;
        
        %[~, tape] = ode45(@ode_func, [0 dt], [q0; v0]);
        [~, tape] = ode15s(@ode_func, [0 dt], [q0; v0]);
        
        
        Handler_State.q = reshape( tape(end, 1:n),       [], 1);
        Handler_State.v = reshape( tape(end, (n+1):2*n), [], 1);
        Handler_State.a = [];
        
        function dz = ode_func(~, z)
            new_q = z(1:n);
            new_v = z((n+1):end);
            
            H = Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(new_q);
            T = Handler_dynamics_generalized_coordinates_model.get_control_map(new_q);
            c = Handler_dynamics_generalized_coordinates_model.get_bais_vector(new_q, new_v);
        
            new_a = H \ (T*u - c);
            %new_a = pinv(H) * (T*u - c);
            
            dz = [new_v; new_a];
        end
    end

end