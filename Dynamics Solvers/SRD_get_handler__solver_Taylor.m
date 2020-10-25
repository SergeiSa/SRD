function Handler_solver_Taylor = SRD_get_handler__solver_Taylor(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__solver_Taylor';
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
        error('do not attempt to save Handler_solver_Taylor; create a new one on the fly instead')
    end


    function Update(Handler_State, Handler_Controller, ...
            Handler_dynamics_generalized_coordinates_model, Handler_Simulation)
        
        dt = Handler_Simulation.TimeLog(Handler_Simulation.CurrentIndex + 1) - Handler_Simulation.TimeLog(Handler_Simulation.CurrentIndex);
        
        q = Handler_State.q;
        v = Handler_State.v;
        
        iH = Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix_inverse(q);
        T = Handler_dynamics_generalized_coordinates_model.get_control_map(q);
        c = Handler_dynamics_generalized_coordinates_model.get_bais_vector(q, v);
        
        u = Handler_Controller.u;
        
        a = iH * (T*u - c);
        %a = pinv(H) * (T*u - c);
        
        v = v + dt * a;
        q = q + dt * v + 0.5 * dt^2 * a;
        
        Handler_State.q = q;
        Handler_State.v = v;
        Handler_State.a = a;
    end

end