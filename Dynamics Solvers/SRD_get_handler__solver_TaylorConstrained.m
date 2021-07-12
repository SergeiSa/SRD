function Handler_solver_TaylorConstrained = SRD_get_handler__solver_TaylorConstrained(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__solver_Taylor';
Parser.addOptional('Handler_State', []);
Parser.addOptional('Handler_Controller', []);
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_Time', []);
Parser.addOptional('Handler_Constraints_Model', []);

Parser.parse(varargin{:});

Handler_solver_TaylorConstrained = SRDHandler_Solver;

Handler_solver_TaylorConstrained.Update = @() Update(...
    Parser.Results.Handler_State, ...
    Parser.Results.Handler_Controller, ...
    Parser.Results.Handler_dynamics_generalized_coordinates_model, ...
    Parser.Results.Handler_Time, ...
    Parser.Results.Handler_Constraints_Model);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_solver_TaylorConstrained.SerializationPrepNeeded = true;
Handler_solver_TaylorConstrained.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_solver_Taylor; create a new one on the fly instead')
    end


    function Update(Handler_State, Handler_Controller, ...
            Handler_dynamics_generalized_coordinates_model, Handler_Time, Handler_Constraints_Model)
        
        dt = Handler_Time.TimeLog(Handler_Time.CurrentIndex + 1) - Handler_Time.TimeLog(Handler_Time.CurrentIndex);
        
        q = Handler_State.q;
        v = Handler_State.v;
        u = Handler_Controller.u;
        
        n = Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
        k = Handler_Constraints_Model.dof_Constraint;
        
        H = Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(q);
        T = Handler_dynamics_generalized_coordinates_model.get_control_map(q);
        c = Handler_dynamics_generalized_coordinates_model.get_bias_vector(q, v);
        
        F  = Handler_Constraints_Model.get_Jacobian(q);
        dF = Handler_Constraints_Model.get_Jacobian_derivative(q, v);
        
        
        M = [H, -F';
             F,  zeros(k, k)];
        
        vec = pinv(M) * [(T*u - c); -dF*v];
         
        
        a = vec(1:n);
        
        v = v + dt * a;
        q = q + dt * v + 0.5 * dt^2 * a;
        
        Handler_State.q = q;
        Handler_State.v = v;
        Handler_State.a = a;
    end

end