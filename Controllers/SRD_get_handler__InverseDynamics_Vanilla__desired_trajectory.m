function Handler_InverseDynamics = SRD_get_handler__InverseDynamics_Vanilla__desired_trajectory(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__InverseDynamics_Vanilla';
Parser.addOptional('Handler_ControlInput', []);
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_Time', []);

Parser.parse(varargin{:});

Handler_InverseDynamics = SRDHandler_Controller;

Handler_InverseDynamics.Update = @() Update(...
    Handler_InverseDynamics, ...
    Parser.Results.Handler_ControlInput, ...
    Parser.Results.Handler_dynamics_generalized_coordinates_model, ...
    Parser.Results.Handler_Time);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_InverseDynamics.SerializationPrepNeeded = true;
Handler_InverseDynamics.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_InverseDynamics; create a new one on the fly instead')
    end


    function Update(Handler_InverseDynamics, Handler_ControlInput, ...
            Handler_dynamics_generalized_coordinates_model, Handler_Time)
        
        t = Handler_Time.CurrentTime;
        
        desired = Handler_ControlInput.get_position_velocity_acceleration(t);
        desired_q = desired(:, 1);
        desired_v = desired(:, 2);
        desired_a = desired(:, 3);
        
        H = Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(desired_q);
        T = Handler_dynamics_generalized_coordinates_model.get_control_map(desired_q);
        c = Handler_dynamics_generalized_coordinates_model.get_bais_vector(desired_q, desired_v);
       
        u = pinv(T)* (H*desired_a + c);
        
        Handler_InverseDynamics.u = u;
    end

end