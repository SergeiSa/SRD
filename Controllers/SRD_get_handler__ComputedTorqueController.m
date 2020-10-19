function Handler_ComputedTorqueController = SRD_get_handler__ComputedTorqueController(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__ComputedTorqueController';
Parser.addOptional('Handler_State', []);
Parser.addOptional('Handler_ControlInput', []);
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_Simulation', []);
Parser.addOptional('Handler_InverseDynamics', []);
Parser.addOptional('Kp', []);
Parser.addOptional('Kd', []);

Parser.parse(varargin{:});

Handler_ComputedTorqueController = SRDHandler_Controller;

Handler_ComputedTorqueController.Update = @() Update(...
    Handler_ComputedTorqueController, ...
    Parser.Results.Handler_State, ...
    Parser.Results.Handler_ControlInput, ...
    Parser.Results.Handler_dynamics_generalized_coordinates_model, ...
    Parser.Results.Handler_Simulation, ...
    Parser.Results.Handler_InverseDynamics, ...
    Parser.Results.Kp, ...
    Parser.Results.Kd);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_ComputedTorqueController.SerializationPrepNeeded = true;
Handler_ComputedTorqueController.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_ComputedTorqueController; create a new one on the fly instead')
    end


    function Update(Handler_ComputedTorqueController, Handler_State, Handler_ControlInput, ...
            Handler_dynamics_generalized_coordinates_model, Handler_Simulation, Handler_InverseDynamics, ...
            Kp, Kd)
        
        t = Handler_Simulation.CurrentTime;
        
        desired = Handler_ControlInput.get_position_velocity_acceleration(t);
        desired_q = desired(:, 1);
        desired_v = desired(:, 2);
        
        H = Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(Handler_State.q);
        T = Handler_dynamics_generalized_coordinates_model.get_control_map(Handler_State.q);
        
        e  = reshape( (desired_q - Handler_State.q), [], 1);
        de = reshape( (desired_v - Handler_State.v), [], 1);
        
        u_FB = pinv(T)*(H*(Kp*e + Kd*de));
        
        u_FF = Handler_InverseDynamics.u;
        
        Handler_ComputedTorqueController.u = u_FB + u_FF;
%         Handler_ComputedTorqueController.u = u_FB;
           
    end

end