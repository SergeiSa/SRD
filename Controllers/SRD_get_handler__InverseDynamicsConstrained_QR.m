%Mistry, M., Buchli, J. and Schaal, S., 2010, May. Inverse dynamics control of floating base 
%systems using orthogonal decomposition. In 2010 IEEE international conference on robotics 
%and automation (pp. 3406-3412). IEEE.
function Handler_InverseDynamics = SRD_get_handler__InverseDynamicsConstrained_QR(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__InverseDynamicsConstrained_QR';
Parser.addOptional('Handler_ControlInput', []);
Parser.addOptional('Handler_Constraints_Model', []);
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_Simulation', []);

Parser.parse(varargin{:});

Handler_InverseDynamics = SRDHandler_Controller;

Handler_InverseDynamics.Update = @() Update(...
    Handler_InverseDynamics, ...
    Parser.Results.Handler_ControlInput, ...
    Parser.Results.Handler_dynamics_generalized_coordinates_model, ...
    Parser.Results.Handler_Constraints_Model, ...
    Parser.Results.Handler_Simulation);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_InverseDynamics.SerializationPrepNeeded = true;
Handler_InverseDynamics.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_InverseDynamics; create a new one on the fly instead')
    end


    function Update(Handler_InverseDynamics, Handler_ControlInput, ...
            Handler_dynamics_generalized_coordinates_model, ...
            Handler_Constraints_Model, ...
            Handler_Simulation)
        
        t = Handler_Simulation.CurrentTime;
        
        n = Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
        k = Handler_Constraints_Model.dof_Constraint;

        desired = Handler_ControlInput.get_position_velocity_acceleration(t);
        desired_q = desired(:, 1);
        desired_v = desired(:, 2);
        desired_a = desired(:, 3);
        
        H = Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(desired_q);
        T = Handler_dynamics_generalized_coordinates_model.get_control_map(desired_q);
        c = Handler_dynamics_generalized_coordinates_model.get_bias_vector(desired_q, desired_v);
        
        F = Handler_Constraints_Model.get_Jacobian(desired_q);
        
        F = orth(F');
        F = F';
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % %
        
        [Q, R] = qr(F');
        R_c = R(1:k, :);
        
        I = eye(n);
        I_c = I(1:k, :);
        I_u = I((k+1):end, :);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % %
        
        u_FF = pinv(I_u * Q' * T) * I_u * Q' * (H*desired_a + c);
        
        lambda = pinv(R_c) * I_c * Q' * (H*desired_a + c - T*u_FF);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % %
        
        Handler_InverseDynamics.u = u_FF;
        
        Handler_InverseDynamics.State.lambda = lambda;
    end

end