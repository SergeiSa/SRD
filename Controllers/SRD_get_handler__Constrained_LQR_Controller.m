function Handler_LQR = SRD_get_handler__Constrained_LQR_Controller(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__LQR_Controller';
Parser.addOptional('Handler_State', []);
Parser.addOptional('Handler_State_StateSpace', []);
Parser.addOptional('Handler_ControlInput_StateSpace', []);
Parser.addOptional('Handler_dynamics_Linearized_Model', []);
Parser.addOptional('Handler_Constraints_Model', []);
Parser.addOptional('Handler_Time', []);
Parser.addOptional('Handler_InverseDynamics', []);
Parser.addOptional('Q', []);
Parser.addOptional('R', []);

Parser.parse(varargin{:});

Handler_LQR = SRDHandler_Controller;

Handler_LQR.Update = @() Update(...
    Handler_LQR, ...
    Parser.Results.Handler_State, ...
    Parser.Results.Handler_State_StateSpace, ...
    Parser.Results.Handler_ControlInput_StateSpace, ...
    Parser.Results.Handler_dynamics_Linearized_Model, ...
    Parser.Results.Handler_Time, ...
    Parser.Results.Handler_InverseDynamics, ...
    Parser.Results.Handler_Constraints_Model, ...
    Parser.Results.Q, ...
    Parser.Results.R);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_LQR.SerializationPrepNeeded = true;
Handler_LQR.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_ComputedTorqueController; create a new one on the fly instead')
    end


    function Update(Handler_LQR, ...
            Handler_State, ...
            Handler_State_StateSpace, ...
            Handler_ControlInput_StateSpace, ...
            Handler_dynamics_Linearized_Model, ...
            Handler_Time, ...
            Handler_InverseDynamics, ...
            Handler_Constraints_Model, ...
            Q, R)
        
        t = Handler_Time.CurrentTime;
        
        desired = Handler_ControlInput_StateSpace.get_x_dx(t);
        desired_x =  desired(:, 1);
        
        q = Handler_State.q;
        v = Handler_State.v;
        
        A = Handler_dynamics_Linearized_Model.get_A();
        B = Handler_dynamics_Linearized_Model.get_B();
        
        F  = Handler_Constraints_Model.get_Jacobian(q);
        dF = Handler_Constraints_Model.get_Jacobian_derivative(q, v);
        
        G = [F, dF];
        
        N = null(G);
        
        An = N'*A*N;
        Bn = N'*B;
        Qn = N'*Q*N;
        
        Kn = lqr(An, Bn, Qn, R);
        K = Kn*N';
        
        e  = reshape( ( Handler_State_StateSpace.x - desired_x), [], 1);
        
        u_FB = -K*e;
        
        u_FF = Handler_InverseDynamics.u;
        
        Handler_LQR.u = u_FB + u_FF;
    end

end