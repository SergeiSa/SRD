%Continuous-time LQR controller (LTI)
function Handler_LQR = SRD_get_handler__LQR_Controller(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__LQR_Controller';
Parser.addOptional('Handler_State_StateSpace', []);
Parser.addOptional('Handler_ControlInput_StateSpace', []);
Parser.addOptional('Handler_dynamics_Linearized_Model', []);
Parser.addOptional('Handler_Time', []);
Parser.addOptional('Handler_InverseDynamics', []);
Parser.addOptional('Q', []);
Parser.addOptional('R', []);
Parser.addOptional('ToLog', true);

Parser.parse(varargin{:});

Handler_LQR = SRDHandler_Controller;
Handler_LQR.ToLog = Parser.Results.ToLog;

Handler_LQR.Update = @() Update(...
    Handler_LQR, ...
    Parser.Results.Handler_State_StateSpace, ...
    Parser.Results.Handler_ControlInput_StateSpace, ...
    Parser.Results.Handler_dynamics_Linearized_Model, ...
    Parser.Results.Handler_Time, ...
    Parser.Results.Handler_InverseDynamics, ...
    Parser.Results.Q, ...
    Parser.Results.R);


    %this function needs to be called every time one needs teh controller
    %to update its control law; typically it is called once on every
    %simulation step.
    function Update(Handler_LQR, ...
            Handler_State_StateSpace, ...
            Handler_ControlInput_StateSpace, ...
            Handler_dynamics_Linearized_Model, ...
            Handler_Time, ...
            Handler_InverseDynamics, ...
            Q, R)
        
        t = Handler_Time.CurrentTime;
        
        desired = Handler_ControlInput_StateSpace.get_x_dx(t);
        desired_x =  desired(:, 1);
        
        A = Handler_dynamics_Linearized_Model.get_A();
        B = Handler_dynamics_Linearized_Model.get_B();
        
        K = lqr(A, B, Q, R);
        
        e  = reshape( ( Handler_State_StateSpace.x - desired_x), [], 1);
        
        u_FB = -K*e;
        
        u_FF = Handler_InverseDynamics.u;
        
        Handler_LQR.u = u_FB + u_FF;
        
        %logging
        if Handler_LQR.ToLog
            Handler_LQR.control_law.K  = K;
            Handler_LQR.control_law.e  = e;
            Handler_LQR.control_law.u_FB  = u_FB;
            Handler_LQR.control_law.u_FF  = u_FF;
        end
    end

end