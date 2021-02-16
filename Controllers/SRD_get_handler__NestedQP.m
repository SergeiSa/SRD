function Handler_NestedQP = SRD_get_handler__NestedQP(varargin)

Parser = inputParser;
Parser.FunctionName = 'SRD_get_handler__NestedQP_Controller';
Parser.addOptional('Handler_State', []);
Parser.addOptional('Handler_ControlInput', []);
Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
Parser.addOptional('Handler_Simulation', []);
Parser.addOptional('Handler_InnerController', []);
Parser.addOptional('Handler_Constraints_Model', []);


Parser.parse(varargin{:});

Handler_NestedQP = SRDHandler_Controller;

Handler_NestedQP.Update = @() Update(...
    Handler_NestedQP,...
    Parser.Results.Handler_State, ...
    Parser.Results.Handler_ControlInput, ...
    Parser.Results.Handler_dynamics_generalized_coordinates_model, ...
    Parser.Results.Handler_Simulation, ...
    Parser.Results.Handler_InnerController, ...
    Parser.Results.Handler_Constraints_Model);

%implementing serialization for arbitrary cell arrays of handlers seems to
%be more pain than it is worth
Handler_NestedQP.SerializationPrepNeeded = true;
Handler_NestedQP.PreSerializationPrepFunction = @PreSerializationPrepFunction;
    function PreSerializationPrepFunction(~)
        error('do not attempt to save Handler_ComputedTorqueController; create a new one on the fly instead')
    end


    function Update(Handler_NestedQP,...
            Handler_State, Handler_ControlInput, ...
            Handler_dynamics_generalized_coordinates_model,...
            Handler_Simulation,...
            Handler_InnerController, ...
            Handler_Constraints_Model)
        
        t = Handler_Simulation.CurrentTime;
        
        desired   = Handler_ControlInput.get_position_velocity_acceleration(t);
        desired_q = desired(:, 1);
        desired_v = desired(:, 2);
        desired_a = desired(:, 3);
        
        q = Handler_State.q;
        v = Handler_State.v;
        a = Handler_State.a;
        
        

        
        H = Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(Handler_State.q);
        T = Handler_dynamics_generalized_coordinates_model.get_control_map(Handler_State.q);       
        c = Handler_dynamics_generalized_coordinates_model.get_bais_vector(desired_q, desired_v);

        tau = Handler_InnerController.u;
        %  Handler_ComputedTorqueController.u = tau;
        %  Handler_InverseDynamics.u = tau;
        
        F  = Handler_Constraints_Model.get_Jacobian(q);
        dF  =  Handler_Constraints_Model.get_Jacobian_derivative(q, v);
        
        k1   =  size(F,1);

        
        n   =  Handler_dynamics_generalized_coordinates_model.dof_control;
        m   =  size(tau,1);
        
        Aeq = [H             T               F'                  zeros(n, n);...
               F             zeros(k1,n)     zeros(k1,k1)        zeros(k1,n);...
               zeros(n,n)    T               F'                  ones(n, n)];
        
        beq = [H * desired_a+c;...
               F* desired_a+dF*Handler_State.v;...
               tau];
           

        P = blkdiag(ones(n,n), ones(m,m), ones(k1,k1), 10*ones(n, n));
%         P = blkdiag(ones(n*3,n*3),100*ones(k1, k1));
        A=[];
        b= [];

        ub=[];
        lb=[];
        x0=[];
        f=[];
        options = optimset('MaxIter',200, 'Display', 'off');
%         options = optimset('MaxIter',200);
        

        x= quadprog(P,f,A,b,Aeq,beq,lb,ub,x0,options);
        u_optim=x(n+1:n+m);
%         lambda_optim=x(n*2+1:n*2+k1);
        s = x(n+m+k1+1:end);
%         Handler_NestedQP.u = tau+ F'* lambda_optim+s;
        Handler_NestedQP.u = u_optim;
        
       
    end

end