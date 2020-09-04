%This class generates equations for kinematics and dynamics
%last update 19.10.17
classdef SRDControl < SRDSimulation
    properties
        
        InverseDynamics = [];
        %this is a function handle used to compute inverse dynamics. Use 
        %method .GetInverseDynamics to get an appropriate funtion
        
        use_pinv_on_control_maps = false;
        %if true, matrix B in H*a + c = B*u + F'l eq. will be inverted with
        %pinv instead of \
        
        Math;
    end
    methods
        % class constructor
        function obj = SRDControl(LinkArray)
            obj = obj@SRDSimulation(LinkArray);
        end
        
        % this function can be called before simulation, it sets up
        % .ForcesForComputedTorqueController property
        function Initialization(obj)
            Initialization@SRDSimulation(obj);
            
            obj.InverseDynamics = obj.GetInverseDynamics('Inverse Dynamics');
            
            obj.Math = MathClass();
        end
        
        %This function provides a collection of tester control input 
        %generators
        %
        %Type = 'ControlInput 0' - a control input generator that always outputs zeroes
        %Type = 'Constant_IC_ControlInput' - a control input generator that
        %always outputs IC values
        %Type = 'Constant_ControlInput' - a control input generator that
        %always outputs set values
        function function_handle = GetPlugInput(obj, Type, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDControl.GetPlugInput';
            Parser.addOptional('ControllerOutputDimentions', 1);
            Parser.addOptional('value_q', obj.IC.q, @(x) size(x, 1)==obj.dof);
            Parser.addOptional('number_of_constraints', []);
            Parser.parse(varargin{:});
            
            function [desired_q, desired_v, desired_a] = ControlInput0(~)
                desired_q = zeros(obj.dof, 1);
                desired_v = zeros(obj.dof, 1);
                desired_a = zeros(obj.dof, 1);
            end
            
            function [desired_q, desired_v, desired_a] = Constant_IC_ControlInput(~)
                desired_q = obj.IC.q;
                desired_v = zeros(obj.dof, 1);
                desired_a = zeros(obj.dof, 1);
            end
            
            function [desired_q, desired_v, desired_a] = Constant_ControlInput(~)
                desired_q = Parser.Results.value_q;
                desired_v = zeros(obj.dof, 1);
                desired_a = zeros(obj.dof, 1);
            end
            
            function [desired_lambda] = zero_desired_lambda(~)
                if ~isempty(Parser.Results.number_of_constraints)
                    k = Parser.Results.number_of_constraints;
                else
                    k = obj.Constraint_dof;
                end
                desired_lambda = zeros(k, 1);
            end

            switch Type
                case 'ControlInput 0'
                    function_handle = @ControlInput0;
                case 'Constant_IC_ControlInput'
                    function_handle = @Constant_IC_ControlInput;
                case 'Constant_ControlInput'
                    function_handle = @Constant_ControlInput;
                case 'zero_desired_lambda'
                    function_handle = @zero_desired_lambda;
                otherwise
                    error('Invalid plug requested');
            end 
        end
        
        %This function provides a collection of tester controllers and
        %control input generators
        %
        %Type = 'Controller 0' - a controller that always outputs zeroes
        function ControllerWrapper = GetPlugController(obj, Type, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDControl.GetPlugController';
            Parser.addOptional('ControllerOutputDimentions', 1);
            Parser.addOptional('value_q', obj.IC.q, @(x) size(x, 1)==obj.dof);
            Parser.parse(varargin{:});
            
            ControllerWrapper = SRDControllerWrapper();
            
            function Controller0(~, ~)
                ControllerWrapper.u = zeros(Parser.Results.ControllerOutputDimentions, 1);
            end

            switch Type
                case 'Controller 0'
                    ControllerWrapper.Controller = @Controller0;
                otherwise
                    error('Invalid controller type');
            end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Controller collection
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % This function returns a function handle - a controller. The
        % controller will be compatible with Simulation method of
        % SRDSimulation class. The input of the controller is a standart
        % SRD controller input structure with fields, including vectors
        % of generalized coordinates, velocities, 
        % the desired values for coordinates, velocities and accelerations. 
        % To chose which controller you get, pass its name in Type 
        % parameter. Kp and Kd are gain matrices. For adaptive controllers 
        % diagonal matrices are suggested. 
        % For details see Spong, Ortega 1989 "Adaptive motion control of 
        % rigid robots: A tutorial"
        function ControllerWrapper = GetPDcontroller(obj, Type, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDControl.GetPDcontroller';
            Parser.addOptional('Kp', []);
            Parser.addOptional('Kd', []);
            Parser.addOptional('unified_Kp', 1000);
            Parser.addOptional('unified_Kd', 100);
            Parser.addOptional('get_desired_lambda', []); %ComputedTorquePDController_wpinv parameter
            Parser.addOptional('alpha_control', 10^-10); %ComputedTorquePDController_wpinv parameter
            Parser.addOptional('alpha_lambda', 10^-16); %ComputedTorquePDController_wpinv parameter
            Parser.addOptional('WeightedPseudoinverseType', 'Tikhonov regularization'); %ComputedTorquePDController_wpinv parameter
            Parser.parse(varargin{:});
            
            if nargin < 2
                Type = 'PD';
            end
            
            ControllerWrapper = SRDControllerWrapper();
            
            %this function is only needed to avoid replication of this same
            %code in every PD controller
            function [Kp, Kd] = helper_GetGains(dimentions)
                if ~isempty(Parser.Results.Kp)
                    Kp = Parser.Results.Kp;
                else
                    Kp = eye(dimentions)*Parser.Results.unified_Kp;
                end
                if ~isempty(Parser.Results.Kd)
                    Kd = Parser.Results.Kd;
                else
                    Kd = eye(dimentions)*Parser.Results.unified_Kd; 
                end
            end
            
            function PDcontroller(~, ~)
                
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                e = SensorData.desired_q - SensorData.q;
                de = SensorData.desired_v - SensorData.v;
                
                %get PD parameters
                [Kp, Kd] = helper_GetGains(obj.dof);
                
                ControllerWrapper.u = Kp*e + Kd*de;
            end
            
            function PD_InverseDynamics(~, ~)
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                e = SensorData.desired_q - SensorData.q;
                de = SensorData.desired_v - SensorData.v;
                
                %get PD parameters
                [Kp, Kd] = helper_GetGains(obj.dof);
                
                InverseDynamicsOutput = obj.InverseDynamics(SensorData);

                ControllerWrapper.u = Kp*e + Kd*de + InverseDynamicsOutput.u0;
            end
                        
            function VaryingGainsPDcontroller(~, ~)
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                e = SensorData.desired_q - SensorData.q;
                de = SensorData.desired_v - SensorData.v;
                
                JSIM = obj.ModelHandler.get_estimated_JSIM(SensorData.desired_q);
                B = obj.ModelHandler.get_estimated_ControlMap(SensorData.desired_q);
                
                %get PD parameters
                m = size(B, 2);
                [Kp, Kd] = helper_GetGains(m);
                
                if obj.use_pinv_on_control_maps
                    ControllerWrapper.u = pinv(B)*JSIM*(SensorData.desired_a + Kp*e + Kd*de);
                else
                    ControllerWrapper.u = B\JSIM*(SensorData.desired_a + Kp*e + Kd*de);
                end
            end
  
            % ForcesForComutedTorqueController should be a function handle
            % for the function that computes all the forces that
            % should appear as F in the following eq:
            % M = B^-1 * (A*(ddqs + Kd*de + Kp*e) + F)
            function ComputedTorquePDController(~, ~)
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                e = SensorData.desired_q - SensorData.q;
                de = SensorData.desired_v - SensorData.v;
                
                InverseDynamicsOutput = obj.InverseDynamics(SensorData);
                u0 = InverseDynamicsOutput.u0;
                H  = InverseDynamicsOutput.H;
                B  = InverseDynamicsOutput.B;
                
                %get PD parameters
                m = size(B, 2);
                [Kp, Kd] = helper_GetGains(m);
                
                if obj.use_pinv_on_control_maps
                    ControllerWrapper.u = pinv(B)*(H*(Kp*e + Kd*de)) + u0;
                else
                    ControllerWrapper.u = B\(H*(Kp*e + Kd*de)) + u0;
                end
                
                ControllerWrapper.State.u0 = u0;
            end   

            % ForcesForComutedTorqueController should be a function handle
            % for the function that computes all the forces that
            % should appear as c in the following eq:
            % H*ddq + c = B*u + F'*l;
            %
            % the idea is to write the control problem as:
            % B*u + F'*l = H*(ddq_des + Kd*de + Kp*e) + c
            % and then solve for u and l using Tikhonov regulirization or
            % weighted pseudoinverse            
            function ComputedTorquePDController_wpinv(~, ~)
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                e = SensorData.desired_q - SensorData.q;
                de = SensorData.desired_v - SensorData.v;
                
                alpha_control = Parser.Results.alpha_control; 
                alpha_lambda = Parser.Results.alpha_lambda;
                WeightedPseudoinverseType = Parser.Results.WeightedPseudoinverseType;
                
                
                Dynamics = obj.ModelHandler.get_Dynamics(SensorData.q, SensorData.v);
                H = Dynamics.H;
                c = Dynamics.c;
                B = obj.ModelHandler.get_estimated_ControlMap(SensorData.q);
                
                %calculate needed quantities
                F = g_dynamics_LagrangeMultiplier_ConstraintJacobian(SensorData.desired_q);
                
                desired_lambda = Parser.Results.get_desired_lambda(SensorData);
                
                m = size(B, 2); k = size(F, 1);
                
                %get PD parameters
                [Kp, Kd] = helper_GetGains(m);
                
                %find the right hand side of the problem 
                RHS = H*(SensorData.desired_a + Kd*de + Kp*e) + c;
                
                %solve
                P = obj.Math.WeightedPseudoinverse(B, F', alpha_control, alpha_lambda, WeightedPseudoinverseType);
                u_and_lambda = P * (RHS - F'*desired_lambda);
                
                ControllerWrapper.u = u_and_lambda(1:m);
                ControllerWrapper.State.lambda_desired = desired_lambda;
                ControllerWrapper.State.lambda = desired_lambda + u_and_lambda((m+1):(m+k));
            end    
            
            
            %this is a PD control that accompanies inverse dynamics 
            %algorithm proposed in the paper:
            %"Inverse Dynamics Control of Floating Base Systems Using
            %Orthogonal Decomposition", 2010, Mistry et al.
            function Schaal_PD(~, ~)
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                %obj.InverseDynamics = obj.GetInverseDynamics('Schaal Inverse Dynamics');
                InverseDynamicsOutput = obj.InverseDynamics(SensorData);
                
                %load precomputed values
                B = InverseDynamicsOutput.B;
                JSIM = InverseDynamicsOutput.JSIM;
                Forces = InverseDynamicsOutput.Forces;
                
                GeneralizedTorques = InverseDynamicsOutput.GeneralizedTorques;
                S = InverseDynamicsOutput.S;
                Sc = InverseDynamicsOutput.Sc;
                R_unique = InverseDynamicsOutput.Decomposition.R_unique;
                Q_unique = InverseDynamicsOutput.Decomposition.Q_unique;
                
                m = size(B, 2); n = obj.dof; 
                
                e = SensorData.desired_q - SensorData.q;
                de = SensorData.desired_v - SensorData.v;
                
                %get PD parameters
                [Kp, Kd] = helper_GetGains(m);
                
                Torques = GeneralizedTorques + Kp*S*e + Kd*S*de;
                
                if obj.use_pinv_on_control_maps
                    u = pinv(B)*[Torques; zeros(n-m, 1)];
                else
                    u = B \ [Torques; zeros(n-m, 1)];
                end
                
                %calculate reactions
                lambda = pinv(R_unique) * Sc * Q_unique' * (JSIM * SensorData.desired_a + Forces - B*u);
                
                ControllerWrapper.u = u;
                ControllerWrapper.State.lambda = lambda;
                ControllerWrapper.State.InverseDynamicsOutput = InverseDynamicsOutput;
            end      
            
            
            switch Type
                case {'PD', 'PDcontroller'}
                    ControllerWrapper.Controller = @PDcontroller;
                case {'PD+ID', 'PD_InverseDynamics'}
                    ControllerWrapper.Controller = @PD_InverseDynamics;
                case {'Varying gains PD', 'VaryingGainsPDcontroller'}
                    ControllerWrapper.Controller = @VaryingGainsPDcontroller;
                case {'Computed torque PD', 'ComputedTorquePDController'}
                    ControllerWrapper.Controller = @ComputedTorquePDController;
                case {'Computed torque PD wpinv', 'ComputedTorquePDController_wpinv'}
                    ControllerWrapper.Controller = @ComputedTorquePDController_wpinv;
                case {'Schaal PD', 'Schaal_PD'}
                    ControllerWrapper.Controller = @Schaal_PD;
                otherwise
                    error('Invalid controller type');
            end
        end
        
        %Parameters is a structure, it has fields .Q, .R (quadratic cost 
        %matices, see description of lqr controller), it can have
        %field .ILQR_TimeStep which determines how often ILQR will
        %recalculate its gains
        function ControllerWrapper = GetLQRcontroller(obj, Type, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDControl.GetLQRcontroller';
            Parser.addOptional('Q', []);
            Parser.addOptional('R', []);
            Parser.addOptional('unified_Q', 1);
            Parser.addOptional('unified_R', 0.01);
            
            Parser.addOptional('ILQR_TimeStep', []);
            
            Parser.addOptional('get_desired_lambda', []); %ComputedTorquePDController_wpinv parameter
            Parser.addOptional('alpha_control', 10^-10); %ComputedTorquePDController_wpinv parameter
            Parser.addOptional('alpha_lambda', 10^-16); %ComputedTorquePDController_wpinv parameter
            Parser.addOptional('WeightedPseudoinverseType', 'Tikhonov regularization'); 
            %ComputedTorquePDController_wpinv parameter
            Parser.parse(varargin{:});
             
            if nargin < 2
                Type = 'LQR';
            end
            
            ControllerWrapper = SRDControllerWrapper();
            
            %if user didn't provide weight matrices Q and R but
            %provided scalar weights unified_Q and unified_R - use them to
            %calculate Q and R
            if isempty(Parser.Results.Q)
                Q = eye(obj.dof*2) * Parser.Results.unified_Q;
            else
                Q = Parser.Results.Q;
            end
            if isempty(Parser.Results.R)
                R = eye(obj.Control_dof) * Parser.Results.unified_R;
            else
                R = Parser.Results.R;
            end
            
            ControllerWrapper.State.Cost.Q = Q;
            ControllerWrapper.State.Cost.R = R;
            ControllerWrapper.State.LQR = [];
            
            function LQRcontroller(~, ~)
                
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                %we find current desired control action
                InverseDynamicsOutput = obj.InverseDynamics(SensorData);
                u0 = InverseDynamicsOutput.u0;
                
                %we find the time remained till the next LQR gains update
                remainder = 1;
                if ~isempty(Parser.Results.ILQR_TimeStep)
                    remainder = mod(SensorData.t, Parser.Results.ILQR_TimeStep);
                end
                
                %if the update is due, do it
                if isempty(ControllerWrapper.State.LQR) || (remainder == 0)
                    %linearize the model around the current point
                    LinearModel = obj.GetLinearization(SensorData.x0, u0);
                    A = LinearModel.A;
                    B = LinearModel.B;
                    
                    %update LQR gains
                    [K, S] = lqr(A, B, Q, R);
                    ControllerWrapper.State.LQR.K = K;
                    ControllerWrapper.State.LQR.S = S;
                end
                    
                e = SensorData.x - SensorData.x0;
                ControllerWrapper.u = u0 - ControllerWrapper.State.LQR.K*e;
                ControllerWrapper.State.u0 = u0;
            end
            
            %see "Balancing and Walking Using Full Dynamics LQR Control With
            %Contact Constraints" https://arxiv.org/abs/1701.08179
            function ProjectedLQRcontroller(~, ~)
                
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                %we find current desired control action 
                InverseDynamicsOutput = obj.InverseDynamics(SensorData);
                u0 = InverseDynamicsOutput.u0;
                
                %we find the time remained till the next LQR gains update
                remainder = 1;
                if ~isempty(Parser.Results.ILQR_TimeStep)
                    remainder = mod(SensorData.t, Parser.Results.ILQR_TimeStep);
                end
                
                %if the update is due, do it
                if isempty(ControllerWrapper.State.LQR) || (remainder == 0)
                    %linearize the model around the current point
                    LinearModel = obj.GetLinearization(SensorData.x0, u0);
                    A = LinearModel.A;
                    B = LinearModel.B;
                    
                    q = SensorData.x0(1:obj.dof);
                    v = SensorData.x0((obj.dof+1):(2*obj.dof));
                    
                    F = g_dynamics_LagrangeMultiplier_ConstraintJacobian(q);
                    dF = g_dynamics_LagrangeMultiplier_ConstraintSecondJacobian(q, v);
                    
                    JM = [F, zeros(size(F));
                          dF, F];
                      
                    N = null(JM);
                    Am = N'*A*N;
                    Bm = N'*B;
                    Qm = N'*Q*N;
                    
                    %update LQR gains
                    [Km, S] = lqr(Am, Bm, Qm, R);
                    K = Km*N';
                    
                    ControllerWrapper.State.LQR.K = K;
                    ControllerWrapper.State.LQR.S = S;
                end
                    
                e = SensorData.x - SensorData.x0;
                ControllerWrapper.u = u0 - ControllerWrapper.State.LQR.K*e;
                ControllerWrapper.State.u0 = u0;
            end
            
            function LQRcontroller_wpinv(~, ~)

                SensorData = obj.SensorHandler.ReadCurrentData;
                
                %we find current desired control action
                InverseDynamicsOutput = obj.InverseDynamics(SensorData);
                u0 = InverseDynamicsOutput.u0;
                
                %linearize the model around the current point
                LinearModel = obj.GetLinearization(SensorData.x0, u0);
                A = LinearModel.A;
                B = LinearModel.B;
                
                m = size(B, 2);
                
                %we find the time remained till the next LQR gains update
                remainder = 1;
                if ~isempty(Parser.Results.ILQR_TimeStep)
                    remainder = mod(SensorData.t, Parser.Results.ILQR_TimeStep);
                end
                
                %if the update is due, do it
                if isempty(ControllerWrapper.State.LQR) || (remainder == 0)
                    
                    pseudo_B = [zeros(obj.dof); eye(obj.dof)];
                    pseudo_R = eye(obj.dof) * Parser.Results.unified_R;
                    
                    %update LQR gains
                    [K, S] = lqr(A, pseudo_B, Q, pseudo_R);
                    ControllerWrapper.State.LQR.K = K;
                    ControllerWrapper.State.LQR.S = S;
                end
                   
                %find desired lambda
                desired_lambda = Parser.Results.get_desired_lambda(SensorData);
                
                %%%%%%%%%%%%%
                % there are two ways to calculate u and lambda correctly.
                % the trick lies in the way you treat your dynamics eqs.
                % in the linear model they are dx = A*x + B*u + c
                % the lack on inertia matrix on the LHS means that
                % everything on the RHS was multiplied by its inverse.
                % Since calculation of lambda requires F (constrains
                % jacobian) it needs to be multiplied by the inverse of 
                % inertia matrix also. Alternatively, a different dynamics 
                % eqs need to be used: H*ddq + c = B*u

                InverseDynamicsOutput = obj.InverseDynamics(SensorData);
                
                %load precomputed values
                JSIM = InverseDynamicsOutput.JSIM;
                Forces = InverseDynamicsOutput.Forces;
                
                u1 = JSIM * SensorData.desired_a + Forces;
                
                RHS = u1 - ControllerWrapper.State.LQR.K*(SensorData.x - SensorData.x0);

                %find constraint jacobian
                F = g_dynamics_LagrangeMultiplier_ConstraintJacobian(SensorData.desired_q);
                %find control map
                small_B = g_dynamics_ControlMap(SensorData.desired_q);
                
                %calculate weighted pseudoinverse
                P = obj.Math.WeightedPseudoinverse(small_B, F', ...
                    Parser.Results.alpha_control, Parser.Results.alpha_lambda, Parser.Results.WeightedPseudoinverseType);
                u_and_lambda = P * (RHS - F'*desired_lambda);
                
                ControllerWrapper.u = u_and_lambda(1:m);
                ControllerWrapper.State.lambda = desired_lambda + u_and_lambda((m+1):length(u_and_lambda));
                ControllerWrapper.State.lambda_desired = desired_lambda;
                ControllerWrapper.State.u0 = u1;
            end
            
            switch Type
                case {'LQR', 'LQRcontroller'}
                    ControllerWrapper.Controller = @LQRcontroller;
                case {'ProjectedLQR', 'ProjectedLQRcontroller'}
                    ControllerWrapper.Controller = @ProjectedLQRcontroller;
                case {'LQR wpinv', 'LQRcontroller_wpinv'}
                    ControllerWrapper.Controller = @LQRcontroller_wpinv;
                otherwise
                    error('Invalid controller type');
            end            
        end

        %This function provides a Nested Constraints Controller (NCC) that
        %is an upper level controller, build on top of a lower lever one
        %(PD, Computed Torque Controller, LQR, MPC). NCC finds a way to
        %minimize the motor torques (control actions) needed to perform the
        %control task
        function ControllerWrapper = GetNestedConstraintsController(obj, LowLevelController, Type, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDControl.GetNestedConstraintsController';
            Parser.addOptional('U_cost', 1);
            Parser.addOptional('lambda_cost', 0.001);
            Parser.addOptional('error_cost', 0.1);
            Parser.addOptional('slack_cost', 10);
            Parser.addOptional('problem', []);
            Parser.parse(varargin{:});
            
            if nargin < 2
                error('You need to provide a function handle LowLevelController to a controller');
            end
            
            if nargin < 3
                Type = 'NCCviaQP';
            end
            
            ControllerWrapper = SRDControllerWrapper();
            
            problem = Parser.Results.problem;
            problem.solver = 'quadprog';
            
            % passing user-defined options
            if isfield(problem, 'options')
                options = problem.options;
            else
                options = [];
            end
            if isempty(options)
                problem.options = optimoptions('quadprog', 'Display', 'none');
            else
                problem.options = options;
            end
             
            control_dof = obj.Control_dof; %number of available motors
            
            
            
            function NCCviaQP(~, ~)
                
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                %values for eq. of dynamics: H*ddq - c = B*u + F'*lambda;
                B = g_dynamics_ControlMap(SensorData.q);
                F = g_dynamics_LagrangeMultiplier_ConstraintJacobian(SensorData.q);
                
                %number of constraints
                constraint_dof = size(F, 1);
                
                LowLevelController.Controller();
                Q = B * LowLevelController.u;                        

                %QP problem, decision variables x = [u; labda]
                problem.Aeq = [B,  F'];
                problem.beq = Q;     
                           
                problem.H = blkdiag(Parser.Results.U_cost*eye(control_dof), ...
                                    Parser.Results.lambda_cost*eye(constraint_dof));
                           
                [x, fval, exitflag] = quadprog(problem);           
                
                ControllerWrapper.u(:, 1) = x(1:control_dof);
                
                ControllerWrapper.State.lambda(:, 1) = x((control_dof + 1):(size(x, 1)));
                ControllerWrapper.State.fval = fval;
                ControllerWrapper.State.exitflag = exitflag;
                
                ControllerWrapper.State.problem = problem;
            end
            
            %This one works, it seems
            function NQPC(~, ~)
                
                %here we work with eq.:
                % 1) B*u + F'*lambda + s = Q;  <- connection to the inner level controller
                % 2) H*dde + B*u + F'*lambda = H*ddq_desired + c; <- system's dynamics
                % 3) F*dde = F*ddq_desired + dF*dq;   <- constraints
                % s is a slack variable
                
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                Dynamics = obj.ModelHandler.get_Dynamics(SensorData.q, SensorData.v);
                H = Dynamics.H;
                c = Dynamics.c;
                B = obj.ModelHandler.get_estimated_ControlMap(SensorData.q);
                
                F = g_dynamics_LagrangeMultiplier_ConstraintJacobian(SensorData.q);
                dF = g_dynamics_LagrangeMultiplier_ConstraintSecondJacobian(SensorData.q, SensorData.v);
                
                %B = g_dynamics_ControlMap(SensorData.q);
                %JSIM = g_dynamics_JSIM(SensorData.q);
                %c = obj.ModelHandler.get_estimated_ForcesForComputedTorqueController(SensorData.q, SensorData.v);
                
                %number of constraints
                constraint_dof = size(F, 1);
                                
                LowLevelController.Controller();
                Q = B * LowLevelController.u;                                                  
        
                           
                problem.Aeq = [zeros(obj.dof, obj.dof), B, F', eye(obj.dof);
                               H, B, F', zeros(obj.dof, obj.dof);
                               F, zeros(constraint_dof, control_dof), zeros(constraint_dof, constraint_dof), zeros(constraint_dof, obj.dof)];
                problem.beq = [Q;
                               H*SensorData.desired_a + c;
                               F*SensorData.desired_a + dF*SensorData.v];                            
                           
                W1 = Parser.Results.error_cost * eye(obj.dof);
                W2 = Parser.Results.U_cost     * eye(control_dof);
                W3 = Parser.Results.lambda_cost* eye(constraint_dof);
                W4 = Parser.Results.slack_cost * eye(obj.dof);         
                 
                problem.H = blkdiag(W1, W2, W3, W4);
                           
                [x, fval, exitflag] = quadprog(problem);           
                
                
                ControllerWrapper.u(:, 1) = x((obj.dof + 1):(obj.dof + control_dof));
                
                ControllerWrapper.State.lambda(:, 1) = x((obj.dof + control_dof + 1):(obj.dof + control_dof + constraint_dof));
                ControllerWrapper.State.slack(:, 1) = x((obj.dof + control_dof + constraint_dof + 1):(2*obj.dof + control_dof + constraint_dof));
                
                ControllerWrapper.State.fval = fval;
                ControllerWrapper.State.exitflag = exitflag;
                
                ControllerWrapper.State.problem = problem;
            end       
            
            switch Type
                case 'NCCviaQP'
                    ControllerWrapper.Controller = @NCCviaQP;
                case 'NQPC'
                    ControllerWrapper.Controller = @NQPC;
                otherwise
                    error('Invalid controller type');
            end              
        end
        
        %This function provides model predictive controller
        %Parameters is a structure, it has fields .LinearCost in case of linear cost
        %and .Q, .R for quadratic cost. Instead of matrices and vectors
        %user can pass scalars .unified_Q, .unified_R, .unified_LinearCost,
        %they will be used to produce diagonal weight matrices Q and R and
        %a weight vector LinearCost of the appropriate dimentions with the
        %elements equal to the mentioned scalars.
        %Parameters also has a field
        %.MP_TimeStep which determines the prediction horizon for a single
        %step
        %
        %Generated controller will have an input ControllerInput, a structure with fields, including 
        %.ControlInput - a function that provides desired position, velocity and acceleration.
        %.t - current time, .x or .q, .v - current position.
        function ControllerWrapper = GetMPcontroller(obj, Type, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDControl.GetNestedConstraintsController';
            Parser.addOptional('Q', []);
            Parser.addOptional('R', []);
            Parser.addOptional('unified_Q', 1);
            Parser.addOptional('unified_R', 0.01);
            Parser.addOptional('LinearCost', []);
            Parser.addOptional('unified_LinearCost', 0);
            
            Parser.addOptional('MP_PredictionTimeStep', 0.01);
            Parser.addOptional('MP_UpdateTimeStep', 0.01);
            Parser.addOptional('NumberOfPredictionSteps', 4);
            
            Parser.addOptional('quadprog_options', optimoptions('quadprog',...
                        'Algorithm', 'interior-point-convex', 'Display', 'none'));
            Parser.addOptional('UseWarmStart', false);
            Parser.parse(varargin{:});
            
            if nargin < 2
                Type = 'MP';
            end
            
            ControllerWrapper = SRDControllerWrapper();
            
            dt = Parser.Results.MP_PredictionTimeStep;
            LinearCost = Parser.Results.LinearCost;
            Q = Parser.Results.Q;
            R = Parser.Results.R;
            
            function OneStepMPcontroller(~, ~)
                
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                t = SensorData.t;
                remainder = mod(t, Parser.Results.MP_UpdateTimeStep);
                
                %if the update is due, do it
                if isempty(ControllerWrapper.State) || (remainder == 0)
                    
                    %first we find the current desired position and the desired position on the next step
                    [desired_q, desired_v, desired_a] = obj.ControlInput(t);
                    [desired_q1, desired_v1, ~] = obj.ControlInput(t + dt);
                    
                    %we find the point of linearization in state
                    %coordinates and control coordinates
                    ID_input.desired_q = desired_q; ID_input.desired_v = desired_v; ID_input.desired_a = desired_a;
                    InverseDynamicsOutput = obj.InverseDynamics(ID_input);
                    u0 = InverseDynamicsOutput.u0;
                    x0 = [desired_q; desired_v];
                    x1 = [desired_q1; desired_v1];
                    
                    %we linearize, LinearModel has fileds .A, .B, .c
                    LinearModel = obj.GetLinearization(x0, u0);
                    A = LinearModel.A;
                    B = LinearModel.B;
                    c = LinearModel.c;
                    
                    x = SensorData.x;
                    
                    size_of_x = size(A, 2);
                    size_of_u = size(B, 2);
                    
                    %if user didn't provide linear const weights vector
                    %LinearCost, but provided a scalar unified_LinearCost, set
                    %LinearCost to be a vector where each elements is of
                    %unified_LinearCost;
                    if isempty(LinearCost)
                        LinearCost = ones((size_of_x + size_of_u), 1) * Parser.Results.unified_LinearCost;
                    end
                    %if user didn't provide weight matrices Q and R but
                    %provided scalar weights unified_Q and unified_R
                    %instead - use them to calculate Q and R
                    if isempty(Q)
                        if ~isempty(Parser.Results.unified_Q)
                            Q = eye(size_of_x) * Parser.Results.unified_Q;
                        else
                            error('either Q or unified_Q need to be provided as fields of Parameters input');
                        end
                    end
                    if isempty(R)
                        if ~isempty(Parser.Results.unified_R)
                            R = eye(size_of_u) * Parser.Results.unified_R;
                        else
                            error('either R or unified_R need to be provided as fields of Parameters input');
                        end
                    end
                    
                    %an identity matrix of size equal to A
                    Ix = eye(size_of_x);
                    
                    Aeq = [Ix, -dt*B];
                    beq = (Ix + dt*A)*x + dt*c + dt*B*u0 - x1;
                    
                    %var = linprog(LinearCost, [], [], Aeq, beq);
                    
                    H = blkdiag(Q, R);
                    
                    var = quadprog(H, LinearCost, [], [], Aeq, beq, [], [], x, Parser.Results.quadprog_options);
                    
                    ControllerWrapper.u = u0 + var((size_of_x + 1):(size_of_x + size_of_u));
                    
                    ControllerWrapper.State.predicted_x = x1 + var(1:size_of_x);
                    ControllerWrapper.State.u0 = u0;
                end
            end
            
            
            function MPcontroller(~, ~)
                
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                N = Parser.Results.NumberOfPredictionSteps;
                
                t = SensorData.t;
                remainder = mod(t, Parser.Results.MP_UpdateTimeStep);
                
                %if the update is due, do it
                if isempty(ControllerWrapper.State) || (remainder == 0)
                    
                    size_of_x = obj.dof * 2;
                    size_of_u = obj.Control_dof;
                    
                    %first we find the desired position for N+1 steps;
                    desired_q_array = zeros(obj.dof, (N + 1));
                    desired_v_array = zeros(obj.dof, (N + 1));
                    desired_a_array = zeros(obj.dof, (N + 1));
                    for i = 1:(N + 1)
                        [desired_q, desired_v, desired_a] = obj.ControlInput(t + i*dt);
                        
                        desired_q_array(:, i) = desired_q;
                        desired_v_array(:, i) = desired_v;
                        desired_a_array(:, i) = desired_a;
                    end
                    
                    %we find desired control actions for N+1 steps;
                    desired_u_array = zeros(size_of_u, N);
                    for i = 1:Parser.Results.NumberOfPredictionSteps
                        
                        ID_input.desired_q = desired_q_array(:, i);
                        ID_input.desired_v = desired_v_array(:, i);
                        ID_input.desired_a = desired_a_array(:, i);
                        InverseDynamicsOutput = obj.InverseDynamics(ID_input);
                        
                        desired_u_array(:, i) = InverseDynamicsOutput.u0;
                    end
                    
                    %we linearize, LinearModel has fileds .A, .B, .c
                    for i = 1:N
                        LinearModel(i) = obj.GetLinearization([desired_q_array(:, i); desired_v_array(:, i)], ...
                            desired_u_array(:, i));
                    end
                    
                    %an identity matrix of size equal to A
                    Ix = eye(size_of_x);
                    
                    %we fill in the big A and B matrices (the A and B
                    %matrices for the combined system of linear dynamics
                    %eq.), as well as big vector c;
                    big_A_matrix = zeros(size_of_x*N, size_of_x*(N+1));
                    for i = 1:N
                        index = (i - 1)*size_of_x + 1;
                        
                        A = LinearModel(i).A;
                        big_A_matrix(index:(index + size_of_x - 1), index:(index + size_of_x - 1)) = -(Ix + dt*A);
                        big_A_matrix(index:(index + size_of_x - 1), (index + size_of_x):(index + 2*size_of_x - 1)) = Ix;
                    end
                    
                    big_B_matrix = zeros(size_of_x*N, size_of_u*N);
                    for i = 1:N
                        index_horizontal = (i - 1)*size_of_u + 1;
                        index_vertical = (i - 1)*size_of_x + 1;
                        
                        B = LinearModel(i).B;
                        big_B_matrix(index_vertical:(index_vertical + size_of_x - 1), ...
                            index_horizontal:(index_horizontal + size_of_u - 1)) = -dt*B;
                    end
                    
                    big_c_vector = zeros(size_of_x*N, 1);
                    for i = 1:N
                        index = (i - 1)*size_of_x + 1;
                        
                        c = LinearModel(i).c;
                        big_c_vector(index:(index + size_of_x - 1)) = dt*c;
                    end
                    
                    big_desired_x = zeros(size_of_x*(N + 1), 1);
                    for i = 1:(N + 1)
                        index = (i - 1)*size_of_x + 1;
                        
                        x = [desired_q_array(:, i); desired_v_array(:, i)];
                        big_desired_x(index:(index + size_of_x - 1)) = x;
                    end
                    
                    big_desired_u = zeros(size_of_u*N, 1);
                    for i = 1:N
                        index = (i - 1)*size_of_u + 1;
                        
                        big_desired_u(index:(index + size_of_u - 1)) = desired_u_array(:, i);
                    end
                    
                    %write dynamics over all prediction steps as
                    %RHS*e = LHS,
                    %where e is error over state and control actions
                    %e = [x - desired_x; u - desired_u];
                    RHS = big_c_vector - big_A_matrix*big_desired_x - big_B_matrix*big_desired_u;
                    
                    LHS = [big_A_matrix, big_B_matrix];
                    
                    IC_x = SensorData.x;
                    
                    %find desired IC
                    desired_IC_x = [desired_q_array(:, 1); desired_v_array(:, 1)];
                    
                    %add initial conditions;
                    IC_LHS = zeros(size_of_x, size(LHS, 2));
                    IC_LHS(1:size_of_x, 1:size_of_x) = eye(size_of_x);
                    LHS = [LHS; IC_LHS];
                    
                    RHS = [RHS; (IC_x - desired_IC_x)];
                    
                    %if user didn't provide linear const weights vector
                    %LinearCost, but provided a scalar unified_LinearCost, set
                    %LinearCost to be a vector where each elements is of
                    %unified_LinearCost;
                    if isempty(LinearCost)
                        LinearCost = ones((size_of_x*(N + 1) + size_of_u*N), 1) ...
                            * Parser.Results.unified_LinearCost;
                    end
                    %if user didn't provide weight matrices Q and R but
                    %provided scalar weights unified_Q and unified_R
                    %instead - use them to calculate Q and R
                    if isempty(Q)
                        if ~isempty(Parser.Results.unified_Q)
                            Q = eye(size_of_x*(N + 1)) * Parser.Results.unified_Q;
                        else
                            error('either Q or unified_Q need to be provided as fields of Parameters input');
                        end
                    end
                    if isempty(R)
                        if ~isempty(Parser.Results.unified_R)
                            R = eye(size_of_u*N) * Parser.Results.unified_R;
                        else
                            error('either R or unified_R need to be provided as fields of Parameters input');
                        end
                    end
                    
                    %prepare quadratic program
                    Aeq = LHS;
                    beq = RHS;
                    
                    H = blkdiag(Q, R);
                    
                    H = sparse(H);
                    Aeq = sparse(Aeq);
                    
                    %if warm start is requested - attemt to use previous
                    %solution as a starting point
                    if Parser.Results.UseWarmStart
                        if ~isempty(ControllerWrapper.State)
                            PreviousSolution = ControllerWrapper.State.solution;
                        else
                            PreviousSolution = [];
                        end
                    else
                        PreviousSolution = [];
                    end
                    
                    %solve QP
                    solution = quadprog(H, LinearCost, [], [], Aeq, beq, [], [], PreviousSolution, ...
                        Parser.Results.quadprog_options);
                    
                    u0 = desired_u_array(:, 1);
                    ControllerWrapper.u = u0 + ...
                        solution((size_of_x*(N + 1) + 1):(size_of_x*(N + 1) + size_of_u));
                    
                    ControllerWrapper.State.u0 = u0;
                    ControllerWrapper.State.solution = solution;
                end
            end
            
            switch Type
                case {'One step MP', 'OneStepMPcontroller'}
                    ControllerWrapper.Controller = @OneStepMPcontroller;
                case {'MP', 'MPcontroller'}
                    ControllerWrapper.Controller = @MPcontroller;
                otherwise
                    error('Invalid controller type');
            end  
        end
        
        % This function returns a function handle - a controller, which
        % serves as a spring-damper system immitator. The controller
        % is compatible with Simulation method of SRDSimulation class.
        %
        % Inputs:
        %'Kp', 'Kd' - spring and damper coefficient matrices
        %'unified_Kp', 'unified_Kd' - pass scalars to generate Kp and
        % Kd with diagonal matrices with uniform gains
        %'relaxed_q', 'relaxed_v' - valus of the gen. coordinates and
        %velocotoes that correspond to the spring-damper system producing
        %zero action
        function ControllerWrapper = GetSpringImmitator(obj, Type, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDControl.GetSpringImmitator';
            Parser.addOptional('Kp', []);
            Parser.addOptional('Kd', []);
            Parser.addOptional('unified_Kp', 100);
            Parser.addOptional('unified_Kd', 10);
            Parser.addOptional('relaxed_q', zeros(obj.Guess_control_dof, 1));
            Parser.addOptional('relaxed_v', zeros(obj.Guess_control_dof, 1));
            
            Parser.addOptional('ElasticDrive_ActualController', []);
            Parser.addOptional('ElasticDrive_JSIM', eye(obj.Guess_control_dof));
            Parser.addOptional('ElasticDrive_Map',  eye(obj.Guess_control_dof));
            Parser.addOptional('ElasticDrive_mu',   zeros(obj.Guess_control_dof));
            %Parser.addOptional('ElasticDrive_q',  []);
            %Parser.addOptional('ElasticDrive_v',  []);
            
            Parser.parse(varargin{:});
            
            if nargin < 2
                Type = 'Springs';
            end
            
            ControllerWrapper = SRDControllerWrapper();
            ControllerWrapper.State.ElasticDrive.q = Parser.Results.ElasticDrive_Map * obj.IC.q;
            ControllerWrapper.State.ElasticDrive.v = Parser.Results.ElasticDrive_Map * obj.IC.v;
            
            %this function is only needed to avoid replication of this same
            %code in every PD controller
            function [Kp, Kd] = helper_GetGains(dimentions)
                if ~isempty(Parser.Results.Kp)
                    Kp = Parser.Results.Kp;
                else
                    Kp = eye(dimentions)*Parser.Results.unified_Kp;
                end
                if ~isempty(Parser.Results.Kd)
                    Kd = Parser.Results.Kd;
                else
                    Kd = eye(dimentions)*Parser.Results.unified_Kd; 
                end
            end
            
            %puts spring-dampers to the robot's joints. The robot model 
            %does not know about them
            function Springs(~, ~)
                SensorData = obj.SensorHandler.ReadCurrentData_True;
                
                e = Parser.Results.relaxed_q - SensorData.q;
                de = Parser.Results.relaxed_v - SensorData.v;
                [Kp, Kd] = helper_GetGains(obj.dof);
                
                ControllerWrapper.u = Kp*e + Kd*de;
            end
            
            %adds elastic drive (a spring-damper between the motor and the
            %joint)
            function AddElasticDrive(~, ~)
                
                if isempty(Parser.Results.ElasticDrive_ActualController)
                    error(['Elastic drive requires a controller', ...
                           'Use ''ElasticDrive_ActualController'' to pass the controller']);
                else
                    Parser.Results.ElasticDrive_ActualController.Controller();
                end
                u_controller = Parser.Results.ElasticDrive_ActualController.u;
                
                
                SensorData = obj.SensorHandler.ReadCurrentData_True;
                
                [Kp, Kd] = helper_GetGains(length(u_controller));
                q = ControllerWrapper.State.ElasticDrive.q;
                v = ControllerWrapper.State.ElasticDrive.v;
                n = size(q, 1);
                x = [q; v];
                
                q_desired = Parser.Results.ElasticDrive_Map * SensorData.q;
                v_desired = Parser.Results.ElasticDrive_Map * SensorData.v;
                
                u_springs = Kp*(q_desired - q) + Kd*(v_desired - v);
                
                %implicit Euler - start
                dt = obj.TimeStep;
                M = [eye(n), zeros(n);
                     Kd,     Parser.Results.ElasticDrive_JSIM];
                A = [zeros(n), eye(n);
                    -Kp,       -Parser.Results.ElasticDrive_mu];
                b = [zeros(n, 1);
                     u_controller + Kp*q_desired + Kd*v_desired];
                 
                x_updated = pinv(M - dt*A) * (M*x + dt*b);
                
                q = x_updated(1:n, 1);
                v = x_updated((n+1):(2*n), 1);
                %implicit Euler - end
                
                ControllerWrapper.State.ElasticDrive.q = q;
                ControllerWrapper.State.ElasticDrive.v = v;
                
                ControllerWrapper.u = -u_springs;
            end
            
            
            switch Type
                case {'Springs'}
                    ControllerWrapper.Controller = @Springs;
                case {'AddElasticDrive'}
                    ControllerWrapper.Controller = @AddElasticDrive;
                otherwise
                    error('Invalid controller type');
            end
        end
            
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Meta controller functions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %This function is taking a cell array of controllers' function
        %handles as an input and produces a new controller that is summing
        %their outputs
        %
        %Inputs:
        %Controllers - a cell array of controllers
        %Output:
        %ControllerWrapper - a class object
        function ControllerWrapper = SumControllers(~, Controllers)
            
            ControllerWrapper = SRDControllerWrapper();
            
            function NewController(~, ~)
                u = [];
                for i = 1:length(Controllers)
                    Controllers{i}.Controller();
                    if isempty(u)
                        u = Controllers{i}.u;
                    else
                        u = u + Controllers{i}.u;
                    end
                end
                ControllerWrapper.u = u;
            end
            
            ControllerWrapper.Controller = @NewController;
        end
        
        
        %This function provides a wrapper for a controller that
        %additionally limits its output by value Cap;
        %
        %Cap can be a scalar or a vector with tsame dimentions as
        %Controller's Output.u
        function ControllerWrapper = LimitControllerOutput(~, OldController, Cap, LowerBound)
            
            if nargin < 4
                LowerBound = -Cap;
            end
            
            ControllerWrapper = SRDControllerWrapper();
            
            function NewController(~, ~)
                OldController.Controller();
                u = OldController.u;
                
                n = length(u);
                for i = 1:n
                    if isscalar(Cap)
                        if u(i) > Cap
                            u(i) = Cap;
                        end
                        if u(i) < LowerBound
                            u(i) = LowerBound;
                        end
                    else
                        if u(i) > Cap(i)
                            u(i) = Cap(i);
                        end
                        if u(i) < LowerBound(i)
                            u(i) = LowerBound(i);
                        end
                    end
                end
                
                ControllerWrapper.u = u;
            end
                
            ControllerWrapper.Controller = @NewController;
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Other
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %This method returns a function handle for an additive quadratic
        %cost function. If Type is 'Generalized coordinates'  then the
        %structure Coefficients should have the following fields:
        %.Q1 and .Q2 - quadratic matrices of dimentions dof
        %.R - a quadratic matrix of dimentions m, where m is the number of
        %control impunt
        %If Type is 'State Space'  then the
        %structure Coefficients should have teh following fields:
        %.Q - quadratic matrices of dimentions 2*dof
        %.R - a quadratic matrix of dimentions m, where m is the number of
        %control impunt
        function CostFunction = GetQuadraticCostFunction(~, Type, Coefficients)
            
            function Cost = InGeneralizedCoordinates(ControllerInput, ControllerOutput, SolverOutputStructure)
                e =  ControllerInput.desired_q - SolverOutputStructure.q;
                de = ControllerInput.desired_v - SolverOutputStructure.v;
                
                Cost = e'*Coefficients.Q1*e + de'*Coefficients.Q2*de + ControllerOutput.u'*Coefficients.R*ControllerOutput.u;
            end
            
            function Cost = InStateSpace(ControllerInput, ControllerOutput, SolverOutputStructure)
                e = ControllerInput.x0 - SolverOutputStructure.s;
                delta_u = ControllerOutput.State.u0 - ControllerOutput.u;
                
                Cost = e'*Coefficients.Q*e + delta_u'*Coefficients.R*delta_u;
            end
            
            switch Type
                case 'Generalized coordinates'
                    CostFunction = @InGeneralizedCoordinates;
                case 'State Space'
                    CostFunction = @InStateSpace;
                otherwise
                    warning('Invalid cost function type');
            end
        end
        
        % For dynamics in the form:
        % dx/dt = f(x, u)         (1)
        % this function produces a state-space linearization as follows:
        % dx/dt = A*x + B*u + c, where
        % A = df/dx;  B = df/du;  c = f - A*x - B*u;
        % To do that another linearisation is used:  
        % H*dx/dt = rA*x + rB*u + rc (see description of 
        % UpdateLinearizedDynamics method of the SRDControlEquations class  
        % for details)
        % then A = inv(H)*rA, B = inv(H)*rB, c = inv(H)*rc.
        %
        % The output is a structure with filds A, B and c
        %
        % Linearization is does around point x0, u0.
        % Before using, call GenerateLinearizationFunctions or 
        % DoLinearization method of the SRDControlEquations class.
        function LinearModel = GetLinearization(~, x0, u0)
            H = g_dynamics_Linearization_SSIM(x0);
            rA = g_dynamics_Linearization_RHS_A(x0, u0);
            rB = g_dynamics_Linearization_RHS_B(x0);
            rc = g_dynamics_Linearization_RHS_c(x0, u0);
            
            LinearModel.A = H \ rA;
            LinearModel.B = H \ rB;
            LinearModel.c = H \ rc;
            LinearModel.H = H;
        end
        
        % This methods provides a function that computes inverse dynamics
        % control actions
        function controller = GetInverseDynamics(obj, Type, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDControl.GetInverseDynamics';
            Parser.addOptional('get_desired_lambda', []); %ComputedTorquePDController_wpinv parameter
            Parser.addOptional('alpha_control', 10^-10);  %ComputedTorquePDController_wpinv parameter
            Parser.addOptional('alpha_lambda', 10^-16); %ComputedTorquePDController_wpinv parameter
            Parser.addOptional('WeightedPseudoinverseType', 'Tikhonov regularization'); %ComputedTorquePDController_wpinv parameter
            Parser.parse(varargin{:});
            
            if nargin < 2
                Type = 'Inverse Dynamics';
            end
               
            %Inverse dynamics for systems without explicit mechanical
            %constraints
            function Output = InverseDynamics(SensorData)
                q = SensorData.desired_q; v = SensorData.desired_v; a = SensorData.desired_a;
                
                Dynamics = obj.ModelHandler.get_Dynamics(q, v);
                H = Dynamics.H;
                c = Dynamics.c;
                B = obj.ModelHandler.get_estimated_ControlMap(q);
                
                if obj.use_pinv_on_control_maps
                    Output.u0 = pinv(B)*(H*a + c);
                else
                    Output.u0 = B \ (H*a + c);
                end
                
                Output.H = H;
                Output.B = B;
                Output.c = c;
            end
            
            
            %Inverse dynamics for systems with explicit mechanical
            %constraints
            function Output = pinvInverseDynamics(SensorData)
                q = SensorData.desired_q; v = SensorData.desired_v; a = SensorData.desired_a;
                
                Dynamics = obj.ModelHandler.get_Dynamics(q, v);
                H = Dynamics.H;
                c = Dynamics.c;
                B = obj.ModelHandler.get_estimated_ControlMap(q);
                
                F = g_dynamics_LagrangeMultiplier_ConstraintJacobian(q);
                
                res = pinv([B, F'])*(H*a + c);
                
                Output.u0 = res(1:size(B, 2));
                Output.lambda0 = res((size(B, 2)+1):end);
                
                Output.H = H;
                Output.B = B;
                Output.c = c;
            end

            %this is a inverse dynamics algorithm proposed in paper
            %"Inverse Dynamics Control of Floating Base Systems Using
            %Orthogonal Decomposition", 2010, Mistry et al.
            function Output = SchaalInverseDynamics(SensorData)
                q = SensorData.desired_q; v = SensorData.desired_v; a = SensorData.desired_a;
                
                JSIM = g_dynamics_JSIM(q);
                B = g_dynamics_ControlMap(q);
                F = g_dynamics_LagrangeMultiplier_ConstraintJacobian(q);
                Forces = obj.ModelHandler.get_estimated_ForcesForComputedTorqueController(q, v);

                
                m = size(B, 2); k = size(F, 1); n = obj.dof;
                
                %generate switching matrices
                S = [eye(m), zeros(m, n-m)];
                Sc = [eye(k), zeros(k, n-k)];
                Su = [zeros(n-k, k), eye(n-k)];
                
                %generate constraint jacobian decomposition
                [Decomposition.Q, Decomposition.R] = qr(F');
                
                %replace found decomposition with the unique one
                s1 = size(Decomposition.R, 1); s2 = size(Decomposition.R, 2); 
                Decomposition.R_padded = [Decomposition.R, zeros(s1, s1-s2)];
                Decomposition.R_SignChanger = diag(sign(diag(Decomposition.R_padded)));
                Decomposition.Q_unique = Decomposition.Q * Decomposition.R_SignChanger; 
                Decomposition.R_unique_padded = Decomposition.R_SignChanger * Decomposition.R_padded;
                %there might be problems with the zeros at the end, is a good question~~
                Decomposition.R_unique = Decomposition.R_unique_padded(1:k, 1:s2);
                
                GeneralizedTorques = pinv(Su* Decomposition.Q_unique' * S') * Su * ...
                    Decomposition.Q_unique'*(JSIM*a + Forces);
                
                %check if the system's equations are set up correctly. This
                %ID algorithm requires a particular structure of these
                %eqs., namely matrix B should have the following structure
                %B = [I, 0]'; iff B = [I; 0], u = GeneralizedTorques
                is_Schaal_system = true;
                if ~isequal(B(1:m, :), eye(m))
                    is_Schaal_system = false;
                end
                if n > m
                    if ~isequal(B((m+1):n, :), zeros(n-m, m))
                        is_Schaal_system = false;
                    end
                end
                if ~is_Schaal_system
                    warning('Matrix equations structure in not suitable for the chosen inverse dynamics algorithm;');
                    warning('Control map (matrix B in the H*a + c = B*u + F''l eq.) should be B = [I, 0]''');
                    warning('Read https://pdfs.semanticscholar.org/b806/510b083107d33ae37287c42a9b1d5458e7cd.pdf');
                    %check norm(B*u - [GeneralizedTorques; zeros(n-m, 1)])
                end
                
                if obj.use_pinv_on_control_maps
                    u = pinv(B)*[GeneralizedTorques; zeros(n-m, 1)];
                else
                    u = B \ [GeneralizedTorques; zeros(n-m, 1)];
                end
                
                lambda = pinv(Decomposition.R_unique) * Sc * Decomposition.Q_unique' * ...
                    (JSIM*a + Forces - B*u);
                
                Output.u0 = u;
                Output.lambda = lambda;
                
                Output.GeneralizedTorques = GeneralizedTorques;
                Output.is_Schaal_system = is_Schaal_system;
                Output.S = S; Output.Sc = Sc; Output.Su = Su;
                Output.Decomposition = Decomposition;
                
                Output.JSIM = JSIM; Output.B = B; Output.F = F;
                Output.Forces = Forces;
            end                  
            
            switch Type
                case 'Inverse Dynamics'
                    controller = @InverseDynamics;
                case 'Schaal Inverse Dynamics'
                    controller = @SchaalInverseDynamics;
                case 'Inverse Dynamics wpinv'
                    controller = @InverseDynamics_wpinv;
                case 'pinv Inverse Dynamics'
                    controller = @pinvInverseDynamics;
                otherwise
                    warning('Invalid controller type');
            end 
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% helper functions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %this function allows you to make a good guess at the about of
        %motors (control dimentions / control degrees of freedom) the
        %system has. If the value obj.Control_dof is set, then it is known
        %exactly, if not - obj.dof is used. It is up to the user to provide
        %obj.Control_dof if obj.Control_dof ~= obj.dof
        function m = Guess_control_dof(obj)
            if ~isempty(obj.Control_dof)
                m = obj.Control_dof;
            else
                m = obj.dof;
            end
        end
        
        
    end
end