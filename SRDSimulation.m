%This class provides simulation functionality
%last update 10.12.17
classdef SRDSimulation < SRDChain
    properties
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % simulation
        
        Time = 10;       %this is the time of the simulation
        TimeStep = 0.001; %this is the time step for the numerical algorithms solving ODE
        
        IC;              
        %Initial conditions, a structure with fields .q and .v, all column vectors
        
        SimulationBlowUpThreshold = 10^6;
        %The simulation is aborted when the norm of generalised coordinates
        %vector exeeds this field; Set to Inf to disable
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % solvers
        CustomSolverType = 'Taylor'; %This determines which custom solver will be used in simulation
        
        SolverDebugOn = false;
        %If this property is true then the solvers will output additional
        %information, such as condition numbers of relevant matrices.
        
        DAE_CorrectionViaProjectionOn = true;
        % If this property is true the DAE solvers will attempt to correct 
        % the obtained solutions by linearizing constraints and then  
        % projecting the solution onto the constraint's null space.
        
        User_provided_solver;
        %function handle to a user-provided solver; define if needed.
        
        use_pinv_forward_dymamics = false;
        %if true, forward dynamics uses pinv rather than back slash for the
        %cases where the matrices can be poorly conditioned
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % output

        SaveSimulationResultsToArray = true;
        SaveSensorDataToArray = false;
        SaveReactionsDataToArray = false;
        %This determines whether of not the simulation functions will write
        %down and output an array that contains the solution for equations
        %of dynamics and other relevant information. Set to false for
        %speed.
        
        SimulationOutputType = 'OutputArray';
        SimulationOutput = [];
        %this property determines what will happen with the simulation
        %results
        %'OutputArray' - they are output by the simulation function
        %'SaveArray' - they are saved to a property .SimulationOutput
          
        LogControllerOutput = false;
        %if true - the controller output would be logged;
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % properties of the modeled system
        
        ControlInput = [];
        %function handle - function that generates control input
        
        ModelHandler = [];
        %handle of SRDModelHandler class
        SensorHandler = [];
        %handle of SRDSensorHandler class
        
        Control_dof = [];
        % dimentions of the control input
        Constraint_dof = [];
        % dimentions of the control input
    end
    properties (Access = private)
        ValidCustomSolverTypes = {'Euler', 'Taylor', 'Runge', 'Implicit Euler', 'DAE Taylor', 'DAE Runge'}; %Valid values for CustomSolverType property
        ConstraintsSolverTypes = {'DAE Taylor', 'DAE Runge'};
    end
    
    
    methods
        % class constructor, it sets default values for initial conditions
        % (all zeroes) and fills ResultArrayColumns property
        function obj = SRDSimulation(LinkArray)
            obj = obj@SRDChain(LinkArray);
            
            obj.IC.q = zeros(obj.dof, 1);
            obj.IC.v = zeros(obj.dof, 1);
            obj.IC.u = [];
            
            obj.ModelHandler = SRDModelHandler;
        end
        
        function Initialization(obj)
            if exist('datafile_settings_Control_dof', 'file') == 2
                temp = load('datafile_settings_Control_dof');
                obj.Control_dof = temp.Control_dof;
            end            
        end
        
        % this function takes parameters from SymbolicEngine that are
        % needed here
        function PassParametersFromSymbolicEngine(obj, SymbolicEngine)
            obj.Control_dof = max(size(SymbolicEngine.u));
        end
        
        % This function evaluates the vector of generalized acceleration of
        % the mechanism for the given state
        function a = Dynamics(obj, q, v, u)
            Dynamics = obj.ModelHandler.get_Dynamics(q, v);
            ControlMap = obj.ModelHandler.get_actual_ControlMap(q);
            
            RHS = ControlMap * u - Dynamics.c;
            a = Dynamics.H \ RHS;
        end
        
        % This function evaluates the vector ds/dt, the state-spase form of
        % dynamics equations: ds/dt = f(s, u)
        %
        % Convertion to generalized coordinates is carried through the
        % following relations:
        % s = [q; v]; ds = [v; a]
        function ds = DynamicsStateSpace(obj, s, u)
            q = s(1:obj.dof);
            v = s((obj.dof + 1):(2*obj.dof));
            
            Dynamics = obj.ModelHandler.get_Dynamics(q, v);
            ControlMap = obj.ModelHandler.get_actual_ControlMap(q);
            
            RHS = ControlMap * u - Dynamics.c;
            a = Dynamics.H \ RHS;
            
            ds = [v; a];
        end
        

        %This function realizes an update function for Euler ODE solver
        %
        %OutputStructure will have fields .s, .ds
        function OutputStructure = Solver_EulerUpdate(obj)
            
            CurrentData = obj.SensorHandler.ReadCurrentData_True;
            s = [CurrentData.q; CurrentData.v];
            u = CurrentData.u;
            
            ds = obj.DynamicsStateSpace(s, u);
            s = s + ds*obj.TimeStep;
            
            OutputStructure.s = s;
            OutputStructure.ds = ds;
            
            OutputStructure.q = s(1:obj.dof);
            OutputStructure.v = s((obj.dof + 1):(2*obj.dof));
            OutputStructure.a = ds((obj.dof + 1):(2*obj.dof));
        end
        
        %This function realizes an update function for Taylor expantion ODE
        %solver
        %
        %OutputStructure will have fields .q, .v, .a
        function OutputStructure = Solver_TaylorUpdate(obj)
            
            CurrentData = obj.SensorHandler.ReadCurrentData_True;
            q = CurrentData.q;
            v = CurrentData.v;
            u = CurrentData.u;
            
            a = obj.Dynamics(q, v, u);
            v = v + a*obj.TimeStep;
            q = q + v*obj.TimeStep + 0.5*a*obj.TimeStep^2;
            
            OutputStructure.q = q;
            OutputStructure.v = v;
            OutputStructure.a = a;
            
            OutputStructure.s = [q; v];
            OutputStructure.ds = [v; a];
        end
        
        % This function realizes an update function for Runge-Kutta ODE solver
        % s = [q; v];
        %
        %InputStructure should have fields .s, .u
        %OutputStructure will have fields .s, .ds
        function OutputStructure = Solver_RungeUpdate(obj)
            
            CurrentData = obj.SensorHandler.ReadCurrentData_True;
            s = [CurrentData.q; CurrentData.v];
            u = CurrentData.u;
            
            k1 = obj.DynamicsStateSpace(s, u);
            k2 = obj.DynamicsStateSpace(s + 0.5*k1*obj.TimeStep, u);
            k3 = obj.DynamicsStateSpace(s + 0.5*k2*obj.TimeStep, u);
            k4 = obj.DynamicsStateSpace(s + k3*obj.TimeStep, u);
            Slope = (k1 + 2*k2 + 2*k3 + k4)/6;
            s = s + Slope*obj.TimeStep;
            
            OutputStructure.s = s;
            OutputStructure.ds = k1;
            
            OutputStructure.q = s(1:obj.dof);
            OutputStructure.v = s((obj.dof + 1):(2*obj.dof));
            OutputStructure.a = OutputStructure.ds((obj.dof + 1):(2*obj.dof));
        end
        
        %This function realizes an update function for implicit Euler ODE
        %solver
        %
        %OutputStructure will have fields .s, .ds
        function OutputStructure = Solver_ImplicitEulerUpdate(obj)
            
            CurrentData = obj.SensorHandler.ReadCurrentData_True;
            s = [CurrentData.q; CurrentData.v];
            u = CurrentData.u;
            
            SSIM = g_dynamics_Linearization_SSIM(s);
            A = g_dynamics_Linearization_RHS_A(s, u);
            B = g_dynamics_Linearization_RHS_B(s);
            c = g_dynamics_Linearization_RHS_c(s, u);
            
            h = obj.TimeStep;
            
            s_old = s;
            if obj.use_pinv_forward_dymamics
                s = pinv(SSIM - h*A) * (SSIM*s + h*B*u + h*c);
            else
                s = (SSIM - h*A) \ (SSIM*s + h*B*u + h*c);
            end
            ds = (s - s_old)/h;
            
            OutputStructure.s = s;
            OutputStructure.ds = ds;
            
            OutputStructure.q = s(1:obj.dof);
            OutputStructure.v = s((obj.dof + 1):(2*obj.dof));
            OutputStructure.a = ds((obj.dof + 1):(2*obj.dof));
        end       
        
        %This function realizes an update function for DAE solver using
        %Taylor expantion
        %
        %OutputStructure will have fields .q, .v, .a, .N
        function OutputStructure = Solver_DAE_TaylorUpdate(obj)
            
            CurrentData = obj.SensorHandler.ReadCurrentData_True;
            q = CurrentData.q;
            v = CurrentData.v;
            u = CurrentData.u;
            
                Dynamics = obj.ModelHandler.get_Dynamics(q, v);
                
                B = obj.ModelHandler.get_actual_ControlMap(q);
                F = g_dynamics_LagrangeMultiplier_ConstraintJacobian(q);
                dF = g_dynamics_LagrangeMultiplier_ConstraintSecondJacobian(q, v);
                
            
                LHSm = [Dynamics.H, -F';
                        F,           zeros(size(F, 1))];
                    
                RHSv = [(B*u - Dynamics.c); 
                        (-dF*v)];
                    
%             LHSm = g_dynamics_LagrangeMultiplier_LHSm(q);
%             RHSv = g_dynamics_LagrangeMultiplier_RHSv(q, v, u);
            
            if obj.use_pinv_forward_dymamics
                g = pinv(LHSm) * RHSv;
            else
                g = LHSm \ RHSv;
            end
            a = g(1:size(q, 1)); %accelerations
            N = g((size(q, 1) + 1):size(g, 1)); %lagrange multipliers, i.e. reactions
            
            v = v + a*obj.TimeStep;
            q = q + v*obj.TimeStep + 0.5*a*obj.TimeStep^2;
            
            OutputStructure.q = q;
            OutputStructure.v = v;
            OutputStructure.a = a;
            OutputStructure.N = N;
            
            OutputStructure.s = [q; v];
            OutputStructure.ds = [v; a];

            if obj.SolverDebugOn
                DAE_LHS_Cond = cond(LHSm);
                OutputStructure.DAE_LHS_Cond = DAE_LHS_Cond;
            end        
        end        
     
        %This function realizes an update function for DAE solver using
        %Runge Kutta method
        %
        %OutputStructure will have fields .q, .v, .a, .N
        function OutputStructure = Solver_DAE_RungeUpdate(obj)
            
            CurrentData = obj.SensorHandler.ReadCurrentData_True;
            s = [CurrentData.q; CurrentData.v];
            u = CurrentData.u;
            
            function Output = lf_Dynamics(s)
                q1 = s(1:obj.dof);
                v1 = s((obj.dof + 1):(2*obj.dof));
                
                Dynamics = obj.ModelHandler.get_Dynamics(q1, v1);
                
                B = obj.ModelHandler.get_actual_ControlMap(q1);
                F = g_dynamics_LagrangeMultiplier_ConstraintJacobian(q1);
                dF = g_dynamics_LagrangeMultiplier_ConstraintSecondJacobian(q1, v1);
            
                LHSm = [Dynamics.H, -F';
                        F,           zeros(size(F, 1))];
                    
                RHSv = [(B*u - Dynamics.c); 
                        (-dF*v1)];
                
                %LHSm = g_dynamics_LagrangeMultiplier_LHSm(q1);
                %RHSv = g_dynamics_LagrangeMultiplier_RHSv(q1, v1, u);
                if obj.use_pinv_forward_dymamics
                    g = pinv(LHSm) * RHSv;
                else
                    g = LHSm \ RHSv;
                end
                a1 = g(1:size(q1, 1)); %accelerations
                
                Output.ds = [v1; a1];
                Output.N = g((size(q1, 1) + 1):size(g, 1)); %lagrange multipliers, i.e. reactions
            end
 
            Output1 = lf_Dynamics(s);
            Output2 = lf_Dynamics(s + 0.5*Output1.ds*obj.TimeStep);
            Output3 = lf_Dynamics(s + 0.5*Output2.ds*obj.TimeStep);
            Output4 = lf_Dynamics(s +     Output3.ds*obj.TimeStep);
            Slope = (Output1.ds + 2*Output2.ds + 2*Output3.ds + Output4.ds)/6;
            
            if obj.DAE_CorrectionViaProjectionOn
                q = s(1:obj.dof);
                v = s((obj.dof + 1):(2*obj.dof));
                
                %Those are constraint jacobians. If we have a constrain
                %f(q), then F1 = df/dq, F2 = d(fdot)/dq, where fdot = df/dt
                %With this notation we can produce two equalities:
                %
                % F1*v = 0, F2*v + F1*ddq = 0, v = dq/dt.
                %
                F1 = g_dynamics_LagrangeMultiplier_ConstraintJacobian(q);
                F2 = g_dynamics_LagrangeMultiplier_ConstraintSecondJacobian(q, v);
                
                number_of_constraints = size(F1, 1);
                
                %This is matrix A from eq. A*[v; ddq] = 0, or,
                %equivalently, A*ds/dt = 0
                A = [F1, zeros(number_of_constraints, obj.dof);
                     F2, F1];
                 
                %find basis in the null space N of A*q = 0 system
                NSB = null(A);
                
                %find projector onto the null space (space spanned by NSB)
                P = NSB*((NSB'*NSB) \ NSB');
                
                %Project Slope onto the null space
                Slope = P*Slope;
            end
                
            s = s + Slope*obj.TimeStep; 
            
            
            if obj.DAE_CorrectionViaProjectionOn
                q = s(1:obj.dof);
                
                %We have constraint f(q) = 0;
                %We linearize it to F*q + b = 0;
                Constraint = g_dynamics_LagrangeMultiplier_Constraint(q);
                b = Constraint - F1*q;
                
                OutputStructure.ConstraintViolation = Constraint;
                
                
                %find basis for null space N for F1*q = 0 system
                NSB = null(F1);
                
                % find projector onto null space N
                P = NSB*((NSB'*NSB) \ NSB');
                
                %find particular solution for F1*q = -b system
                if obj.use_pinv_forward_dymamics
                    c1 = pinv(F1) * (-b);
                else
                    c1 = F1 \ (-b);
                end
                
                %project the particular solution onto the space orthagonal
                %to the null space N. It is done to make sure q has a zero
                %projection on null space N
                c = c1 - P*c1;
                
                %Project q onto null space N using projector P, and then shift it by c.
                %If q is already in the set of solutions, then it could be decomposed
                %in a sum q = v1 + c, where v1 is in null space N. Then:
                % P*q + c = P*(v1 + c) + c = v1 + c = q, since P*v1 = v1 and P*c = 0, 
                % because we made sure it is so above.
                % This result is important, as it assures the method won't be 
                % needlessly changing already viable solutions                
                q = P*q + c;
                
                s(1:obj.dof) = q;
            end
            
            OutputStructure.s = s;
            OutputStructure.ds = Output1.ds;
            OutputStructure.N = Output1.N;            
            
            OutputStructure.q = s(1:obj.dof);
            OutputStructure.v = s((obj.dof + 1):(2*obj.dof));
            OutputStructure.a = OutputStructure.ds((obj.dof + 1):(2*obj.dof));
        end           
        
        
        
        %This function choses update function according to the preference
        %set by obj.CustomSolverType property
        function Solver_Update = SolverUpdateIterface(obj)

            switch obj.CustomSolverType
                case 'Euler'
                    Solver_Update = @obj.Solver_EulerUpdate;
                case 'Taylor'
                    Solver_Update = @obj.Solver_TaylorUpdate;
                case 'Runge'
                    Solver_Update = @obj.Solver_RungeUpdate;
                case 'Implicit Euler'
                    Solver_Update = @obj.Solver_ImplicitEulerUpdate;
                case 'DAE Taylor'
                    Solver_Update = @obj.Solver_DAE_TaylorUpdate;
                case 'DAE Runge'
                    Solver_Update = @obj.Solver_DAE_RungeUpdate;
                case 'User-provided'
                    Solver_Update = obj.User_provided_solver;
                otherwise
                    warning('Invalid solver type');
            end
        end
        
        % This function simulates the controlled motion of the mechanism.
        % The user needs to provide ControlInput function handle, and
        % ControlInput needs to output desired q, v and a
        % The user also needs to provide Controller function handle, and
        % Controller function needs to output u - vector of control actions
        % of proper dimentions
        %
        % CostFunction - a function handle for a cost function (the
        % function should provide the cost for the current state of the
        % system, which will be evaluated on each iteration and summed up)
        %
        % Tester - a function handle to an arbitrary user defined function.
        %
        % CostFunction and Tester should take as an input 1) controller 
        % input 2) controller output 3) solver output
        
        function Output = Simulation(obj, ControlInput, ControllerWrapper, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDSimulation.Simulation';
            Parser.addOptional('CostFunction', []);
            Parser.addOptional('Tester', []);
            Parser.addOptional('Estimator', []);
            Parser.addOptional('SensorHandler', []);
            Parser.addOptional('Verbose', false);
            Parser.parse(varargin{:});
            
            CostFunction = Parser.Results.CostFunction;
            Tester = Parser.Results.Tester;
            Estimator = Parser.Results.Estimator;
            obj.SensorHandler = Parser.Results.SensorHandler;
            
            obj.ControlInput = ControlInput;
            
            Count = floor(obj.Time / obj.TimeStep);
            
            %set initial conditions
            q = obj.IC.q;
            v = obj.IC.v;
            if isempty(obj.IC.u)
                u = zeros(obj.Control_dof, 1);
            else
                u = obj.IC.u;
            end
            
            %Provide controller with a link to SimulationEngine instance 
            ControllerWrapper.SimulationEngine = obj;
            
            %if required - allocate a cell array for the controller output
            %logging
            if obj.LogControllerOutput
                Output.ControllerOutput = cell(Count, 1);
            end
            
            %If we are calculating additive cost - initialize the variable
            %Output.Cost with 0
            if ~isempty(CostFunction)
                CalculateCost = true;
                Output.Cost = 0;
            else
                CalculateCost = false;
                Output.Cost = [];
            end
            
            %If we are using Tester function - preallocate memory for the
            %Output.TesterOutput
            if ~isempty(Tester)
                SaveTesterOutput = true;
                Output.TesterOutput = cell(Count, 1);
            else
                SaveTesterOutput = false;
                Output.TesterOutput = [];
            end
            
            %If we are using estimator - set the switch on;
            if ~isempty(Estimator)
                UseEstimator = true;
                EstimatorState = [];
                Output.EstimatorOutput = cell(Count, 1);
            else
                UseEstimator = false;
                Output.EstimatorOutput = [];
            end
            
            %If ModelHandler wasn't manually set up previously - do it now
            if obj.ModelHandler.model_type_code == 0
                obj.ModelHandler.Setup();
            end
            
            %if SensorHandler wasn't provided by the user - set it up 
            if isempty(obj.SensorHandler)
                obj.SensorHandler = SRDSensorHandler();
            end
            
            %if we simulate a system with mechanical constraints we need to
            %initialize .Constraint_dof property and CurrentData.lambda
            %parameter
            switch obj.CustomSolverType
                case obj.ConstraintsSolverTypes
                    if isempty(obj.Constraint_dof)
                        obj.Update_Constraint_dof();
                    end
                    CurrentData.lambda = zeros(obj.Constraint_dof, 1);
                    
                    update_lambda = true;
                otherwise
                    update_lambda = false;
            end
            
            %If we are saving results to an array - do memory preallocation
            if obj.SaveSimulationResultsToArray
                OutputArray.Time = zeros(Count, 1);
                OutputArray.Position = zeros(Count, obj.dof);
                OutputArray.Velocity = zeros(Count, obj.dof);
                OutputArray.Acceleration = zeros(Count, obj.dof);
                OutputArray.DesiredPosition = zeros(Count, obj.dof);
                OutputArray.DesiredVelocity = zeros(Count, obj.dof);
                OutputArray.ControlActions = zeros(Count, obj.Control_dof);
            end
            if obj.SaveSensorDataToArray
                OutputArray.MeasuredPosition = zeros(Count, obj.dof);
                OutputArray.MeasuredVelocity = zeros(Count, obj.dof);
            end
            if obj.SaveReactionsDataToArray
                OutputArray.lambda = zeros(Count, obj.Constraint_dof);
            end
            
            %flag showing whether or not the norm of q exceeded the threshold.
            Output.BlowUp = false; 
            
            Solver_Update = obj.SolverUpdateIterface();
             
            for i = 1:Count
                t = i*obj.TimeStep;
                if Parser.Results.Verbose
                    disp(['Time passed - ', num2str(t)]); 
                end
                
                [desired_q, desired_v, desired_a] = ControlInput(t);
                
                CurrentData.q = q;
                CurrentData.v = v;
                CurrentData.a = [];
                CurrentData.x = [q; v];
                CurrentData.desired_q = desired_q;
                CurrentData.desired_v = desired_v;
                CurrentData.desired_a = desired_a;
                CurrentData.x0 = [desired_q; desired_v];
                CurrentData.t = t;
                CurrentData.ControlInput = ControlInput;
                CurrentData.u = u;
                
                obj.SensorHandler.WriteCurrentData(CurrentData);
                
                %Controller update
                ControllerWrapper.Controller();
                u = ControllerWrapper.u;
                
                obj.SensorHandler.UpdateCurrentData('u', u);
                
                SolverOutputStructure = Solver_Update();
                
                q = SolverOutputStructure.q;
                v = SolverOutputStructure.v;
                a = SolverOutputStructure.a;
                if update_lambda
                    CurrentData.lambda = SolverOutputStructure.N;
                end
                
                %Saving to results OutputArray if needed
                if obj.SaveSimulationResultsToArray
                    OutputArray.Time(i) = t;
                    OutputArray.Position(i, :) = q';
                    OutputArray.Velocity(i, :) = v';
                    OutputArray.Acceleration(i, :) = a';
                    OutputArray.DesiredPosition(i, :) = desired_q';
                    OutputArray.DesiredVelocity(i, :) = desired_v';
                    OutputArray.ControlActions(i, :) = u';
                end
                if obj.SaveSensorDataToArray
                    SensorData = obj.SensorHandler.ReadCurrentData;
                    OutputArray.MeasuredPosition(i, :) = SensorData.q';
                    OutputArray.MeasuredVelocity(i, :) = SensorData.v';
                end
                if obj.SaveReactionsDataToArray
                    OutputArray.lambda(i, :) = SolverOutputStructure.N';
                end
                
                if obj.LogControllerOutput
                    Output.ControllerOutput{i} = ControllerOutput;
                end
                
                %Calculating cost if needed
                if CalculateCost
                    Output.Cost = Output.Cost + CostFunction([], [], SolverOutputStructure, obj)*obj.TimeStep;
                end
                
                %Saving tester function output if needed
                if SaveTesterOutput
                    Output.TesterOutput{i} = Tester(SolverOutputStructure, obj);
                end
                
                if UseEstimator
                    Output.EstimatorOutput{i} = Estimator(EstimatorState, [], [], SolverOutputStructure, Output);
                    EstimatorState = Output.EstimatorOutput{i}.State;
                end
                
                %check if the norm of q exceeded the threshold
                if ~(obj.SimulationBlowUpThreshold == Inf)
                    if norm(q) > obj.SimulationBlowUpThreshold
                        Output.BlowUp = true;
                        if CalculateCost
                            Output.Cost = NaN;
                        end
                        break;
                    end
                end
                
            end
            
            %Packing the results into a structure Output
            Output.SimulationOutput = [];
            if obj.SaveSimulationResultsToArray
                
                %trim zeros if the simulation was terminated
                if Output.BlowUp
                    OutputArray.Time = OutputArray.Time(1:i, :);
                    OutputArray.Position = OutputArray.Position(1:i, :);
                    OutputArray.Velocity = OutputArray.Velocity(1:i, :);
                    OutputArray.Acceleration = OutputArray.Acceleration(1:i, :);
                    OutputArray.DesiredPosition = OutputArray.DesiredPosition(1:i, :);
                    OutputArray.DesiredVelocity = OutputArray.DesiredVelocity(1:i, :);
                    OutputArray.ControlActions = OutputArray.ControlActions(1:i, :);
                end
                
                switch obj.SimulationOutputType
                    case 'OutputArray'
                        Output.SimulationOutput = OutputArray;
                    case 'SaveArray'
                        obj.SimulationOutput = OutputArray;
                otherwise
                    warning('Invalid SimulationOutputType value');
                end
            end
        end
        

        
        %this function plots the simulation results
        %if SimulationOutput is not 
        function figure_handle = PlotSimulationResults(obj, SimulationOutput, WhatToPlot)
            
            if nargin < 2
                SimulationOutput = [];
            end
            if isempty(SimulationOutput)
                if ~isempty(obj.SimulationOutput)
                    SimulationOutput = obj.SimulationOutput;
                else
                    warning('No simulation results were povided');
                end
            end
            
            if nargin < 3
                WhatToPlot = 'P, dP; V, U';
            end
            
            BP = BetterPlotsClass;
            BP.ToMakeAnnotationLines = false;
            figure_handle = BP.CreateBigFigure;
    
            switch WhatToPlot
                case {'P, dP; V, U', 'position, desired position; velocity, control action'}
                    BP.ArrowLengthX = 0.4;
                    BP.ArrowLengthY = 0.4;
                    
                    subplot(2, 2, 1);
                    h(1) = BP.Plot(SimulationOutput.Time, SimulationOutput.Position, 't, s', '$$q_i$$', figure_handle);
                    title('Position');
                    
                    subplot(2, 2, 2);
                    h(2) = BP.Plot(SimulationOutput.Time, SimulationOutput.DesiredPosition, 't, s', '$$q_i^*$$', figure_handle);
                    title('Desired position');
                    
                    subplot(2, 2, 3);
                    h(3) = BP.Plot(SimulationOutput.Time, SimulationOutput.Velocity, 't, s', '$$v_i$$', figure_handle);
                    title('Velocity');
                    
                    subplot(2, 2, 4);
                    h(4) = BP.Plot(SimulationOutput.Time, SimulationOutput.ControlActions, 't, s', '$$u_i$$', figure_handle);
                    title('Control actions');
                    BP.FixArrowsAllSubplots(h);
                    drawnow;
                case {'P', 'position'}
                    BP.ArrowLengthX = 0.85;
                    BP.ArrowLengthY = 0.85;
                    h(1) = BP.Plot(SimulationOutput.Time, SimulationOutput.Position, 't, s', '$$q_i$$', figure_handle);
                    title('Position');
                    BP.FixArrowsAllSubplots(h);
                    drawnow;
                case {'P+dP', 'position + desired position'}
                    BP.ArrowLengthX = 0.85;
                    BP.ArrowLengthY = 0.85;
                    h(1) = BP.Plot(SimulationOutput.Time, SimulationOutput.Position, 't, s', '$$q_i$$', figure_handle);
                    hold on;
                    BP.LineWidth = 1;
                    h(2) = BP.Plot(SimulationOutput.Time, SimulationOutput.DesiredPosition, 't, s', '$$q_i^*$$', figure_handle);
                    title('Position + desired position (thiner lines)');
                    BP.FixArrowsAllSubplots(h);
                    drawnow;
                case {'P+dP, V+dV', 'position + desired position, velocity + desired velocity'}
                    BP.ArrowLengthX = 0.4;
                    BP.ArrowLengthY = 0.85;
                    subplot(1, 2, 1);
                    h(1) = BP.Plot(SimulationOutput.Time, SimulationOutput.Position, 't, s', '$$q_i$$', figure_handle);
                    hold on;
                    BP.LineWidth = 1;
                    h(2) = BP.Plot(SimulationOutput.Time, SimulationOutput.DesiredPosition, 't, s', '$$q_i^*$$', figure_handle);
                    title('Position + desired position (thiner lines)');
                    
                    subplot(1, 2, 2);
                    BP.LineWidth = 3;
                    h(3) = BP.Plot(SimulationOutput.Time, SimulationOutput.Velocity, 't, s', '$$q_i$$', figure_handle);
                    hold on;
                    BP.LineWidth = 1;
                    h(4) = BP.Plot(SimulationOutput.Time, SimulationOutput.DesiredVelocity, 't, s', '$$q_i^*$$', figure_handle);
                    title('Velocity + desired velocity (thiner lines)');
                    
                    BP.FixArrowsAllSubplots(h);
                    drawnow;
                otherwise
                    warning('Invalid WhatToPlot value; use one of the following:');
                    disp('position, desired position; velocity, control action');
                    disp('position');
                    disp('position + desired position');
                    disp('position + desired position, velocity + desired velocity');
            end
               
        end
        
        %updates .Constraint_dof field
        function Update_Constraint_dof(obj)
            obj.Constraint_dof = size(g_dynamics_LagrangeMultiplier_ConstraintJacobian(zeros(obj.dof, 1)), 1);
        end
        
    end
end

