%This class provides robot model
%last update 10.01.18
classdef SRDSensorHandler < handle
    properties
        %%%%%%%%%%%%%%%%%%
        %%% data
        
        CurrentData_true
        %structure containing all current data (actual)
        
        CurrentData_available
        %structure containing all current data (available)
        
        GenerateSensorOutputOnUpdate = true;
        %if true - the sensor output will be generates when
        %.WriteCurrentData is called
        
        %%%%%%%%%%%%%%%%%%
        %%% functions
        
        %Reader functions and properties
        Reader = {};
        %is is a cell array of reader handles
        
        
        %Observer functions and properties
        Observer = {};
        %is is a cell array of observer handles
        
        ObserverState = [];
        
        %%%%%%%%%%%%%%%%%%
        %%% SRD handles
        
        SimulationEngine = [];
    end
    methods
        
        function obj = SRDSensorHandler(SimulationEngine)
            
            if nargin >= 1
                obj.SimulationEngine = SimulationEngine;
            end
            
            obj.SetReader('True');
        end
        
        %input true information about the state of the system
        function WriteCurrentData(obj, CurrentData)
            obj.CurrentData_true = CurrentData;
            
            if obj.GenerateSensorOutputOnUpdate
                obj.GenerateSensorOutput()
            end
        end
        
        %update some information about the state of the system
        function UpdateCurrentData(obj, Name, Val)
            obj.CurrentData_true.(Name) = Val;
            obj.CurrentData_available.(Name) = Val;
        end
        
        %input true information about the state of the system
        function GenerateSensorOutput(obj)
            
            Data = obj.CurrentData_true;
            
            if ~isempty(obj.Reader)
                %obj.Reader could contain more than one reader; then
                %they simply stack, one passing its output to the other;
                for i = 1:length(obj.Reader)
                    Data = obj.Reader{i}(Data);
                end
            end
                
            if ~isempty(obj.Observer)
                %obj.Observer could contain more than one observer; then
                %they simply stack, one passing its output to the other;
                for i = 1:length(obj.Observer)
                    Data = obj.Observer{i}(Data);
                end
            end
            
            obj.CurrentData_available = Data;
        end
        
        
        %read sensor information about the state of the system
        function CurrentData = ReadCurrentData(obj)
            CurrentData = obj.CurrentData_available;
        end
        
        %read actual information about the state of the system
        function CurrentData = ReadCurrentData_True(obj)
            CurrentData = obj.CurrentData_true;
        end        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %chose reader (sensor model)
        function Reader = SetReader(obj, ReaderType, varargin)
            switch ReaderType
                case 'True'
                    Reader = obj.get_Reader_True(varargin{:});
                case 'Quantized'
                    Reader = obj.get_Reader_Quantized_qv(varargin{:});
                case 'Quantized_lambda'
                    Reader = obj.get_Reader_Quantized_lambda(varargin{:});
                case 'Noise'
                    Reader = obj.get_Reader_noise_qv(varargin{:});
                case 'Covariance Noise'
                    Reader = obj.get_Reader_covariance_noise_qv(varargin{:});
                case 'Noise_lambda'
                    Reader = obj.get_Reader_noise_lambda(varargin{:});
                otherwise
                    error('incorrect reader type')
            end
        end
        
        %Reader that does nothing
        function Reader = get_Reader_True(~, varargin)
            function CurrentData = Reader_True(Data)
                CurrentData = Data;
            end
            Reader = @Reader_True;
        end
        
        %Reader that quantizes the gen coordinates and velocities
        function Reader = get_Reader_Quantized_qv(~, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDSensorHandler.get_Reader_Quantized_qv';
            Parser.addOptional('quantization_step_q', 0.001);
            Parser.addOptional('quantization_step_v', 0.001);
            Parser.parse(varargin{:});
            
            function CurrentData = Reader_Quantized_qv(Data)
                CurrentData = Data;
                if ~isempty(Parser.Results.quantization_step_q)
                    CurrentData.q = CurrentData.q - rem(CurrentData.q, Parser.Results.quantization_step_q);
                end
                if ~isempty(Parser.Results.quantization_step_v)
                    CurrentData.v = CurrentData.v - rem(CurrentData.v, Parser.Results.quantization_step_v);
                end
                CurrentData.x = [CurrentData.q; CurrentData.v];
            end
            Reader = @Reader_Quantized_qv;
        end
        
        %Reader that quantizes the reactions
        function Reader = get_Reader_Quantized_lambda(~, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDSensorHandler.get_Reader_Quantized_lambda';
            Parser.addOptional('quantization_step', 0.001);
            Parser.parse(varargin{:});
            
            function CurrentData = Reader_Quantized_lambda(Data)
                CurrentData = Data;
                if ~isempty(Parser.Results.quantization_step)
                    CurrentData.lambda = CurrentData.lambda - rem(CurrentData.lambda, Parser.Results.quantization_step);
                end
            end
            Reader = @Reader_Quantized_lambda;
        end
        
        %Reader that adds random noise to the gen coordinates and velocities
        function Reader = get_Reader_noise_qv(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDSensorHandler.get_Reader_noise_qv';
            Parser.addOptional('noise_amplitute_q', ones(obj.SimulationEngine.dof, 1)*0.001);
            Parser.addOptional('noise_amplitute_v', ones(obj.SimulationEngine.dof, 1)*0.001);
            Parser.addOptional('noise_center_q', zeros(obj.SimulationEngine.dof, 1));
            Parser.addOptional('noise_center_v', zeros(obj.SimulationEngine.dof, 1));
            Parser.parse(varargin{:});
            
            NA.Dq = diag(Parser.Results.noise_amplitute_q);
            NA.Dv = diag(Parser.Results.noise_amplitute_v);
            NC.q = Parser.Results.noise_center_q;
            NC.v = Parser.Results.noise_center_v;
            n = obj.SimulationEngine.dof;
            
            function CurrentData = Reader_noise_qv(Data)
                CurrentData = Data;
                CurrentData.q = CurrentData.q + NC.q + ...
                        NA.Dq*(rand(n, 1) - 0.5*ones(n, 1));
                CurrentData.v = CurrentData.v + NC.v + ...
                        NA.Dv*(rand(n, 1) - 0.5*ones(n, 1));
                CurrentData.x = [CurrentData.q; CurrentData.v];
            end
            Reader = @Reader_noise_qv;
        end
        
        
        %Reader that adds random noise with a given mean and covariance to the gen coordinates and velocities
        function Reader = get_Reader_covariance_noise_qv(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDSensorHandler.get_Reader_covariance_noise_qv';
            Parser.addOptional('noise_mu_q', zeros(1, obj.SimulationEngine.dof));
            Parser.addOptional('noise_mu_v', zeros(1, obj.SimulationEngine.dof));
            Parser.addOptional('noise_sigma_q', eye(obj.SimulationEngine.dof)*0.083);
            Parser.addOptional('noise_sigma_v', eye(obj.SimulationEngine.dof)*0.083);
            Parser.parse(varargin{:});
            
            function CurrentData = Reader_covariance_noise_qv(Data)
                CurrentData = Data;
                CurrentData.q = CurrentData.q + mvnrnd(Parser.Results.noise_mu_q, Parser.Results.noise_sigma_q, 1)';
                CurrentData.v = CurrentData.v + mvnrnd(Parser.Results.noise_mu_v, Parser.Results.noise_sigma_v, 1)';
                CurrentData.x = [CurrentData.q; CurrentData.v];
            end
            Reader = @Reader_covariance_noise_qv;
        end
        
        
        %Reader that adds random noise to the gen coordinates and velocities
        function Reader = get_Reader_noise_lambda(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDSensorHandler.get_Reader_noise_lambda';
            Parser.addOptional('noise_amplitute', ones(obj.SimulationEngine.Constraint_dof, 1)*0.001);
            Parser.addOptional('noise_center', zeros(obj.SimulationEngine.Constraint_dof, 1));
            Parser.parse(varargin{:});
            
            D = diag(Parser.Results.noise_amplitute);
            noise_center = Parser.Results.noise_center;
            k = obj.SimulationEngine.Constraint_dof;
            
            function CurrentData = Reader_noise_lambda(Data)
                CurrentData = Data;
                CurrentData.lambda = CurrentData.lambda + noise_center + ...
                        D*(rand(k, 1) - 0.5*ones(k, 1));
            end
            Reader = @Reader_noise_lambda;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %chose reader (sensor model)
        function Observer = SetObserver(obj, ObserverType, varargin)
            switch ObserverType
                case 'LinearOptimal'
                    Observer = obj.get_Observer_LinearOptimal(varargin{:});
                case 'LinearDiagonal'
                    Observer = obj.get_Observer_LinearDiagonal(varargin{:});
                case 'PD'
                    Observer = obj.get_Observer_PD(varargin{:});
                case 'SimpleKalman'
                    Observer = obj.get_Observer_SimpleKalman(varargin{:});
                case 'none'
                    Observer = [];
                otherwise
                    error('incorrect observer type')
            end
        end
        
        
        function Observer = get_Observer_LinearOptimal(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDSensorHandler.get_Observer_LinearOptimal';
            Parser.addOptional('Q', eye(2*obj.SimulationEngine.dof)*10^3);
            Parser.addOptional('R', eye(2*obj.SimulationEngine.dof));
            Parser.addOptional('LinearOptimalObserver_update_TimeStep', 0.01);
            Parser.addOptional('UseConstraintsJacobians', false);
            Parser.parse(varargin{:});
            
            function CurrentData = Observer_LinearOptimal(ReaderData)
                
                %if ObserverState wasn't initialized, initialize it this
                %way
                if ~isfield(obj.ObserverState, 'LinearOptimal')
                    obj.ObserverState.LinearOptimal.z = ReaderData.x;
                    obj.ObserverState.LinearOptimal.L = [];
                end
                
                %linearize the model around the current point
                LinearModel = obj.SimulationEngine.GetLinearization(ReaderData.x0, ReaderData.u);
                    
                %remainder till the next update of the observer
                remainder = 1;
                if ~isempty(Parser.Results.LinearOptimalObserver_update_TimeStep)
                    remainder = mod(ReaderData.t, Parser.Results.LinearOptimalObserver_update_TimeStep);
                end
                %every TimeStep solve Riccati eq
                if isempty(obj.ObserverState.LinearOptimal.L) || (remainder == 0)
                    %update LQR gains
                    obj.ObserverState.LinearOptimal.L = lqr(LinearModel.A, eye(2*obj.SimulationEngine.dof), ...
                        Parser.Results.Q, Parser.Results.R);
                end
                
                %Observer dynamics
                dz = LinearModel.A * obj.ObserverState.LinearOptimal.z + LinearModel.B * ReaderData.u  + ...
                    LinearModel.c + obj.ObserverState.LinearOptimal.L * (ReaderData.x - obj.ObserverState.LinearOptimal.z);
                
                %in case the dynamics has form dx = A*x + B*u + F*l + c,
                %this formulation can be used.
                %F = H^-1 * [0; F0'], where F0 is a constraint jacobian, 
                %F0 = d(r_C) / dq; and H is the inertia matrix of the linearization.
                %see SRDControl.GetLinearization() description for details
                if Parser.Results.UseConstraintsJacobians
                    F0 = obj.SimulationEngine.ModelHandler.get_estimated_ConstraintJacobian(ReaderData.q);
                    f = LinearModel.H \ ([zeros(size(F0')); F0'] * ReaderData.lambda);
                    dz = dz + f;
                end
                
                %Euler update for the observer dynamics
                obj.ObserverState.LinearOptimal.z = obj.ObserverState.LinearOptimal.z + dz * obj.SimulationEngine.TimeStep;
                
                %send the observer output as the current data
                CurrentData = ReaderData;
                CurrentData.x = obj.ObserverState.LinearOptimal.z;
                CurrentData.q = CurrentData.x(1:obj.SimulationEngine.dof);
                CurrentData.v = CurrentData.x((obj.SimulationEngine.dof + 1):(2 * obj.SimulationEngine.dof));
            end
            
            Observer = @Observer_LinearOptimal;
        end
        
        
        function Observer = get_Observer_LinearDiagonal(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDSensorHandler.get_Observer_LinearOptimal';
            Parser.addOptional('L', eye(2*obj.SimulationEngine.dof)*10^3);
            Parser.addOptional('UseConstraintsJacobians', false);
            Parser.parse(varargin{:});
            
            function CurrentData = Observer_LinearDiagonal(ReaderData)
                
                %if ObserverState wasn't initialized, initialize it this
                %way
                if ~isfield(obj.ObserverState, 'LinearDiagonal')
                    obj.ObserverState.LinearDiagonal.z = ReaderData.x;
                end
                
                %linearize the model around the current point
                LinearModel = obj.SimulationEngine.GetLinearization(ReaderData.x0, ReaderData.u);
                
                %Observer dynamics
                dz = LinearModel.A * obj.ObserverState.LinearDiagonal.z + LinearModel.B * ReaderData.u  + ...
                    LinearModel.c + Parser.Results.L * (ReaderData.x - obj.ObserverState.LinearDiagonal.z);
                
                %in case the dynamics has form dx = A*x + B*u + F*l + c,
                %this formulation can be used.
                %F = H^-1 * [0; F0'], where F0 is a constraint jacobian, 
                %F0 = d(r_C) / dq; and H is the inertia matrix of the linearization.
                %see SRDControl.GetLinearization() description for details
                if Parser.Results.UseConstraintsJacobians
                    F0 = obj.SimulationEngine.ModelHandler.get_estimated_ConstraintJacobian(ReaderData.q);
                    f = LinearModel.H \ ([zeros(size(F0')); F0'] * ReaderData.lambda);
                    dz = dz + f;
                end
                
                %Euler update for the observer dynamics
                obj.ObserverState.LinearDiagonal.z = obj.ObserverState.LinearDiagonal.z + dz * obj.SimulationEngine.TimeStep;
                
                %send the observer output as the current data
                CurrentData = ReaderData;
                CurrentData.x = obj.ObserverState.LinearDiagonal.z;
                CurrentData.q = CurrentData.x(1:obj.SimulationEngine.dof);
                CurrentData.v = CurrentData.x((obj.SimulationEngine.dof + 1):(2 * obj.SimulationEngine.dof));
            end
            
            Observer = @Observer_LinearDiagonal;
        end
     

        function Observer = get_Observer_PD(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDSensorHandler.get_Observer_LinearOptimal';
            Parser.addOptional('Lp', eye(obj.SimulationEngine.dof)*2*10^2);
            Parser.addOptional('Ld', eye(obj.SimulationEngine.dof)*10^2);
            Parser.addOptional('UseConstraintsJacobians', false);
            Parser.parse(varargin{:});
            
            function CurrentData = Observer_PD(ReaderData)
                
                %if ObserverState wasn't initialized, initialize it this
                %way
                if ~isfield(obj.ObserverState, 'PD')
                    obj.ObserverState.PD.z = ReaderData.q;
                    obj.ObserverState.PD.dz = ReaderData.v;
                end
                
                %linearize the model around the current point
                JSIM = obj.SimulationEngine.ModelHandler.get_estimated_JSIM(obj.ObserverState.PD.z);
                B = obj.SimulationEngine.ModelHandler.get_estimated_ControlMap(obj.ObserverState.PD.z);
                Forces = obj.SimulationEngine.ModelHandler.get_estimated_ForcesForComputedTorqueController(obj.ObserverState.PD.z, obj.ObserverState.PD.dz);
                
                %Observer dynamics
                ddz = JSIM \ (B * ReaderData.u  - Forces ...
                    + JSIM * Parser.Results.Lp * (ReaderData.q - obj.ObserverState.PD.z) + JSIM * Parser.Results.Ld * (ReaderData.v - obj.ObserverState.PD.dz));
                
                if Parser.Results.UseConstraintsJacobians
                    F = obj.SimulationEngine.ModelHandler.get_estimated_ConstraintJacobian(obj.ObserverState.PD.z);
                    f = JSIM \ (F' * ReaderData.lambda);
                    ddz = ddz + f;
                end
                
                %Euler update for the observer dynamics
                obj.ObserverState.PD.dz = obj.ObserverState.PD.dz + ddz * obj.SimulationEngine.TimeStep;
                obj.ObserverState.PD.z = obj.ObserverState.PD.z + obj.ObserverState.PD.dz * obj.SimulationEngine.TimeStep + 0.5 * ddz * obj.SimulationEngine.TimeStep^2;
                
                %send the observer output as the current data
                CurrentData = ReaderData;
                CurrentData.x = [obj.ObserverState.PD.z; obj.ObserverState.PD.dz];
                CurrentData.q = obj.ObserverState.PD.z;
                CurrentData.v = obj.ObserverState.PD.dz;
            end
            
            Observer = @Observer_PD;
        end        
        
        %Q - covariance matrix for dynamics noise
        %R - covariance matrix for sensor noise
        %C - sensor matrix in the linear dynamical system
        function Observer = get_Observer_SimpleKalman(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDSensorHandler.get_Observer_SimpleKalman';
            Parser.addOptional('Q', zeros(2*obj.SimulationEngine.dof));
            Parser.addOptional('R', eye(2*obj.SimulationEngine.dof)*0.083);
            Parser.addOptional('C', eye(2*obj.SimulationEngine.dof));
            Parser.parse(varargin{:});
            
            function CurrentData = Observer_SimpleKalman(ReaderData)
                
                I = eye(2*obj.SimulationEngine.dof);
                
                %if ObserverState wasn't initialized, initialize it this
                %way
                if ~isfield(obj.ObserverState, 'SimpleKalman')
                    obj.ObserverState.SimpleKalman.z = ReaderData.x;
                    obj.ObserverState.SimpleKalman.P = I;
                end
                
                %linearize the model around the current point
                LinearModel = obj.SimulationEngine.GetLinearization(ReaderData.x0, ReaderData.u);
                
                %find discrete linear model coefficients
                A = LinearModel.H \ LinearModel.A * obj.SimulationEngine.TimeStep + I;
                B = LinearModel.H \ LinearModel.B * obj.SimulationEngine.TimeStep;
                c = LinearModel.H \ LinearModel.c * obj.SimulationEngine.TimeStep;
                    
                %kalman dynamics update
                z_dynamics = A * obj.ObserverState.SimpleKalman.z + B * ReaderData.u + c;
                P_dynamics = A * obj.ObserverState.SimpleKalman.P * A' + Parser.Results.Q;
                
                %kalman sensor update
                S = Parser.Results.R + Parser.Results.C * P_dynamics * Parser.Results.C';
                K = P_dynamics * Parser.Results.C' * (S \ I);
                
                obj.ObserverState.SimpleKalman.z = z_dynamics + K * (ReaderData.x - Parser.Results.C * z_dynamics);
                obj.ObserverState.SimpleKalman.P = (I - K * Parser.Results.C) * P_dynamics * (I - K * Parser.Results.C)' + K * Parser.Results.R * K';
                
                
                %send the observer output as the current data
                CurrentData = ReaderData;
                CurrentData.x = obj.ObserverState.SimpleKalman.z;
                CurrentData.q = CurrentData.x(1:obj.SimulationEngine.dof);
                CurrentData.v = CurrentData.x((obj.SimulationEngine.dof + 1):(2 * obj.SimulationEngine.dof));
            end
            
            Observer = @Observer_SimpleKalman;
        end
    end
end