%This class provides positions estimation functions
%last update 14.12.17
classdef SRDEstimation < SRDControl
    properties
        
    end
    methods
        % class constructor
        function obj = SRDEstimation(LinkArray)
            obj = obj@SRDControl(LinkArray);
        end
        
        function estimator = GetDynamicEstimator(obj, Type, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDEstimation.GetDynamicEstimator';
            
            %OrtegaSpongCTC_Estimator
            Parser.addOptional('Kp', eye(obj.dof)*100);
            Parser.addOptional('Kd', eye(obj.dof)*50);
            Parser.addOptional('Q', eye(2*obj.dof)*100);
            Parser.addOptional('R', eye(length(obj.ModelHandler.estimated_theta))*50);
            Parser.addOptional('ILQR_TimeStep', obj.TimeStep);
            Parser.addOptional('use_inverse_dynamics_in_OrtegaSpong_CTC', true);
            
            %LeastSquares_Estimator
            Parser.addOptional('LeastSquares_TimeStep', obj.TimeStep);
            Parser.addOptional('LeastSquares_NumberOfStates', 10);
            Parser.addOptional('LeastSquares_dt', 0.01);
            Parser.addOptional('LeastSquares_DampingRate', 0);
            Parser.parse(varargin{:});
            
            if nargin < 2
                Type = 'OrtegaSpong_CTC';
            end
            
            %this function implements method described in:
            %"Ortega, R. and Spong, M.W., 1989. Adaptive motion control of
            %rigid robots: A tutorial"
            function Output = OrtegaSpongCTC_Estimator(PreviousState, ~, ~, ~, ~)
                
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                %we find the time remained till the next LQR gains update
                remainder = mod(SensorData.t, Parser.Results.ILQR_TimeStep);
                
                %get parameters; note that Kp and Kd here should be the
                %same as in the computed torque controller
                Kp = Parser.Results.Kp; Kd = Parser.Results.Kd;
                Q = Parser.Results.Q; R = Parser.Results.R;
                
                JSIM = obj.ModelHandler.get_estimated_JSIM(SensorData.q);
                B = obj.ModelHandler.get_estimated_ControlMap(SensorData.desired_q);
                Y = pinv(B)*g_OrtegaSpong_Regressor(SensorData.q, SensorData.v, SensorData.a);
                
                %if the update is due, do it
                if isempty(PreviousState) || (remainder == 0)
                
                    
                    n = obj.dof;
                    d = length(obj.ModelHandler.estimated_theta);
                    
                    %prepare LQR problem;
                    lqr_A = [zeros(n), eye(n);
                             -Kp,     -Kd];
                    lqr_B = [zeros(n, d); (pinv(JSIM) * Y)];
                    
                    %!!!weave in the inverse dynamics
                    %we have dx = lqr_A*x + lqr_B*theta + lqr_c
                    %where lqr_c = [0; Regressor_constant]
                    %maybe treat theta as control input and solve for a
                    %inverse dynamics this way
                    
                    K = lqr(lqr_A, lqr_B, Q, R);
                else
                    K = PreviousState.K;
                end
                
                e = SensorData.desired_q - SensorData.q;
                de = SensorData.desired_v - SensorData.v;
                dde = SensorData.desired_a - SensorData.a;
                
                if Parser.Results.use_inverse_dynamics_in_OrtegaSpong_CTC
                    Yc = pinv(B)*g_OrtegaSpong_Regressor_constant(SensorData.q, SensorData.v, SensorData.a);
                    theta_desired = pinv(Y) * (JSIM * (dde + Kd*de + Kp*e) - Yc);
                else
                    theta_desired = 0;
                end
                 
                x = [e; de];
                
                %estimator update law
                d_theta = K*x + theta_desired;
                %new estimation
                obj.ModelHandler.estimated_theta = obj.ModelHandler.estimated_theta + d_theta*obj.TimeStep;
                
                Output.State.K = K;
            end
            
            %the idea is to take dynamics in the form:
            %H*ddq + c = tau
            %define regressor Y = d(H*ddq + c)/d(theta), and 
            %regressor constant Yc = H*ddq + c - Y*theta
            %see "Ortega, R. and Spong, M.W., 1989. Adaptive motion 
            %control of rigid robots: A tutorial"
            %then state Y*theta + Yc = tau for n moments in time
            %and solve for theta via least squares
            %note: tau = B*u
            function Output = LeastSquares_Estimator(PreviousState, ~, ~, ~, SimulationOutput)
                
                SensorData = obj.SensorHandler.ReadCurrentData;
                
                B = obj.ModelHandler.get_estimated_ControlMap(SensorData.desired_q);
                Y = pinv(B)*g_OrtegaSpong_Regressor(SensorData.q, SensorData.v, SensorData.a);
                Yc = pinv(B)*g_OrtegaSpong_Regressor_constant(SensorData.q, SensorData.v, SensorData.a);   
                
                %we find the time remained till the next
                %LeastSquares_Estimator update
                remainder = mod(SensorData.t, Parser.Results.LeastSquares_TimeStep);
                
                NumberOfStates = Parser.Results.LeastSquares_NumberOfStates;
                dt = Parser.Results.LeastSquares_dt;
                
                index = floor(SensorData.t / obj.TimeStep);
                
                %if the update is due, do it
                if (isempty(PreviousState) || (remainder == 0)) && (index > NumberOfStates)
                    
                    %TimeDepth - long long into the motion history we will
                    %look; if it is longer then the history itself -
                    %shorten dt;
                    TimeDepth = NumberOfStates * dt;
                    if TimeDepth > SensorData.t
                        dt = SensorData.t / NumberOfStates;
                    end
                    
                    m = size(B, 2);
                    h1 = size(Y, 1);
                    h2 = size(Y, 2);
                    
                    Collected_Regressor = zeros(h1*NumberOfStates, h2);
                    Collected_u = zeros(m*NumberOfStates, 1);
                    
                    %collect samples
                    for i = 1:NumberOfStates
                        sample_time = SensorData.t - (i-1)*dt;
                        sample_index = floor(sample_time / obj.TimeStep);
                        if sample_index < index
                            sample_Regressor = SimulationOutput.EstimatorOutput{sample_index}.Regressor;
                            sample_Regressor_constant = SimulationOutput.EstimatorOutput{sample_index}.Regressor_constant;
                        else
                            sample_Regressor = Y;
                            sample_Regressor_constant = Yc;
                        end
                        sample_u = SimulationOutput.ControllerOutput{sample_index}.u;
                        
                        %compound them into one
                        Collected_Regressor(((i-1)*h1 + 1):(i*h1), :) = sample_Regressor;
                        Collected_u(((i-1)*m + 1):(i*m), :) = sample_u - sample_Regressor_constant;
                    end
                    
                    theta = pinv(Collected_Regressor) * Collected_u;
                    
                    %new estimation
                    DampingRate = Parser.Results.LeastSquares_DampingRate;
                    theta_old = obj.ModelHandler.estimated_theta;
                    
                    obj.ModelHandler.estimated_theta = theta_old * DampingRate + (1 - DampingRate)*theta;
                end
                
                Output.State = [];
                Output.Regressor = Y;
                Output.Regressor_constant = Yc;
            end
            
            
            switch Type
                case {'OrtegaSpong_CTC'}
                    estimator = @OrtegaSpongCTC_Estimator;
                case {'LeastSquares'}
                    estimator = @LeastSquares_Estimator;
                otherwise
                    warning('Invalid controller type');
            end
        end
        
    end
end