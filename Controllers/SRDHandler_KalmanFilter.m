classdef SRDHandler_KalmanFilter < SRDHandler
    properties
        Handler_ObserverState;             %x_observer
        Handler_nominal_trajectory_state;  %nominal x u 
        Handler_dynamics_Linearized_Model; %Ax + B*u
        Handler_MeasuredOutput;            %y = C*x
        MainController;                    %u = u(x)
        Handler_Time;                      %t
        
        Q;
        R
        previous_P;
    end
    methods
        function obj = SRDHandler_KalmanFilter(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_KalmanFilter';
            Parser.addOptional('Handler_ObserverState', []);
            Parser.addOptional('Handler_nominal_trajectory_state', []);
            Parser.addOptional('Handler_dynamics_Linearized_Model', []);
            Parser.addOptional('Handler_MeasuredOutput', []);
            Parser.addOptional('MainController', []);
            Parser.addOptional('Handler_Time', []);
            Parser.addOptional('Q', []);
            Parser.addOptional('R', []);
            Parser.addOptional('P0', []);
            Parser.parse(varargin{:});
            
            
            obj.Handler_ObserverState              = Parser.Results.Handler_ObserverState;
            obj.Handler_nominal_trajectory_state   = Parser.Results.Handler_nominal_trajectory_state;
            obj.Handler_dynamics_Linearized_Model  = Parser.Results.Handler_dynamics_Linearized_Model;
            obj.Handler_MeasuredOutput             = Parser.Results.Handler_MeasuredOutput;
            obj.MainController                     = Parser.Results.MainController;
            obj.Handler_Time                       = Parser.Results.Handler_Time;
            
            obj.Q = Parser.Results.Q;
            obj.R = Parser.Results.R;
            
            if isempty(Parser.Results.P0)
                obj.previous_P = zeros(size(Parser.Results.Q));
            else
                obj.previous_P = Parser.Results.P0;
            end
        end
        
        function Update(obj)
            
            P = obj.previous_P;
            
            dt = obj.Handler_Time.dt;
            A = obj.Handler_dynamics_Linearized_Model.get_A();
            B = obj.Handler_dynamics_Linearized_Model.get_B();
            C = obj.Handler_MeasuredOutput.C;
            
            n = size(A, 2);
            
            u = obj.MainController.u;
            y = obj.Handler_MeasuredOutput.y;
            x = obj.Handler_ObserverState.x;
            
            x_nominal  = obj.Handler_nominal_trajectory_state.x;
            u_nominal  = obj.Handler_nominal_trajectory_state.u;
            dx_nominal = obj.Handler_nominal_trajectory_state.dx;
            %f_nominal = obj.Handler_nominal_trajectory_state.f;
            y_nominal  = C*x_nominal;
            
            delta_x = x - x_nominal;
            delta_u = u - u_nominal;
            delta_y = y - y_nominal;
            
            switch obj.Handler_dynamics_Linearized_Model.TemporalType
                case 'ContiniousTime'
                    A              = eye(size(A)) +  A          * dt;
                    B              =                 B          * dt;
                    next_x_nominal = x_nominal    +  dx_nominal * dt;
                case 'DiscreteTime'
                    warning('Was it implemented?..')
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
            P = A*P*A' + obj.Q;
            L = P*C'*pinv(C*P*C' + obj.R);
            P = (eye(n) - L*C)*P;
            
            delta_next_x = A*delta_x + B*delta_u + L*(delta_y - C*delta_x);
            x = next_x_nominal + delta_next_x;
            
            %delta_next_x_1 = obj.Handler_nominal_trajectory_state.function_Original_Model(x, u) * dt + L*(delta_y - C*delta_x);
            %x1 = next_x_nominal + delta_next_x_1;
            
            obj.Handler_ObserverState.x = x;
            obj.previous_P = P;
        end
    end
end