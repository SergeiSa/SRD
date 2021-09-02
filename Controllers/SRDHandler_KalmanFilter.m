classdef SRDHandler_KalmanFilter < SRDHandler
    properties
        Handler_ObserverState;
        Handler_MeasuredOutput;
        Handler_dynamics_Linearized_Model;
        MainController;
        Handler_Time;
        
        Q;
        R
        previous_P;
    end
    methods
        function obj = SRDHandler_KalmanFilter(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_KalmanFilter';
            Parser.addOptional('Handler_ObserverState', []);
            Parser.addOptional('Handler_MeasuredOutput', []);
            Parser.addOptional('Handler_dynamics_Linearized_Model', []);
            Parser.addOptional('MainController', []);
            Parser.addOptional('Handler_Time', []);
            Parser.addOptional('Q', []);
            Parser.addOptional('R', []);
            Parser.addOptional('P0', []);
            Parser.parse(varargin{:});
            
            obj.Handler_ObserverState              = Parser.Results.Handler_ObserverState;
            obj.Handler_MeasuredOutput             = Parser.Results.Handler_MeasuredOutput;
            obj.Handler_dynamics_Linearized_Model  = Parser.Results.Handler_dynamics_Linearized_Model;
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
            
            P = obj.previous_P;  Q = obj.Q;  R = obj.R;
            
            dt = obj.Handler_Time.dt;
            A = obj.Handler_dynamics_Linearized_Model.get_A();
            B = obj.Handler_dynamics_Linearized_Model.get_B();
            C = obj.Handler_MeasuredOutput.C;
            
            switch obj.Handler_dynamics_Linearized_Model.TemporalType
                case 'ContiniousTime'
                    A = eye(size(A)) + A*dt;
                    B =                B*dt;
                case 'DiscreteTime'
                    warning('Was it implemented?..')
            end
            
            u = obj.MainController.u;
            y = obj.Handler_MeasuredOutput.y;
            x = obj.Handler_ObserverState.x;
    
            P = A*P*A' + Q;
            L =  P*C'*pinv(C*P*C' + R);
            x = A*x + B*u + L*(y - C*x);
            
            obj.Handler_ObserverState.x = x;
            obj.previous_P = P;
        end
    end
end