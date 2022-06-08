classdef SRDController_LQR < SRDHandler
    properties
        Handler_nominal_trajectory_state;             %nominal x u  
        Handler_State_StateSpace;                     %x
        Handler_dynamics_Linearized_Model;            %Ax + B*u
        
        Q;                                            %J = x*Q*x + u*R*u
        R;                                            %%%%%%%%%%%%%%%%%%
        
        u;                                            %u = -K*(x - x_nominal) + u_nominal
        
        ToLog
        control_law;
    end
    methods
        function obj = SRDController_LQR(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDController_LQR';
            Parser.addOptional('Handler_nominal_trajectory_state', []);
            Parser.addOptional('Handler_State_StateSpace', []);
            Parser.addOptional('Handler_dynamics_Linearized_Model', []);
            Parser.addOptional('Q', []);
            Parser.addOptional('R', []);
            Parser.addOptional('ToLog', false);
            Parser.parse(varargin{:});
            
            
            obj.Handler_nominal_trajectory_state   = Parser.Results.Handler_nominal_trajectory_state;
            obj.Handler_State_StateSpace           = Parser.Results.Handler_State_StateSpace;
            obj.Handler_dynamics_Linearized_Model  = Parser.Results.Handler_dynamics_Linearized_Model;
            
            obj.Q = Parser.Results.Q;
            obj.R = Parser.Results.R;
            
            obj.ToLog = Parser.Results.ToLog;
        end
        
        function Update(obj)
            
            nominal_x = obj.Handler_nominal_trajectory_state.x;
            nominal_u = obj.Handler_nominal_trajectory_state.u;
            
            x = obj.Handler_State_StateSpace.x;
            
            A = obj.Handler_dynamics_Linearized_Model.get_A();
            B = obj.Handler_dynamics_Linearized_Model.get_B();
            
            K = lqr(A, B, obj.Q, obj.R);
            
            e  = reshape(x - nominal_x, [], 1);
            
            u_FB = -K*e;
            
            obj.u = u_FB + nominal_u;
            
            %logging
            if obj.ToLog
                obj.control_law.K  = K;
                obj.control_law.e  = e;
                obj.control_law.u_FB  = u_FB;
                obj.control_law.u_FF  = nominal_u;
            end
        end
    end
end