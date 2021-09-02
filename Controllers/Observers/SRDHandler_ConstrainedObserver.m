classdef SRDHandler_ConstrainedObserver < SRDHandler
    properties
        Handler_CLQE;
        Handler_ObserverState;
        Handler_MeasuredOutput;
        MainController;
        Handler_InverseDynamics;
        Handler_ControlInput_StateSpace;
        Handler_Time;
        tol;
        
        ObserverSimulation_Euler = true;
        
        testing_inputs_handler;
    end
    methods
        function obj = SRDHandler_ConstrainedObserver(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_ConstrainedObserver';
            Parser.addOptional('Handler_CLQE', []);
            Parser.addOptional('Handler_ObserverState', []);
            Parser.addOptional('Handler_MeasuredOutput', []);
            Parser.addOptional('MainController', []);
            Parser.addOptional('Handler_InverseDynamics', []);
            Parser.addOptional('Handler_ControlInput_StateSpace', []);
            Parser.addOptional('Handler_Time', []);
            Parser.addOptional('tol', 10^(-6));
            Parser.parse(varargin{:});
            
            obj.Handler_CLQE                    = Parser.Results.Handler_CLQE;
            obj.Handler_ObserverState           = Parser.Results.Handler_ObserverState;
            obj.Handler_MeasuredOutput          = Parser.Results.Handler_MeasuredOutput;
            obj.MainController                  = Parser.Results.MainController;
            obj.Handler_InverseDynamics         = Parser.Results.Handler_InverseDynamics;
            obj.Handler_ControlInput_StateSpace = Parser.Results.Handler_ControlInput_StateSpace;
            obj.Handler_Time                    = Parser.Results.Handler_Time;
            obj.tol                             = Parser.Results.tol;
        end
        
        function Update(obj)
            
            E = obj.Handler_CLQE.Solution.Observer.map;
            
            u = obj.MainController.u;
            y = obj.Handler_MeasuredOutput.y;
            chi = E' * obj.Handler_ObserverState.x;
            x0 = obj.Handler_ObserverState.x - E*chi;
            
            dt = obj.Handler_Time.dt;
            t = obj.Handler_Time.CurrentTime;  
            desired = obj.Handler_ControlInput_StateSpace.get_x_dx(t);
            desired_x =  desired(:, 1);
            desired_u = obj.Handler_InverseDynamics.u; 
            
            dchi = obj.Handler_CLQE.Solution.Observer.matrix_chi * (chi - E'*desired_x) + ...
                   obj.Handler_CLQE.Solution.Observer.matrix_u * (u- desired_u) + ...
                   obj.Handler_CLQE.Solution.Observer.matrix_y * (y- obj.Handler_MeasuredOutput.C*desired_x) + ...
                   obj.Handler_CLQE.Solution.Observer.vector;
            
               
            chi = chi + dchi*dt;

            x = x0 + E*chi;
            
            obj.Handler_ObserverState.x = x;
        end
    end
end