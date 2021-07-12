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
            Parser.addOptional('tol', []);
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
            
            %uu = obj.Handler_CLQE.Solution.Controller.matrix_chi*chi + obj.Handler_CLQE.Solution.Controller.vector;
            
            dchi = obj.Handler_CLQE.Solution.Observer.matrix_chi * (chi - E'*desired_x) + ...
                   obj.Handler_CLQE.Solution.Observer.matrix_u * (u- desired_u) + ...
                   obj.Handler_CLQE.Solution.Observer.matrix_y * (y- obj.Handler_MeasuredOutput.C*desired_x) + ...
                   obj.Handler_CLQE.Solution.Observer.vector;
            
%             desired_dx = desired(:, 2);
%             desired_q = desired_x(1:dof);
%             desired_v = desired_x((dof + 1):end);  
               
% % %             System =  obj.Handler_CLQE.Solution.System;
% % %             k = size(System.G, 1);
% % %             n = obj.Handler_CLQE.Solution.sizes.size_x;
% % %             
% % %             N1 = obj.Handler_CLQE.Solution.Matrices.N1;
% % %             
% % %             rhs = System.A * (obj.Handler_ObserverState.x - desired_x) + ...
% % %                   System.B * (u - desired_u) + ...
% % %                   System.g;
% % %             M = [eye(size(System.A, 2)), obj.Handler_CLQE.Solution.Matrices.R; System.G, zeros(k)]; 
% % %             vec = pinv(M)*[rhs; zeros(k, 1)];
% % %             dx_calc = vec(1:n);
% % %             dx = [obj.testing_inputs_handler.v;
% % %                               obj.testing_inputs_handler.a];
% % %             dchi_real = E' * dx;
% % %             dchi_cal = E' * dx_calc;
% % %             dchi_maybe = obj.Handler_CLQE.Solution.Observer.matrix_chi * chi + ...
% % %                    obj.Handler_CLQE.Solution.Observer.matrix_u * u + ...
% % %                    obj.Handler_CLQE.Solution.Observer.matrix_y * y + ...
% % %                    obj.Handler_CLQE.Solution.Observer.vector + ...
% % %                    N1'*(System.A * desired_x + System.B * desired_u);
% % %             dchi_maybe2 = obj.Handler_CLQE.Solution.Observer.matrix_chi * (chi - E'*desired_x) + ...
% % %                    obj.Handler_CLQE.Solution.Observer.matrix_u * (u- desired_u) + ...
% % %                    obj.Handler_CLQE.Solution.Observer.matrix_y * (y- System.C*desired_x)+ ...
% % %                    obj.Handler_CLQE.Solution.Observer.vector;
               
               
            chi = chi + dchi*dt;
            
%             Et = svd_suit(E', obj.tol);
%             x = Et.null*Et.null'*obj.Handler_ObserverState.x + ...
%                 (eye(obj.Handler_CLQE.Solution.sizes.size_x) - Et.null*Et.null') * Et.pinv * chi;

            x = x0 + E*chi;
            
            obj.Handler_ObserverState.x = x;
        end
    end
end