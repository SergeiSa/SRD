%This class solves nonlinear eq. of the form
%f(q) = a, df/dt = b, d^2f/dt^2 = c,
%q = q(t)
classdef TPNonlinEqSolver_discretizationQP < handle
    properties
        
    end
    methods
        %the method solves nonlinear eq. of the form
        %f(q) = RHSa, df/dt = RHSb, d^2f/dt^2 = RHSc,
        %v = dq/dt, a = dv/dt.
        %Input parameter is a structure problem with fields:
        %.FirstJacobian - Jacobian function handle that takes q and returns J1
        %.SecondJacobian - Jacobian function handle that takes q, v and returns J2
        %J1 = df/dq, g = df/dt, J2 = dg/dq.
        %.GetTask - a function of t that returns [RHSa, RHSb, RHSc];
        %.x - initial value of q; (*)
        %.objective - handle to function f(q); (*)
        %
        %(*) see description of problem parameter for MATLAB
        %optimization solvers, like fmincon
        function [q, v, a] = Solve(~, problem, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'TPNonlinEqSolver_discretizationQP.Solve';
            
            %OrtegaSpongCTC_Estimator
            Parser.addOptional('quadprog_problem', []);
            Parser.addOptional('Aineq', []);
            Parser.addOptional('bineq', []);
            Parser.parse(varargin{:});
            
            
            %find RHSa, use it to find the dimentions of the q, v, a
            [task, d_task, dd_task] = problem.GetTask(problem.t);
            
            q0 = problem.past_q;
            q1 = problem.x0;
            
            dt = problem.TimeStep;
            
            J = problem.FirstJacobian(q1);
            dJ = problem.SecondJacobian(q1, problem.current_v);
            
            linearization_constant = problem.objective(q1) - J*q1;
            
            LHS_matrix = [J; (J + dt*dJ); J];
            RHS_vector = [(d_task*dt + J*q1);
                          (dd_task*dt^2 + dJ*q1*dt + J*(2*q1 - q0));
                          (task - linearization_constant)];
                     
            if isempty(Parser.Results.quadprog_problem)
                n = length(q1);
                quadprog_problem.H = eye(n);
                quadprog_problem.f = quadprog_problem.H*q1; %for (q1 - q)'*H*(q1 - q) cost
                quadprog_problem.solver = 'quadprog';
                quadprog_problem.options = optimoptions('quadprog', 'Display', 'off');
            else
                quadprog_problem = Parser.Results.quadprog_problem;
            end
            
            if ~isempty(Parser.Results.Aineq)
                quadprog_problem.Aineq = Parser.Results.Aineq;
                quadprog_problem.bineq = Parser.Results.bineq;
            end
            
            quadprog_problem.Aeq = LHS_matrix;
            quadprog_problem.beq = RHS_vector;
            
            q = quadprog(quadprog_problem);
            v = (q - q1) / dt;
            a = (q - 2*q1 + q0) / dt^2;
        end
    end
end