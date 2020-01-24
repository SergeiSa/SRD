%This class solves nonlinear eq. of the form
%f(q) = a, df/dt = b, d^2f/dt^2 = c,
%q = q(t)
classdef TPNonlinEqSolver_Projection < handle
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
        function [q, v, a] = Solve(~, problem)
            
            %find RHSa, use it to find the dimentions of the q, v, a
            [RHSa, RHSb, ~] = problem.GetTask(problem.t);
            N = length(problem.x0);
            
            function Output = SolveFor_ds(t, s)
                local.q = s(1:N);
                local.v = s((N + 1):(2*N));
                
                %find Jacobians
                local.J1 = problem.FirstJacobian(local.q);
                local.J2 = problem.SecondJacobian(local.q, local.v);
                
                %To solve for v and a we construct a linear system:
                %A*ds = [RHSb; RHSc]; ds = [v; a];
                local.A = [local.J1, zeros(size(local.J1));
                           local.J2, local.J1];
                
                [~, local.RHSb, local.RHSc] = problem.GetTask(t);
                
                %we solve the linear system
                ds = local.A \ [local.RHSb; local.RHSc];
                Output.ds = ds;
            end
            
            %First we find an approximation of the initial point
            %q0 = problem.x0, J1*v0 = RHSb, v = dq/dt
            J1_0 = problem.FirstJacobian(problem.x0);
            s0 = [problem.x0; (J1_0 \ RHSb)];
            
            %then find Slope using the Runge method
            Output1 = SolveFor_ds(problem.t, s0);
            Output2 = SolveFor_ds(problem.t + 0.5 * problem.TimeStep, s0 + 0.5 * Output1.ds * problem.TimeStep);
            Output3 = SolveFor_ds(problem.t + 0.5 * problem.TimeStep, s0 + 0.5 * Output2.ds * problem.TimeStep);
            Output4 = SolveFor_ds(problem.t + problem.TimeStep,       s0 +       Output3.ds * problem.TimeStep);
            
            Slope = (Output1.ds + 2*Output2.ds + 2*Output3.ds + Output4.ds)/6;
            
            %this is value of q found using integration
            q0 = problem.x0 + Slope(1:N) * problem.TimeStep;
            
            v = Output1.ds(1:N);
            a = Output1.ds((N + 1):(2*N));
            
            %We have eq. f(q) = RHSa;
            %We linearize it to J1*q = b; 
            %where b = RHSa - (f(q0) - J1*q0)
            J1 = problem.FirstJacobian(q0);
            b = RHSa - (problem.objective(q0) - J1*q0);
            
            %we denote null space of J1 as N;
            %here we find basis in N
            NSB = null(J1);
            
            %find projector onto N
            %note that NSB has independent columns (basis vectors in the
            %null space of J1, the same length as q), so the following
            %formula is appropriate.
            P = NSB*((NSB'*NSB) \ NSB');
            
            %find particular solution for J1*q0 = b system
            q_particular = J1 \ b;
            
            %project the particular solution onto the kernel of J1 (kernel
            %of J1 is orthagonal to N); It is done to make sure q has a
            %zero projection on null space N
            q_projected = q_particular - P*q_particular;
            
            % Project q0 onto null space N using projector P, and then shift
            % it by projected onto kernel of J1 particular solution q_projected.
            % If q0 is already in the set of solutions, then it could be
            % decomposed in a sum q0 = v1 + c, where v1 is in null space N
            % and c = q_projected. Then:
            % P*q0 + c = P*(v1 + c) + c = v1 + c = q0, since P*v1 = v1 and P*c = 0,
            % because we made sure it is so above by projecting q_projected
            % into the kernel of J1.
            % This result is important, as it assures the method won't be
            % needlessly changing already viable solutions.
            q = P*q0 + q_projected;
        end
        
        
    end
end