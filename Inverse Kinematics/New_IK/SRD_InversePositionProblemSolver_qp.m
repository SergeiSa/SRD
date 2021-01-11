function q = SRD_InversePositionProblemSolver_qp(Task, TaskJacobian, Value, InitialGuess)

% IK without constrain  
% min ||J(q-q0)+r_k-r_k0|| 
% s.t q 


[H,f] = Objective(InitialGuess);
q = quadprog(H,f);

    function [H,f] = Objective(q)
        J = TaskJacobian(q);
        H = J'*J;
        f = -J'*(J*q+Value-Task(q));
    end

end