function q = SRD_InversePositionProblemSolver_pinv(Task, TaskJacobian, Value, InitialGuess)

% IK equality solver using Pseudo inverse Jacobain 
% q = J^+(Jq0+r_k-r_k0)

q0 = InitialGuess;
[F,J] = Objective(q0);
q = pinv(J)*(J*q0+F);

    function [F,J] = Objective(q)
        F =  Value-Task(q);
        J = TaskJacobian(q);
    end

end