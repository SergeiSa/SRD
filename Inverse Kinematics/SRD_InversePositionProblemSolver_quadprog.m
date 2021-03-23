function q = SRD_InversePositionProblemSolver_quadprog(Task, TaskJacobian, Value, q0, opts)

if nargin < 5
    opts = optimoptions(@quadprog, 'Algorithm', 'interior-point-convex');
end

J = TaskJacobian(q0);
f0 = Task(q0);

H = 2*J'*J;
f = 2*(f0 - J*q0 - Value)' * J;

% x = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options) 
q = quadprog(H,f,[],[],[],[],[],[],q0,opts);
end