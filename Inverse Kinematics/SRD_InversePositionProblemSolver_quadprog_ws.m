function wsout = SRD_InversePositionProblemSolver_quadprog_ws(Task, TaskJacobian, Value, ws)

q0 = ws.X;
J = TaskJacobian(q0);
f0 = Task(q0);

H = J'*J;
f = 2*(f0 - Value)' * J;

%opts = optimoptions(@quadprog, 'Algorithm', 'interior-point-convex');
% x = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options) (H,f,A,b,Aeq,beq,lb,ub,ws) 
% ws = optimwarmstart(x0,options)
% [wsout,fval,exitflag,output,lambda] = quadprog(H,f,A,b,Aeq,beq,lb,ub,ws)
wsout = quadprog(H,f,[],[],[],[],[],[],ws);

end