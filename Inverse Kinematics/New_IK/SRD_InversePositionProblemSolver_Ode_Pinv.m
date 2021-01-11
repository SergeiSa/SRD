function dq = SRD_InversePositionProblemSolver_Ode_Pinv(t,q0,dx,TaskJacobian,H)
% IK base on equality solution 
% dq = J^+*(dx)
% dx : task velocity 

J = TaskJacobian(q0);
% v = [-sin(t),cos(t)]
dq = pinv(J)*dx;
end




