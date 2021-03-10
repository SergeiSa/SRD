function dq = SRD_InversePositionProblemSolver_Ode_Dynamics(t,q0,dx,J,H_mat)

% IK base on dq with dynamics parameters 
% min ||q_dot^T.H.q_dot + J.q_dot-x_dot|| 
% st. q_dot 

H = H_mat(q0) +J(q0)'*J(q0);
f = -dx'*J(q0);
A = [];
b = [];

dq = quadprog(H,f,A,b);
end 