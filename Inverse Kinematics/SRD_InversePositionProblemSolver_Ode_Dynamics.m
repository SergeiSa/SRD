
% IK base on dq with dynamics parameters 
% min ||q_dot^T.H.q_dot + J.q_dot-x_dot|| 
% st. q_dot 

function dq_opt = SRD_InversePositionProblemSolver_Ode_Dynamics(q0,dx,J,dof)

H =J(q0)'*J(q0);% D_mat(q0) +
f = -dx'*J(q0);
A = vertcat(0.1*eye(dof),-0.1*eye(dof));
q_min = -0.5*ones([dof,1]);
q_max = 0.5*ones([dof,1]);
b = vertcat(q_max,-q_min);

dq_opt = quadprog(H,f,A,b);


% 
% 
% cvx_begin %quiet
% variables dq(3);
% minimize( norm(J(q0)*dq - dx) );
% subject to
%     dq >= q_min;
%     dq <= q_max;
% %     J(q0)*dq == dx;
% cvx_end
% 
% dq_opt = dq;



end 

