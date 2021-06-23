%Required input fields
%
%System.tol - tolerance
%
% dx/xdt = A*x + B*u + g + F*l
% y = C x
% G dx/xdt = 0
%
%
function Output = LTI_CLQE(System)

A = System.A;
B = System.B;
C = System.C;
G = System.G;
g = System.g;

ControllerCost = System.ControllerCost;
ObserverCost   = System.ObserverCost;

x_desired = System.x_desired;
x_initial = System.x_initial;

tol = System.tol;

size_x = size(A, 2);
size_u = size(B, 2);
size_y = size(C, 1);
size_l = size(G, 1);

G = svd_suit(G, tol);
N = G.null; R = G.row_space; %E = [N, R];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Derivative-State Constraints and Effective States 
%D1*dx/dt + D2*x = 0
dof = size_x/2;
D1 = [eye(dof), zeros(dof)];
D2 = [zeros(dof), eye(dof)];

GG = [G.self, zeros(size_l, size_x); D1, D2];
GG = svd_suit(GG, tol);

V = GG.null((size_x+1):end, :);

temp = svd_suit( ( pinv(N'*A)*(N'*A) * V*pinv(V) * R), tol);
R_used = temp.orth;

E = [N, R_used];

size_z    = size(N,  2);
size_zeta = size(R_used, 2);
size_xi   = size_z+size_zeta;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Kz    = lqr(N'*A*N, N'*B, ControllerCost.Q, ControllerCost.R);
Kzeta = pinv(N'*B) * N'*A*R_used;
K = [Kz, Kzeta];

N1 = [N, zeros(size_x, size_zeta)];

L = lqr((N1'*A*E)', E'*C', ObserverCost.Q, ObserverCost.R);
L = L';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% IC
InitialConditions.x    = x_initial;
InitialConditions.z    = N'     *x_initial;
InitialConditions.zeta = R_used'*x_initial;

InitialConditions.xi_estimate_random = 0.01*randn(size_xi, 1); 

zeta = InitialConditions.zeta;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% desired

desired.x      = x_desired;
desired.z      = N'     *x_desired;
desired.zeta   = R_used'*x_desired; 

desired.derivation.M = svd_suit([N'*A*N, N'*B], tol);
desired.derivation.N = desired.derivation.M.null;
desired.derivation.zu_particular = -desired.derivation.M.pinv * N'*g;

desired.derivation.map.z = [eye(size_z),           zeros(size_z, size_u)];
desired.derivation.map.u = [zeros(size_u, size_z), eye(size_u)];
desired.derivation.z_particular = -desired.derivation.map.z * desired.derivation.M.pinv * N'*g;
desired.derivation.Projector = (desired.derivation.map.z* desired.derivation.N) * pinv(desired.derivation.map.z * desired.derivation.N);

desired.z_corrected = desired.derivation.z_particular + ...
    desired.derivation.Projector * (desired.z - desired.derivation.z_particular);
% desired.z_corrected = (eye(size(desired.derivation.Projector)) - desired.derivation.Projector) * desired.derivation.z_particular + ...
%     desired.derivation.Projector * N'*x_desired;

z_des = desired.z_corrected;
x_des = N*z_des + R_used*zeta;

if norm( (eye(size_z) - (N'*B)*pinv(N'*B)) *(N'*A*N*z_des + N'*g) ) < tol
    u_des = -pinv(N'*B) *(N'*A*N*z_des + N'*g);
else
    warning('cannot create a node');
end

desired.zeta_corrected = zeta;
desired.x_corrected    = x_des;
desired.u_corrected    = u_des;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Projected LTI in z-xi coordinates

closed_loop.z_xi.Matrix = [N'*A*N,   -N'*B*K;
                           L*C*N,     (N1'*A*E - N1'*B*K - L*C*E)];
      
closed_loop.z_xi.Vector = [N'*A*R_used*zeta + N' *B*Kz*z_des + N' *B*u_des + N' *g;
                           L*C*R_used*zeta  + N1'*B*Kz*z_des + N1'*B*u_des + N1'*g];
                 

closed_loop.z_xi.ode_fnc = get_ode_fnc(closed_loop.z_xi.Matrix, closed_loop.z_xi.Vector);                       
closed_loop.z_xi.Y0 = [InitialConditions.z; InitialConditions.xi_estimate_random]; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% LTI in x-xi coordinates

closed_loop.x_xi.M = [eye(size_x),            zeros(size_x, size_xi),   -R;
                      zeros(size_xi, size_x), eye(size_xi,  size_xi),    zeros(size_xi, size_l);
                      G.self,                 zeros(size_l, size_xi),    zeros(size_l, size_l)];
iM = pinv(closed_loop.x_xi.M);
iM11 = iM(1:(size_x+size_xi), 1:(size_x+size_xi));

closed_loop.x_xi.Matrix = iM11*[A,     -B*K;
                                L*C,    (N1'*A*E - N1'*B*K - L*C*E)];
      

closed_loop.x_xi.Vector = iM11*[    B*Kz*z_des + B*u_des     + g;
                                N1'*B*Kz*z_des + N1'*B*u_des + N1'*g];


closed_loop.x_xi.ode_fnc = get_ode_fnc(closed_loop.x_xi.Matrix, closed_loop.x_xi.Vector);                       
closed_loop.x_xi.Y0 = [InitialConditions.x; InitialConditions.xi_estimate_random]; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Output.sizes = struct('size_x', size_x, 'size_u', size_u, 'size_y', size_y, 'size_l', size_l, ...
    'size_z', size_z, 'size_zeta', size_zeta, 'size_xi', size_xi);
Output.InitialConditions = InitialConditions;
Output.desired = desired;
Output.closed_loop = closed_loop;
Output.Matrices = struct('G', G, 'N', N, 'R', R, 'R_used', R_used, 'E', E, ...
    'Kz', Kz, 'Kzeta', Kzeta, 'K', K, 'L', L, ...
    'N1', N1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function fnc = get_ode_fnc(Matrix, Vector)
    fnc = @my_fnc;
    function dy = my_fnc(~, y)
        dy = Matrix * y + Vector;
    end

end


end