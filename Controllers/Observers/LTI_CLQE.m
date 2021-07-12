%Required input fields
%
%System.tol - tolerance
%
% dx/xdt = A*x + B*u + g + F*l
% y = C x
% G dx/xdt = 0
%
%
function Output = LTI_CLQE_test(System, SaveCopmutations)

if nargin < 2
    SaveCopmutations = 0;
end

A = System.A;
B = System.B;
C = System.C;
G = System.G;
g = System.g;

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

% temp = svd_suit([N, GG.row_space((size_x+1):end, :)], tol);
% R_DS = temp.left_null;

NA = svd_suit(N'*A, tol);
temp = svd_suit([NA.null, N, GG.row_space((size_x+1):end, :)], tol);
% R_ES = temp.left_null;
R_used = temp.left_null;

size_z    = size(N,  2);
size_zeta = size(R_used, 2);
size_chi  = size_z+size_zeta;

Output.sizes = struct('size_x', size_x, 'size_u', size_u, 'size_y', size_y, 'size_l', size_l, ...
    'size_z', size_z, 'size_zeta', size_zeta, 'size_chi', size_chi);

E = [N, R_used];
N1 = [N, zeros(size_x, size_zeta)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% observer gains design

if isfield(System.ControllerSettings, 'Cost')
    if size(System.ControllerSettings.Cost.Q, 2) == 1
        costQ = diag(System.ControllerSettings.Cost.Q(1:size_z));
    else
        costQ = System.ControllerSettings.Cost.Q;
    end
    if size(System.ControllerSettings.Cost.R, 2) == 1
        costR = diag(System.ControllerSettings.Cost.R(1:size_u));
    else
        costR = System.ControllerSettings.Cost.R;
    end
    
    Kz = lqr(N'*A*N, N'*B, costQ, costR);
else
    p = System.ControllerSettings.Poles(1:size(N, 2));
    Kz = place(N'*A*N, N'*B, p);
end

if isfield(System.ObserverSettings, 'Cost')
    if size(System.ObserverSettings.Cost.Q, 2) == 1
        costQ = diag(System.ObserverSettings.Cost.Q(1:size_chi));
    else
        costQ = System.ObserverSettings.Cost.Q;
    end
    if size(System.ObserverSettings.Cost.R, 2) == 1
        costR = diag(System.ObserverSettings.Cost.R(1:size_y));
    else
        costR = System.ObserverSettings.Cost.R;
    end
    
    L = lqr((N1'*A*E)', E'*C', costQ, costR);
else
    p = System.ObserverSettings.Poles(1:size(E, 2));
    L = place((N1'*A*E)', E'*C', p);
end
L = L';

Kzeta = pinv(N'*B) * N'*A*R_used;
K = [Kz, Kzeta];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% observer structure

Output.Controller.matrix_chi = K;
Output.Observer.matrix_chi = (N1'*A*E - L*C*E);
Output.Observer.matrix_u = N1'*B;
Output.Observer.matrix_y = L;
Output.Observer.vector = N1'*g;

Output.Observer.map = E;

Output.Matrices = struct('G', G, 'N', N, 'R', R, 'R_used', R_used, 'E', E, ...
    'Kz', Kz, 'Kzeta', Kzeta, 'K', K, 'L', L, ...
    'N1', N1);
Output.System = System;

if SaveCopmutations == 2
    return;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% desired

desired.x      = System.x_desired;
desired.dx     = System.dx_desired;
desired.z      = N'     *desired.x;
desired.dz     = N'     *desired.dx;
%desired.zeta   = R_used'*x_desired; 

desired.derivation.NB = svd_suit(N'*B, tol);
desired.derivation.NB_projector = desired.derivation.NB.left_null * desired.derivation.NB.left_null';
desired.derivation.M = svd_suit([-desired.derivation.NB_projector*N'*A*N, desired.derivation.NB_projector], tol);
desired.derivation.zdz_corrected = ...
    desired.derivation.M.pinv *desired.derivation.NB_projector*N'*g + ...%particular solution
    desired.derivation.M.null*desired.derivation.M.null' * [desired.z; desired.dz];

desired.z_corrected  = desired.derivation.zdz_corrected(1:size_z);
desired.dz_corrected = desired.derivation.zdz_corrected((1+size_z):end);

if norm( desired.derivation.NB.left_null*desired.derivation.NB.left_null'*...
        (desired.dz_corrected - N'*A*N*desired.z_corrected - N'*g) ) < tol

desired.u_corrected = desired.derivation.NB.pinv * (desired.dz_corrected - N'*A*N*desired.z_corrected - N'*g);
Output.Controller.vector = desired.u_corrected;

else
    warning('cannot create a node');
end

    

% desired.derivation.zu_particular = -desired.derivation.M.pinv * N'*g;
% 
% desired.derivation.map.z = [eye(size_z),           zeros(size_z, size_u)];
% desired.derivation.map.u = [zeros(size_u, size_z), eye(size_u)];
% desired.derivation.z_particular = -desired.derivation.map.z * desired.derivation.M.pinv * N'*g;
% desired.derivation.Projector = (desired.derivation.map.z* desired.derivation.N) * pinv(desired.derivation.map.z * desired.derivation.N);
% 
% desired.z_corrected = desired.derivation.z_particular + ...
%     desired.derivation.Projector * (desired.z - desired.derivation.z_particular);
% % desired.z_corrected = (eye(size(desired.derivation.Projector)) - desired.derivation.Projector) * desired.derivation.z_particular + ...
% %     desired.derivation.Projector * N'*x_desired;
% 
% z_des = desired.z_corrected;
% 
% if norm( (eye(size_z) - (N'*B)*pinv(N'*B)) *(N'*A*N*z_des + N'*g) ) < tol
%     u_des = -pinv(N'*B) *(N'*A*N*z_des + N'*g);
% else
%     warning('cannot create a node');
% end


if SaveCopmutations == 1
    Output.desired = desired;
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% IC

x_initial = System.x_initial;

InitialConditions.x    = x_initial;
InitialConditions.z    = N'     *x_initial;
InitialConditions.zeta = R_used'*x_initial;

InitialConditions.chi_estimate_random = 0.01*randn(size_chi, 1); 

Output.InitialConditions = InitialConditions;

zeta = InitialConditions.zeta;

u_des = desired.u_corrected;
z_des = desired.z_corrected ;
x_des = N*z_des + R_used*zeta;

desired.zeta_corrected = zeta;
desired.x_corrected    = x_des;

Output.desired = desired;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Projected LTI in z-chi coordinates

closed_loop.z_chi.Matrix = [N'*A*N,   -N'*B*K;
                            L*C*N,     (N1'*A*E - N1'*B*K - L*C*E)];
      
closed_loop.z_chi.Vector = [N'*A*R_used*zeta + N' *B*Kz*z_des + N' *B*u_des + N' *g;
                            L*C*R_used*zeta  + N1'*B*Kz*z_des + N1'*B*u_des + N1'*g];
                 

closed_loop.z_chi.ode_fnc = get_ode_fnc(closed_loop.z_chi.Matrix, closed_loop.z_chi.Vector);                       
closed_loop.z_chi.Y0 = [InitialConditions.z; InitialConditions.chi_estimate_random]; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% LTI in x-chi coordinates

closed_loop.x_chi.M = [eye(size_x),            zeros(size_x, size_chi),   -R;
                      zeros(size_chi, size_x), eye(size_chi,  size_chi),   zeros(size_chi, size_l);
                      G.self,                  zeros(size_l, size_chi),    zeros(size_l, size_l)];
iM = pinv(closed_loop.x_chi.M);
iM11 = iM(1:(size_x+size_chi), 1:(size_x+size_chi));

closed_loop.x_chi.Matrix = iM11*[A,     -B*K;
                                L*C,    (N1'*A*E - N1'*B*K - L*C*E)];
      

closed_loop.x_chi.Vector = iM11*[    B*Kz*z_des + B*u_des     + g;
                                N1'*B*Kz*z_des + N1'*B*u_des + N1'*g];


closed_loop.x_chi.ode_fnc = get_ode_fnc(closed_loop.x_chi.Matrix, closed_loop.x_chi.Vector);                       
closed_loop.x_chi.Y0 = [InitialConditions.x; InitialConditions.chi_estimate_random]; 

Output.closed_loop = closed_loop;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function fnc = get_ode_fnc(Matrix, Vector)
    fnc = @my_fnc;
    function dy = my_fnc(~, y)
        dy = Matrix * y + Vector;
    end

end


end