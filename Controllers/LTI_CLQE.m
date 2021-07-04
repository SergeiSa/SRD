%Required input fields
%
%System.tol - tolerance
%
% dx/xdt = A*x + B*u + g + F*l
% y = C x
% G dx/xdt = 0
%
%
function Output = LTI_CLQE(System, SaveCopmutations)

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

V = GG.null((size_x+1):end, :);

temp = svd_suit( ( pinv(N'*A)*(N'*A) * V*pinv(V) * R), tol);
R_used = temp.orth;

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
    Kz = lqr(N'*A*N, N'*B, System.ControllerSettings.Cost.Q, System.ControllerSettings.Cost.R);
else
    p = System.ControllerSettings.Poles(1:size(N, 2));
    Kz = place(N'*A*N, N'*B, p);
end

if isfield(System.ObserverSettings, 'Cost')
    L = lqr((N1'*A*E)', E'*C', System.ObserverSettings.Cost.Q, System.ObserverSettings.Cost.R);
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

if SaveCopmutations == 2
    return;
end

x_desired = System.x_desired;

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

if norm( (eye(size_z) - (N'*B)*pinv(N'*B)) *(N'*A*N*z_des + N'*g) ) < tol
    u_des = -pinv(N'*B) *(N'*A*N*z_des + N'*g);
else
    warning('cannot create a node');
end

desired.u_corrected = u_des;

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Output.closed_loop = closed_loop;
Output.Matrices = struct('G', G, 'N', N, 'R', R, 'R_used', R_used, 'E', E, ...
    'Kz', Kz, 'Kzeta', Kzeta, 'K', K, 'L', L, ...
    'N1', N1);

Output.Controller.vector = u_des;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function fnc = get_ode_fnc(Matrix, Vector)
    fnc = @my_fnc;
    function dy = my_fnc(~, y)
        dy = Matrix * y + Vector;
    end

end


end