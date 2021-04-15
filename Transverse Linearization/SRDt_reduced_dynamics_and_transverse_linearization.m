function description = SRDt_reduced_dynamics_transverse_linearization(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_transverse_linearization';
% Parser.addOptional('SymbolicEngine', []);

Parser.addOptional('Symbolic_ToOptimizeFunctions', true);
Parser.addOptional('Casadi_cfile_name', 'transv_dynamics');

%H*ddq + C*dq + g = T*u
Parser.addOptional('H_callable', []);
Parser.addOptional('C_callable', []);
Parser.addOptional('g_callable', []);
Parser.addOptional('T_callable', []);

Parser.addOptional('FunctionName_alpha', 'get_alpha');
Parser.addOptional('FunctionName_beta', 'get_beta');
Parser.addOptional('FunctionName_gamma', 'get_gamma');


Parser.addOptional('FunctionName_U', 'get_U_ff');
Parser.addOptional('FunctionName_N', 'get_N');
Parser.addOptional('FunctionName_A', 'get_A_transv');
Parser.addOptional('FunctionName_B', 'get_B_transv');

Parser.addOptional('Path', 'autogen_transv_dynamics/');

Parser.addOptional('N_dof', []);

Parser.addOptional('c0', []); % s = c0*q
Parser.addOptional('H0', []); % H0*q = Phi(s)

Parser.parse(varargin{:});



% if isempty(Parser.Results.SymbolicEngine)
%     error('Please provide SymbolicEngine')
% else
%     SymbolicEngine = Parser.Results.SymbolicEngine;
% end

if ~isempty(Parser.Results.Path)
    if ~exist(Parser.Results.Path, 'dir')
        mkdir(Parser.Results.Path)
    end
end

N_dof = Parser.Results.N_dof;
c0 = Parser.Results.c0;
H0 = Parser.Results.H0;

% in this script I derive the transverse dynamics for the case when
% the motion generator is the linear combination of generalized coordinates

run_sanity_checks = 1;

% % path to urdf file
% path_to_urdf = 'pendubot_description/planar_manip.urdf';
% % create pendubot instance
% robot = Pendubot(path_to_urdf); % pendubot instance

% create symbolic variabes
% ---------------------------------------------------------------------
% transversal coordinates
y = sym('y', [N_dof-1,1], 'real');
y_dot = sym('y_dot', [N_dof-1,1], 'real');
y_2dot = sym('y_2dot', [N_dof-1,1], 'real');

% motion generator
s = sym('s', 'real');
s_dot = sym('s_dot', 'real');
s_2dot = sym('s_2dot', 'real');

% virtual constraints H0*q = phi(s)
phi = sym('phi', [N_dof-1,1], 'real');
phi_prm = sym('phi_prm', [N_dof-1,1], 'real');
phi_2prm = sym('phi_2prm', [N_dof-1,1], 'real');

% control signal of the linearized system
v = sym('v', [N_dof-1,1], 'real');

% dummy variable used in computations
% d = sym('d', 'real');
% d = sym('d', [N_dof-1,1], 'real');

% Symbolic variables for virtual constraints q = Phi(s)

Phi = sym('phi', [N_dof,1], 'real');
Phi_prm = sym('phi%d_prm', [N_dof,1], 'real');
Phi_2prm = sym('phi%d_2prm', [N_dof,1], 'real');

% ----------------------------------------------------------------------
% Compute reduced dynamics
% ----------------------------------------------------------------------
% Annihilator of B matrix
B = Parser.Results.T_callable();
B_anh = null(B')';

% Change of coordinates from q to Phi
M_phi = Parser.Results.H_callable(Phi);
C_phi = Parser.Results.C_callable(Phi, Phi_prm); 
G_phi = Parser.Results.g_callable(Phi);

% Obtaining alpha, beta, gamma expressions
alpha = B_anh * M_phi * Phi_prm;
beta = B_anh * (M_phi * Phi_2prm + C_phi * Phi_prm );
gamma = B_anh * G_phi;


% Generate functions
% -------------------------------------------------------------------
FileName_alpha = [Parser.Results.Path, Parser.Results.FunctionName_alpha];
FileName_beta = [Parser.Results.Path, Parser.Results.FunctionName_beta];
FileName_gamma = [Parser.Results.Path, Parser.Results.FunctionName_gamma];
        
        
matlabFunction(alpha, 'File', FileName_alpha, 'Vars', {Phi, Phi_prm});
matlabFunction(beta, 'File', FileName_beta, 'Vars', {Phi, Phi_prm, Phi_2prm});
matlabFunction(gamma, 'File', FileName_gamma, 'Vars', {Phi});


% perform change of coordinates. here H0*q = phi; s = c0*q
% ---------------------------------------------------------------------
q = [H0;c0]\[phi+y;s];
% q_dot = L*[y_dot; s_dot];
L = jacobian(q, [y; s]) + [zeros(N_dof, N_dof-1) jacobian(q, phi)*phi_prm];
% q_2dot = L*[y_2dot; s_2dot] + L_dot*[y_dot; s_dot]
% L_dot = diff(L, phi_prm)*phi_2prm*s_dot; % old version for 2 degrees
L_dot = zeros(N_dof,N_dof);
for i = 1:N_dof-1
    L_dot = L_dot + diff(L, phi_prm(i))*phi_2prm(i)*s_dot;
end


% getting dynamics in new coordinates
% --------------------------------------------------------------------

M_ys = Parser.Results.H_callable(q);
C_ys = Parser.Results.C_callable(q, L*[y_dot; s_dot]);
g_ys = Parser.Results.g_callable(q);
B_ys = Parser.Results.T_callable();


inv_M_ys = M_ys\eye(N_dof);
inv_L = L\eye(N_dof);

% dynamics of tranversal coordinates y_2dot = R + N*u
N = [eye(N_dof-1) zeros(N_dof-1,1)]*inv_L*inv_M_ys*B_ys;
R = -[eye(N_dof-1) zeros(N_dof-1,1)]*(inv_L*inv_M_ys*(C_ys*L*[y_dot; s_dot] + g_ys) + ...
            inv_L*L_dot*[y_dot; s_dot]);
% feedforward control term
U = -N\R; 
% y dynamics in open-loop
F = R + N*U;

% dynamics of the motion generator
s_dyn = [zeros(1,N_dof-1) 1]*(M_ys*(L*[y_2dot; s_2dot] + L_dot*[y_dot; s_dot]) + ...
               C_ys*L*[y_dot; s_dot] + g_ys);
% zero transversal coordinates (the system is on zero manifold)
s_abg = simplify(subs(s_dyn, [y, y_dot, y_2dot], zeros(N_dof-1,3)));

alpha = simplify(diff(s_abg, s_2dot));
beta = 0.5*simplify(diff(diff(s_abg, s_dot), s_dot));
gamma = subs(s_abg, [s_dot; s_2dot], zeros(2,1));

if run_sanity_checks
    alpha2 = get_alpha([H0;c0]\[phi;s], ...
                             [H0;c0]\[phi_prm;1]);
    beta2 = get_beta([H0;c0]\[phi;s], ...
                           [H0;c0]\[phi_prm;1], ...
                           [H0;c0]\[phi_2prm;0]);
    gamma2 = get_gamma([H0;c0]\[phi;s]);

    assert(double(simplify(alpha - alpha2)) == 0);
    assert(double(simplify(beta - beta2)) == 0);
    assert(double(simplify(gamma - gamma2)) == 0);
end

% get right hand side of the alpha-beta-gamma (i.e. when the system 
% is not on zero manifold)
W = -s_dyn + alpha*s_2dot + beta*s_dot^2 + gamma;

if run_sanity_checks
   assert(double(simplify(subs(W, [y; y_dot; y_2dot], zeros(3*(N_dof-1),1)))) == 0); 
end

% W depends on y_2d, in case of feedback linearization we can substitute it
% with control signal v, because the dynamics is of double integrator
W = subs(W, y_2dot, v);


% transverse linearization
% --------------------------------------------------------------------
% Linearize integral dynamics
temp = simplify(subs(W, [y_dot; v], zeros(2*(N_dof-1),1)));
W_y = simplify(jacobian(temp, y)); 

temp = simplify(subs(W, [y; v], zeros(2*(N_dof-1),1)));
W_yd = simplify(jacobian(temp, y_dot)); 
        
temp = simplify(subs(W, [y; y_dot], zeros(2*(N_dof-1),1)));
W_v = simplify(jacobian(temp, v)); 

temp = simplify(subs(W, [y; y_dot; v], zeros(3*(N_dof-1),1)));
W_I = simplify((s_dot*simplify(diff(temp, s_dot)) - ...
                s_2dot*simplify(diff(temp,s)))/(2*(s_dot^2 + s_2dot^2)));
            
W_y = subs(W_y, y, zeros(N_dof-1,1));
W_yd = subs(W_yd, y_dot, zeros(N_dof-1,1));
W_v = subs(W_v, v, zeros(N_dof-1,1));
            
a = simplify(2*s_dot/(alpha));
a_11 = simplify(a*(W_I - simplify(beta)));
a_12 = simplify(a*W_y);
a_13 = simplify(a*W_yd);
b1 = simplify(a*W_v);

% Here we linearize y dynamics
temp = simplify(subs(F, [y; y_dot; v], zeros(3*(N_dof-1),1)));
A21 = simplify((s_dot*simplify(diff(temp,s_dot)) - ...
               s_2dot*simplify(diff(temp,s)))/(2*(s_dot^2 + s_2dot^2)));
temp = simplify(subs(F, [y_dot; v], zeros(2*(N_dof-1),1)));
A22 = subs(jacobian(temp,y), y, zeros(N_dof-1,1));
temp = simplify(subs(F, [y; v], zeros(2*(N_dof-1),1)));
A23 = subs(jacobian(temp,y_dot), y_dot, zeros(N_dof-1,1) );
B2 = eye(N_dof-1); 

% Here we put linearized transverse dynamics together
A = [a_11 a_12 a_13; zeros(N_dof-1,N_dof) eye(N_dof-1); A21 A22 A23];
B = [b1; zeros(N_dof-1,N_dof-1); B2];

% substitute s_2dot
A = simplify(subs(A,s_2dot,(-beta*s_dot^2 - gamma)/alpha));


% Generate functions
% -------------------------------------------------------------------
FileName_U = [Parser.Results.Path, Parser.Results.FunctionName_U];
FileName_N = [Parser.Results.Path, Parser.Results.FunctionName_N];
FileName_A = [Parser.Results.Path, Parser.Results.FunctionName_A];
FileName_B = [Parser.Results.Path, Parser.Results.FunctionName_B];
        
        
matlabFunction(U, 'File', FileName_U, 'Vars', ...
               {s, s_dot, y, y_dot, phi, phi_prm, phi_2prm});
matlabFunction(N, 'File', FileName_N, 'Vars', ...
               {s, y, phi, phi_prm});
matlabFunction(A, 'File', FileName_A,'Vars', ...
               {s, s_dot, phi, phi_prm, phi_2prm});
matlabFunction(B,'File',FileName_B,'Vars', ...
               {s, s_dot, phi, phi_prm});
           
end