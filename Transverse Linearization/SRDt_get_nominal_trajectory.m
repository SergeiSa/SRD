function  [s_str, sd_str, q_str, qd_str, qdd_str, T, A, B] = SRDt_get_nominal_trajectory(varargin)
    Parser = inputParser;
    Parser.FunctionName = 'get_nominal_trajectory';
    Parser.addOptional('Handler_reduced_dynamics_and_transverse_linearization', []);
    Parser.addOptional('p', []); % virtual constraints parameter vector, e.g. phi(s) = p(1) + p(2)*s...
    Parser.addOptional('vrtl_cnstr_obj', []); % virtual constraints class object
    Parser.addOptional('s0', []); % initial value of motion generator
    Parser.addOptional('dt', []); % discretization time
    
    Parser.parse(varargin{:});
    
    h = Parser.Results.Handler_reduced_dynamics_and_transverse_linearization;
    p = Parser.Results.p;

    vrtl_cnstr = Parser.Results.vrtl_cnstr_obj;
    
    %     func1 = @(s) get_alpha(vrtl_cnstr.Phi(s, p), vrtl_cnstr.Phi_prm(s, p));
    %     func2 = @(s) get_gamma(vrtl_cnstr.Phi(s, p));
    %     s_interval = [-10 10];
    % fplot(func1,s_interval)
    % roots1 = fzero(func1,0)
    % fplot(func2,s_interval)
    % roots2 = fzero(func2,0)

    tspan= 0:Parser.Results.dt:10;
    x0 = [Parser.Results.s0;0];
    optns = odeset('RelTol',1e-9,'AbsTol',1e-9,'NormControl','on');

    [t, x] = ode45( @(t,x)reduced_dynamics_ode(t,x,vrtl_cnstr.Phi(x(1), p), ...
        vrtl_cnstr.Phi_prm(x(1), p),...
        vrtl_cnstr.Phi_2prm(x(1), p), h), tspan, x0, optns);

    [~,locs] = findpeaks(x(:,1));
    T = t(locs(1)); 
    T_ind = locs(1);    
    
    s_str = x(1:T_ind,1);
    sd_str = x(1:T_ind,2);
    
    sdd_str = zeros(T_ind,1);

    for i=1:length(T_ind)
        sdd_str(i) = -1/h.get_alpha(vrtl_cnstr.Phi(x(i,1), p), vrtl_cnstr.Phi_prm(x(i,1), p))*...
                  (h.get_gamma(vrtl_cnstr.Phi(x(i,1), p)) + ...
                  h.get_beta(vrtl_cnstr.Phi(x(i,1), p), vrtl_cnstr.Phi_prm(x(i,1), p), ...
                  vrtl_cnstr.Phi_2prm(x(i,1), p))*x(i,2)^2);
    end

    q_str = zeros(2, T_ind);
    qd_str = zeros(2, T_ind);
    qdd_str = zeros(2, T_ind);
    for i = 1:T_ind
        q_str(:,i) = vrtl_cnstr.Phi(s_str(i), p);
        qd_str(:,i) = vrtl_cnstr.Phi_prm(s_str(i), p)*sd_str(i);
        qdd_str(:,i) = vrtl_cnstr.Phi_2prm(s_str(i), p)*sd_str(i)^2 + ...
            vrtl_cnstr.Phi_prm(s_str(i), p)*sdd_str(i);
    end

    A = zeros(  2*h.N_dof-1,2*h.N_dof-1,length(s_str));
    B = zeros(2*h.N_dof-1,h.N_dof-1,length(s_str));

    for i = 1:length(s_str)
        s_i = s_str(i);
        sd_i = sd_str(i);
        phi_i = h.H0*vrtl_cnstr.Phi(s_i, p);
        phi_prm_i = h.H0*vrtl_cnstr.Phi_prm(s_i, p);
        phi_2prm_i = h.H0*vrtl_cnstr.Phi_2prm(s_i, p);
        A(:,:,i) = h.get_A(s_i,sd_i,phi_i,phi_prm_i,phi_2prm_i);
        B(:,:,i) = h.get_B(s_i,sd_i,phi_i,phi_2prm_i);
    end
    
    function z_dot = reduced_dynamics_ode(t, z, Phi, Phi_prm, Phi_2prm, h)
     z_dot = [z(2); -1/h.get_alpha(Phi, Phi_prm)*...
                    (h.get_gamma(Phi) + ...
                        h.get_beta(Phi, Phi_prm, Phi_2prm)*z(2)^2)];
    end

end





