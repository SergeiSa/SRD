function  [s_str, sd_str, q_str, qd_str, qdd_str, T, A, B] = SRDt_get_nominal_trajectory(varargin)
    Parser = inputParser;
    Parser.FunctionName = 'get_nominal_trajectory';
    Parser.addOptional('N_dof', []);
    Parser.addOptional('c0', []); % s = c0*q
    Parser.addOptional('H0', []); % H0*q = phi(s)
    Parser.addOptional('p', []); % virtual constraints parameter vector, e.g. phi(s) = p(1) + p(2)*s...
    Parser.addOptional('s0', []); % initial value of motion generator
    Parser.addOptional('dt', []); % discretization time
    
    Parser.parse(varargin{:});
    
    p = Parser.Results.p;

    vrtl_cnstr = VirtualConstraint(Parser.Results.N_dof, length(Parser.Results.p)-1,...
        'ordinary', Parser.Results.c0, Parser.Results.H0);
    
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
        vrtl_cnstr.Phi_2prm(x(1), p)), tspan, x0, optns);

    [~,locs] = findpeaks(x(:,1));
    T = t(locs(1)); 
    T_ind = locs(1);    
    
    s_str = x(1:T_ind,1);
    sd_str = x(1:T_ind,2);
    
    sdd_str = zeros(T_ind,1);

    for i=1:length(T_ind)
        sdd_str(i) = -1/get_alpha(vrtl_cnstr.Phi(x(i,1), p), vrtl_cnstr.Phi_prm(x(i,1), p))*...
                  (get_gamma(vrtl_cnstr.Phi(x(i,1), p)) + ...
                  get_beta(vrtl_cnstr.Phi(x(i,1), p), vrtl_cnstr.Phi_prm(x(i,1), p), ...
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

    A = zeros(  2*Parser.Results.N_dof-1,2*Parser.Results.N_dof-1,length(s_str));
    B = zeros(2*Parser.Results.N_dof-1,Parser.Results.N_dof-1,length(s_str));

    for i = 1:length(s_str)
        s_i = s_str(i);
        sd_i = sd_str(i);
        phi_i = Parser.Results.H0*vrtl_cnstr.Phi(s_i, p);
        phi_prm_i = Parser.Results.H0*vrtl_cnstr.Phi_prm(s_i, p);
        phi_2prm_i = Parser.Results.H0*vrtl_cnstr.Phi_2prm(s_i, p);
        A(:,:,i) = get_A_transv(s_i,sd_i,phi_i,phi_prm_i,phi_2prm_i);
        B(:,:,i) = get_B_transv(s_i,sd_i,phi_i,phi_2prm_i);
    end

end





