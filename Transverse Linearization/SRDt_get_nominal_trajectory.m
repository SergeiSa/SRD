function  nominal_trajectory = SRDt_get_nominal_trajectory(varargin)
    Parser = inputParser;
    Parser.FunctionName = 'get_nominal_trajectory';
    Parser.addOptional('Handler_reduced_dynamics_and_transverse_linearization', []);
    Parser.addOptional('p', []); % virtual constraints parameter vector, e.g. phi(s) = p(1) + p(2)*s...
    Parser.addOptional('virtual_constraints_obj', []); % virtual constraints class object
    Parser.addOptional('s0', []); % initial value of motion generator
    Parser.addOptional('dt', []); % discretization time
    Parser.addOptional('Time', 10); % discretization time
    
    Parser.parse(varargin{:});
    
    h = Parser.Results.Handler_reduced_dynamics_and_transverse_linearization;
    p = Parser.Results.p;

    virtual_constraints_obj = Parser.Results.virtual_constraints_obj;
    
    %     func1 = @(s) get_alpha(vrtl_cnstr.Phi(s, p), vrtl_cnstr.Phi_prm(s, p));
    %     func2 = @(s) get_gamma(vrtl_cnstr.Phi(s, p));
    %     s_interval = [-10 10];
    % fplot(func1,s_interval)
    % roots1 = fzero(func1,0)
    % fplot(func2,s_interval)
    % roots2 = fzero(func2,0)

    tspan= 0:Parser.Results.dt:Parser.Results.Time;
    x0 = [Parser.Results.s0; 0];
    optns = odeset('RelTol',1e-9,'AbsTol',1e-9,'NormControl','on');

    [t, x] = ode45( @(t,x)reduced_dynamics_ode(t,x,virtual_constraints_obj.Phi(x(1), p), ...
        virtual_constraints_obj.Phi_prm(x(1), p),...
        virtual_constraints_obj.Phi_2prm(x(1), p), h), tspan, x0, optns);

    [~,locs] = findpeaks(x(:,1));
    nominal_trajectory.T = t(locs(1)); 
    T_ind = locs(1);    
    
    nominal_trajectory.s = x(1:T_ind,1);
    nominal_trajectory.sd = x(1:T_ind,2);
    
    nominal_trajectory.sdd = zeros(T_ind,1);

    for i=1:T_ind
        nominal_trajectory.sdd(i) = -1/h.get_alpha(virtual_constraints_obj.Phi(x(i,1), p), virtual_constraints_obj.Phi_prm(x(i,1), p))*...
                  (h.get_gamma(virtual_constraints_obj.Phi(x(i,1), p)) + ...
                  h.get_beta(virtual_constraints_obj.Phi(x(i,1), p), virtual_constraints_obj.Phi_prm(x(i,1), p), ...
                  virtual_constraints_obj.Phi_2prm(x(i,1), p))*x(i,2)^2);
    end

    nominal_trajectory.q = zeros(2, T_ind);
    nominal_trajectory.qd = zeros(2, T_ind);
    nominal_trajectory.qdd = zeros(2, T_ind);
    for i = 1:T_ind
        nominal_trajectory.q(:,i) = virtual_constraints_obj.Phi(nominal_trajectory.s(i), p);
        nominal_trajectory.qd(:,i) = virtual_constraints_obj.Phi_prm(nominal_trajectory.s(i), p)*nominal_trajectory.sd(i);
        nominal_trajectory.qdd(:,i) = virtual_constraints_obj.Phi_2prm(nominal_trajectory.s(i), p)*nominal_trajectory.sd(i)^2 + ...
            virtual_constraints_obj.Phi_prm(nominal_trajectory.s(i), p)*nominal_trajectory.sdd(i);
    end

    nominal_trajectory.A = zeros(2*h.N_dof-1,2*h.N_dof-1,length(nominal_trajectory.s));
    nominal_trajectory.B = zeros(2*h.N_dof-1,h.N_dof-1,length(nominal_trajectory.s));
    
    nominal_trajectory.p = p;

    for i = 1:T_ind
        s_i = nominal_trajectory.s(i);
        sd_i = nominal_trajectory.sd(i);
        phi_i = h.H0*virtual_constraints_obj.Phi(s_i, p);
        phi_prm_i = h.H0*virtual_constraints_obj.Phi_prm(s_i, p);
        phi_2prm_i = h.H0*virtual_constraints_obj.Phi_2prm(s_i, p);
        nominal_trajectory.A(:,:,i) = h.get_A(s_i,sd_i,phi_i,phi_prm_i,phi_2prm_i);
        nominal_trajectory.B(:,:,i) = h.get_B(s_i,sd_i,phi_i,phi_2prm_i);
    end
    
    function z_dot = reduced_dynamics_ode(t, z, Phi, Phi_prm, Phi_2prm, h)
     z_dot = [z(2); -1/h.get_alpha(Phi, Phi_prm)*...
                    (h.get_gamma(Phi) + ...
                        h.get_beta(Phi, Phi_prm, Phi_2prm)*z(2)^2)];
    end

end





