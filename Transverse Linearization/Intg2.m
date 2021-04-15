function I = Intg2(s, s_d, s_0, s_d0, p, vrtlCnstr)
% Function computes integral of the motion
if s == s_0
    I = s_d^2 - s_d0^2;
    return
end

n = 4; % number of intervals to numerically compute integral
xi = linspace(s_0,s,n+1);

f = zeros(n+1, 1);
for i=1:n+1
    f(i) = intg_fi(s_0, xi(i), p, vrtlCnstr);
end
% f = @(s) intg_f(s,s_0);
int = simpsons(f, s_0, s, n);


% See 24-25 in "Constructive tool..." 2005
I = s_d^2 - intg_psi(s_0, s, p, vrtlCnstr)*(s_d0^2 - int);


% ---------------------------------------------------------------------
% Local Function
% ---------------------------------------------------------------------
function f = intg_fi(s0, s, p, vrtlCnstr)
    f = intg_psi(s, s0, p, vrtlCnstr)* 2 .*gamma_over_alpha(s, p, vrtlCnstr);
end

function psi = intg_psi(s0, s, p, vrtlCnstr)
    %psi as in paper: Constructive tool...2005
    %for L = -1.5, g=9.81
    if s0 == s
        psi = 1;
        return
    end
          
    ff = zeros(n+1, 1);
    xj = linspace(s0,s,n+1);
    for j = 1:n+1
        ff(j) = beta_over_alpha(xj(j), p, vrtlCnstr);
    end
    int1 = simpsons(ff, s0, s, n);
    psi = exp(-2*int1);
end

function out = beta_over_alpha(s, p, vrtlCnstr)
    Phi = vrtlCnstr.Phi(s, p);
    Phi_prm = vrtlCnstr.Phi_prm(s, p);
    Phi_2prm = vrtlCnstr.Phi_2prm(s, p);
    out = get_beta(Phi, Phi_prm, Phi_2prm)/get_alpha(Phi, Phi_prm);
end

function out = gamma_over_alpha(s, p, vrtlCnstr)
    Phi = vrtlCnstr.Phi(s, p);
    Phi_prm = vrtlCnstr.Phi_prm(s, p);
    out = get_gamma(Phi)/get_alpha(Phi, Phi_prm);
end

end

