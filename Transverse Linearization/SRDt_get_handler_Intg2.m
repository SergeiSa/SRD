function h = SRDt_get_handler_Intg2(p, vrtlCnstr, g)
h = @(s, s_d, s_0, s_d0) Intg2(s, s_d, s_0, s_d0, p, vrtlCnstr, g);
function I = Intg2(s, s_d, s_0, s_d0, p, vrtlCnstr, g)
% Function computes integral of the motion
if s == s_0
    I = s_d^2 - s_d0^2;
    return
end

n = 4; % number of intervals to numerically compute integral
xi = linspace(s_0,s,n+1);

f = zeros(n+1, 1);
for i=1:n+1
    f(i) = intg_fi(s_0, xi(i), p, vrtlCnstr, g);
end
% f = @(s) intg_f(s,s_0);
int = simpsons(f, s_0, s, n);


% See 24-25 in "Constructive tool..." 2005
I = s_d^2 - intg_psi(s_0, s, p, vrtlCnstr, g)*(s_d0^2 - int);


% ---------------------------------------------------------------------
% Local Function
% ---------------------------------------------------------------------
function f = intg_fi(s0, s, p, vrtlCnstr, g)
    f = intg_psi(s, s0, p, vrtlCnstr, g)* 2 .*gamma_over_alpha(s, p, vrtlCnstr, g);
end

function psi = intg_psi(s0, s, p, vrtlCnstr, g)
    %psi as in paper: Constructive tool...2005
    %for L = -1.5, g=9.81
    if s0 == s
        psi = 1;
        return
    end
          
    ff = zeros(n+1, 1);
    xj = linspace(s0,s,n+1);
    for j = 1:n+1
        ff(j) = beta_over_alpha(xj(j), p, vrtlCnstr, g);
    end
    int1 = simpsons(ff, s0, s, n);
    psi = exp(-2*int1);
end

function out = beta_over_alpha(s, p, vrtlCnstr, g)
    Phi = vrtlCnstr.Phi(s, p);
    Phi_prm = vrtlCnstr.Phi_prm(s, p);
    Phi_2prm = vrtlCnstr.Phi_2prm(s, p);
    out = g.get_beta(Phi, Phi_prm, Phi_2prm)/g.get_alpha(Phi, Phi_prm);
end

function out = gamma_over_alpha(s, p, vrtlCnstr, g)
    Phi = vrtlCnstr.Phi(s, p);
    Phi_prm = vrtlCnstr.Phi_prm(s, p);
    out = g.get_gamma(Phi)/g.get_alpha(Phi, Phi_prm);
end

function I = simpsons(f,a,b,n)
    if numel(f)>1 % If the input provided is a vector
        n=numel(f)-1; h=(b-a)/n;
        I= h/3*(f(1)+2*sum(f(3:2:end-2))+4*sum(f(2:2:end))+f(end));
    else % If the input provided is an anonymous function
        h=(b-a)/n; xi=a:h:b;    
        I= h/3*(f(xi(1))+2*sum(f(xi(3:2:end-2)))+4*sum(f(xi(2:2:end)))+f(xi(end)));
    end
end

end

end

