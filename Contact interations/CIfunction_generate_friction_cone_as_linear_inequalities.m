%this function generates linear inequalities that represent a friction cone
%it represents the cone in terms of its linear coefficients. Namely, x1 and
%x2 are coefficients for tau1 and tau2 - unit vectors, tangent to the
%contact surface at the contact point and y is the coefficient for n, a
%unit vector normal to the contact surface at the contact point. Thus, the
%constraints are placed on x1, x2 and y and not on the forces expressed in
%cartesian coordinates.
%F_friction = x1*tau1 + x2*tau2;
%N = y*n;
%
%actual friction cone constraints: F_friction <= mu*N;
%
%inputs:
%mu - friction coefficient
%Nmax - max value of normal reaction, use Inf if there is no limit
%number_of_vertices - number of vertices in the linear approximation of the
%friction cone
function [A, b] = CIfunction_generate_friction_cone_as_linear_inequalities(mu, Nmax, number_of_vertices, order)

if nargin < 3
    number_of_vertices = 4;
end
if nargin < 4
    order = [];
end

if isnumeric(mu) &&isnumeric(Nmax) 
    numeric = true;
else
    numeric = false;
end

dphi = 2 * pi / number_of_vertices;
Vertices = zeros(number_of_vertices, 3);

if ~numeric
    Vertices = sym(Vertices);
end

%generate vertices
for i = 1:number_of_vertices
    phi = (i - 1)*dphi;
    T = [cos(phi), -sin(phi), 0; 
         sin(phi),  cos(phi), 0; 
         0,         0,        1];
    
    P = T*[mu; 0; 0] + [0; 0; 1];
    Vertices(i, :) = P';
end

%allocate memory for the constraints. If Nmax = Inf to the constraint that
%caps the friction cone from the above is missing
if isfinite(Nmax)
    A = zeros(number_of_vertices+1, 3);
    b = zeros(number_of_vertices+1, 1);
else
    A = zeros(number_of_vertices, 3);
    b = zeros(number_of_vertices, 1);
end

if ~numeric
    A = sym(A);
    b = sym(b);
end

%generate linear constraints (sides of the approximation)
for i = 1:number_of_vertices
    next_i = i + 1;
    if next_i > number_of_vertices
        next_i = 1;
    end
    
    P1 = Vertices(i, :)';
    P2 = Vertices(next_i, :)';
    
    a = -cross(P1, P2);
    A(i, :) = a / norm(a);
    %note that elements of b are all zero, since the half-planes a are all
    %intersectig at the point [0; 0; 0]
end

if isfinite(Nmax)
    A(end, :) = [0; 0; 1];
    b(end) = Nmax;
end

if ~isempty(order)
    switch order
        case 'x1 x2 y'
        case 'y x1 x2'
            A = [A(:, 3), A(:, 1), A(:, 2)];
    end
end

end