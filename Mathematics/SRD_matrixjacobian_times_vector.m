%test:
%q = sym('q', [2, 1]); T = SRD_RotationMatrix3D_x(q(1)); P = matrixjacobian_times_vector(T, q, [0;0;1])
function P = SRD_matrixjacobian_times_vector(M, var, vec)

s1 = size(M);
s2 = length(vec);
s3 = length(var);

P = zeros(s1(1), s3); 
if isa(M, 'sym') || isa(var, 'sym') || isa(vec, 'sym')
    P = sym(P);
end
if isa(M, 'casadi.SX') || isa(var, 'casadi.SX') || isa(vec, 'casadi.SX')
    P = casadi_zeros(P);
end

T = jacobian(M(:), var);

for i = 1:s3
    P(:, i) = reshape(T(:, i), s1) * vec;
end

    function P = casadi_zeros(P)
        import casadi.*
        P = SX.zeros(size(P));
    end
        
end