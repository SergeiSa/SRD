function P = Tensor3MatrixProduct(A, B)

s1 = size(A);
s2 = size(B);

P = zeros(s1(1), s2(2), s1(3)); 
if isa(A, 'sym') || isa(B, 'sym')
    P = sym(P);
end

for i = 1:s1(3)
    P(:, :, i) = A(:, :, i)*B; 
end

end