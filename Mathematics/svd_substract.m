%deletes from A dimention B; A and B are matrices or SVD decompositions
function suit = svd_substract(A, B, tol, ToCheck)

if nargin < 4 
    ToCheck = true;
end

%matlab-style automatic tolerance
if nargin < 3 
    suit.tol = 10^(-10);
else
    suit.tol = tol;
end

if ~isfield(A, 'left_null')
    As = svd_suit(A, suit.tol); A_left_null = As.left_null;
else
    A_left_null = A.left_null;
end
if ~isfield(B, 'self')
    Bs = B;
else
    Bs = B.self;
end

if ToCheck
    if norm( A_left_null*A_left_null'*Bs ) > suit.tol
        error('substracting from A subspace that does not lie in range(A)')
    end
end

res = svd_suit([A_left_null, Bs], suit.tol);

suit.self      = res.left_null;
suit.orth      = res.left_null;
suit.left_null = res.orth;
suit.row_space = eye(size(res.left_null, 2));
suit.null      = zeros(size(res.left_null, 2), 0);

suit.pinv = res.left_null';
suit.rank = size(res.left_null, 2);

end
