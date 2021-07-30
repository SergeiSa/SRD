function suit = svd_intersection(A, B, tol, A_is_leftnull, B_is_leftnull)

if nargin < 3 
    suit.tol = 10^(-10);
else
    suit.tol = tol;
end

if nargin < 4
    A_is_leftnull = false;
end
if nargin < 5
    B_is_leftnull = false;
end    

if A_is_leftnull
    A_left_null = A;
else
    if ~isfield(A, 'left_null')
        As = svd_suit(A, suit.tol); 
        A_left_null = As.left_null;
    else
        A_left_null = A.left_null;
    end
end
if B_is_leftnull
    B_left_null = B;
else
    if ~isfield(B, 'left_null')
        Bs = svd_suit(B, suit.tol); 
        B_left_null = Bs.left_null;
    else
        B_left_null = B.left_null;
    end
end

res = svd_suit([A_left_null, B_left_null], suit.tol);

suit.self      = res.left_null;
suit.orth      = res.left_null;
suit.left_null = res.orth;
suit.row_space = eye(size(res.left_null, 2));
suit.null      = zeros(size(res.left_null, 2), 0);

suit.pinv = res.left_null';
suit.rank = size(res.left_null, 2);

end
