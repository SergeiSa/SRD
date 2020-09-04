clear;

tol = 0.00001;

for k = 1:100
    
    M{1} = rand(10, 5);
    M{2} = rand(5, 10);
    M{3} = rand(10, 10);
    M{4} = rand(10, 1);
    M{5} = rand(1, 10);
    M{6} = zeros(10, 10);
    M{7} = ones(10, 10);
    
    
    for i = 1:7
        
        N = svd_suit(M{i}, tol);
        
        if norm( rank(N.self, tol) - N.rank ) > tol
            warning('rank is calulates with lower accuracy than tolerance')
        end
        
        if norm( null(N.self) - N.null ) > tol
            warning('null is calulates with lower accuracy than tolerance')
        end
        
        if norm( orth(N.self) - N.orth ) > tol
            warning('orth is calulates with lower accuracy than tolerance')
        end
        
        if norm( pinv(N.self) - N.pinv ) > tol
            warning('pinv is calulates with lower accuracy than tolerance')
        end
        
    end

end