% displaces LocalVertices to fin into one of the regions in RegionsArray
% RegionsArray is a cell array with fields .A and .b
% LocalVertices is an aaray, each row is a vertex
% QP_cost_matrix is H matrix in quadprog description
function displacement = SPDisplaceToFreeRegion(RegionsArray, LocalVertices, QP_cost_matrix)

M = size(RegionsArray{1}.A, 2);
N = max(size(RegionsArray));
NumberOfVertices = size(LocalVertices, 1);

if nargin < 3
    problem.H = eye(M);
else
    problem.H = QP_cost_matrix;
end

problem.solver = 'quadprog';
problem.options = optimoptions('quadprog', 'Display', 'off');

displacements_array = zeros(N, M);
performance = zeros(N, 1);

for i = 1:N
    A = RegionsArray{i}.A;
    b = RegionsArray{i}.b;
    
    [lx, ly] = size(A);
    
    toll_b = zeros(lx*NumberOfVertices, 1);
    toll_A = zeros(lx*NumberOfVertices, ly);
    for j = 1:NumberOfVertices
        index = (j - 1)*lx;
        v = LocalVertices(j, :)';
        
        toll_b((index + 1):(index + lx)) = b - A*v;
        toll_A((index + 1):(index + lx), :) = A;
    end
    
    problem.Aineq = toll_A;
    problem.bineq = toll_b;
    [x, fval, exitflag] = quadprog(problem);
    if exitflag == 1
        performance(i) = fval;
        displacements_array(i, :) = x';
    else
        performance(i) = Inf;
    end
end

[~, best_index] = min(performance);
displacement = displacements_array(best_index, :)';
end