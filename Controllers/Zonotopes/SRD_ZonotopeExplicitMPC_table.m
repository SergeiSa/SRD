%A_table, B_table: 
%dim 1,2 are the state, 
%dim 3 is the iteration index, 
%dim 4 is the parameter index
function [K_table, G_table, T_table] = SRD_ZonotopeExplicitMPC_table(varargin)
cvx_clear;

Parser = inputParser;
Parser.FunctionName = 'SRD_ZonotopeExplicitMPC_table';
Parser.addOptional('cvx_solver', 'SDPT3');
Parser.addOptional('BarHomogeneous', 1);
Parser.addOptional('CrossoverBasis', 1);
Parser.addOptional('NumericFocus',   3);
Parser.addOptional('Method',         3);
Parser.addOptional('cvx_precision', 'best');

Parser.addOptional('A_table',   []);
Parser.addOptional('B_table',   []);
Parser.addOptional('W_matrix',  diag([0.001 0.001 0.001 0.001]));

Parser.addOptional('zonotope_order',   250);
Parser.addOptional('cost_weights_G',     [1;1;1;1]);
Parser.addOptional('cost_weights_T',     1);
Parser.addOptional('cost_weights_b',     1);


Parser.addOptional('cyclical',     false);


Parser.parse(varargin{:});

A_table = Parser.Results.A_table;
B_table = Parser.Results.B_table;
W_matrix = Parser.Results.W_matrix;

size_z  = Parser.Results.zonotope_order;
cost_weights_G  = Parser.Results.cost_weights_G;
cost_weights_T  = Parser.Results.cost_weights_T;
cost_weights_b  = Parser.Results.cost_weights_b;
cyclical = Parser.Results.cyclical;

size_x = size(A_table, 2);
size_u = size(B_table, 2);
Count  = size(A_table, 3);
size_p = size(A_table, 4);


cvx_solver(Parser.Results.cvx_solver)
cvx_begin
cvx_solver_settings( 'BarHomogeneous', Parser.Results.BarHomogeneous )
cvx_solver_settings( 'CrossoverBasis', Parser.Results.CrossoverBasis )
cvx_solver_settings( 'NumericFocus', Parser.Results.NumericFocus )
cvx_solver_settings( 'Method', Parser.Results.Method )
cvx_precision(Parser.Results.cvx_precision)

variable G(size_x, size_z+size_x, (Count+1))
variable T(size_u, size_z+size_x, Count)

% variable x(n, (Count+1))
% variable u(m, (Count+1))

% variable Gamma_end(n, z+n)
% variable betta_end(z+n, 1)

%variable Gamma_u(m, z+n, Count)
%variable betta_u(m, 1, Count)

variable bounding_box_margin(size_x, Count) nonnegative

cost_norm = 2;

Cost = cost_weights_b * norm(bounding_box_margin(:), cost_norm);

if length(cost_weights_T) == 1
    Cost = Cost + cost_weights_T * norm(T(:), cost_norm);
else
    for i = 1:size_u
        Cost = Cost + cost_weights_T(i) * norm(reshape(T(i, :, :), [], 1), cost_norm);
    end
end

if length(cost_weights_G) == 1
    Cost = Cost + cost_weights_G * norm(G(:), cost_norm);
else
    for i = 1:size_x
        Cost = Cost + cost_weights_G(i) * norm(reshape(G(i, :, :), [], 1), cost_norm);
    end
end

minimize( Cost )

subject to

if cyclical
    G(:, :, 1) == G(:, :, Count+1);
end

%goal
%G(:, :, Count+1) == G_end*Gamma_end
%x_g - x(:,Count+1) == G_start*betta_end
%norm(Gamma_end, inf) <= 1

for i = 1:Count
    
    %control limit
    %T(:, :, i) == T_max*Gamma_u(:,:,i)
    %0 - u(:,i) == T_max*betta_u(:,:,i)
    %norm([Gamma_u(:,:,i)],Inf) <= 1
    
    for j = 1:size_p
        GG(:, :, j) = A_table(:,:, i, j)*G(:, :, i) + B_table(:,:, i, j)*T(:,:,i);
    end
    
    F_TEMP = [(GG(:, :, 1)+GG(:, :, 2)) (GG(:, :, 1)-GG(:, :, 2))]/2;
    
    %ReaZOR
    F_rea = [F_TEMP(:, 1:size_x), F_TEMP(:, 9:end), F_TEMP(:, 5:8)];
    F_1 = F_rea(:,1:size_z-size_x);
    F_2 = F_rea(:, (size_z-size_x+1):end);
    for j = 1:size_x
        norm(F_2(j,:),1) <= bounding_box_margin(j,i);
    end
    G(:, :, i+1) == [F_1 diag(bounding_box_margin(:,i)), W_matrix];
    
end
cvx_end

if strcmp(cvx_status, 'Inaccurate/Solved') || strcmp(cvx_status, 'Solved')
K_table = zeros(size_u, size_x, Count);
    for i = 1:Count
        K_table(:,:,i) = T(:, :, i) * pinv(G(:,:, i));
    end
    
    G_table = full(G);
    T_table = full(T);
end


end