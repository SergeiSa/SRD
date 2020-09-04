%This function generates and solves a quadratic program to check if the
%body defined by its contact points and external forces and torques is
%stabilizable
%
%inputs:
%'ContactPoints' - cell array of CI_ContactPoint objects
%'ExternalActions' - cell array of CI_ExternalAction objects
%'Dimentions' - '2d', '3d', defined the problem dimentions
%'Number_of_vertices_lin_approx' - number of vertices for linear
%approximation of friction cones (only for 3d problems)
%'Center' - coordinates of the center for the torque balance eq.
%'Verbose' - 1 for for display, 0 for none
%'Symbolic' - true if the problem needs to be generates in symbolic form
function output = CIfunction_check_if_stabilizable(varargin)
Parser = inputParser;
Parser.FunctionName = 'CIfunction_check_if_stabilizable';
Parser.addOptional('ContactPoints', []); 
Parser.addOptional('ExternalActions', []);
Parser.addOptional('Dimentions', '3d');
Parser.addOptional('Number_of_vertices_lin_approx', 8);
Parser.addOptional('Center', []);
Parser.addOptional('Verbose', 1);
Parser.addOptional('Symbolic', false);
Parser.parse(varargin{:});

switch Parser.Results.Dimentions
    case {'2d', '3d'}
    otherwise
        error('incorrect Dimentions type, chose 2d or 3d');
end

Math = MathClass();

%find center (point aroud which the torque eqs will be balanced
if isempty(Parser.Results.Center) 
    switch Parser.Results.Dimentions
        case '2d'
            Center = [0; 0];
            if (Parser.Results.Verbose > 0); disp("CIfunction_check_if_stabilizable: Assumed Center = [0; 0]"); end
        case '3d'
            Center = [0; 0; 0];
            if (Parser.Results.Verbose > 0); disp("CIfunction_check_if_stabilizable: Assumed Center = [0; 0; 0]"); end
    end
else
    Center = Parser.Results.Center;
end

%build cross product matrices (CPM)
n1 = length(Parser.Results.ContactPoints);
ContactPoints = cell(n1, 1);

for i = 1:n1
    r = Parser.Results.ContactPoints{i}.Position - Center;
    switch Parser.Results.Dimentions
        case '2d'
            CPM = Math.CrossProductMatrix2d(r);
            ContactPoints{i}.tau = Parser.Results.ContactPoints{i}.Tangent;
        case '3d'
            CPM = Math.CrossProductMatrix(r);
            ContactPoints{i}.tau1 = Parser.Results.ContactPoints{i}.Tangent(:, 1);
            ContactPoints{i}.tau2 = Parser.Results.ContactPoints{i}.Tangent(:, 2);
    end
    
    ContactPoints{i}.CPM = CPM;
    ContactPoints{i}.n = Parser.Results.ContactPoints{i}.Normal;
    ContactPoints{i}.mu = Parser.Results.ContactPoints{i}.friction_coefficient;
    ContactPoints{i}.Nmax = Parser.Results.ContactPoints{i}.max_normal_force;
end


%build force and torque arrays
n2 = length(Parser.Results.ExternalActions);
index_force = 0; index_torque = 0; Forces = {};  Torques = {};
for i = 1:n2
    switch Parser.Results.ExternalActions{i}.Type
        case 'force'
            
            r = Parser.Results.ExternalActions{i}.Position - Center;
            switch Parser.Results.Dimentions
                case '2d'
                    CPM = Math.CrossProductMatrix2d(r);
                case '3d'
                    CPM = Math.CrossProductMatrix(r);
            end
            
            index_force = index_force + 1;
            Forces{index_force}.M = CPM * Parser.Results.ExternalActions{i}.Value;
            Forces{index_force}.F = Parser.Results.ExternalActions{i}.Value;
            
        case 'torque'
            index_torque = index_torque + 1;
            Torques{index_torque}.M = Parser.Results.ExternalActions{i}.Value;
    end
end
    
%the code below is a little messy: the 2d version is written with 'manual'
%construction of the constraints matrix, while the 3d version uses SRD
%optimization helper functions; 
switch Parser.Results.Dimentions
    case '2d'
        
        %build constraints
        A_torque_row = [];
        A_force_row = [];
        
        A_ineq_friction_cone = [];
        
        A_ineq_N_min_max = [];
        b_ineq_N_min_max = [];

        if Parser.Results.Symbolic
            zero_line = sym(zeros(2, 2*n1));
        else
            zero_line = zeros(2, 2*n1);
        end
           
        for i = 1:n1
            index = 2*(i - 1) + 1;
            
            %constraints associated with torque balance eq.
            A_torque_row = [A_torque_row, ...
                (ContactPoints{i}.CPM * ContactPoints{i}.tau), ...
                (ContactPoints{i}.CPM * ContactPoints{i}.n)];
            
            %constraints associated with force balance eq.
            A_force_row = [A_force_row, ContactPoints{i}.tau, ContactPoints{i}.n];
            
            %constraints associated with friction cone
            a = zero_line; a(1, index) =  1; a(1, index + 1) = -ContactPoints{i}.mu;
                           a(2, index) = -1; a(2, index + 1) = -ContactPoints{i}.mu;
            A_ineq_friction_cone = [A_ineq_friction_cone;  a];
            
            %constraints associated with normal reaction limits
            a = zero_line; a(1, index) = 0; a(1, index + 1) = -1;
                           a(2, index) = 0; a(2, index + 1) = 1;
            A_ineq_N_min_max = [A_ineq_N_min_max; a];
            
            b_ineq_N_min_max = [b_ineq_N_min_max; 0; ContactPoints{i}.Nmax];
        end
        
        b_torque_row = 0;
        for i = 1:index_force
            b_torque_row = b_torque_row - Forces{i}.M;
        end
        for i = 1:index_torque
            b_torque_row = b_torque_row - Torques{i}.M;
        end
        
        b_force_row = 0;
        for i = 1:index_force
            b_force_row = b_force_row - Forces{i}.F;
        end
        
        A_eq = [A_torque_row; A_force_row];
        b_eq = [b_torque_row; b_force_row];
        
        A_ineq = [A_ineq_friction_cone; 
                  A_ineq_N_min_max];
        
        b_ineq = [zeros(size(A_ineq_friction_cone, 1), 1); 
                  b_ineq_N_min_max];
 
    case '3d'
        
        %build constraints
        x1 = sym('x1', [n1, 1]); assume(x1, 'real');
        x2 = sym('x2', [n1, 1]); assume(x2, 'real');
        y =  sym('y', [n1, 1]);  assume(y, 'real');
        var = [];
        
        Torque_balance_eq_LHS = sym(zeros(3, 1));
        Torque_balance_eq_RHS = sym(zeros(3, 1));
        
        Force_balance_eq_LHS = sym(zeros(3, 1));
        Force_balance_eq_RHS = sym(zeros(3, 1));
        
        FrictionCones_LHS = [];
        FrictionCones_RHS = [];
        
        for i = 1:n1
            
            Torque_balance_eq_LHS = Torque_balance_eq_LHS + ...
                x1(i) * ContactPoints{i}.CPM * ContactPoints{i}.tau1 + ...
                x2(i) * ContactPoints{i}.CPM * ContactPoints{i}.tau2 + ...
                 y(i) * ContactPoints{i}.CPM * ContactPoints{i}.n;
                
            Force_balance_eq_LHS = Force_balance_eq_LHS + ...
                x1(i) * ContactPoints{i}.tau1 + ...
                x2(i) * ContactPoints{i}.tau2 + ...
                 y(i) * ContactPoints{i}.n;
            
            [Ac, bc] = CIfunction_generate_friction_cone_as_linear_inequalities(ContactPoints{i}.mu, ...
                ContactPoints{i}.Nmax, Parser.Results.Number_of_vertices_lin_approx);
            
            FrictionCones_LHS = [FrictionCones_LHS; 
                                 (Ac * [x1(i); x2(i); y(i)])];
            FrictionCones_RHS = [FrictionCones_RHS;
                                 bc];
            
            var = [var; [x1(i); x2(i); y(i)]];
        end
        
        for i = 1:index_force
            Torque_balance_eq_RHS = Torque_balance_eq_RHS - Forces{i}.M;
        end
        for i = 1:index_torque
            Torque_balance_eq_RHS = Torque_balance_eq_RHS - Torques{i}.M;
        end
        
        for i = 1:index_force
            Force_balance_eq_RHS = Force_balance_eq_RHS - Forces{i}.F;
        end
        
        Equality_constraints_LHS = [Torque_balance_eq_LHS; Force_balance_eq_LHS];
        Equality_constraints_RHS = [Torque_balance_eq_RHS; Force_balance_eq_RHS];
        
        [A_eq, b_eq] = OHfunction_generate_linear_constraints('LHS', Equality_constraints_LHS, ...
            'RHS', Equality_constraints_RHS, 'var', var, 'Numeric', ~Parser.Results.Symbolic);
        
        [A_ineq, b_ineq] = OHfunction_generate_linear_constraints('LHS', FrictionCones_LHS, ...
            'RHS', FrictionCones_RHS, 'var', var, 'Numeric', ~Parser.Results.Symbolic);
        
end

%generate QP
if Parser.Results.Verbose > 0
    DisplayLevel = 'iter';
else
    DisplayLevel = 'none';
end

problem.solver = 'quadprog';
problem.options = optimoptions('quadprog', 'Display', DisplayLevel);
problem.H = eye(size(A_eq, 2));
problem.Aeq = A_eq;
problem.beq = b_eq;

problem.Aineq = A_ineq;
problem.bineq = b_ineq;

%solve QP
if ~Parser.Results.Symbolic
    [x,fval,exitflag] = quadprog(problem);
    output.x = x;
    output.fval = fval;
    output.exitflag = exitflag;
end
output.problem = problem;

%print info
if ~Parser.Results.Symbolic
    if (Parser.Results.Verbose > 0) && (output.exitflag == 1)
        for i = 1:n1
            switch Parser.Results.Dimentions
                case '2d'
                    index = 2*(i - 1) + 1;
                    disp(['x', num2str(index), ' - f', num2str(i), ' = ', num2str(x(index)), ...
                        ', friction for contact point #', num2str(i)]);
                    disp(['x', num2str(index + 1), ' - n', num2str(i), ' = ', num2str(x(index + 1)), ...
                        ', normal reaction for contact point #', num2str(i)]);
                case '3d'
                    warning('not implemented yet');
            end
        end
    end
end

end