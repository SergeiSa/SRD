classdef SRDUnilateralConstraints < handle
    properties
        %%% SRD classes' objects
        SymbolicEngine;
        SimulationEngine;
        Math;
        
        %symbolic computations settings 
        ToSimplify = true;
        UseParallelizedSimplification = false;
        
        %%%lists containting handles of generated functions
        ConstraintsList;
        ConstraintJacobiansList;
        ConstraintJacobianDerivativesList;
        ConstraintLinearizationConstantsList;
        
        %%% Cartesian constraints testers
        intersection_tester; 
        get_constraint_properties;
        %see TPUnilateralConstraintCollection for examples of how these
        %function handle should be generated
        
    end
    methods
        function obj = SRDUnilateralConstraints(SymbolicEngine)
            obj.SymbolicEngine = SymbolicEngine;
            obj.Math = MathClass;
        end
        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% following are symbolic derivation methods
        
        %generates constraints functions
        %takes 3xn symbolic arrays as inputs
        %
        %example: 
        %PointsArray{1} = SymbolicEngine.LinkArray(2).AbsoluteBase;
        %PointsArray{2} = SymbolicEngine.LinkArray(2).AbsoluteFollower(:, 1);
        %PointsArray{3} = SymbolicEngine.LinkArray(2).AbsoluteFollower(:, 2);
        %.GenerateConstrainsFunctionsCell(PointsArray)
        function GenerateConstrainsFunctionsCell(obj, PointsArray)
            number_of_points = length(PointsArray);
            
            obj.Math.UseParallel = obj.UseParallelizedSimplification;
            
            Constraints = cell(number_of_points, 1);
            ConstraintJacobians = cell(number_of_points, 1);
            ConstraintJacobianDerivatives = cell(number_of_points, 1);
            ConstraintLinearizationConstants = cell(number_of_points, 1);
            
            q = obj.SymbolicEngine.q;
            v = obj.SymbolicEngine.v;
            
            for i = 1:number_of_points
                P = PointsArray{i};
                
                F = jacobian(P, q);
                if obj.ToSimplify
                    disp(['Started simplifying constraint jacobian ', int2str(i), ' out of ', ...
                        int2str(number_of_points)]);
                    if obj.UseParallelizedSimplification
                        F = obj.Math.ParallelizedSimplification(F, ['constraint jacobian ', int2str(i)]);
                    else
                        F = simplify(F);
                    end
                end
                
                dF = obj.Math.MatrixDerivative(F, q, v);
                if obj.ToSimplify
                    disp(['Started simplifying constraint jacobian derivative ', int2str(i), ' out of ', ...
                        int2str(number_of_points)]);
                    if obj.UseParallelizedSimplification
                        dF = obj.Math.ParallelizedSimplification(dF, ['constraint jacobian derivative ', int2str(i)]);
                    else
                        dF = simplify(dF);
                    end
                end
                
                LinearizationConstant = P - F*q;
                if obj.ToSimplify
                    disp(['Started simplifying constraint linearization constant ', int2str(i), ' out of ', ...
                        int2str(number_of_points)]);
                    if obj.UseParallelizedSimplification
                        LinearizationConstant = obj.Math.ParallelizedSimplification(LinearizationConstant, ...
                            ['constraint linearization constant ', int2str(i)]);
                    else
                        LinearizationConstant = simplify(LinearizationConstant);
                    end
                end
                
                Constraints{i}.Expression = P;
                Constraints{i}.Inputs = q;
                Constraints{i}.ExpressionName = ['Constrained point ', int2str(i)];
                
                ConstraintJacobians{i}.Expression = F;
                ConstraintJacobians{i}.Inputs = q;
                ConstraintJacobians{i}.ExpressionName = ['Constrained point jacobian ', int2str(i)];
                
                ConstraintJacobianDerivatives{i}.Expression = dF;
                ConstraintJacobianDerivatives{i}.Inputs = {q, v};
                ConstraintJacobianDerivatives{i}.ExpressionName = ...
                    ['Constrained point jacobian derivative ', int2str(i)];
                
                ConstraintLinearizationConstants{i}.Expression = LinearizationConstant;
                ConstraintLinearizationConstants{i}.Inputs = q;
                ConstraintLinearizationConstants{i}.ExpressionName = ...
                    ['Constrained point linearization constant ', int2str(i)];
            end
            
            obj.ConstraintsList = obj.Math.Generate_functions_en_masse_Cell('g_UnilateralConstraint', Constraints);
            obj.ConstraintJacobiansList = ...
                obj.Math.Generate_functions_en_masse_Cell('g_UnilateralConstraintJacobian', ConstraintJacobians);
            obj.ConstraintJacobianDerivativesList = ...
                obj.Math.Generate_functions_en_masse_Cell('g_UnilateralConstraintJacobianDerivative', ConstraintJacobianDerivatives);
            obj.ConstraintLinearizationConstantsList = ...
                obj.Math.Generate_functions_en_masse_Cell('g_UnilateralConstraintLinearizationConstant', ConstraintLinearizationConstants);
            
            rehash();
        end
        
        %generates constraints functions
        %takes 3xn symbolic arrays as inputs
        %
        %example:
        %rO2 = SymbolicEngine.LinkArray(2).AbsoluteBase;
        %rF2 = SymbolicEngine.LinkArray(2).AbsoluteFollower;
        %.GenerateConstrainsFunctions(rO2, rF2)
        %
        %a wrapper for .GenerateConstrainsFunctionsCell method
        function GenerateConstrainsFunctions(obj, varargin)
            number_of_inputs = length(varargin);
            PointsArray = cell(number_of_inputs, 1);
            
            index = 0;
            for i = 1:number_of_inputs
                input = varargin{i};
                if size(input, 1) ~= 3
                    error(['input ', inputname(i + 1), ' has needs to have three rows']);
                end
                
                %the following code allows to handle both vectors and
                %matrices
                k = size(input, 2);
                for j = 1:k
                    index = index + 1;
                    PointsArray{index} = input(:, j);
                end
            end
            
            obj.GenerateConstrainsFunctionsCell(PointsArray);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% following are simulation methods
        
        
        %constructs the active set of cnstraints for the currecnt
        %configuration of the mechanism
        %returns a cell array with structure elements 
        function ActiveSet = GetActiveSet(obj, q, v)
            number_of_points = length(obj.ConstraintsList);
            
            index = 0; ActiveSet = {};
            for i = 1:number_of_points
                Point = obj.ConstraintsList{i}.FunctionHandle(q);
                if obj.intersection_tester(Point)
                    index = index + 1;
                    ActiveSet{index} = obj.get_constraint_properties(Point);
                    
                    n = ActiveSet{index}.Normal;
                    rC = ActiveSet{index}.ConstraintPoint;
                    V = ActiveSet{index}.TangentBasis;
                    
                    F = obj.ConstraintJacobiansList{i}.FunctionHandle(q);
                    dF = obj.ConstraintJacobianDerivativesList{i}.FunctionHandle(q, v);
                    d = obj.ConstraintLinearizationConstantsList{i}.FunctionHandle(q);
                    
                    ActiveSet{index}.NormalJacobian = n'*F;
                    ActiveSet{index}.TangentJacobian = V'*F;
                    ActiveSet{index}.NormalJacobianDerivative = n'*dF;
                    ActiveSet{index}.TangentJacobianDerivative = V'*dF;
                    ActiveSet{index}.NormalLinearizationConstant = n'*(rC - d);
                    ActiveSet{index}.TangentLinearizationConstant = V'*(rC - d);
                end
            end
        end
        
        
        function OutputStructure = SolveLCP_YALMIP(~, InputStructure)
            
            dof = InputStructure.dof;
            number_of_constraints = InputStructure.number_of_constraints;

            ddq = sdpvar(dof, 1);
            multiplier_normal = sdpvar(number_of_constraints, 1);
            multiplier_tangent = sdpvar(2*number_of_constraints, 1);
            
            JSIM = InputStructure.JSIM;
            RHS = InputStructure.RHS;
            
            q = InputStructure.q;
            v = InputStructure.v;
            dt = InputStructure.TimeStep;
            
            NormalJacobian = InputStructure.NormalJacobian;
            TangentJacobian = InputStructure.TangentJacobian;
            
            NormalJacobianDerivative = InputStructure.NormalJacobianDerivative;
            TangentJacobianDerivative = InputStructure.TangentJacobianDerivative;
            
            NormalLinearizationConstant = InputStructure.NormalLinearizationConstant;
            TangentLinearizationConstant = InputStructure.TangentLinearizationConstant;
            
            z1 = zeros(number_of_constraints, 1);
            z2 = zeros(2*number_of_constraints, 1);
            
            Constraints = [JSIM*ddq == RHS + NormalJacobian'*multiplier_normal + TangentJacobian'*multiplier_tangent];
            Constraints = [Constraints, (multiplier_normal >= z1)];
            
%             Constraints = [Constraints, (diag(multiplier_normal)*(NormalJacobian*ddq + NormalJacobianDerivative*(v + ddq*dt)) == z2)];
            Constraints = [Constraints, (NormalJacobian*(v + ddq*dt)  >= z1)];
%             Constraints = [Constraints, (NormalJacobian*(q + v*dt + ddq*0.5*dt^2) == NormalLinearizationConstant)];
            
%             Constraints = [Constraints, (TangentJacobian*ddq + TangentJacobianDerivative*(v + ddq*dt) == z2)];
%             Constraints = [Constraints, (TangentJacobian*(v + ddq*dt)  == z2)];
%             Constraints = [Constraints, (TangentJacobian*(q + v*dt + ddq*0.5*dt^2) == TangentLinearizationConstant)];
            
%             Objective = norm(ddq) + norm(multiplier_normal) + norm(multiplier_tangent);
            Objective = 0;
%             Objective = Objective + norm(diag(multiplier_normal)*(NormalJacobian*ddq + NormalJacobianDerivative*(v + ddq*dt)));
            Objective = Objective + norm(TangentJacobian*ddq + TangentJacobianDerivative*(v + ddq*dt));
            Objective = Objective + norm(TangentJacobian*(v + ddq*dt));
            Objective = Objective + norm(TangentJacobian*(q + v*dt + ddq*0.5*dt^2) - TangentLinearizationConstant);
            
            mu = 0.01;
            Objective = Objective + mu*(norm(ddq) + norm(multiplier_normal) + norm(multiplier_tangent));
            
            ops = sdpsettings('verbose', 0);
            % Solve the problem
            sol = optimize(Constraints, Objective, ops);
            
            if sol.problem == 0
                % Extract and display value
                
                OutputStructure.ddq = value(ddq);
                OutputStructure.multiplier_normal = value(multiplier_normal);
                OutputStructure.multiplier_tangent = value(multiplier_tangent);
            else
                display('YALMIP failed to solve the problem');
                sol.info
                yalmiperror(sol.problem)
                
                OutputStructure = [];
            end    
        end
        
        
        
        function Solver = GetDynamicsSolver(obj)
            function OutputStructure = InfiniteFrictionSolver(InputStructure)
                q = InputStructure.q;
                v = InputStructure.v;
                u = InputStructure.u;
                
                JSIM = g_dynamics_JSIM(q);
                RHS = g_dynamics_RHS(q, v, u);
                
                %get active set of cinstraints
                ActiveSet = obj.GetActiveSet(q, v);
                
                if ~isempty(ActiveSet)
                    
                    %find number of constraints
                    number_of_constraints = length(ActiveSet);
                    dof = obj.SimulationEngine.dof;
                    
                    %construct Jacobians and constants; ActiveSet is a cell
                    %array, here we convert it into matrices and vectors
                    NormalJacobian = zeros(number_of_constraints, dof);
                    TangentJacobian = zeros(number_of_constraints, dof);
                    
                    NormalJacobianDerivative = zeros(number_of_constraints, dof);
                    TangentJacobianDerivative = zeros(2*number_of_constraints, dof);
                    
                    NormalLinearizationConstant = zeros(number_of_constraints, 1);
                    TangentLinearizationConstant = zeros(2*number_of_constraints, 1);
                    
                    for i = 1:number_of_constraints
                        NormalJacobian(i, :) = ActiveSet{i}.NormalJacobian;
                        NormalJacobianDerivative(i, :) = ActiveSet{i}.NormalJacobianDerivative;
                        NormalLinearizationConstant(i) = ActiveSet{i}.NormalLinearizationConstant;
                        
                        index = (i - 1)*2 + 1;
                        TangentJacobian(index, :) = ActiveSet{i}.TangentJacobian(1, :);
                        TangentJacobian(index+1, :) = ActiveSet{i}.TangentJacobian(2, :);
                        TangentJacobianDerivative(index, :) = ActiveSet{i}.TangentJacobianDerivative(1, :);
                        TangentJacobianDerivative(index+1, :) = ActiveSet{i}.TangentJacobianDerivative(2, :);
                        TangentLinearizationConstant(index) = ActiveSet{i}.TangentLinearizationConstant(1);
                        TangentLinearizationConstant(index+1) = ActiveSet{i}.TangentLinearizationConstant(2);
                    end
                    
                    %form LCP_Sover_InputStructure
                    LCP_Sover_InputStructure.JSIM = JSIM;
                    LCP_Sover_InputStructure.RHS = RHS;
                    LCP_Sover_InputStructure.q = q;
                    LCP_Sover_InputStructure.v = v;
                    LCP_Sover_InputStructure.NormalJacobian = NormalJacobian;
                    LCP_Sover_InputStructure.TangentJacobian = TangentJacobian;
                    LCP_Sover_InputStructure.NormalJacobianDerivative = NormalJacobianDerivative;
                    LCP_Sover_InputStructure.TangentJacobianDerivative = TangentJacobianDerivative;
                    LCP_Sover_InputStructure.NormalLinearizationConstant = NormalLinearizationConstant;
                    LCP_Sover_InputStructure.TangentLinearizationConstant = TangentLinearizationConstant;
                    LCP_Sover_InputStructure.TimeStep = obj.SimulationEngine.TimeStep;
                    LCP_Sover_InputStructure.dof = dof;
                    LCP_Sover_InputStructure.number_of_constraints = number_of_constraints;
                    
                    Sover_OutputStructure = obj.SolveLCP_YALMIP(LCP_Sover_InputStructure);
                    
                    if isempty(Sover_OutputStructure)
                        a = zeros(dof, 1);
                    else
                        a = Sover_OutputStructure.ddq
                        L1 = Sover_OutputStructure.multiplier_normal
                        L2 = Sover_OutputStructure.multiplier_tangent
                    end
                else
                    Sover_OutputStructure = [];
                    a = JSIM\RHS;
                end
                
                dt = obj.SimulationEngine.TimeStep;
                v = v + a*dt;
                q = q + v*dt + 0.5*a*dt^2;
                
                OutputStructure.q = q;
                OutputStructure.v = v;
                OutputStructure.a = a;
            end
            
            Solver = @InfiniteFrictionSolver;
        end
        
        
        
        
        
        
        
        
        
    end
end