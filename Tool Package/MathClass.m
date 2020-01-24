%This class implements some useful math operations
%last update 09.10.17
classdef MathClass
    properties
        %%%%%%%%%%%%%
        %%% parallel computation settings
        
        UseParallel = true;
        %determines whether or not parallel computations are used
        
        NumberOfWorkers = 8;
        %determines the number of workers in the parallel pull
        
        %%%%%%%%%%%%%
        %%% function generation settings
        
        ToOptimizeFunctions = true;
        %determines whether or not functions generated from simbolic
        %expression are simplified
        
        %%%%%%%%%%%%%
        %%% symbolics settings
        
        simplify_Steps = 1;
        
    end
    methods
        
        %class constructor
        function obj = MathClass(NumberOfWorkers)
            if nargin < 1
                obj.NumberOfWorkers = 8;
            else
                obj.NumberOfWorkers = NumberOfWorkers;
            end
        end
        
        %%%%%%%%%%%%%
        %%% Matrix operations
        
        %2D Rotation Matrix
        function T = RotationMatrix2D(~, q)
            T = [cos(q), -sin(q);
                sin(q),  cos(q)];
        end
        
        %Rotation Matrix around x axis
        function T = RotationMatrix3D_x(~, q)
            T = [1, 0,       0;
                0, cos(q), -sin(q);
                0, sin(q),  cos(q)];
        end
        
        %Rotation Matrix around y axis
        function T = RotationMatrix3D_y(~, q)
            T = [cos(q),  0, sin(q);
                0,       1, 0;
                -sin(q), 0, cos(q)];
        end
        
        %Rotation Matrix around z axis
        function T = RotationMatrix3D_z(~, q)
            T = [cos(q), -sin(q), 0;
                sin(q), cos(q),  0;
                0,      0,       1];
        end
        
        %returns cross product matrix for the input vector (only for three
        %dimentional vectors)
        function CPM = CrossProductMatrix(~, input)
            CPM = [0,       -input(3), input(2);
                   input(3), 0,       -input(1);
                  -input(2), input(1), 0];
        end
        %returns cross product matrix for the input vector (only for three
        %dimentional vectors)
        function CPM = CrossProductMatrix2d(~, input)
            CPM = [-input(2), input(1)];
        end
        
        %returns rotation matrix around a given axis for a given angle
        function T = RotationAroundAxis(obj, Axis, theta)
            U = Axis*Axis';
            Uc = obj.CrossProductMatrix(Axis);
            T = cos(theta)*eye(3) + sin(theta)*Uc + (1 - cos(theta))*U;
        end

        %Implements a weighted pseudoinverse based on Tikhonov
        %regulirization.
        %The problem is stated as follows:
        %Ax + By = z;
        %find x and y, with different weights placed on them.
        %
        %it is solved as follows:
        %M := [A B]; T = [I1*alpha1, 0; 0, I2*alpha2], 
        %where I1 and I2 are identity matrices with sizes dependant on x
        %and y.
        %
        %[x; y] = (M'*M + T'*T) * M'z;
        %This function returns P = pinv(M'*M + T'*T) * M' - a weighted pseudoinverse
        function P = WeightedPseudoinverse(~, A, B, alpha1, alpha2, Type)
            if nargin < 6
                Type = 'Tikhonov regularization';
            end
            
            n1 = size(A, 2);
            n2 = size(B, 2);
            
            M = [A, B];
            
            switch Type
                case 'Tikhonov regularization'
                    T = blkdiag(eye(n1)*alpha1, eye(n2)*alpha2);
                    P = pinv(M'*M + T) * M';
                case 'Weighted pseudoinverse'
                    invT = blkdiag(eye(n1)*(1/alpha1), eye(n2)*(1/alpha2));
                    P = invT * Mat' * pinv(Mat * invT * Mat');
                otherwise
                    error('Incorrect value of Type parameter for .WeightedPseudoinverse method, MathClass');
            end
        end
        
        %%%%%%%%%%%%%
        %%% Indexing
        
        %finds the index into an array where the closest value to the
        %requested one lies.
        function Index = FindPlaceInArray(~, Array, Value, Type)
            if nargin < 4
                Type = 'Closest';
            end
            
            switch Type
                case 'Closest'
                    A = abs(Array - Value);
                    [~, Index] = min(A);
                case 'Closest smaller'
                    A = Array - Value;
                    A(A > 0) = -Inf;
                    [val, Index] = max(A);
                    if val == -Inf
                        Index = -1;
                    end
                case 'Closest bigger'
                    A = Array - Value;
                    A(A < 0) = Inf;
                    [val, Index] = min(A);
                    if val == Inf
                        Index = -1;
                    end
            end
        end
        
        %%%%%%%%%%%%%
        %%% Strings
        
        %returns the name of the input variable
        function VariableName = GetVariableName(~, ~)
            VariableName = inputname(2);
        end
            
        
        %%%%%%%%%%%%%
        %%% Geometry      
        
        %returns an array of points that are vertices in the convex hull of
        %the input array Vertices; columns of both arrays correspond to
        %coordinates; works with 2D and 3D vectors
        function ConvexHullVertices = convhull(~, Vertices)
            switch size(Vertices, 2)
                case 2
                    K = convhull(Vertices);
                    N = size(K, 1);
                    ConvexHullVertices = zeros(N, 2);
                    for i = 1:N
                        ConvexHullVertices(i, :) = Vertices(K(i), :);
                    end
                case 3
                    K = convhull(Vertices);
                    K = unique(K(:));
                    N = size(K, 1);
                    ConvexHullVertices = zeros(N, 3);
                    for i = 1:N
                        ConvexHullVertices(i, :) = Vertices(K(i), :);
                    end
                otherwise
                    warning('.convhull method of MathClass only works with 2 or 3 dimentional vectors')
            end 
        end
        
        %fixes input angle to lie within bounds [0 2*pi]
        function phi_fixed = AnglesIn_0to2pi(~, phi)
            phi_fixed = phi;
            if phi_fixed < 0
                phi_fixed = phi_fixed + 2*pi;
            end
            if phi_fixed > 2*pi
                phi_fixed = phi_fixed - 2*pi;
            end
        end        
        

        %%%%%%%%%%%%%
        %%% parallel computations
        
        %starts parallel pool
        function StartParallelPool(obj)
            ParallelPool = gcp('nocreate');
            if isempty(ParallelPool)
                Cluster = parcluster('local');
                Cluster.NumWorkers = obj.NumberOfWorkers;
                parpool(Cluster, Cluster.NumWorkers);
            end
        end
        

        %a wrapper for CallForAllCombinations() method, gives output as a
        %double array
        %
        %calls function z = function_handle(x, y) for every combination of
        %elements in vectors Input1, Input2 and returns three arrays:
        %2 arrays of inputs X, Y, and an array of outputs Z; Z is a double
        %array;
        function Output = CallForAllCombinationsNum(obj, Input1, Input2, function_handle)   
            Output = obj.CallForAllCombinations(Input1, Input2, function_handle);
            Output.InputArray1 = cell2mat(Output.InputArray1);
            Output.InputArray2 = cell2mat(Output.InputArray2);
            Output.OutputArray = cell2mat(Output.OutputArray);
        end
        
        
        %calls function z = function_handle(x, y) for every combination of
        %elements in vectors Input1, Input2 and returns three arrays:
        %2 arrays of inputs X, Y, and an array of outputs Z; Z is a cell
        %array;
        function Output = CallForAllCombinations(obj, Input1, Input2, function_handle)
            
            obj.StartParallelPool();
            
            n1 = length(Input1);
            n2 = length(Input2);
            
            if ~iscell(Input1)
                Input1 = num2cell(Input1);
            end
            if ~iscell(Input2)
                Input2 = num2cell(Input2);
            end
            
            InputArray1 = cell(n1, n2);
            InputArray2 = cell(n1, n2);
            OutputArray = cell(n1, n2);
            
            for i = 1:n1
                InputArray1(i, :) = Input1;
            end
            for i = 1:n2
                InputArray2(:, i) = Input2;
            end
            
            Line1 = InputArray1(:);
            Line2 = InputArray2(:);
            OutputLine = OutputArray(:);
            
            NumberOfPoints = length(Line1);
            
            index = 1;
            while index <= NumberOfPoints
                range_end = index + obj.NumberOfWorkers - 1;
                if range_end > NumberOfPoints
                    range_end = NumberOfPoints;
                end
                range = index:range_end;
                range_length = (range_end - index + 1);
                
                temp_Line1 = Line1(range);
                temp_Line2 = Line2(range);
                temp = cell(range_length, 1);
                
                disp(['Started call to function ', inputname(4), ' for elements # ', int2str(range), '; ', ...
                        int2str(NumberOfPoints - range(end)), ' others remaining']);
                parfor j = 1:range_length
                    in1 = temp_Line1{j};
                    in2 = temp_Line2{j};
                    temp{j} = function_handle(in1, in2);
                end
                OutputLine(range) = temp;
                index = range_end + 1;
            end
            
            OutputArray = reshape(OutputLine, [n1, n2]);
            
            Output.InputArray1 = InputArray1;
            Output.InputArray2 = InputArray2;
            Output.OutputArray = OutputArray;
        end
        
        
        %%%%%%%%%%%%%
        %%% Symbolic operations
        
        function X = simplify(obj, X, Name)
            if nargin < 3
                Name = inputname(2);
            end
            
            if isnumeric(X)
                X = sym(X);
            end
            
            if obj.UseParallel
                X = obj.ParallelizedSimplification(X, Name);
            else
                X = simplify(X, 'Steps', obj.simplify_Steps);
                %X = simplify(X, 'Steps', obj.simplify_Steps, 'IgnoreAnalyticConstraints', false);
            end
        end
        
        %This function implements palallelized element-wise simplification
        function X = ParallelizedSimplification(obj, X, Name)
            total = numel(X);
            
            if obj.UseParallel

                obj.StartParallelPool();
                SZ = size(X);
                x = X(:);
                
                NumberOfIterations = floor(total / obj.NumberOfWorkers);
                
                for i = 1:NumberOfIterations
                    index = (i - 1)*obj.NumberOfWorkers + 1;
                    range = index:(index + obj.NumberOfWorkers - 1);
                    temp = x(range);
                    C = sym(zeros(obj.NumberOfWorkers, 1));
                    
                    disp(['Started simplifying elements # ', int2str(range), ' of ', Name, '; ', ...
                        int2str(total - range(end)), ' others remaining']);
                    parfor j = 1:obj.NumberOfWorkers
                        C(j) = simplify(temp(j));
                    end
                    x(range) = C;
                end
                
                length = total - NumberOfIterations*obj.NumberOfWorkers;
                range = (NumberOfIterations*obj.NumberOfWorkers + 1):total;
                temp = x(range);
                C = sym(zeros(length, 1));
                
                disp(['Started simplifying last elements # ', int2str(range), ' of ', Name]);
                parfor j = 1:length
                    C(j) = simplify(temp(j));
                end
                x(range) = C;
                
                X = reshape(x, SZ);
                
            else
                for j = 1:size(X, 2)
                    for i = 1:size(X, 1)
                        index = (j - 1)*size(X, 1) + i;
                        
                        disp(['Started simplifying element in row ', int2str(i), ', column of ', Name, '; ', ...
                            int2str(j), ', ', int2str(total - index), ' others remaining'])
                        X(i, j) = simplify(X(i, j));
                    end
                end
            end
        end
        
        %this function deffirentiates symbolic matrix X(q); v = dq/dt
        function dX = MatrixDerivative(obj, X, q, v)
            
            dX = sym(zeros(size(X)));
            
            if obj.UseParallel
                obj.StartParallelPool();
                
                NumberOfIterations = floor(size(X, 2) / obj.NumberOfWorkers);
                for i = 1:NumberOfIterations
                    
                    index = (i - 1)*obj.NumberOfWorkers + 1;
                    range = index:(index + obj.NumberOfWorkers - 1);
                    temp = X(:, range);
                    C = sym(zeros(size(temp)));
                    
                    parfor j = 1:obj.NumberOfWorkers
                        C(:, j) = jacobian(temp(:, j), q)*v;
                    end
                    
                    dX(:, range) = C;
                end
                
                leftover = size(X, 2) - NumberOfIterations*obj.NumberOfWorkers;
                range = (NumberOfIterations*obj.NumberOfWorkers + 1):size(X, 2);
                temp = X(:, range);
                C = sym(zeros(size(temp)));
                parfor j = 1:leftover
                    C(:, j) = jacobian(temp(:, j), q)*v;
                end
                dX(:, range) = C;
                
            else
                for j = 1:size(X, 2)
                    V = X(:, j);
                    dX(:, j) = jacobian(V, q)*v;
                end
            end
            
        end
        
        %generates m-file functions from symbolic expressions
        %Prefix defines the prefix for the names of these functions
        %ExpressionArray is a cell array with fields:
        %.Expression - expression that will be turned into a function
        %.Inputs - inputs to that function
        %.ExpressionName - the name of teh expression (optional)
        %
        %example:
        %ExpressionArray{1}.Expression = func1;
        %ExpressionArray{1}.Inputs = x;
        %ExpressionArray{1}.ExpressionName = 'func1';
        %ExpressionArray{2}.Expression = func2;
        %ExpressionArray{2}.Inputs = x;
        %ExpressionArray{2}.ExpressionName = 'func2';
        %.Generate_functions_en_masse('MyFunctions_', ExpressionArray)
        function Output = Generate_functions_en_masse_Cell(obj, Prefix, ExpressionArray)
            number_of_functions = length(ExpressionArray);
            
            Output = cell(number_of_functions, 1);
            for i = 1:number_of_functions
                Output{i}.FunctionName = [Prefix, num2str(i)];
                Output{i}.Inputs = ExpressionArray{i}.Inputs;
                
                disp(['started generating function ', Output{i}.FunctionName, ' ', num2str(number_of_functions - i), ...
                    ' remains']);
                
                if iscell(ExpressionArray{i}.Inputs)
                    function_inputs = ExpressionArray{i}.Inputs;
                else
                    function_inputs = {ExpressionArray{i}.Inputs};
                end
                
                Output{i}.FunctionHandle = matlabFunction(ExpressionArray{i}.Expression, ...
                    'File', Output{i}.FunctionName, 'Vars', function_inputs, ...
                    'Optimize', obj.ToOptimizeFunctions);
                
                if isfield(ExpressionArray{i}, 'ExpressionName')
                    Output{i}.ExpressionName = ExpressionArray{i}.ExpressionName;
                else
                    Output{i}.ExpressionName = ['Expression', num2str(i)];
                end
            end
        end
        
        %generates m-file functions from symbolic expressions
        %Prefix defines the prefix for the names of these functions
        %function_inputs defines the inputs to the functions
        %takes symbolic expressions or cell arrays of symbolic expressions
        %
        %example: .Generate_functions_en_masse('MyFunctions_', x, fnc1, func2, {func4, func5})
        %
        %it is a wrapper for .Generate_functions_en_masse_Cell function
        function Output = Generate_functions_en_masse(obj, Prefix, function_inputs, varargin)
            
            number_of_inputs = length(varargin);
            
            index = 0; 
            ExpressionArray = cell(number_of_inputs, 1);
            
            for i = 1:number_of_inputs
                input = varargin{i};
                if iscell(input)
                    number_of_expressions = length(input);
                    for j = 1:number_of_expressions
                        index = index + 1;
                        ExpressionArray{index, 1}.Expression = input{j};
                        ExpressionArray{index, 1}.ExpressionName = [inputname(i + 3), str2double(j)];
                        ExpressionArray{index, 1}.Inputs = function_inputs;
                    end
                else
                    index = index + 1;
                    ExpressionArray{index, 1}.Expression = input;
                    ExpressionArray{index, 1}.ExpressionName = inputname(i + 3);
                    ExpressionArray{index, 1}.Inputs = function_inputs;
                end
            end
            
            Output = obj.Generate_functions_en_masse_Cell(Prefix, ExpressionArray);
        end
        
    end
end