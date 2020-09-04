%This class implements some useful math operations
classdef TP_3Dmatrices
    properties
        %%%%%%%%%%%%%
        %%% parallel computation settings
        
        UseParallel = true;
        %determines whether or not parallel computations are used
        
        NumberOfWorkers = 8;
        %determines the number of workers in the parallel pull
    end
    methods
        %class constructor
        function obj = TP_3Dmatrices(NumberOfWorkers)
            if nargin < 1
                obj.NumberOfWorkers = 8;
            else
                obj.NumberOfWorkers = NumberOfWorkers;
            end
        end
        
        %multiplices
        function C = MatrixProduct(~, A, B)
            n1 = size(A, 1); n2 = size(B, 1);
            m1 = size(A, 2); m2 = size(B, 2);
            k1 = size(A, 3); k2 = size(B, 3);
            
            %To visualise what is goin on here, think about 3d matrix A as
            %sliced vertically; every slice is multiplied by a scalar
            %element from i-th column of B, and then all slices are summed up,
            %to form a slice of the resulting matrix. This is done for all
            %columns of B
            if (k1 > 1) && (k2 == 1)
                C = zeros(n1, m2, k1);
                if ~(isnumeric(A) && isnumeric(B))
                    C = sym(C);
                end
                
                for i = 1:m2
                    for j = 1:m1
                        C(:, i, :) = C(:, i, :) + A(:, j, :)*B(j, i);
                    end
                end
            end
            %To visualise what is goin on here, think about 3d matrix B as
            %sliced horisontally; every slice is multiplied by a scalar
            %element from i-th row of A, and then all slices are summed up,
            %to form a slice of the resulting matrix. This is done for all
            %rows of A
            if (k1 == 1) && (k2 > 1)
                C = zeros(n1, m2, k2);
                if ~(isnumeric(A) && isnumeric(B))
                    C = sym(C);
                end
                
                for i = 1:n1
                    for j = 1:n2
                        C(i, :, :) = C(i, :, :) + B(j, :, :)*A(i, j);
                    end
                end
            end
            if (k1 == 1) && (k2 == 1)
                C = A*B;
            end
            if (k1 > 1) && (k2 > 1)
                error('method MatrixProduct can not multiply two 3d matrices')
            end
        end
        
        %Input a NxMx1 or a Nx1xK or a 1xMxK matrix and get NxM = NxK or a MxK matrix
        function C = ConvertTo2D(~, X)
            n = size(X, 1);
            m = size(X, 2);
            k = size(X, 3);
            
            if n == 1
                C = zeros(m, k);
                if ~isnumeric(X)
                    C = sym(C);
                end
                for i = 1:k
                    C(:, i) = X(1, :, i);
                end
            end
            
            if m == 1
                C = zeros(n, k);
                if ~isnumeric(X)
                    C = sym(C);
                end
                for i = 1:k
                    C(:, i) = X(:, 1, i);
                end
            end
            
            if k == 1
                C = zeros(n, m);
                if ~isnumeric(X)
                    C = sym(C);
                end
                for i = 1:m
                    C(:, i) = X(:, i, 1);
                end
            end
            
            if (n ~= 1) && (m ~= 1) && (k ~= 1)
                error('Full 3d matrix detected')
            end
        end
        
        
        %finds jacobians of the columns of the input matrix A with
        %argument x
        %
        %(!) can speed up by using parallelization
        function C = MatrixJacobian(~, A, x)
            n = size(A, 1);
            m = size(A, 2);
            k = length(x);
            
            C = zeros(n, m, k);
            C = sym(C);
            
            for i = 1:m
                C(:, i, :) = jacobian(A(:, i), x);
            end
        end
        
        %computes jacobian of a matrix product J = d(A*B)/dx
        %you have options to provide dA/dx or dB/dx, to make computation
        %faster
        %
        %this is useful when you want to find d(A*B)/dx and already know
        %either dA/dx or dB/dx
        function C = MatrixProductJacobian(obj, A, B, x, dAdx, dBdx)
            if nargin < 6
                dBdx = [];
            end
            if nargin < 5
                dAdx = [];
            end
            
            if isempty(dAdx)
                dAdx = obj.MatrixJacobian(A, x);
            end
            if isempty(dBdx)
                dBdx = obj.MatrixJacobian(B, x);
            end
            
            C = obj.MatrixProduct(dAdx, B) + obj.MatrixProduct(A, dBdx);
        end
        
    end
end