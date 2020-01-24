%This class provides blockmatrix operations
%last update 17.12.17
classdef TPBlockMatrix < handle & matlab.mixin.SetGet
    properties
        Matrix;
        %the matrix we are working with
        
        matrix_size_x;
        matrix_size_y;
        %dimentions of the matrix we are working with
        
        matrix_size_in_blocks_x;
        matrix_size_in_blocks_y;
        %dimentions of the matrix we are working with (in blocks)
        
        block_size_x;
        block_size_y;
        %dimentions of the block we are working with
    end
    methods
        %create matrix of n by m blocks Block
        function obj = TPBlockMatrix(Block, n, m)
            [obj.block_size_x, obj.block_size_y] = size(Block);
            
            obj.matrix_size_in_blocks_x = n;
            obj.matrix_size_in_blocks_y = m;
            
            obj.matrix_size_x = n * obj.block_size_x;
            obj.matrix_size_y = m * obj.block_size_y;
            
            obj.Matrix = zeros(obj.matrix_size_x, obj.matrix_size_y);
            
            if ~isnumeric(Block)
                obj.Matrix = sym(obj.Matrix);
            end
        end
        
        %read block number (i, j)
        function res = subsref(obj, S)
            switch S.type
                case '.'
                    res = get(obj, S.subs);
                case '()'
                    i = S.subs{1}; j = S.subs{2};
                    
                    if strcmp(i, ':')
                        range_x = 1:obj.matrix_size_x;
                    else
                        if (i > 0) && (i <= obj.matrix_size_in_blocks_x)
                            range_x = (1 + obj.block_size_x*(i-1)):(obj.block_size_x*i);
                        else
                            error('first index out of range');
                        end
                    end
                    if strcmp(j, ':')
                        range_y = 1:obj.matrix_size_y;
                    else
                        if (j > 0) && (j <= obj.matrix_size_in_blocks_y)
                            range_y = (1 + obj.block_size_y*(j-1)):(obj.block_size_y*j);
                        else
                            error('second index out of range');
                        end
                    end
                    
                    res = obj.Matrix(range_x, range_y);
                    
                otherwise
                    error('This class does not know how to do that');
            end
        end
        
        %write block number (i, j)
        function obj = subsasgn(obj, S, Block)
            i = S.subs{1}; j = S.subs{2};
            if (i > 0) && (i <= obj.matrix_size_in_blocks_x) && ...
                    (j > 0) && (j <= obj.matrix_size_in_blocks_y)
                obj.Matrix((1 + obj.block_size_x*(i-1)):(obj.block_size_x*i), ...
                    (1 + obj.block_size_y*(j-1)):(obj.block_size_y*j)) = Block;
            else
                error('index out of range');
            end
        end      


        function Value = get.Matrix(obj)
            Value = obj.Matrix;
        end
        function Value = get.matrix_size_x(obj)
            Value = obj.matrix_size_x;
        end
        function Value = get.matrix_size_y(obj)
            Value = obj.matrix_size_y;
        end
        function Value = get.matrix_size_in_blocks_x(obj)
            Value = obj.matrix_size_in_blocks_x;
        end
        function Value = get.matrix_size_in_blocks_y(obj)
            Value = obj.matrix_size_in_blocks_y;
        end
        function Value = get.block_size_x(obj)
            Value = obj.block_size_x;
        end
        function Value = get.block_size_y(obj)
            Value = obj.block_size_y;
        end
        
    end
end