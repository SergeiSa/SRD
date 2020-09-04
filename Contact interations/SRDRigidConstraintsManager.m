%This class allows modelling rigid mechanical constraints. Its purpose is
%to provide a combined constraints Jacobian for a mechanical system with a
%number of constraints, some of which can be inactive.
classdef SRDRigidConstraintsManager < handle
    properties
        ConstarintsArray = {}
    end
    methods
        % rK(q) = rK0; F = d(rK)/dq
        % 
        % Constraint - function handle for the constraint's left hand side, rK(q)
        % Position   - number, constraint's right hand side, rK0
        % Jacobian   - function handle for the constraint's jacobian, F
        % JacobianDerivative - function handle, derivative of the F
        % Active     - binary, true/false
        % Name       - constraint's name
        function Add(obj, Constraint, Position, Jacobian, JacobianDerivative, Active, Name)
            if nargin < 7
                Name = [];
            end
            
            n = length(obj.ConstarintsArray);
            
            obj.ConstarintsArray(n + 1).Constraint = Constraint;
            obj.ConstarintsArray(n + 1).Position = Position;
            obj.ConstarintsArray(n + 1).Jacobian = Jacobian;
            obj.ConstarintsArray(n + 1).JacobianDerivative = JacobianDerivative;
            obj.ConstarintsArray(n + 1).Active = Active;
            obj.ConstarintsArray(n + 1).Name = Name;
        end
        
        function NoA = GetNumberOfActiveConstraints(obj)
            n = length(obj.ConstarintsArray);
            
            NoA = 0;
            for i = 1:n
                if obj.ConstarintsArray(i).Active
                    NoA = NoA + 1;
                end
            end
        end
        
        function Constraint = Read_Constraint(obj, q)
            n = length(obj.ConstarintsArray);
            
            Constraint = [];
            for i = 1:n
                if obj.ConstarintsArray(i).Active
                    Constraint = [Constraint; obj.ConstarintsArray(i).Constraint(q)];
                end
            end
        end
        
        function Position = Read_Position(obj)
            n = length(obj.ConstarintsArray);
            
            Position = [];
            for i = 1:n
                if obj.ConstarintsArray(i).Active
                    Position = [Position; obj.ConstarintsArray(i).Position];
                end
            end
        end
        
        function Jacobian = Read_Jacobian(obj, q)
            n = length(obj.ConstarintsArray);
            
            Jacobian = [];
            for i = 1:n
                if obj.ConstarintsArray(i).Active
                    Jacobian = [Jacobian; obj.ConstarintsArray(i).Jacobian(q)];
                end
            end
        end
        
        function JacobianDerivative = Read_JacobianDerivative(obj, q, v)
            n = length(obj.ConstarintsArray);
            
            JacobianDerivative = [];
            for i = 1:n
                if obj.ConstarintsArray(i).Active
                    JacobianDerivative = [JacobianDerivative; obj.ConstarintsArray(i).JacobianDerivative(q, v)];
                end
            end
        end
        
        %%%%%%%%%%%%%%%%%%%
        
        function NoR = GetNumberOfReactionComponents(obj)
            n = length(obj.ConstarintsArray);
            
            NoR = 0;
            for i = 1:n
                NoR = NoR + length(obj.ConstarintsArray(i).Position);
            end
        end
        
        function NoR = GetNumberOfActiveReactionComponents(obj)
            n = length(obj.ConstarintsArray);
            
            NoR = 0;
            for i = 1:n
                if obj.ConstarintsArray(i).Active
                    NoR = NoR + length(obj.ConstarintsArray(i).Position);
                end
            end
        end
        
        %takes in a vector of reactions produced by the current set of
        %active constraints and maps it to a vector of all possible
        %reactions.
        function ReactionVector = Parse_lambda(obj, lambda)
            
            n = length(obj.ConstarintsArray);
            
            NoR = obj.GetNumberOfReactionComponents();
            ReactionVector = zeros(NoR, 1);
            
            cursor_ReactionVector = 1;
            cursor_lambda = 1;
            
            for i = 1:n
                L = length(obj.ConstarintsArray(i).Position);
                
                if obj.ConstarintsArray(i).Active
                    
                    range_ReactionVector = cursor_ReactionVector:(cursor_ReactionVector + L - 1);
                    range_lambda         = cursor_lambda:(cursor_lambda + L - 1);
                    
                    ReactionVector(range_ReactionVector) = lambda(range_lambda);
                    
                    cursor_lambda = cursor_lambda + L;
                end
                
                cursor_ReactionVector = cursor_ReactionVector + L;
            end
        end
            
            
    end
end