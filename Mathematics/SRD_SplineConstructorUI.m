%This class works with polynomial splines
%This is a user interface for SplineClass, it is less flexible, but easier
%to use for some common applications
%
% to do:
% -check for consistent lengths of the NodeValues arrays and NodeTimes
% -check that first and last values in NodeValues.ZeroOrderDerivativeNodes
% array are numbers
classdef SRD_SplineConstructorUI < handle
    properties
        NodeTimes;
        %This array holds all node time for all the splines
        
        NodeValues;
        %This is a structure with three elements:
        % .ZeroOrderDerivativeNodes - cell array that determines what values 
        % splines should take at the nodes
        % .FirstOrderDerivativeNodes - cell array that determines what  
        % values first derivatives of the splines should take at the nodes
        % .SecondOrderDerivativeNodes - cell array that determines what  
        % values second derivatives of the splines should take at the nodes
        
        SplineArray;
        %This is a structure with three fields:
        % .ZeroOrderDerivative - contains splines themselves
        % .FirstOrderDerivative - contains first order time derivatives
        % of the splines
        % .SecondOrderDerivative - contains second order time derivatives
        % of the splines
        
        NumberOfSplines;
        %Number of splines in the .SplineArray (each of its fields)
        
        OutOfBoundariesBehaviour = 'Loop';
        %This determines the Out Of Boundaries Behaviour for all splines.
        %can use 'Loop', 'None', 'LastValue', 'Warning and LastValue' 'OutOfBoundariesValue'
        OutOfBoundariesValue = [];
    end
    methods
        
        function GenerateSplines(obj, NodeTimes, ZeroOrderDerivativeNodes, FirstOrderDerivativeNodes, SecondOrderDerivativeNodes)
            
            %Here we check is the inputs are correct
            %This check is for input dimentions:
            if max(size(NodeTimes)) ~= size(ZeroOrderDerivativeNodes, 2)
                error('The number of columns in ZeroOrderDerivativeNodes array should be the same as the number of time nodes')
            end
            if max(size(NodeTimes)) ~= size(FirstOrderDerivativeNodes, 2)
                error('The number of columns in FirstOrderDerivativeNodes array should be the same as the number of time nodes')
            end
            if max(size(NodeTimes)) ~= size(SecondOrderDerivativeNodes, 2)
                error('The number of columns in SecondOrderDerivativeNodes array should be the same as the number of time nodes')
            end
            %This check is to make sure that all spline starting points are defined 
            for i = 1:size(ZeroOrderDerivativeNodes, 1)
                if ~isnumeric(ZeroOrderDerivativeNodes{i, 1})
                    error('First ellements in every row of ZeroOrderDerivativeNodes should be numeric')
                end
            end
            
            %Saving data to the fields
            obj.NodeTimes = NodeTimes;
            obj.NodeValues.ZeroOrderDerivativeNodes = ZeroOrderDerivativeNodes;
            obj.NodeValues.FirstOrderDerivativeNodes = FirstOrderDerivativeNodes;
            obj.NodeValues.SecondOrderDerivativeNodes = SecondOrderDerivativeNodes;
            
            obj.NumberOfSplines = size(obj.NodeValues.ZeroOrderDerivativeNodes, 1);
            
            for i = 1:obj.NumberOfSplines
                %Here we want to create one spline. First some reassingment
                %for cleaner code.
                V1 = obj.NodeValues.ZeroOrderDerivativeNodes(i, :);
                V2 = obj.NodeValues.FirstOrderDerivativeNodes(i, :);
                V3 = obj.NodeValues.SecondOrderDerivativeNodes(i, :);
                
                Nodes = []; %Creating a vector of all nodes for a particular spline;
                for j = 1:size(V1, 2)
                    if isnumeric(V1{j}) || isnumeric(V2{j}) || isnumeric(V3{j})
                        Node.Time = obj.NodeTimes(j);
                        if isnumeric(V1{j}) 
                            Node.V1 = V1{j}; %zero-order derivatives
                        else
                            Node.V1 = [];
                        end
                        if isnumeric(V2{j}) 
                            Node.V2 = V2{j}; %first-order derivatives
                        else
                            Node.V2 = [];
                        end
                        if isnumeric(V3{j}) 
                            Node.V3 = V3{j}; %second-order derivatives
                        else
                            Node.V3 = [];
                        end
                        
                        Nodes = [Nodes; Node];
                    end
                end
                %Now we know all the nodes for this spline, and we can
                %proceed to create its segments
                
                clear Segments;
                for j = 1:(size(Nodes, 1) - 1)
                    
                    Node1 = Nodes(j);
                    Node2 = Nodes(j+1);
                    
                    Array = [];
                    %zero-order derivatives
                    if ~isempty(Node1.V1)
                        Array = [Array, [Node1.Time; Node1.V1; 0]];
                    end
                    if ~isempty(Node2.V1)
                        Array = [Array, [Node2.Time; Node2.V1; 0]];
                    end
                    
                    %first-order derivatives
                    if ~isempty(Node1.V2)
                        Array = [Array, [Node1.Time; Node1.V2; 1]];
                    end
                    if ~isempty(Node2.V2)
                        Array = [Array, [Node2.Time; Node2.V2; 1]];
                    end
                    
                    %second-order derivatives
                    if ~isempty(Node1.V3)
                        Array = [Array, [Node1.Time; Node1.V3; 2]];
                    end
                    if ~isempty(Node2.V3)
                        Array = [Array, [Node2.Time; Node2.V3; 2]];
                    end
                        
                    Segments{j} = Array;
                end
                
                Times_for_this_spline = zeros(size(Nodes, 1), 1);
                for j = 1:(size(Nodes, 1))
                    Times_for_this_spline(j) = Nodes(j).Time;
                end
                
                %Generating splines
                obj.SplineArray.ZeroOrderDerivative(i) = SRD_SplineConstructor(Segments, Times_for_this_spline);
                obj.SplineArray.ZeroOrderDerivative(i).OutOfBoundariesBehaviour = obj.OutOfBoundariesBehaviour;
                
                %Generating their derivative splines
                obj.SplineArray.FirstOrderDerivative(i) = obj.SplineArray.ZeroOrderDerivative(i).GenerateDerivatveSpline(1);
                obj.SplineArray.SecondOrderDerivative(i) = obj.SplineArray.ZeroOrderDerivative(i).GenerateDerivatveSpline(2);
            end
            obj.UpdateOutOfBoundariesBehaviour;
        end
        
        %This function changes .UpdateOutOfBoundariesBehaviour property of
        %all splines (SplineClass objects) to the value stored in 
        %obj.UpdateOutOfBoundariesBehaviour property of this class
        %and makes their derivatives act appropriately
        function UpdateOutOfBoundariesBehaviour(obj)
            for i = 1:obj.NumberOfSplines
                obj.SplineArray.ZeroOrderDerivative(i).OutOfBoundariesBehaviour = obj.OutOfBoundariesBehaviour;
                switch obj.OutOfBoundariesBehaviour
                    case {'Loop', 'None'}
                        obj.SplineArray.FirstOrderDerivative(i).OutOfBoundariesBehaviour = obj.OutOfBoundariesBehaviour;
                        obj.SplineArray.SecondOrderDerivative(i).OutOfBoundariesBehaviour = obj.OutOfBoundariesBehaviour;
                    case {'LastValue', 'Warning and LastValue', 'OutOfBoundariesValue'}
                        obj.SplineArray.ZeroOrderDerivative(i).OutOfBoundariesValue = obj.OutOfBoundariesValue;
                        obj.SplineArray.FirstOrderDerivative(i).OutOfBoundariesBehaviour = 'OutOfBoundariesValue';
                        obj.SplineArray.FirstOrderDerivative(i).OutOfBoundariesValue = 0;
                        obj.SplineArray.SecondOrderDerivative(i).OutOfBoundariesBehaviour = 'OutOfBoundariesValue';
                        obj.SplineArray.SecondOrderDerivative(i).OutOfBoundariesValue = 0;
                    otherwise
                        warning('Invalid OutOfBoundariesBehaviour type');
                end
            end
        end
        
        %This function evaluates spline at a time t
        function q = EvaluateQ(obj, t)
            q = zeros(obj.NumberOfSplines, 1);
            for i = 1:obj.NumberOfSplines
                q(i) = obj.SplineArray.ZeroOrderDerivative(i).EvaluateSpline(t);
            end
        end
        function handle = get_EvaluateQ_handle(obj)
            handle = @(t) EvaluateQ(obj, t);
        end
        
        %This function evaluates the first derivative of a spline at a time t
        function v = EvaluateV(obj, t)
            v = zeros(obj.NumberOfSplines, 1);
            for i = 1:obj.NumberOfSplines
                v(i) = obj.SplineArray.FirstOrderDerivative(i).EvaluateSpline(t);
            end
        end
        function handle = get_EvaluateV_handle(obj)
            handle = @(t) EvaluateV(obj, t);
        end
        
        %This function evaluates the second derivative of a spline at a time t
        function a = EvaluateA(obj, t)
            a = zeros(obj.NumberOfSplines, 1);
            for i = 1:obj.NumberOfSplines
                a(i) = obj.SplineArray.SecondOrderDerivative(i).EvaluateSpline(t);
            end
        end
        function handle = get_EvaluateA_handle(obj)
            handle = @(t) EvaluateA(obj, t);
        end
        
        %This function evaluates spline at a time t, well as its first and
        %second derivatives
        function [q, v, a] = EvaluateAll(obj, t)
            q = obj.EvaluateQ(t);
            v = obj.EvaluateV(t);
            a = obj.EvaluateA(t);
        end
        
        %generates a cell array filled with filler or asterisks '*' if the 
        %filler isn't specified
        function EmptyArray = GenerateEmptyArray(~, Template, filler)
            if nargin < 3
                filler = '*';
            end
            
            EmptyArray = cell(size(Template));
            for i = 1:size(Template, 1)
                for j = 1:size(Template, 2)
                    EmptyArray{i, j} = filler;
                end
            end
        end
                  
    end
end














