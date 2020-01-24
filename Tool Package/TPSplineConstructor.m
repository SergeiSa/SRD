%This class works with polynomial splines
classdef TPSplineConstructor < handle
    properties
        Coefficients %Coefficients of the spline
        Times %a column vector that determines for which segment of time will
              %each polynomial be used.
        
        %This property determines what the spline will do after 
        OutOfBoundariesBehaviour = 'Loop';
        OutOfBoundariesValue = [];
    end
    properties (Access = private) 
        ValidValues_OutOfBoundariesBehaviour = {'Loop', 'None', 'LastValue', 'Warning and LastValue', 'OutOfBoundariesValue'};
    end
    methods

        %Segments - a cell array, each element of the cell array - an array 
        %with three rows, first row has times of points through which
        %the polynomials go, the second row has the values of the function,  
        %and the third row has the orders of the derivatives for which the 
        %values are given
        %
        %Times - a column vector that determines for which segment of time will
        %each polynomial be used.
        %Times has n + 1 elements, where n -  number of segments 
        %(polynomials that comprize the spline)
        %
        % Example:
        %
        % Segments{1} = [0, 0, 2, 4, 4;  - time row
        %                0, 1, 5, 1, 0;  - value row
        %                1, 0, 0, 0, 1]; - derivative order row
        %
        % Segments{2} = [4, 4, 6, 6;  - time row
        %                1, 0, 0, 1;  - value row
        %                0, 1, 1, 0]; - derivative order row
        %
        % Times = [0; 4; 6];
        %
        % Spline = TPSplineConstructor(Segments, Times)
        %
        function obj = TPSplineConstructor(Segments, Times)
            if nargin == 2
            t = sym('t');
            obj.Times = Times;
            
            NumberOfSegments = size(Times, 1) - 1; %this is explained above
            for i = 1:NumberOfSegments
                S = Segments{i};
                NumberOfPoints = size(S, 2);
                
                DefaultMatrixLine = sym(zeros(NumberOfPoints, 1)); %Memory preallocation
                for j = 1:NumberOfPoints                             % Here we make a vector [t^n; t^(n-1); t^(n-2); ... t; 1] 
                    DefaultMatrixLine(j) = t^(NumberOfPoints - j);   % It is used to construct the matrix for SLAE that is
                end                                                  % being solved for polynomial coefficients
                % The mentioned SLAE has form: Matrix*Coefficients{i} = Values

                Matrix = zeros(NumberOfPoints, NumberOfPoints); %Memory preallocation                
                Values = zeros(NumberOfPoints, 1);              %Memory preallocation   
                for j = 1:NumberOfPoints
                    % this should be read as:
                    % d^DerivativeOrder(f(t))/dt = Values(j)
                    % where f(t) = t^n + t^(n-1) + t^(n-2) + ... + t + 1;
                    % There are NumberOfPoints of these equalities, and they form a square matrix mentioned above 
                    Time = S(1, j);
                    Values(j) = S(2, j);
                    DerivativeOrder = S(3, j);

                    MatrixLine = DefaultMatrixLine;
                    for k = 1:DerivativeOrder                 % here we differentiate
                        MatrixLine = jacobian(MatrixLine, t); % MatrixLine n times,  
                    end                                       % n = DerivativeOrder
                    MatrixLine = subs(MatrixLine, t, Time); %here we substitute Time (numerical value) 
                                                            %for t (which is a symbolic variable)          
                    Matrix(j, :) = eval(MatrixLine)';  %Here we are evaluating MatrixLine to transform it into 
                                                       %a numerical vector
                end
                obj.Coefficients{i} = Matrix\Values;      
            end
            obj.Coefficients{NumberOfSegments + 1} = obj.Coefficients{NumberOfSegments}; %this is meant to handle the case
                                                                                         %when the time for which the spline is 
                                                                                         %evaluated exceeds the range where it is defined
            end                                                                                 
        end
        
        % Evaluate the n-th derivative of the spline for time t,
        % where n = Order;
        function Value = EvaluateSpline(obj, Time, Order)
            
            if nargin < 3
                Order = 0;
            end
            
            Value = [];
            
            if Time > obj.Times(size(obj.Times, 1))  %this means that t is bigger than any
            %element of obj.Times and there is possibly a need to change it
                if strcmp(obj.OutOfBoundariesBehaviour, 'OutOfBoundariesValue')
                    Value = obj.OutOfBoundariesValue;
                else
                    Time = obj.OutOfBoundariesFix(Time);
                end
            end                                      
 
            if isempty(Value)
                
                NumberOfSegments = size(obj.Times, 1) - 1; %this is explained above
                for i = 1:NumberOfSegments
                    if ((Time >= obj.Times(i)) && (Time <= obj.Times(i + 1))) %here we find which segment
                        C = obj.Coefficients{i};                              %(a polynomial function) needs to be used
                    end
                end
                
                if Order > 0                                   %when Order > 0 we first need to
                    for j = 1:Order                            %differentiate the polynomial n times (n = Order)
                        C = polyder(C);
                    end
                end
                
                Value = polyval(C, Time); %evaluating the function
                
            end
        end
        
        %This function changes the time for which the polynomial is being
        %evaluated in the case when the time is out of range for which the
        %spline is defined
        %the change is done according to the settings (obj.OutOfBoundariesBehaviour)
        function CorrectedTime = OutOfBoundariesFix(obj, t)
            switch obj.OutOfBoundariesBehaviour
                case 'Loop'
                    Period = obj.Times(size(obj.Times, 1)); %this is the period of the function
                    NumberOfPeriods = floor(t / Period);    
                    CorrectedTime = t - NumberOfPeriods*Period;
                case 'None'
                    CorrectedTime = t;
                case 'LastValue'
                    CorrectedTime = obj.Times(size(obj.Times, 1));
                case 'Warning'
                    CorrectedTime = obj.Times(size(obj.Times, 1));
                    warning('The time is out of bounds');
                otherwise
                    warning('Invalid OutOfBoundariesBehaviour type'); 
            end                   
        end
        
        % This function generates a new Spline (an object of this class)
        % that is composed of derivatives of the polynomals of the spline obj
        % It is useful because evaluating the generated spline is faster
        % than evaluating a derivative of the existing spline
        function Spline = GenerateDerivatveSpline(obj, Order)
            
            Spline = TPSplineConstructor;
            Spline.Coefficients = obj.Coefficients;
            Spline.Times = obj.Times;
            Spline.OutOfBoundariesBehaviour = obj.OutOfBoundariesBehaviour;
            
            
            NumberOfSegments = size(obj.Times, 1) - 1; %this is explained above
            for i = 1:NumberOfSegments 
                C = obj.Coefficients{i};
                for j = 1:Order %differentiate the polynomial n times (n = Order)
                    C = polyder(C);
                end
                Spline.Coefficients{i} = C;
            end
        end
        
    end
end