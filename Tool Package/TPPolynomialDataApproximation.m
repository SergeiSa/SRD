%This class provides polynomial approximation functionality
%It allows to approximate data by a single polynomial or by a piece-wise
%polynomial function. It also allows to evaluate the polynomial.
%Last updated 06.10.2016
classdef TPPolynomialDataApproximation < handle
    properties
    
        TimeStart;
        %time where the approximation begins
        
        TimeEnd;
        %time where the approximation ends
        
        TotalTime;
        %duration of the approximation
        
        SegmentLength;
        %duration of a single segment of the approximation
        
        NumberOfFunctions;
        %This is the number of approximated functions
        
        NumberOfSegments; 
        % This is the number of polynomials that were
        % used to approxidate functions (number of segments).
        
        ApproximatedFunctions
        % This is a rectangular cell array, with .NumberOfFunctions rows
        % and .NumberOfPolynomials columns
        % Each element of .ApproximatedFunctions is a structure with
        % fields:
        % .p, .S, .mu
        % .p - contains the polynomial, .S and .mu - see description of
        % polyfit function output.
        % Use P ONLY with mu, see description of polyfit and polyval
        % functions
        
        PolynomialDegree;
        %Defines what degree polynomials will be used for approximation
        
        Overshoot = 0.2;
        %This defines the potion of the neibouring segments that will be
        %used in approximation of the current one.
        
        ProvidedData;
        %This is a structure that contains the data provided for
        %approximation. It has fields:
        %.Data, .Time, see description of .Approximator() method for
        %details
        
        SaveProvidedData = true;
        %If this value is false provided approximation data won't be saved
        %in .ProvidedData structure
        
        AllowedOvershootSize = 0.01;
        %Determines how far outside th etime boundaries the user can go 
        %before program will start generating warrnings. 
        
    end
    methods
        
        function obj = TPPolynomialDataApproximation()
        end
        
        %This function does the approximation. It generates
        %.ApproximatedFunctions field.
        %
        % Input Time is a column vector, contains times for data points
        % Input Data is an array, each column corresponds to a function.
        % NumberOfSegments determines how many polynomyal functions will be
        % used for approximations.
        % PolynomialDegree determines the degree of the used polynomials
        function Approximator(obj, Time, Data, NumberOfSegments, PolynomialDegree)
            
            if size(Time, 1) ~= size(Data, 1)
                error('Time and Data arrays should have the same number of rows');
            end
                
            %We want to make sure there is enough data points for the
            %approximation
            if (PolynomialDegree*NumberOfSegments) >= size(Data, 1)
                error(['There are only ', num2str(size(Data, 1)), ' data points, which gives ', ...
                    num2str(size(Data, 1)/NumberOfSegments), ...
                    ' points per segment, which is less than the degree of polynomials (',...
                    num2str(PolynomialDegree), ') requested.']);
            end
            
            obj.NumberOfFunctions = size(Data, 2);
            obj.NumberOfSegments = NumberOfSegments;
            obj.PolynomialDegree = PolynomialDegree;
            
            %find duration of the approximation
            obj.TimeStart = Time(1);
            obj.TimeEnd = Time(size(Time, 1));
            obj.TotalTime = obj.TimeEnd - obj.TimeStart;
            
            obj.SegmentLength = obj.TotalTime / NumberOfSegments;
            
            %preallocation
            obj.ApproximatedFunctions = cell(obj.NumberOfFunctions, NumberOfSegments);
                
            for j = 1:NumberOfSegments
                %find the time interval for the current segment
                segment_lower_boundary = (j - 1 - obj.Overshoot) * obj.SegmentLength;
                segment_upper_boundary = (j + obj.Overshoot) * obj.SegmentLength;
                
                %find logical index for the current segment in the Time and Data arrays
                logical_index = ((Time > segment_lower_boundary) & (Time < segment_upper_boundary));
                
                %This is an array that has the time points for this segment
                segment_time = Time(logical_index);
                
                for i = 1:obj.NumberOfFunctions
                    %This is an array that has the data points for this
                    %segment
                    segment_values = Data(logical_index, i);
                    
                    %approximation
                    [p, S, mu] = polyfit(segment_time, segment_values, PolynomialDegree);
                    
                    obj.ApproximatedFunctions{i, j}.p = p;
                    obj.ApproximatedFunctions{i, j}.S = S;
                    obj.ApproximatedFunctions{i, j}.mu = mu;
                end
            end
            
            %Saving the provided data
            if obj.SaveProvidedData
                obj.ProvidedData.Time = Time;
                obj.ProvidedData.Data = Data;
            end 
        end
        
        %This function evaluates approximation at the point t
        function value = Evaluate(obj, t)
            
            %Check if t is between obj.TimeStart and obj.TimeEnd
            if ((obj.TimeEnd + obj.AllowedOvershootSize - t) < 0) || ((t - obj.TimeStart + obj.AllowedOvershootSize) < 0)
                warning(['Evaluation outside approximation boundaries, t = ', num2str(t), ...
                    ' and boundaries are [', num2str(obj.TimeStart), ', ', num2str(obj.TimeEnd), ']']);
                    
            end
            
            %find what segment t belongs to
            segment_index = floor((t - obj.TimeStart) / obj.SegmentLength) + 1;
            if segment_index > obj.NumberOfSegments
                segment_index = obj.NumberOfSegments;
            end
            if segment_index < 1
                segment_index = 1;
            end
            
            value = zeros(obj.NumberOfFunctions, 1);
            for i = 1:obj.NumberOfFunctions
                value(i) = polyval(obj.ApproximatedFunctions{i, segment_index}.p, t, [], obj.ApproximatedFunctions{i, segment_index}.mu);
            end
        end
        
        %wrapper for Evaluate method, extending it beyond its boundaries by
        %assuming boundary values for out-of-boundary points
        function value = EvaluateExtended(obj, t)
            if (obj.TimeEnd - t) < 0
                t = obj.TimeEnd;
            end
            if (t - obj.TimeStart) < 0
                t = obj.TimeStart;
            end
            
            value = obj.Evaluate(t);
        end
        
        %This function plots the approximation results
        %1. CustomInterval - a vector with two elements, start and end time 
        %for which to plot the approximation. Doesn't affect the provided
        %data plotting.
        %2. ShowProvidedData - if true will plot the data provided for the
        %approximation. The data will be ploted on top of the approximation
        %with width 1.
        %3. ShowEveryPlotOnItsOwnSubplot - if true, the figure will be
        %arranged as a column of subplots, each with one function plotted
        %on it.
        %4. CustomTimeStep - will determine the tome step for the plots.
        %Deault value is calculated such that to have 100 points on each
        %plot.
        %5. LineWidth - determines the value for LineWidth property for the
        %graphs. Affects the approximating functions plots.
        %6. Matrix - A matrix that approximation vector will be multiplied
        %by. Passing nonsquare matrix is a convinient way to reduce the
        %number of plots. Passing diagonal matrix is a convinient way to
        %scale the plots.
        %
        %All inputs can be omitted.
        function fh = PlotApproximation(obj, CustomInterval, ShowProvidedData, ...
                ShowEveryPlotOnItsOwnSubplot, CustomTimeStep, LineWidth, Matrix, CustomPlotGrid)
            %checking inputs, fill in the omitted ones
            if nargin < 8
                CustomPlotGrid = [];
            end
            if nargin < 7
                Matrix = eye(obj.NumberOfFunctions);
            end
            if nargin < 6
                LineWidth = 3;
            end
            if nargin < 5
                CustomTimeStep = [];
            end
            if nargin < 4
                ShowEveryPlotOnItsOwnSubplot = false;
            end
            if nargin < 3
                ShowProvidedData = false;
            end
            if nargin < 2
                CustomInterval = [];
            end
            
            %determine the interval to be plotted
            if ~isempty(CustomInterval)
                TimeInterval = CustomInterval;
            else
                TimeInterval = [obj.TimeStart; obj.TimeEnd];
            end
            
            %determine time step
            if ~isempty(CustomTimeStep)
                dt = CustomTimeStep;
            else
                dt = (TimeInterval(2) - TimeInterval(1))/100;
            end
               
            NumberOfPlots = size(Matrix, 1);
            
            if ~isempty(CustomPlotGrid)
                PlotGrid = CustomPlotGrid;
            else
                PlotGrid = [NumberOfPlots, 1];
            end
            
            Count = floor((TimeInterval(2) - TimeInterval(1)) / dt) + 1; 
            TimeArray = zeros(Count, 1);
            DataArray = zeros(Count, size(Matrix, 1));
            
            for i = 1:Count
                t = (i - 1)*dt + TimeInterval(1);
                value = obj.Evaluate(t);
                
                TimeArray(i) = t;
                DataArray(i, :) = Matrix*value;
            end
            
            %Plotting the graphs
            BP = BetterPlotsClass;
            BP.LineWidth = LineWidth;
            
            fh = BP.CreateBigFigure;
            if ShowEveryPlotOnItsOwnSubplot
                BP.ToAddArrows = false;
                Error = obj.CalculateMeanError;
                
                for j = 1:NumberOfPlots
                    subplot(PlotGrid(1), PlotGrid(2), j);
                    BP.Plot(TimeArray, DataArray(:, j), 't, s', '', fh);
                    if ShowProvidedData
                        hold on;
                        plot(obj.ProvidedData.Time, obj.ProvidedData.Data(:, j), 'LineWidth', 1);
                    end
                    title(['Polynomial approximation, function #', num2str(j), ' mean error - ', ...
                        num2str(Error(j))]);
                end
            else
                BP.Plot(TimeArray, DataArray, 't, s', '', fh);
                if ShowProvidedData
                    hold on;
                    plot(obj.ProvidedData.Time, obj.ProvidedData.Data, 'LineWidth', 1);
                end
                title('Polynomial approximation');
            end
        end
        
        %This function calculates mean error in the following way:
        %e = sqrt( summ(se_i) / N), where N is the number of data points
        %provided and se_i is the square error:
        %se_i = (y_i - f(t_i))^2, where t_i are moments of time for
        %which the data was provided, y_i is the provided data, f is the
        %function approximation.
        function Error = CalculateMeanError(obj)
            
            Count = size(obj.ProvidedData.Data, 1);
            summ = 0;
            
            for i = 1:Count
               t = obj.ProvidedData.Time(i);
               y = obj.ProvidedData.Data(i, :)';
               f = obj.Evaluate(t);
               
               summ = summ + (y - f).^2;
            end 
            summ = summ / Count;
            
            Error = sqrt(summ);
        end
        
        
    end
end