%This class provides functionality for plotting graphs
%last update 2.10.16
classdef BetterPlotsClass < handle
    properties
        LineWidth = 3;
        %line width for the graphs
        
        FigureColor = 'white';
        %color of the figure (space outside the plots)
        
        %grid properties:
        XGrid = 'on';
        YGrid = 'on';
        ZGrid = 'on';
        XMinorGrid = 'on';
        YMinorGrid = 'on';
        ZMinorGrid = 'on';
        %If 'on', the grid will be displayed
        
        GridAlpha = 0.5;
        MinorGridAlpha = 0.3;
        %determines grid transparency;
        
        GridLineWidth = 0.5;
        %determines grid line width;
        
        %text properties
        AxisFontName = 'Times New Roman';
        AxisFontSize = 14;
        
        %if true, the y axis label will not be rotated (as it happens in
        %default Matlab settings)
        Prevent_ylabel_rotation = false;
        
        %Arrows
        ToAddArrows = true;
        % if true, annotation arrows will be added to the axes where
        % possible
        ArrowLineWidth = 1;
        ArrowLengthX = 0.815;
        ArrowLengthY = 0.872;
        
        %graphs' annotations
        ToMakeAnnotationLines = false;
        %if true annotation lines will be added
        
        AnnotationLineWidth = 0.75;
        AnnotationColor = 'black'
        
        AnnotationLengthRatioX = 0.15;
        AnnotationLengthRatioY = 0.15;
        %Defines the ratio between the annotation length and the
        %corresponding side of the plot
        
        %Labels property
        ToMakeLabeles = true;
        %if true there will be labels for the plots
        
        LabelsInterpreter = 'latex';
        
    end
    methods
        
        function output = Plot(obj, X, Y, label_x, label_y, figure_handle)
            if nargin  < 6
                figure_handle = [];
            end
            if nargin  < 5
                label_y = '';
            end
            if nargin  < 4
                label_x = '';
            end
            
            %if FigureToUse is defined, it will be used for plotting
            if ~isempty(figure_handle)
                figure(figure_handle);
            else
                figure_handle = figure;
            end
                
            plot_handle = plot(X, Y, 'LineWidth', obj.LineWidth); hold on;
            
            axes_handle = obj.FixCurrentFigure;
       
            %if requested make axis labels
            if obj.ToMakeLabeles
                temp = obj.AddLatexLabels(label_x, label_y, [], figure_handle);
                output.xlabel_handle = temp.xlabel_handle;
                output.ylabel_handle = temp.ylabel_handle;
            end
            
            %if requested adds arrows to the plot
            if obj.ToAddArrows
                temp = obj.AddArrows(figure_handle, axes_handle);
                
                output.arrow_x_handle = temp.arrow_x_handle;
                output.arrow_y_handle = temp.arrow_y_handle;
            end
            
            %if requested adds annotations to the graphs
            if obj.ToMakeAnnotationLines
                temp = obj.AddGraphAnnotations(X, Y, figure_handle);
                output.quiver_handle = temp.quiver_handle;
            end
            
            output.figure_handle = figure_handle;
            output.plot_handle = plot_handle;
            output.axes_handle = axes_handle;
            
        end
        
        %sets axis and figure properties, as defined by the class'
        %properties
        function axes_handle = FixCurrentFigure(obj)
            axes_handle = gca;
            axes_handle.XGrid = obj.XGrid;
            axes_handle.YGrid = obj.YGrid;
            axes_handle.ZGrid = obj.ZGrid;
            axes_handle.XMinorGrid = obj.XMinorGrid;
            axes_handle.YMinorGrid = obj.YMinorGrid;
            axes_handle.ZMinorGrid = obj.ZMinorGrid;
            axes_handle.GridAlpha = obj.GridAlpha;
            axes_handle.MinorGridAlpha = obj.MinorGridAlpha;
            axes_handle.LineWidth = obj.GridLineWidth;
            axes_handle.FontName = obj.AxisFontName;
            axes_handle.FontSize = obj.AxisFontSize;
            
            figure_handle = gcf;
            figure_handle.Color = 'white';
        end
        
        %Adds arrows to the axes
        function output = AddArrows(obj, figure_handle, axes_handle)
            if nargin < 2
                figure_handle = gcf;
            end
            if nargin < 3
                axes_handle = gca;
            end
            
            if ~isempty(figure_handle)
                figure(figure_handle);
            else
                figure_handle = figure;
            end
            
            arrow_x_handle = annotation('textarrow');
            arrow_x_handle.Position = [axes_handle.Position(1), axes_handle.Position(2), obj.ArrowLengthX, 0];
            arrow_x_handle.LineWidth = obj.ArrowLineWidth;
            
            arrow_y_handle = annotation('textarrow');
            arrow_y_handle.Position = [axes_handle.Position(1), axes_handle.Position(2), 0, obj.ArrowLengthY];
            arrow_y_handle.LineWidth = obj.ArrowLineWidth;
            
            
            output.figure_handle = figure_handle;
            output.axes_handle = axes_handle;
            output.arrow_x_handle = arrow_x_handle;
            output.arrow_y_handle = arrow_y_handle;
        end
        
        %fixes axes arrows in case they messed up, takes output of Plot()
        %method as the input
        function FixArrows(obj, Plot_output)
            
            P = Plot_output.axes_handle.Position;
            
            %the following code is here to avoid the situation where the
            %head of the arrow goes off screen.
            x_length = obj.ArrowLengthX + P(1);
            y_length = obj.ArrowLengthY + P(2);
            
            if x_length > 1
                actual_ArrowLengthX = obj.ArrowLengthX - (x_length - 1);
            else
                actual_ArrowLengthX = obj.ArrowLengthX;
            end
            
            if y_length > 1
                actual_ArrowLengthY = obj.ArrowLengthY - (y_length - 1);
            else
                actual_ArrowLengthY = obj.ArrowLengthY;
            end
            
            %fising arrow position
            Plot_output.arrow_x_handle.Position = [P(1), P(2), actual_ArrowLengthX, 0];
            Plot_output.arrow_y_handle.Position = [P(1), P(2), 0, actual_ArrowLengthY];
        end
        
        %plot_array - is an array shaped as the subplots, each elements
        %sorresponds to the output of Plot() method
        function FixArrowsAllSubplots(obj, plot_array)
            
            %this is needed, because the positions of the arrows gets 
            %messed up during the drawing, likely because of the labels.
            drawnow;
            N = size(plot_array, 1);
            M = size(plot_array, 2);
            
            for i = 1:N
                for j = 1:M
                    obj.FixArrows(plot_array(i, j));
                end
            end  
        end
        
        
        %Adds annotation lines to the plots
        function output = AddGraphAnnotations(obj, X, Y, figure_handle)
            if nargin >= 4
                figure(figure_handle);
            end
            
            %find number of plots and points in them
            NumberOfPlots = size(Y, 2);
            NumberOfPoints = size(X, 1);
            
            %find the range of the plot in X
            rangeX = max(X) - min(X);
            
            %find the range of the plot in Y
            rangeY = max(max(Y)) - min(min(Y));

            for i = 1:NumberOfPlots
                
                
                %find the point on the graph to which the annotation will
                %be attached
                index = floor(i*NumberOfPoints/(NumberOfPlots*1.2));
                Point = [X(index); Y(index, i)];
                
                %find teh length of the annotation
                displacement = [obj.AnnotationLengthRatioX*rangeX; obj.AnnotationLengthRatioY*rangeY];
                
                %create the annotation
                quiver_handle(i) = quiver(Point(1), Point(2), displacement(1), displacement(2), 0);
                
                quiver_handle(i).ShowArrowHead = 'off';
                quiver_handle(i).LineWidth = obj.AnnotationLineWidth;
                quiver_handle(i).Color = obj.AnnotationColor;
            end
            
            output.quiver_handle = quiver_handle;
        end
        
        %creates labels
        function output = AddLatexLabels(obj, code_x, code_y, code_z, figure_handle)
            if nargin < 5
                figure_handle = gcf;
            end
            if nargin < 4
                code_z = [];
            end
            
            if ~isempty(figure_handle)
                figure(figure_handle);
            else
                figure_handle = figure;
            end
            
            xlabel_handle = xlabel(code_x);
            xlabel_handle.Interpreter = obj.LabelsInterpreter;
            
            ylabel_handle = ylabel(code_y);
            ylabel_handle.Interpreter = obj.LabelsInterpreter;
            
            if obj.Prevent_ylabel_rotation
                ylabel_handle.Rotation = 0;
            end
            
            if ~isempty(code_z)
                zlabel_handle = ylabel(code_z);
                zlabel_handle.Interpreter = obj.LabelsInterpreter;
            else
                zlabel_handle = [];
            end
            
            output.figure_handle = figure_handle;
            output.xlabel_handle = xlabel_handle;
            output.ylabel_handle = ylabel_handle;
            output.zlabel_handle = zlabel_handle;
        end
        
        function figure_handle = CreateBigFigure(~)
            figure_handle = figure('units', 'normalized', 'outerposition', [0 0 1 1]);
        end
        
    end
end

