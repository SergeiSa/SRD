classdef IPStepSequencer < handle
    properties
        SpatialTube;
        %object of SpatialTubeClass class
        
        GeneralStepSequencer;
        %object of WRStepSequencer class
        
        ProjectedStepSequence;
        %cell array containing the Projected Step Sequence
        
        InitialStepSequence;
        %cell array containing the Initial Step Sequence
    end
    methods
        
        function obj = IPStepSequencer(SpatialTube)
            obj.SpatialTube = SpatialTube;
        end
        
        %fixes the pregenerated step to match the height map of the
        %unwrapped tube
        function Output = StepFix(obj, Input)
            Output = Input;
            
            P = Output.StepTo;
            P(3) = obj.SpatialTube.HeightMap(P(1), P(2));
            Output.StepTo = P;
            
            Output.NewState(Output.LegNumber, :) = Output.StepTo';
        end
        
        %fixes the pregenerated step to match the height map of the
        %unwrapped tube
        function GenerateProjectedStepSequence(obj, NumberOfSteps)
            obj.ProjectedStepSequence = cell(NumberOfSteps, 1);
            obj.InitialStepSequence = cell(NumberOfSteps, 1);
            
            for i = 1:NumberOfSteps
                Output = obj.GeneralStepSequencer.MakeNextStep();
                
                StepPosition = Output.StepTo;
                StepPosition3D = obj.SpatialTube.Map_onto_3D_tube(StepPosition);
                
                obj.ProjectedStepSequence{i}.Position = StepPosition3D;
                obj.ProjectedStepSequence{i}.LegNumber = Output.LegNumber;   
                
                obj.InitialStepSequence{i}.Position = StepPosition;
                obj.InitialStepSequence{i}.LegNumber = Output.LegNumber; 
            end
            
        end
        
        %Draws the generated ProjectedStepSequence
        function DrawProjectedStepSequence(obj, StepSequence)
            if nargin < 2
                StepSequence = obj.ProjectedStepSequence;
            end
            
            NumberOfSteps = size(StepSequence, 1);
            for i = 1:NumberOfSteps
                obj.GeneralStepSequencer.DrawOneStep(StepSequence{i}.Position, ...
                    StepSequence{i}.LegNumber);
            end
        end
        
        %Draws the generated ProjectedStepSequence
        function DrawStepSequence(obj, StepSequence)
            if nargin < 2
                StepSequence = obj.InitialStepSequence;
            end
            
            NumberOfSteps = size(StepSequence, 1);
            for i = 1:NumberOfSteps
                obj.GeneralStepSequencer.DrawOneStep(StepSequence{i}.Position, ...
                    StepSequence{i}.LegNumber);
            end
        end
    end
end