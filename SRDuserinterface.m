%This class provides user interface for general SRD functionality. Its
%purpose is to hide unnesessary details, make end-user code cleaner.
classdef SRDuserinterface < SRDinterface

    methods
        function obj = SRDuserinterface()
        end
       
        function Ground = CreateGroundLink(~)
            RelativeBase = [0; 0; 0];
            RelativeFollower = [0; 0; 0];
            RelativeCoM = [0; 0; 0];
            Mass = 0;
            Inertia = eye(3);
            Name = 'Ground';
            save('datafile_ground', 'RelativeBase', 'RelativeFollower', 'RelativeCoM', 'Mass', 'Inertia', 'Name');
            rehash;
            
            Ground = SRDLinkWithJoint('none', 0, 'datafile_ground', [], []);
            Ground.RelativeOrientation = eye(3);
        end
        
        % This function creates an SRD object that will be used for
        % simulation.
        % LinkArray - an array of links, objects of the class
        % SRDLinkWithJoint. User needs to assign them their generalized
        % coordinates before passing them here (using
        % .SetUsedGenCoordinates method of SRDLinkWithJoint class)
        %
        % InitialPosition - initial value of the vector of the generalized
        % coordinates
        %
        % AxisLimits - defines the limits for the axes
        % ViewAngle - defines the camera view angle
        function SimulationEngine = CreateRobotStructure(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'CreateRobotStructure';
            Parser.addOptional('LinkArray', []);
            Parser.addOptional('InitialPosition', []);     
            Parser.addOptional('AxisLimits', [-1; 1; -1; 1; -1; 1]);  
            Parser.addOptional('ViewAngle', [-37.5, 30]); 
            Parser.addOptional('ToDrawFrames', false);
            Parser.addOptional('ToDrawMeshes', false);
            Parser.addOptional('ToAnimateRobotAfterGeneration', false);
            Parser.parse(varargin{:});
                        
            InitialPosition = Parser.Results.InitialPosition;
              
            %We create SimulationEngine, it will be used for simulation and
            %other things
            SimulationEngine = SRDSimulationEngine(Parser.Results.LinkArray);
            
            %Pass InitialPosition to SimulationEngine and also save it
            %Update the mechanism, so it will take initial configuration
            SimulationEngine.IC.q = InitialPosition;
            SimulationEngine.Update(InitialPosition);
            
            obj.SaveSimulationEngine(SimulationEngine);
            
            rehash;
            
%             if Parser.Results.ToAnimateRobotAfterGeneration
%                 %Display the initial position of the mechanism
%                 Animation = SRDAnimation();
%                 Animation.ToDrawFrames = Parser.Results.ToDrawFrames;
%                 if Parser.Results.ToDrawMeshes
%                     Animation.DrawType = 'STL';
%                 end
%                 Animation.DrawIC();
%                 xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
%             end
        end
        
        
        function visuals_config = SetupVisuals(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SetupAnimation';
            Parser.addOptional('AxisLimits', [-1; 1; -1; 1; -1; 1]);  
            Parser.addOptional('ViewAngle', [-37.5, 30]); 
            Parser.addOptional('ToDrawFrames', false);
            Parser.addOptional('ToDrawMeshes', false);
            
            Parser.addOptional('Animation_ToUseGrid', true);
            Parser.addOptional('Animation_ToUseGridMinor', true);
            
            Parser.addOptional('DrawRobot_Default_RobotColor', [0.6 0.3 0]);
            Parser.addOptional('DrawRobot_Default_EdgeAlpha', 0.3);
            Parser.addOptional('DrawRobot_Default_FaceAlpha', 1);
            Parser.addOptional('DrawRobot_Default_LineWidth', 0.5);
            
            Parser.addOptional('DrawRobot_STL_FaceColor', [0.8 0.8 1.0]);
            Parser.addOptional('DrawRobot_STL_EdgeColor', 'none');
            Parser.addOptional('DrawRobot_STL_FaceLighting', 'gouraud');
            Parser.addOptional('DrawRobot_STL_AmbientStrength', 0.15);
            Parser.addOptional('DrawRobot_STL_camlight', 'headlight');
            Parser.addOptional('DrawRobot_STL_material', 'dull');
            
            Parser.addOptional('DrawRobot_Frame_Scale', 0.2);
            Parser.addOptional('DrawRobot_Frame_LineWidth', 1);
        
            Parser.addOptional('FileName_visuals_config', []);
            Parser.parse(varargin{:});
            
            if ~isempty(Parser.Results.FileName_visuals_config)
                obj.FileName_visuals_config = Parser.Results.FileName_visuals_config;
            end
            
            visuals_config.AxisLimits = Parser.Results.AxisLimits;
            visuals_config.ViewAngle = Parser.Results.ViewAngle;
            visuals_config.AxisLimits = Parser.Results.AxisLimits;
            
            visuals_config.ToDrawFrames = Parser.Results.ToDrawFrames;
            visuals_config.ToDrawMeshes = Parser.Results.ToDrawMeshes;
            
            visuals_config.Animation_ToUseGrid = Parser.Results.Animation_ToUseGrid;
            visuals_config.Animation_ToUseGridMinor = Parser.Results.Animation_ToUseGridMinor;
            
            visuals_config.DrawRobot_Default_RobotColor = Parser.Results.DrawRobot_Default_RobotColor;
            visuals_config.DrawRobot_Default_EdgeAlpha = Parser.Results.DrawRobot_Default_EdgeAlpha;
            visuals_config.DrawRobot_Default_FaceAlpha = Parser.Results.DrawRobot_Default_FaceAlpha;
            visuals_config.DrawRobot_Default_LineWidth = Parser.Results.DrawRobot_Default_LineWidth;
            
            visuals_config.DrawRobot_STL_FaceColor = Parser.Results.DrawRobot_STL_FaceColor;
            visuals_config.DrawRobot_STL_EdgeColor = Parser.Results.DrawRobot_STL_EdgeColor;
            visuals_config.DrawRobot_STL_FaceLighting = Parser.Results.DrawRobot_STL_FaceLighting;
            visuals_config.DrawRobot_STL_AmbientStrength = Parser.Results.DrawRobot_STL_AmbientStrength;
            visuals_config.DrawRobot_STL_camlight = Parser.Results.DrawRobot_STL_camlight;
            visuals_config.DrawRobot_STL_material = Parser.Results.DrawRobot_STL_material;
            
            visuals_config.DrawRobot_Frame_Scale = Parser.Results.DrawRobot_Frame_Scale;
            visuals_config.DrawRobot_Frame_LineWidth = Parser.Results.DrawRobot_Frame_LineWidth;
            
            save(obj.FileName_visuals_config, 'visuals_config');
        end
        
        
        %This function needs .CreateRobotStructure to have been called.
        %
        %dissipation_coefficients - pass vector to assign each
        %individually, a scalar to assign them uniformly, or nothing to
        %have them all set as 1
        function SymbolicEngine = DeriveEquationsForSimulation(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'DeriveEquationsForSimulation';
            Parser.addOptional('LinkArray', []);
            Parser.addOptional('UseCasadi', false);
            Parser.addOptional('ToLinearize', false);     
            Parser.addOptional('ToSimplify', true);  
            Parser.addOptional('dissipation_coefficients', []); 
            Parser.addOptional('ToRecreateSymbolicEngine', true);
            %if true - method will create new SymbolicEngine; 
            %if false, the method will attempt to load usiting engine;
            
            Parser.addOptional('ToSaveSymbolicEngine', true);  
            
            Parser.addOptional('NumberOfWorkers', 8); 
            %Defines the number of MATLAB workers that will be used in 
            %parallel computing
            
            Parser.addOptional('ToUseParallelizedSimplification', false); 
            %If true, the programm will simplify the elements of symbolic 
            %vector expressions in parallel and it will report the progress
            
            Parser.addOptional('ToOptimizeFunctions', true); 
            %This property will be used to set the same property of the
            %SymbolicEngine
            
            
            Parser.parse(varargin{:});
            
            %load created previously LinkArray 
            LinkArray = Parser.Results.LinkArray;
            
            %Create SymbolicEngine that will be used for deriving equations
            if Parser.Results.ToRecreateSymbolicEngine
                SymbolicEngine = SRDSymbolicEngine(LinkArray, Parser.Results.UseCasadi);
            else
                SymbolicEngine = obj.GetSymbolicEngine(true);
                if isempty(SymbolicEngine)
                    SymbolicEngine = SRDSymbolicEngine(LinkArray);
                end
            end
            
            %if UseParallelizedSimplification or NumberOfWorkers properties
            %are defined, pass them to the SymbolicEngine
            SymbolicEngine.UseParallelizedSimplification = Parser.Results.ToUseParallelizedSimplification;
            SymbolicEngine.NumberOfWorkers = Parser.Results.NumberOfWorkers;
            SymbolicEngine.ToOptimizeFunctions = Parser.Results.ToOptimizeFunctions;
            
            %Assignment of the dissipation cefficients
            if isempty(Parser.Results.dissipation_coefficients)
                SymbolicEngine.dissipation_coefficients = ones(SymbolicEngine.dof, 1);
            else
                if length(Parser.Results.dissipation_coefficients) == 1
                    SymbolicEngine.dissipation_coefficients = Parser.Results.dissipation_coefficients * ones(SymbolicEngine.dof, 1);
                else
                    SymbolicEngine.dissipation_coefficients = Parser.Results.dissipation_coefficients;
                end
            end
            
            %Create dynamics eq. 
            SymbolicEngine.BuildDynamicsEquations(Parser.Results.ToSimplify, false);
            %Generate nesessary function from those equations
%             if Parser.Results.UseCasadi
%                 SymbolicEngine.GenerateForwardDynamicsFunctions_Casadi();
%             else
%                 SymbolicEngine.GenerateForwardDynamicsFunctions();
%             end
%             
%             %If requested generate linearized version of dynamics eq
%             if Parser.Results.ToLinearize
%                 SymbolicEngine.DoLinearization(Parser.Results.ToSimplify);
%             end
            
            if Parser.Results.ToSaveSymbolicEngine
                obj.SaveSymbolicEngine(SymbolicEngine);
            end
            rehash;            
        end
        
        %Task - symbolic expression of the inverse kinematics task
        function InverseKinematicsEngine = SetupSymbolicInverseKinematics(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDuserinterface.SetupSymbolicInverseKinematics';
            Parser.addOptional('Task', []);
            Parser.addOptional('SymbolicEngine', []);
            Parser.addOptional('ToSaveInverseKinematicsEngine', true);
            Parser.parse(varargin{:});
            
            if isempty(Parser.Results.SymbolicEngine)
                SymbolicEngine = obj.GetSymbolicEngine();
            else
                SymbolicEngine = Parser.Results.SymbolicEngine;
            end
            
            %create InverseKinematicsEngine
            InverseKinematicsEngine = SRDInverseKinematics;
            %derive nessesary symbolic functions
            InverseKinematicsEngine.IKsetup(SymbolicEngine, Parser.Results.Task);
            
            if Parser.Results.ToSaveInverseKinematicsEngine
                obj.SaveInverseKinematicsEngine(InverseKinematicsEngine);
            end
            rehash;
        end
        
        %This function solves inverse kinematics problem numerically,
        %and approximates the solution. SetupSymbolicInverseKinematics
        %needs to have been called.
        %
        %DesiredTask - fuction handle, the output needs to match dimentions
        %of Task, the input of .SetupSymbolicInverseKinematics() called
        %earlier.
        function SetupNumericInverseKinematics(obj, varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDuserinterface.SetupNumericInverseKinematics';
            Parser.addOptional('DesiredTask', []);
            Parser.addOptional('TimeRange', []);
            Parser.addOptional('PolynomialDegree', 5);
            Parser.addOptional('NumberOfSegments', []);
            Parser.addOptional('SolverType', 'lsqnonlin');
            Parser.addOptional('LookupTableTimeStep', 0.001);
            Parser.addOptional('TimeStep', 0.01);
            Parser.addOptional('problem', []);
            Parser.addOptional('ToPlot', true);
            Parser.addOptional('Verbose', true);
            Parser.parse(varargin{:});
            
            %load created previously InverseKinematicsEngine
            FileName = 'datafile_InverseKinematicsEngine.mat';
            if exist(FileName, 'file') == 2
                temp = load(FileName);
                InverseKinematicsEngine = temp.InverseKinematicsEngine;
            else
                error(['File ', FileName, ' does not exist. Create the inverse kinematics engine before using it']);
            end
            
            %%%%%%%%%%%%%%%%%
            %input parameters processing
            if isempty(Parser.Results.DesiredTask)
                error('provide DesiredTask')
            end
            
            TimeRange = Parser.Results.TimeRange;
            if ~isempty(TimeRange)
                %give time range for inverse kinematics problem
                InverseKinematicsEngine.TimeStart = TimeRange(1);
                InverseKinematicsEngine.TimeEnd = TimeRange(2);
            end
            
            NumberOfSegments = Parser.Results.NumberOfSegments;
            if isempty(NumberOfSegments)
                NumberOfSegments = floor((TimeRange(2) - TimeRange(1)) / ...
                    (2 * Parser.Results.PolynomialDegree * InverseKinematicsEngine.dt));
            end
            %%%%%%%%%%%%%%%%%
            
            
            %load InitialPosition and pass it to InverseKinematicsEngine
            InverseKinematicsEngine.InitialGuess = obj.GetInitialPosition();
            
            %set IK solver type
            if ~isempty(Parser.Results.SolverType)
                InverseKinematicsEngine.SolverType = Parser.Results.SolverType;
            end
            
            %set IK time step
            InverseKinematicsEngine.dt = Parser.Results.TimeStep;
            
            %Solve the inverse kinematics problem
            if Parser.Results.Verbose; disp('Called .SolveAndApproximate procedure'); end
            InverseKinematicsEngine.SolveAndApproximate(Parser.Results.DesiredTask, ...
                Parser.Results.PolynomialDegree, NumberOfSegments, Parser.Results.problem);
            
            if Parser.Results.LookupTableTimeStep ~= 0
                InverseKinematicsEngine.LookupTable_dt = Parser.Results.LookupTableTimeStep;
                
                if Parser.Results.Verbose; disp('Called .GenerateLookupTable procedure'); end
                InverseKinematicsEngine.GenerateLookupTable;
            end
            
            %save the solution
            save('datafile_InverseKinematicsEngine_processed', 'InverseKinematicsEngine');
            
            %plot the solution
            if Parser.Results.ToPlot
                InverseKinematicsEngine.PlotGraphsFromEvaluatePolynomialApproximation;
            end
        end
            

    end
end