%This class provides user interface for general SRD functionality. Its
%purpose is to hide unnesessary details, make end-user code cleaner.
%last update 12.04.18
classdef SRDuserinterface < handle
    properties
        
        RecreateSymbolicEngine = true;
        %if true - the DeriveEquationsForSimulation method will create
        %SymbolicEngine each time it is used; if false, the methid will
        %attempt to load the engine;
        
        UseParallelizedSimplification = [];
        %If true, the programm will simplify the elements of symbolic 
        %vector expressions in parallel and it will report the progress
        
        NumberOfWorkers = [];
        %Defines the number of MATLAB workers that will be used in parallel
        %computing       
        
        ToOptimizeFunctions = true;
        %This property will be used to set the same property of the
        %SymbolicEngine
        
        AnimateRobot = true;
        %determines if the robot will be animated when appropriate
        
    end
    methods
        function obj = SRDuserinterface(UseParallelizedSimplification, NumberOfWorkers)
            if nargin < 1
                UseParallelizedSimplification = [];
            end
            if ~isempty(UseParallelizedSimplification)
                obj.UseParallelizedSimplification = UseParallelizedSimplification;
            end
            
            if nargin < 2
                NumberOfWorkers = [];
            end
            if ~isempty(NumberOfWorkers)
                obj.NumberOfWorkers = NumberOfWorkers;
            end
                
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
        function CreateRobotStructure(obj, LinkArray, InitialPosition, AxisLimits, ViewAngle)
            
            if nargin < 4
                AxisLimits = [];
            end
            if isempty(AxisLimits)
                AxisLimits = [-1; 1; -1; 1; -1; 1];
            end
            
            if nargin < 5
                ViewAngle = [];
            end
            if isempty(ViewAngle)
                ViewAngle = [-37.5, 30];
            end
            
            %Save LinkArray
            save('datafile_LinkArray', 'LinkArray');
            %Save AxisLimits
            save('datafile_AxisLimits', 'AxisLimits');
            %Save ViewAngle
            save('datafile_ViewAngle', 'ViewAngle');
            
            %We create SimulationEngine, it will be used for simulation and
            %other things
            SimulationEngine = SRDSimulationEngine(LinkArray);
            
            %Pass InitialPosition to SimulationEngine and also save it
            SimulationEngine.IC.q = InitialPosition;
            save('datafile_InitialPosition', 'InitialPosition');
            
            %Update the mechanism, so it will take initial configuration
            SimulationEngine.Update(InitialPosition);
            save('datafile_SimulationEngine', 'SimulationEngine');
            
            rehash;
            
            if obj.AnimateRobot
                %Display the initial position of the mechanism
                Animation = SRDAnimation();
                Animation.DrawIC();
                xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
            end
        end
        
        %This function needs .CreateRobotStructure to have been called.
        function DeriveEquationsForSimulation(obj, ToLinearize, ToSimplify, dissipation_coefficients)
            if nargin < 2
                ToLinearize = false;
            end
            if nargin < 3
                ToSimplify = false;
            end
            
            %load created previously LinkArray 
            LinkArray = obj.GetLinkArray;
            
            %Create SymbolicEngine that will be used for deriving equations
            if obj.RecreateSymbolicEngine
                SymbolicEngine = SRDSymbolicEngine(LinkArray);
            else
                SymbolicEngine = obj.GetSymbolicEngine(true);
                if isempty(SymbolicEngine)
                    SymbolicEngine = SRDSymbolicEngine(LinkArray);
                end
            end
            
            %if UseParallelizedSimplification or NumberOfWorkers properties
            %are defined, pass them to the SymbolicEngine
            if ~isempty(obj.UseParallelizedSimplification)
                SymbolicEngine.UseParallelizedSimplification = obj.UseParallelizedSimplification;
            end
            if ~isempty(obj.NumberOfWorkers)
                SymbolicEngine.NumberOfWorkers = obj.NumberOfWorkers;
            end
            
            %This needs to be worked on
            if nargin < 4
                dissipation_coefficients = ones(SymbolicEngine.dof, 1);
            end
            SymbolicEngine.dissipation_coefficients = dissipation_coefficients;
            SymbolicEngine.ToOptimizeFunctions = obj.ToOptimizeFunctions;
            
            %Create dynamics eq. 
            SymbolicEngine.BuildDynamicsEquations(ToSimplify, false);
            %Generate nesessary function from those equations
            SymbolicEngine.GenerateForwardDynamicsFunctions();
            
            %If requested generate linearized version of dynamics eq
            if ToLinearize
                SymbolicEngine.DoLinearization(ToSimplify);
            end
            %Save SymbolicEngine
            save('datafile_SymbolicEngine', 'SymbolicEngine');
            
            rehash;            
        end
        
        %Task - symbolic expression of the inverse kinematics task
        function SetupSymbolicInverseKinematics(obj, Task)
            SimulationEngine = obj.GetSimulationEngine();
            n = SimulationEngine.dof;
            q = sym('q', [n, 1]); assume(q, 'real');
            
            %create InverseKinematicsEngine
            InverseKinematicsEngine = SRDInverseKinematics;
            %derive nessesary symbolic functions
            InverseKinematicsEngine.IKsetup(q, Task);
            % save InverseKinematicsEngine
            save('datafile_InverseKinematicsEngine', 'InverseKinematicsEngine');
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
            
        %loads LinkArray from a file
        function LinkArray = GetLinkArray(~)
            FileName = 'datafile_LinkArray.mat';
            if exist(FileName, 'file') == 2
                %load created previously LinkArray 
                temp = load(FileName);
                LinkArray = temp.LinkArray;
            else
                warning(['File ', FileName, ' does not exist. Create the LinkArray before using it']);
                LinkArray = [];
            end
        end       
        
        %loads SymbolicEngine from a file
        function SymbolicEngine = GetSymbolicEngine(~, SilentMode)
            
            if nargin < 2
                SilentMode = false;
            end
            
            FileName = 'datafile_SymbolicEngine.mat';
            if exist(FileName, 'file') == 2
                %load created previously SymbolicEngine
                temp = load(FileName);
                SymbolicEngine = temp.SymbolicEngine;
                SymbolicEngine.SetAssumptions();
            else
                if ~SilentMode
                    warning(['File ', FileName, ' does not exist. Set up the symbolic engine before using it']);
                end
                SymbolicEngine = [];
            end
        end
        
        %creates new SymbolicEngine
        %set ToUpdateGeometry = true if need to use .GeometryArray field of
        %the SymbolicEngine
        function SymbolicEngine = GetNewSymbolicEngine(obj, ToUpdateGeometry)
            
            if nargin < 2
                ToUpdateGeometry = false;
            end
            
            %load created previously LinkArray 
            LinkArray = obj.GetLinkArray;
            SymbolicEngine = SRDSymbolicEngine(LinkArray);
            
            %if UseParallelizedSimplification or NumberOfWorkers properties
            %are defined, pass them to the SymbolicEngine
            if ~isempty(obj.UseParallelizedSimplification)
                SymbolicEngine.UseParallelizedSimplification = obj.UseParallelizedSimplification;
            end
            if ~isempty(obj.NumberOfWorkers)
                SymbolicEngine.NumberOfWorkers = obj.NumberOfWorkers;
            end            
            
            %if requested - udate .GeometryArray field in the SymbolicEngine
            if ToUpdateGeometry
                SymbolicEngine.UpdateGeometryArray(true);
            end
        end
        
        %loads SimulationEngine from a file
        %PutIntoInitialPosition - if true, the mechanism will be put into
        %its original position, as defined in the file
        %datafile_InitialPosition. It is not nesessary, as the mechanism
        %should be in that position already if the default procedure for
        %setting it up was used
        function SimulationEngine = GetSimulationEngine(obj, PutIntoInitialPosition)
            
            if nargin < 2
                PutIntoInitialPosition = false;
            end
            
            FileName = 'datafile_SimulationEngine.mat';
            if exist(FileName, 'file') == 2
                %load created previously SimulationEngine
                temp = load(FileName);
                SimulationEngine = temp.SimulationEngine;
                
                %if requested put the mechanism into its initial position.
                if PutIntoInitialPosition
                    InitialPosition = obj.GetInitialPosition();
                    SimulationEngine.IC.q = InitialPosition;
                    SimulationEngine.Update(InitialPosition);
                end
                
                %if the control dof are not calculated for SimulationEngine
                %- attempt to load the info from the file
                TryToLoad_Control_dof = false;
                if ~isfield(SimulationEngine, 'Control_dof')
                    TryToLoad_Control_dof = true;
                else
                    if isempty(SimulationEngine.Control_dof)
                        TryToLoad_Control_dof = true;
                    end
                end
                if TryToLoad_Control_dof
                    FileName_Control_dof = 'datafile_settings_Control_dof.mat';
                    if exist(FileName_Control_dof, 'file') == 2
                        temp = load(FileName_Control_dof);
                        SimulationEngine.Control_dof = temp.Control_dof;
                    end
                end
                
                SimulationEngine.Initialization;
            else
                warning(['File ', FileName, ' does not exist. Set up the simulation engine before using it']);
                SimulationEngine = [];
            end
        end
        
        %loads InitialPosition from a file
        function InitialPosition = GetInitialPosition(~)
            FileName = 'datafile_InitialPosition.mat';
            if exist(FileName, 'file') == 2
                %load created previously InitialPosition
                temp = load(FileName);
                InitialPosition = temp.InitialPosition;
            else
                warning(['File ', FileName, ' does not exist. Define the initial position before using it']);
                InitialPosition = [];
            end
        end
        
        %loads InverseKinematicsEngine from a file
        function InverseKinematicsEngine = GetInverseKinematicsEngine(~)
            FileName = 'datafile_InverseKinematicsEngine_processed.mat';
            if exist(FileName, 'file') == 2
                %load created previously InverseKinematicsEngine with the
                %solved IK problem
                temp = load(FileName);
                InverseKinematicsEngine = temp.InverseKinematicsEngine;
            else
                warning(['File ', FileName, ...
                    ' does not exist. Set up and process the inverse kinematics engine before using it']);
                InverseKinematicsEngine = [];
            end
        end
        
        
        %loads ExternalForcesEngine from a file
        function ExternalForcesEngine = GetExternalForcesEngine(~)
            FileName = 'datafile_ExternalForcesEngine.mat';
            if exist(FileName, 'file') == 2
                %load created previously InverseKinematicsEngine with the
                %solved IK problem
                temp = load(FileName);
                ExternalForcesEngine = temp.ExternalForcesEngine;
            else
                warning(['File ', FileName, ...
                    ' does not exist. Set up and process the External Forces Engine before using it']);
                ExternalForcesEngine = [];
            end
        end
        
        %loads AxisLimits from a file
        function AxisLimits = GetAxisLimits(~)
            FileName = 'datafile_AxisLimits.mat';
            if exist(FileName, 'file') == 2
                %load created saved previously AxisLimits
                temp = load(FileName);
                AxisLimits = temp.AxisLimits;
            else
                warning(['File ', FileName, ' does not exist']);
                AxisLimits = [];
            end
        end
        
        %loads AxisLimits from a file
        function ViewAngle = GetViewAngle(~)
            FileName = 'datafile_ViewAngle.mat';
            if exist(FileName, 'file') == 2
                %load created saved previously ViewAngle
                temp = load(FileName);
                ViewAngle = temp.ViewAngle;
            else
                warning(['File ', FileName, ' does not exist']);
                ViewAngle = [];
            end
        end
        
        %saves SymbolicEngine
        function SaveSymbolicEngine(~, SymbolicEngine)
            FileName = 'datafile_SymbolicEngine.mat';
            save(FileName, 'SymbolicEngine');
        end
        
        %saves SimulationEngine
        function SaveSimulationEngine(~, SimulationEngine)
            FileName = 'datafile_SimulationEngine.mat';
            save(FileName, 'SimulationEngine');
        end
        
        %saves InverseKinematicsEngine
        function SaveInverseKinematicsEngine(~, InverseKinematicsEngine)
            FileName = 'datafile_InverseKinematicsEngine.mat';
            save(FileName, 'InverseKinematicsEngine');
        end
        
        %saves SimulationEngine
        function SaveInitialPosition(~, InitialPosition)
            FileName = 'datafile_InitialPosition.mat';
            save(FileName, 'InitialPosition');
        end
        
        %saves SimulationEngine
        function SaveExternalForcesEngine(~, ExternalForcesEngine)
            FileName = 'datafile_ExternalForcesEngine.mat';
            save(FileName, 'ExternalForcesEngine');
        end
        
    end
end