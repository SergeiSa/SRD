%This class provides user interface for general SRD functionality. Its
%purpose is to hide unnesessary details, make end-user code cleaner.
classdef SRDinterface < handle
    properties
        
        FileName_LinkArray               = 'datafile_LinkArray.mat';
        FileName_SymbolicEngine          = 'datafile_SymbolicEngine.mat';
        FileName_InverseKinematicsEngine = 'datafile_InverseKinematicsEngine.mat';
        FileName_SimulationEngine        = 'datafile_SimulationEngine.mat';
        FileName_InitialPosition         = 'datafile_InitialPosition.mat';
        
        FileName_visuals_config          = 'datafile_VisualSettings.mat';
        
        FileName_AxisLimits              = 'datafile_AxisLimits.mat';
        FileName_ViewAngle               = 'datafile_ViewAngle.mat';
        
    end
    methods
        function obj = SRDinterface()
        end

        %loads LinkArray from a file
        function LinkArray = GetLinkArray(obj)
            if exist(obj.FileName_LinkArray, 'file') == 2
                %load created previously LinkArray 
                temp = load(obj.FileName_LinkArray);
                LinkArray = temp.LinkArray;
            else
                warning(['File ', obj.FileName_LinkArray, ' does not exist. Create the LinkArray before using it']);
                LinkArray = [];
            end
        end       
        
        %loads SymbolicEngine from a file
        function SymbolicEngine = GetSymbolicEngine(obj)
            
            
            if exist(obj.FileName_SymbolicEngine, 'file') == 2
                %load created previously SymbolicEngine
                temp = load(obj.FileName_SymbolicEngine);
                SymbolicEngine = temp.SymbolicEngine;
                if ~SymbolicEngine.Casadi
                    SymbolicEngine.SetAssumptions();
                end
            else
                warning(['File ', obj.FileName_SymbolicEngine, ' does not exist. Set up the symbolic engine before using it']);
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
            LinkArray = obj.GetLinkArray();
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
        function SimulationEngine = GetSimulationEngine(obj, PutInInitialPosition)
            
            if nargin < 2
                PutInInitialPosition = false;
            end
            
            if exist(obj.FileName_SimulationEngine, 'file') == 2
                %load created previously SimulationEngine
                temp = load(obj.FileName_SimulationEngine);
                SimulationEngine = temp.SimulationEngine;
                
                %if requested put the mechanism into its initial position.
                if PutInInitialPosition
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
                warning(['File ', obj.FileName_SimulationEngine, ' does not exist. Set up the simulation engine before using it']);
                SimulationEngine = [];
            end
        end
        
        %loads InitialPosition from a file
        function InitialPosition = GetInitialPosition(obj)
            if exist(obj.FileName_InitialPosition, 'file') == 2
                %load created previously InitialPosition
                temp = load(obj.FileName_InitialPosition);
                InitialPosition = temp.InitialPosition;
            else
                warning(['File ', obj.FileName_InitialPosition, ' does not exist. Define the initial position before using it']);
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
        function AxisLimits = GetAxisLimits(obj)
            if exist(obj.FileName_AxisLimits, 'file') == 2
                %load created saved previously AxisLimits
                temp = load(obj.FileName_AxisLimits);
                AxisLimits = temp.AxisLimits;
            else
                warning(['File ', obj.FileName_AxisLimits, ' does not exist']);
                AxisLimits = [];
            end
        end
        
        %loads AxisLimits from a file
        function ViewAngle = GetViewAngle(obj)
            if exist(obj.FileName_ViewAngle, 'file') == 2
                %load created saved previously ViewAngle
                temp = load(obj.FileName_ViewAngle);
                ViewAngle = temp.ViewAngle;
            else
                warning(['File ', obj.FileName_ViewAngle, ' does not exist']);
                ViewAngle = [];
            end
        end
        
        %saves SymbolicEngine
        function SaveSymbolicEngine(obj, SymbolicEngine)            
            save(obj.FileName_SymbolicEngine, 'SymbolicEngine');
        end
        
        %saves SimulationEngine
        function SaveSimulationEngine(obj, SimulationEngine)
            save(obj.FileName_SimulationEngine, 'SimulationEngine');
        end
        
        %saves InverseKinematicsEngine
        function SaveInverseKinematicsEngine(obj, InverseKinematicsEngine)
            save(obj.FileName_InverseKinematicsEngine, 'InverseKinematicsEngine');
        end
        
        %saves SimulationEngine
        function SaveInitialPosition(obj, InitialPosition)
            save(obj.FileName_InitialPosition, 'InitialPosition');
        end
        
        %saves SimulationEngine
        function SaveExternalForcesEngine(~, ExternalForcesEngine)
            FileName = 'datafile_ExternalForcesEngine.mat';
            save(FileName, 'ExternalForcesEngine');
        end
        
    end
end