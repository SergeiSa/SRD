%this class tunes PD controller
classdef TPPDtuner < handle
    properties
 
        %%%%%%%%%%%%%%%%%%%%%%%
        % external function handles     
        
        SimulationEngine;
        %handle for SRDControl/SRDSimulation class object
        
        ControlInput;
        %handle to a function that provides control input
        
        AdditiveCostFunction;
        %handle to a function that is used to calculate additive cost
        FinalCostFunction = [];
        %handle to a function that is used to calculate final cost
        %cost = integral of AdditiveCostFunction() over time + FinalCostFunction()
        
        SimulationFunction;
        %handle to a function that is used to simulate robot motion
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % optimization settings        
        
        OptimizerType = 'PSO';
        %Optimizer type. Valid types are 'PSO', 'GA', 'Sobol'
 
        SaveResults = true;
        %inf true the computation results will be saved in a file.
        
        DisplayMessageOnEvaluatingObjective = true;
        %if true, a custom message will be shown each time objective
        %function is evaluated;
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % Simulation settings  
        
        UseRandomIC = true;
        %if true, the simulation will use a randomized initial position,
        %producing control error in the beginning of teh simulation
        
        NumberOfRandomizedIC = 5;
        % the number of randomized initial positions for which the 
        % simulations will be done
        
        RandomizedIC_radius = 0.1;
        %defines how much the the randomization can change the given IC
        
        ICarray = [];
        %this array will contain all the initial positions that will be
        %used in simulation
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % controller settings     
        ControllerType = 'PD';
        %type of the PD controller that is tuned
        
        UpperBound = 5000;
        %the upper bound for the gains
        
        initial_guess_inified_kp = 1000;
        initial_guess_inified_kd = 100;
        %those values define the initial guess for optimization, where it
        %is needed. 
        %Kp = eye()*initial_guess_inified_kp
        %Kd = eye()*initial_guess_inified_kd
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % parallel computing
        
        NumberOfWorkers = 8;
        %Defines the number of MATLAB workers that will be used in parallel
        %computing
        UseParallelComputing = true;
        %if true the algorithms will attempt to use parallel computing
    end
    methods
        %class constructor
        function obj = PDtuner(SimulationEngine, AdditiveCostFunction, ControlInput, SimulationFunction)
            obj.SimulationEngine = SimulationEngine;
            obj.AdditiveCostFunction = AdditiveCostFunction;
            obj.ControlInput = ControlInput;
            obj.SimulationFunction = SimulationFunction;
        end
        
        %converts a vector with even number of elements k into two
        %diagonal matrices, Kp and Kd, where Kp has first half of k'
        %elements on its diagonal, and Kd - the second half.
        function [Kp, Kd] = ConvertToMatrices(~, k)
                n = max(size(k));
                
                k1 = k(1:(n/2));
                k2 = k((n/2 + 1):n);
                
                Kp = diag(k1);
                Kd = diag(k2);
        end
        
        %This method provides the objective function used in optimization
        function Objective_handle = GetObjective(obj)
            
            function J = Objective(k)
                [Kp, Kd] = obj.ConvertToMatrices(k);
                
                Controller = obj.SimulationEngine.GetPDcontroller(obj.ControllerType, Kp, Kd);
                
                J = 0;
                ICq = obj.SimulationEngine.IC.q;
                for i = 1:size(obj.ICarray, 1)
                    obj.SimulationEngine.IC.q = obj.ICarray(i, :)';
                    Res = obj.SimulationFunction(obj.ControlInput, Controller, obj.AdditiveCostFunction);
                    J = J + Res.Cost;
                end
                obj.SimulationEngine.IC.q = ICq;
                
                FinalCostInputStructure.Kp = Kp;
                FinalCostInputStructure.Kd = Kd;
                FinalCostInputStructure.k = k;
                
                if ~isempty(obj.FinalCostFunction)
                    FinalCost = obj.FinalCostFunction(FinalCostInputStructure);
                else
                    FinalCost = 0;
                end
                    
                J = J + FinalCost;
                
                if obj.DisplayMessageOnEvaluatingObjective
                    disp(['Evaluated objective for k = ', mat2str(k, 4), ' to get ', num2str(J)]);
                end
            end
            
            Objective_handle = @Objective;
        end
        
        %This function tune PD controller
        function Output = TunePD(obj, Parameters)
            
            if nargin < 2
                Parameters = [];
            end
            
            %Start parallel pool if needed
            if obj.UseParallelComputing
                ParallelPool = gcp('nocreate');
                if isempty(ParallelPool)
                    Cluster = parcluster('local');
                    Cluster.NumWorkers = obj.NumberOfWorkers;
                    parpool(Cluster, Cluster.NumWorkers);
                end
            end
            
            obj.PrepareICarray();
            SaveSimulationResultsToArray_buffer = obj.SimulationEngine.SaveSimulationResultsToArray;
            obj.SimulationEngine.SaveSimulationResultsToArray = false;
            

            Objective = obj.GetObjective();            
            
            n = obj.SimulationEngine.dof * 2;
            
            switch obj.OptimizerType
                case 'PSO'
                    if isfield(Parameters, 'PSO_problem')
                        PSO_problem = Parameters.PSO_problem;
                    else
                        
                        if isfield(Parameters, 'PSO_SwarmSize')
                            PSO_SwarmSize = Parameters.PSO_SwarmSize;
                        else
                            PSO_SwarmSize = n*20;
                        end
                        
                        if isfield(Parameters, 'Display')
                            Display = Parameters.Display;
                        else
                            Display = 'iter';
                        end
                        
                        if isfield(Parameters, 'MaxIterations')
                            MaxIterations = Parameters.MaxIterations;
                        else
                            MaxIterations = 1000*n;
                        end
                            
                        PSO_problem.lb = ones(n, 1)*0;
                        PSO_problem.ub = ones(n, 1)*obj.UpperBound;
                        PSO_problem.options = optimoptions('particleswarm', 'SwarmSize', PSO_SwarmSize, 'Display', Display, 'MaxIterations', MaxIterations, ...
                            'UseParallel', obj.UseParallelComputing);
                    end
                    PSO_problem.solver = 'particleswarm';
                    PSO_problem.objective = Objective;
                    PSO_problem.nvars = n;
                    
                    [k, fval, exitflag, log] = particleswarm(PSO_problem);
                    Output.k = k;
                    Output.fval = fval;
                    Output.exitflag = exitflag;
                    Output.log = log;
                case 'GA'
                    
                case 'Sobol'
                    
                otherwise
                   warning('Invalid optimizer type. Use ''PSO'', ''GA'' or ''Sobol''');
            end
            
            if obj.SaveResults
                if isfield(Parameters, 'FileName')
                    FileName = Parameters.FileName;
                else
                    FileName = ['OptimizationOutput ', date, ' ', num2str(now), '.mat'];
                end
                save(FileName, 'Output');
            end
            
            obj.SimulationEngine.SaveSimulationResultsToArray = SaveSimulationResultsToArray_buffer;
        end
        
        
        %This function provides a final cost function that uses spectral
        %norms of Kp and Kd matrices. The cost is calsulated as follows:
        % J = w(1)*|Kp| + w(2)*|Kd|
        %w - weights vector
        function SpectralNormFinalCost_handle = GetSpectralNormFinalCost(~, weights)
            if nargin < 2
                weights = [1; 1];
            end
            
            function J = SpectralNormFinalCost(FinalCostInputStructure)
                J = weights(1)*norm(FinalCostInputStructure.Kp) + weights(2)*norm(FinalCostInputStructure.Kd);
            end
            
            SpectralNormFinalCost_handle = @SpectralNormFinalCost;
        end
        
        %This function prepares .ICarray property, filling it with the
        %initial positions that will be used in simulations.
        function PrepareICarray(obj)
            
            if obj.UseRandomIC
                n = obj.SimulationEngine.dof;
                for i = 1:obj.NumberOfRandomizedIC
                    R = (2*rand(1, n) - ones(1, n))*obj.RandomizedIC_radius;
                    obj.ICarray(i, :) = obj.SimulationEngine.IC.q' + R;
                end
            else
                obj.ICarray = obj.SimulationEngine.IC.q';
            end
            
        end
        
    end
end