classdef SRDAddExternalForces < handle
    properties
        SymbolicEngine;
        
        ForwardDynamicsStructure;
        
        vars;
    end
    methods
        % class constructor
        % initializes properties s and calls the superclass constructor.
        function obj = SRDAddExternalForces(SymbolicEngine)
            obj.SymbolicEngine = SymbolicEngine;
        end
        
        % This function generates symbolic expressions eeded to add
        % external force. To use them - call .UpdateModel() on every
        % iteration
        %
        %
        %Force - symbolic expression for the added generilized force; 
        %
        %vars - parameters defining the force (for example, Cartesian
        %components of the force producing the gen. Force.
        %
        %Use example: (in the symbolic derivation (step 4) routine)
            % SymbolicEngine = SRD.GetSymbolicEngine();
            % ExternalForcesEngine = SRDAddExternalForces(SymbolicEngine);
            % f = sym('f', [2, 1]);
            % L = SymbolicEngine.RetreaveLinkInLinkArray("Torso");
            % r = L.AbsoluteFollower;
            % J = jacobian(r, SymbolicEngine.q);
            % Force = J'*[f(1); 0; f(2)];
            % ExternalForcesEngine.AddForce(Force, f); 
            % SRD.SaveExternalForcesEngine(ExternalForcesEngine);
        %
        function AddForce(obj, Force, vars)
            timerVal = tic;

            disp('Started calculating the right hand side of the manipulator equations - with external forces');
            obj.ForwardDynamicsStructure.RightHandSide_ofManipulatorEq = ...
                obj.SymbolicEngine.ForwardDynamicsStructure.RightHandSide_ofManipulatorEq + Force;
            
            disp('Started calculating the skew vector (forces for the computed torque controller) - with external forces');
            obj.ForwardDynamicsStructure.ForcesForComputedTorqueController = ...
                obj.SymbolicEngine.ForwardDynamicsStructure.ForcesForComputedTorqueController + Force;
            
            disp('Finished updating symbolic exprssions to take into account external forces');
            
            
            disp('Started generating Right Hand Side of Manipulator Eq function - with external forces');
            matlabFunction(obj.ForwardDynamicsStructure.RightHandSide_ofManipulatorEq, 'File', ...
                'g_dynamics_RHS_ext', 'Vars', {obj.SymbolicEngine.q, obj.SymbolicEngine.v, obj.SymbolicEngine.u, vars}, ...
                'Optimize', obj.SymbolicEngine.ToOptimizeFunctions);
            
            disp('Started generating function for forces for the computed torque controller - with external forces');
            matlabFunction(obj.ForwardDynamicsStructure.ForcesForComputedTorqueController, 'File', ...
                'g_control_ForcesForComputedTorqueController_ext', 'Vars', {obj.SymbolicEngine.q, obj.SymbolicEngine.v, vars}, ...
                'Optimize', obj.SymbolicEngine.ToOptimizeFunctions); 
            
            disp('Finished generating functions to take into account external forces');

            obj.vars = vars;
            
            toc(timerVal);
        end
        
        %updates model functions in the SRDModelHandler
        %
        %value - value of parameters vars that are currently acting
        %
        % Example: (in the simulation (step 7) routine)
        %
        %     function f = GetExternalForces()
        %         SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
        %         t = SensorData.t;
        %         f = [sin(t); 1];
        %     end
        %     SimulationEngine.CustomSolverType = 'User-provided';
        %     function OutputStructure = User_provided_solver()
        %         f = GetExternalForces();
        %         ExternalForcesEngine.UpdateModel(f, SimulationEngine.ModelHandler);
        %         OutputStructure = SimulationEngine.Solver_TaylorUpdate();
        %     end
        %     SimulationEngine.User_provided_solver = @User_provided_solver;
        %
        function UpdateModel(~, value, ModelHandler)
            
            ModelHandler.g_dynamics_RHS = @(q, v, u) g_dynamics_RHS_ext(q, v, u, value);
            ModelHandler.ForcesForComputedTorqueController = @(q, v) g_control_ForcesForComputedTorqueController_ext(q, v, value);
            
        end
    end
end
