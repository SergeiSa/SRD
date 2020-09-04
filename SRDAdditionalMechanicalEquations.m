classdef SRDAdditionalMechanicalEquations < handle
    properties
        %object of SRDMechanicalEquations or SRDControlEquations classes
        SymbolicEngine;
    end
    methods
        %class constructor, gets or loads SymbolicEngine
        function obj = SRDAdditionalMechanicalEquations(SymbolicEngine)
            if nargin < 1
                SRD = SRDuserinterface;
                obj.SymbolicEngine = SRD.GetSymbolicEngine();
            else
                obj.SymbolicEngine = SymbolicEngine;
            end
        end
        
        %provides symbolic equations modelling springs installed in robot's
        %joints
        %
        %Parameters: 
        %'SpringCoefficientsType': 'Numeric', 'NumericUnified', 'Symbolic',  'SymbolicUnified', 'SymbolicAuto'
        %'EquilibriumPositionType': 'Numeric', 'NumericUnified', 'Symbolic',  'SymbolicUnified', 'SymbolicAuto'
        function Output = GetJointSprings(obj, varargin)
            %input options (name-value pairs)
            p = inputParser;
            p.FunctionName = 'SRDAdditionalMechanicalEquations.GetJointSprings';
            p.addOptional('SpringCoefficients', []);
            p.addOptional('SpringCoefficientUnified', 100);
            p.addOptional('SpringCoefficientsType', 'NumericUnified');
            p.addOptional('EquilibriumPositions', []);
            p.addOptional('EquilibriumPositionUnified', 0);
            p.addOptional('EquilibriumPositionType', 'NumericUnified');
            p.parse(varargin{:});
            
            IndexArray = [];
            for i = 1:length(obj.SymbolicEngine.LinkArray)
                switch obj.SymbolicEngine.LinkArray(i).JointType
                    case {'pivotX', 'pivotY', 'pivotZ', 'prismaticX', 'prismaticY', 'prismaticZ'}
                        IndexArray = [IndexArray; i];
                end
            end
            n = length(IndexArray);
            
            switch p.Results.SpringCoefficientsType
                case 'Numeric'
                    k = p.Results.SpringCoefficients;
                case 'NumericUnified'
                    k = ones(n, 1) * p.Results.SpringCoefficientUnified;
                case 'Symbolic'
                    k = p.Results.SpringCoefficients;
                case 'SymbolicUnified'
                    k = ones(n, 1) * p.Results.SpringCoefficientUnified;
                case 'SymbolicAuto'
                    k = sym('k', [n, 1]);
            end
            switch p.Results.EquilibriumPositionType
                case 'Numeric'
                    alpha = p.Results.EquilibriumPositions;
                case 'NumericUnified'
                    alpha = ones(n, 1) * p.Results.EquilibriumPositionUnified;
                case 'Symbolic'
                    alpha = p.Results.EquilibriumPositions;
                case 'SymbolicUnified'
                    alpha = ones(n, 1) * p.Results.EquilibriumPositionUnified;
                case 'SymbolicAuto'
                    alpha = sym('alpha', [n, 1]);
            end            
            
            Torques = sym(zeros(obj.SymbolicEngine.dof, 1));
            for i = 1:n
                index = IndexArray(i);
                GenCoordinateIndex = abs(obj.SymbolicEngine.LinkArray(index).UsedGenCoordinates);
                Torques(GenCoordinateIndex) = -k(GenCoordinateIndex) * (obj.SymbolicEngine.q(GenCoordinateIndex) - alpha(GenCoordinateIndex));
            end
            
            Output.Torques = Torques;
            Output.k = k;
            Output.alpha = alpha;
            Output.IndexArray = IndexArray;
        end
   
        %provides symbolic equations modelling angular dampers installed in
        %robot's joints
        %
        %Parameters: 
        %'DamperCoefficientsType': 'Numeric', 'NumericUnified', 'Symbolic',  'SymbolicUnified', 'SymbolicAuto'
        function Output = GetJointDamper(obj, varargin)
            %input options (name-value pairs)
            p = inputParser;
            p.FunctionName = 'SRDAdditionalMechanicalEquations.GetJointDamper';
            p.addOptional('DamperCoefficients', []);
            p.addOptional('DamperCoefficientUnified', 100);
            p.addOptional('DamperCoefficientsType', 'NumericUnified');
            p.parse(varargin{:});
            
            IndexArray = [];
            for i = 1:length(obj.SymbolicEngine.LinkArray)
                switch obj.SymbolicEngine.LinkArray(i).JointType
                    case {'pivotX', 'pivotY', 'pivotZ', 'prismaticX', 'prismaticY', 'prismaticZ'}
                        IndexArray = [IndexArray; i];
                end
            end
            n = length(IndexArray);
            
            switch p.Results.DamperCoefficientsType
                case 'Numeric'
                    mu = p.Results.DamperCoefficients;
                case 'NumericUnified'
                    mu = ones(n, 1) * p.Results.DamperCoefficientUnified;
                case 'Symbolic'
                    mu = p.Results.DamperCoefficients;
                case 'SymbolicUnified'
                    mu = ones(n, 1) * p.Results.DamperCoefficientUnified;
                case 'SymbolicAuto'
                    mu = sym('mu', [n, 1]);
            end         
            
            Torques = sym(zeros(obj.SymbolicEngine.dof, 1));
            for i = 1:n
                index = IndexArray(i);
                GenCoordinateIndex = obj.SymbolicEngine.LinkArray(index).UsedGenCoordinates;
                Torques(GenCoordinateIndex) = -mu(GenCoordinateIndex) * obj.SymbolicEngine.v(GenCoordinateIndex);
            end
            
            Output.Torques = Torques;
            Output.mu = mu;
            Output.IndexArray = IndexArray;
        end        
        
    end
end







