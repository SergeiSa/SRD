%This class stores info over the use generalized coordinates and alike
classdef SRDSymbolicEncoding < SRD_Chain
    properties
            
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Symbolic variables, used for derivations 
        
        q; %the vector of generalized coordinates;
        v; %the vector of generalized velocities;
        
        u; %the vector of control actions (scalar values of torques that
        %are being produced by the motors)

        
        Casadi = [];
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% External classes
        
        %Math
        %object of MathClass class; drovides useful functions;
        
    end
    methods
        % class constructor, it sets dof property,
        % initializes properties q, v, a, M,
        % and calls the superclass constructor.
        % it also assignes a valuse to dissipation_coefficients
        % property
        function obj = SRDSymbolicEncoding(LinkArray, Casadi)
            obj = obj@SRD_Chain(LinkArray);
            
            obj.Casadi = Casadi;
            
            if obj.Casadi
                obj.CasadiInitialization;
            else
                obj.q = sym('q', [obj.dof, 1]);
                obj.v = sym('v', [obj.dof, 1]);
                obj.u = sym('u', [obj.control_dof, 1]);
                obj.SetAssumptions();
            end
        end
        
        function CasadiInitialization(obj)
            import casadi.*
            obj.q = SX.sym('q', [obj.dof, 1]);
            obj.v = SX.sym('v', [obj.dof, 1]);
            obj.u = SX.sym('u', [obj.control_dof, 1]);
        end
        
        
        function InitializeLinkArray(obj)
            % Here we deal with the problem that RelativeFollower field
            % is initialised as a numeric array, and then the code will
            % attempt to modify it column by column; so to avoid Matlab
            % trying to convert symbolic to numerical we need to make
            % sure all matrices are converted to symbolic beforehand
            for i = 1:size(obj.LinkArray, 1)
                switch class(obj.q)
                    case 'sym'
                        obj.LinkArray(i).RelativeFollower = sym(obj.LinkArray(i).RelativeFollower);
                    case 'double'
                        %obj.LinkArray(i).RelativeFollower = obj.LinkArray(i).RelativeFollower;
                    case 'casadi.SX'
                        %obj.LinkArray(i).RelativeFollower = obj.LinkArray(i).RelativeFollower;
                    otherwise
                        error('invalid type of q')
                end
            end
            
            obj.Update(obj.q);       
        end
        
        
        %This function sets assumptions on symbolic variables;
        function SetAssumptions(obj)
            assume(obj.q, 'real');
            assume(obj.v, 'real');
        end
    end
end