classdef SRD_CLQE < SRDHandler_Controller
    properties
        Solution;
        
        Handler_Time;
        
        Handler_ControlInput_StateSpace;
        Handler_State;
        
        Handler_InverseDynamics;
        Handler_dynamics_generalized_coordinates_model;
        Handler_dynamics_Linearized_Model;
        Handler_Constraints_Model;
        C;
        
        ControllerSettings;
        ObserverSettings;
        R_type;
        R_custom;
        
        tol;
        
        
        SaveCopmutations;
        SystemExtraParameters;
        
        other;
        
    end
    methods
        function obj = SRD_CLQE(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRD_CLQE';
            Parser.addOptional('Handler_Time', []);
            Parser.addOptional('Handler_ControlInput_StateSpace', []);
            Parser.addOptional('Handler_State', []);
            Parser.addOptional('Handler_InverseDynamics', []);
            Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
            Parser.addOptional('Handler_dynamics_Linearized_Model', []);
            Parser.addOptional('Handler_Constraints_Model', []);
            Parser.addOptional('C', []);
            Parser.addOptional('ControllerSettings', []);
            Parser.addOptional('ObserverSettings', []);
            Parser.addOptional('R_type', 'ES');
            Parser.addOptional('R_custom', []);
            Parser.addOptional('SaveCopmutations', 1);
            Parser.addOptional('SystemExtraParameters', {});
            Parser.addOptional('tol', []);
            
            Parser.parse(varargin{:});
            
            obj.Handler_Time = Parser.Results.Handler_Time;
            
            obj.Handler_ControlInput_StateSpace                = Parser.Results.Handler_ControlInput_StateSpace;
            obj.Handler_State                                  = Parser.Results.Handler_State;
            
            obj.Handler_InverseDynamics                        = Parser.Results.Handler_InverseDynamics;
            obj.Handler_dynamics_generalized_coordinates_model = Parser.Results.Handler_dynamics_generalized_coordinates_model;
            obj.Handler_dynamics_Linearized_Model              = Parser.Results.Handler_dynamics_Linearized_Model;
            obj.Handler_Constraints_Model                      = Parser.Results.Handler_Constraints_Model;
            obj.C                                              = Parser.Results.C;
            
            obj.ControllerSettings                             = Parser.Results.ControllerSettings;
            obj.ObserverSettings                               = Parser.Results.ObserverSettings;
            obj.R_type                                         = Parser.Results.R_type;
            obj.R_custom                                       = Parser.Results.R_custom;
            
            obj.SaveCopmutations                               = Parser.Results.SaveCopmutations;
            obj.SystemExtraParameters                          = Parser.Results.SystemExtraParameters;
            
            obj.tol                                            = Parser.Results.tol;
            
%             obj.Update = @() obj.MyUpdate(...
%                 Parser.Results.Handler_Time, ...
%                 Parser.Results.Handler_ControlInput_StateSpace, ...
%                 Parser.Results.Handler_InverseDynamics, ...
%                 Parser.Results.Handler_dynamics_generalized_coordinates_model, ...
%                 Parser.Results.Handler_dynamics_Linearized_Model, ...
%                 Parser.Results.Handler_Constraints_Model, ...
%                 Parser.Results.C, ...
%                 Parser.Results.ControllerSettings, ...
%                 Parser.Results.ObserverSettings, ...
%                 Parser.Results.R_type, ...
%                 Parser.Results.R_type, ...
%                 Parser.Results.tol...
%                 );
            
            obj.Update = @() obj.MyUpdate();
            
        end
        
        
%         function MyUpdate(...
%                 obj, ...
%                 Handler_Time, ...
%                 Handler_ControlInput_StateSpace, ...
%                 Handler_InverseDynamics, ...
%                 Handler_dynamics_generalized_coordinates_model, ...
%                 Handler_dynamics_Linearized_Model, ...
%                 Handler_Constraints_Model, ...
%                 C, ...
%                 ControllerSettings, ...
%                 ObserverSettings, ...
%                 R_type, ...
%                 tol)
        function MyUpdate(obj)           
            
            t = obj.Handler_Time.CurrentTime;
            
            dof = obj.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
            k = obj.Handler_Constraints_Model.dof_Constraint;
            
            desired = obj.Handler_ControlInput_StateSpace.get_x_dx(t);
            desired_x =  desired(:, 1);
            desired_dx = desired(:, 2);
            desired_q = desired_x(1:dof);
            desired_v = desired_x((dof + 1):end);
            
            desired_u = obj.Handler_InverseDynamics.u;
            
            %f0 = Handler_dynamics_generalized_coordinates_model.get_FirstOrderSystem_qv(desired_x, desired_u);
            H = obj.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(desired_q);
            c = obj.Handler_dynamics_generalized_coordinates_model.get_bias_vector(desired_q, desired_v);
            T = obj.Handler_dynamics_generalized_coordinates_model.get_control_map(desired_q);
            n = obj.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
                      
            F  = obj.Handler_Constraints_Model.get_Jacobian(desired_q);
            dF = obj.Handler_Constraints_Model.get_Jacobian_derivative(desired_q, desired_v);
            G = [F, zeros(k, dof); dF, F];
            k = obj.Handler_Constraints_Model.dof_Constraint;
            
            A = obj.Handler_dynamics_Linearized_Model.get_A();
            B = obj.Handler_dynamics_Linearized_Model.get_B();
            
            switch obj.Handler_dynamics_Linearized_Model.LinearizationType
                case 'normal'
                    R = [F',              zeros(size(F'));
                         zeros(size(F')), pinv(H)*F'];
                     
                    iHf = pinv(H)*(T*desired_u - c); 
                    
                    M_GR = eye(2*n) - R*pinv(G*R)*G;
                    
                    va = M_GR * [desired_v; iHf];
                    a0 = va((n+1):(2*n));
                    
                    A = M_GR * A;
                    B = M_GR * B;
                    
                case 'constained'
                    M = [H, -F';
                        F, zeros(k, k)];
                    iM = pinv(M);
                    Ma = iM(1:n, :);
                    
                    a0 = Ma*[(T*desired_u - c); -dF*desired_v];
            end
            g = [desired_v; a0];
            
            System = struct('A', A, 'B', B, 'C', obj.C, 'G', G, 'g', g, 'tol', obj.tol, ...
                'R_type', obj.R_type, 'R_custom', obj.R_custom, ...
                'ControllerSettings', obj.ControllerSettings, 'ObserverSettings', obj.ObserverSettings, ...
                'x_desired', desired_x, 'dx_desired', desired_dx, ...
                obj.SystemExtraParameters{:});
            
            obj.Solution = LTI_CLQE(System, obj.SaveCopmutations);
        end
        
    end
end




