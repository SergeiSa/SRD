classdef tpTransverseLinearization < handle
    properties
        A_table       % nominal state matrix along trajectory
        B_table       % nominal control matrix along trajectory
        x_table       % nominal state trajectory
        u_table       % nominal control evaluation
        dx_table       % nominal state derivative trajectory
        
        robot   % pendubot instance
        n = [];  % number of states
        m = [];  % number of states
        N       % number of samples
        P       % rotation matrix for finidng z: z = P*f(x,u)/|P*f(x,u)|
    end
    
    methods
        function obj = tpTransverseLinearization(A_table, B_table, x_table, u_table, dx_table, P)
            % class constructor
            % inputs:
            %   output of SRD_LinearModel_GenerateTable
            
            obj.A_table = A_table;
            obj.B_table = B_table;
            obj.x_table = x_table;
            obj.u_table = u_table;
            obj.dx_table = dx_table;
            obj.P = P;
            
            obj.n = size(x_table, 1);
            obj.m = size(u_table, 1);
            obj.N = size(x_table, 2);
        end
        
        
        function [z, xi] = compute_moving_orthonormal_system(obj, e)
            % computes moving orthonormal system along periodic orbit
            % inputs:
            %   e   orthonormal basis
            % ouputs:
            %   z   periodic vector function tangent to trajectory - 
            %       phase acceleration. It is chosen as f(x,u)/|f(x,u)| 
            %   xi  transverse coordinates
            
            z = zeros(obj.n, obj.N);
            F_X_U = [];
            for k = 1:obj.N
                f_x_u = obj.dx_table(:, k);
                F_X_U = [F_X_U, f_x_u];
                z(:,k) = (obj.P*f_x_u)./norm(obj.P*f_x_u);
            end
            
            % check that e1 is not equal to z(tau)
            delta = zeros(obj.n, obj.N);
            delta_norm = zeros(1, obj.N);
            for k = 1:obj.N
                delta(:,k) = z(:,k) - e(:,1);
                delta_norm(k) = norm(delta(:,k),2);
                if delta_norm(k) < 1e-6
                    error('e1 is equal to z');
                end
            end
            
            xi = zeros(obj.n, obj.n-1, obj.N);
            for k = 1:obj.N
               for j = 1:obj.n-1
                   coef = e(:,j+1)'*z(:,k)/(1 + e(:,1)'*z(:,k));
                   xi(:,j,k) = e(:,j+1) - coef*(e(:,1) + z(:,k));
               end
            end
        end
        
        
        function [dz_dtau, dxi_dtau] = compute_moving_orthonormal_system_derivative(obj, e, z)
            % computes the derivative of the moving orthonormal system
            % inputs:
            %   e   orthonormal basis
            %   z   periodic vector function tangent to trajectory
            % outputs:
            %   dz_dtau     time derivative of (flow)
            %   dxi_dtau    time derivative of the transverse coordinates
            
            dz_dtau = zeros(obj.n, obj.N);
            for k = 1:obj.N
                f_x_u = obj.dx_table(:, k);

                df_dx = obj.A_table(:, :, k);
                dx_dtau = f_x_u;
                
                Pf_norm = norm(obj.P*f_x_u, 2);
                dz_dtau(:,k) = (obj.P*df_dx*dx_dtau*Pf_norm - ...
                                obj.P*f_x_u*(obj.P*f_x_u)'*obj.P*df_dx*dx_dtau/Pf_norm)/...
                               Pf_norm^2;
            end

            dxi_dtau = zeros(obj.n, obj.n-1, obj.N);
            for k = 1:obj.N
                for j = 1:obj.n-1
                    coef = e(:,j+1)'*z(:,k)/(1 + e(:,1)'*z(:,k));

                    t1 = e(:,j+1)'*dz_dtau(:,k)*(1 + e(:,1)'*z(:,k));
                    t2 = e(:,j+1)'*z(:,k)*e(:,1)'*dz_dtau(:,k);
                    t3 = (t2 - t1)/(1 + e(:,1)'*z(:,k))^2;

                    dxi_dtau(:,j,k) = t3*(e(:,1) + z(:,k)) - coef*dz_dtau(:,k);
                end
            end
        end
        
        
        function [A, B] = compute_transverse_linearization(obj)
            % computes transverse linearization according to Hale. It works
            % in case when z = f(x)/|f(x)| !!!!!!!!!!
            % output:
            %   A   state-matrix of the linearized transverse dynamics
            %   B   input-matrix of the linearized transverse dynamics
            
            e = obj.generate_orthonormal_basis(obj.n);
            [z, xi] = obj.compute_moving_orthonormal_system(e);
            [dz_dtau, dxi_dtau] = obj.compute_moving_orthonormal_system_derivative(e, z);
            
            % compute projection operator and its derivative
            Pi = zeros(obj.n-1, obj.n, obj.N);
            dPi_dtau = zeros(obj.n-1, obj.n, obj.N);
            for k = 1:obj.N
                Pi(:,:, k) = xi(:,:,k)';
                dPi_dtau(:,:,k) = dxi_dtau(:,:,k)';
            end

            A = zeros(obj.n-1, obj.n-1, obj.N);
            B = zeros(obj.n-1, obj.m, obj.N);
            for k = 1:obj.N
                f_x_u = obj.dx_table(:, k);
                df_dx = obj.A_table(:, :, k);
                
                dtau = (z(:,k)'*obj.P*df_dx*Pi(:,:,k)' + dz_dtau(:,k)'*obj.P*Pi(:,:,k)')/...
                       (z(:,k)'*obj.P*f_x_u);
                   
                A(:,:,k) = dPi_dtau(:,:,k)*Pi(:,:,k)' + Pi(:,:,k)*df_dx*Pi(:,:,k)' - ...
                            Pi(:,:,k)*f_x_u*dtau;
                                        
                df_du = obj.B_table(:, :, k);
                
                B(:,:,k) = Pi(:,:,k)*(eye(obj.n) - ...
                            (f_x_u*z(:,k)'*obj.P)/(z(:,k)'*obj.P*f_x_u))*df_du;
            end
        end
        
    end
    
    methods (Static)
        function e = generate_orthonormal_basis(n)
            % generates orthonormal basis
            % inputs:
            %   n - size of basis vectors
            
            is_orthonormal_basis_found = 0;
            while ~is_orthonormal_basis_found
                A = rand(n); % generate matrix
                B = A'*A; % make it symmetric

                [P, ~] = eig(B); % find eigenvectors and eigenvalues of B
                if ((P'*P - eye(n)) < eps) 
                    is_orthonormal_basis_found = 1;
                    e = P;
                end
            end
        end
     
    end
end