function SRD_dynamics_derive_JacobiansForLinkArray(varargin)
Parser = inputParser;
Parser.FunctionName = 'SRD_dynamics_derive_JacobiansForLinkArray';
Parser.addOptional('SymbolicEngine', []);
% Parser.addOptional('Symbolic_UseParallelizedSimplification', false);


Parser.parse(varargin{:});

if isempty(Parser.Results.SymbolicEngine)
    error('Please provide SymbolicEngine')
else
    SymbolicEngine = Parser.Results.SymbolicEngine;
end

disp('* Derivation of Jacobians started');

% Each link has center of mass rC, and angular velocity w
% 
% Jacobian_CenterOfMass = d(rC) / dq
% Jacobian_AngularVelocity = d(w) / dv
%
% v = dq/dt

if SymbolicEngine.Casadi
    for i = 1:length(SymbolicEngine.LinkArray)
        
        SymbolicEngine.LinkArray(i).Jacobian_CenterOfMass = ...
            jacobian(SymbolicEngine.LinkArray(i).AbsoluteCoM, SymbolicEngine.q);
        
        %%%%%%%%%%%
        
        Tvec = reshape(SymbolicEngine.LinkArray(i).AbsoluteOrientation, 9, 1);
        
        SymbolicEngine.LinkArray(i).AbsoluteOrientation_derivative = ...
            reshape( jacobian(Tvec, SymbolicEngine.q) * SymbolicEngine.v, [3, 3]);
        
        % this is the so-called Poisson formula, that defines the
        % relations between the matrix of directional cosines and
        % the angular velocity in a skew-simmetric form (angular
        % velocity tensor)
        %
        % There are two eq. of interest for angular velocity
        % tensor;
        % first: (0)W = dT*T' where T is the matrix of directional
        % cosines for a local frame (basis), dT is its derivative,
        % (0)W denotes W expressed in the world frame;
        % second: (l)W = T'*dT, (1)W denotes W expressed in the local
        % frame. The following equality also holds:
        % (0)W = T*(l)W*T';
        %
        % For finding kinetic energy we need (l)W, because the tensor
        % of inertia will be expressed in the local frame.
        
        Omega = SymbolicEngine.LinkArray(i).AbsoluteOrientation' * SymbolicEngine.LinkArray(i).AbsoluteOrientation_derivative;
        
        %The following gives us angular velocity w, expressed in the
        %local frame, see description of the angular velocity tensor.
        SymbolicEngine.LinkArray(i).AngularVelocity = [-Omega(2, 3); Omega(1, 3); -Omega(1, 2)];
        
        SymbolicEngine.LinkArray(i).Jacobian_AngularVelocity = ...
            jacobian(SymbolicEngine.LinkArray(i).AngularVelocity, SymbolicEngine.v);
        
    end
else
    for i = 1:length(SymbolicEngine.LinkArray)
        
        SymbolicEngine.LinkArray(i).Jacobian_CenterOfMass = ...
            jacobian(SymbolicEngine.LinkArray(i).AbsoluteCoM, SymbolicEngine.q);
        
        SymbolicEngine.LinkArray(i).Jacobian_CenterOfMass = simplify(SymbolicEngine.LinkArray(i).Jacobian_CenterOfMass);
        %%%%%%%%%%%
        
        Tvec = reshape(SymbolicEngine.LinkArray(i).AbsoluteOrientation, 9, 1);
        
        SymbolicEngine.LinkArray(i).AbsoluteOrientation_derivative = ...
            reshape( jacobian(Tvec, SymbolicEngine.q) * SymbolicEngine.v, [3, 3]);
        
        SymbolicEngine.LinkArray(i).AbsoluteOrientation_derivative = simplify(SymbolicEngine.LinkArray(i).AbsoluteOrientation_derivative);
        
        % this is the so-called Poisson formula, that defines the
        % relations between the matrix of directional cosines and
        % the angular velocity in a skew-simmetric form (angular
        % velocity tensor)
        %
        % There are two eq. of interest for angular velocity
        % tensor;
        % first: (0)W = dT*T' where T is the matrix of directional
        % cosines for a local frame (basis), dT is its derivative,
        % (0)W denotes W expressed in the world frame;
        % second: (l)W = T'*dT, (1)W denotes W expressed in the local
        % frame. The following equality also holds:
        % (0)W = T*(l)W*T';
        %
        % For finding kinetic energy we need (l)W, because the tensor
        % of inertia will be expressed in the local frame.
        
        Omega = SymbolicEngine.LinkArray(i).AbsoluteOrientation' * SymbolicEngine.LinkArray(i).AbsoluteOrientation_derivative;
        
        %The following gives us angular velocity w, expressed in the
        %local frame, see description of the angular velocity tensor.
        SymbolicEngine.LinkArray(i).AngularVelocity = [-Omega(2, 3); Omega(1, 3); -Omega(1, 2)];
        
        SymbolicEngine.LinkArray(i).AngularVelocity = simplify(SymbolicEngine.LinkArray(i).AngularVelocity);
        
        
        SymbolicEngine.LinkArray(i).Jacobian_AngularVelocity = ...
            jacobian(SymbolicEngine.LinkArray(i).AngularVelocity, SymbolicEngine.v);
        
        SymbolicEngine.LinkArray(i).Jacobian_AngularVelocity = simplify(SymbolicEngine.LinkArray(i).Jacobian_AngularVelocity);
        
    end    
end
            
disp('* Derivation of Jacobians finished');
   
end