classdef SRDt_VirtualConstraint
   properties
       Ndof {mustBePositive}
       degree {mustBeNumeric, mustBePositive}
       type string % 'ordinary' or 'bezier'
       c0 % s = c0*q
       H0 % H0*q = phi
   end
   
   methods
       function obj = SRDt_VirtualConstraint(Ndof, degree, type, c0, H0) % constructor of the class
           obj.Ndof = Ndof; % number of DoF
           obj.degree = degree; % degree of virtual constraints polynomials
           obj.type = type; % type of virtual constraint
           obj.c0 = c0; % define motion generator as linear combination of q_i
           obj.H0 = H0; % H0*q - phi -->0
       end
       
       
       function out = Phi(obj, s, p)
           if ((size(p,2) ~= obj.degree+1 || size(p,1) ~= obj.Ndof-1))
               error('Size of the polynomial coefficients and degree do not coincide')
           end
           
           if isa(s, 'casadi.SX')
               v = casadi.SX.zeros(obj.degree + 1, 1);
           else
               v = zeros(obj.degree + 1, 1);
           end
           
           if strcmp('ordinary', obj.type)  
               for i = 1:obj.degree + 1
                   v(i) = s^(i-1); 
               end
           else
               for i = 1:obj.degree + 1
                   v(i) = obj.bernstein_polynomial(obj.degree, i-1, s); 
               end
           end
           
           if isa(s, 'casadi.SX')
               out = casadi.SX.zeros(obj.Ndof, 1);
               out = [obj.H0;obj.c0]\[p*v;s];
           else
               out = [obj.H0;obj.c0]\[p*v;s];               
           end

       end
       
       
       function out = Phi_prm(obj, s, p)
           if ((size(p,2) ~= obj.degree+1 || size(p,1) ~= obj.Ndof-1))
               error('Size of the polynomial coefficients and degree do not coincide')
           end
           
           if isa(s, 'casadi.SX')
               v = casadi.SX.zeros(obj.degree + 1, 1);
           else
               v = zeros(obj.degree + 1, 1);
           end
               
           if strcmp('ordinary', obj.type)
               for i = 2:obj.degree + 1
                  v(i) = (i-1)*s^(i-2); 
               end
           else
               for i = 1:obj.degree
                  t = obj.degree*obj.bernstein_polynomial(obj.degree-1, i-1, s);
                  v(i) = v(i) - t;
                  v(i+1) = v(i+1) + t;
               end
           end
           
           if isa(s, 'casadi.SX')
               out = casadi.SX.zeros(obj.Ndof, 1);
               out = [obj.H0;obj.c0]\[p*v;1];
           else
               out = [obj.H0;obj.c0]\[p*v;1];               
           end
       end
       
       
       function out = Phi_2prm(obj, s, p)
           if ((size(p,2) ~= obj.degree+1 || size(p,1) ~= obj.Ndof-1))
               error('Size of the polynomial coefficients and degree do not coincide')
           end
           
           if isa(s, 'casadi.SX')
               v = casadi.SX.zeros(obj.degree + 1, 1);
           else
               v = zeros(obj.degree + 1, 1);
           end
           
           if strcmp('ordinary', obj.type)
               for i = 3:obj.degree + 1
                  v(i) = (i-1)*(i-2)*s^(i-3); 
               end
           else
               for i = 1:obj.degree-1
                  t = obj.degree*(obj.degree-1)*obj.bernstein_polynomial(obj.degree-2, i-1, s);
                  v(i) = v(i) + t;
                  v(i+1) = v(i+1) - 2*t;
                  v(i+2) = v(i+2) + t;
               end
           end
           
           if isa(s, 'casadi.SX')
               out = casadi.SX.zeros(obj.Ndof, 1);
               out = [obj.H0;obj.c0]\[p*v;0];
           else
               out = [obj.H0;obj.c0]\[p*v;0];               
           end
       end
   end
   
   
   methods (Static)
       function b = bernstein_polynomial(n,i,x)
           % Bernsterin polynomila (https://tinyurl.com/yd298elt)
           % B_{n,i}(x)
           t1 = factorial(n)./(factorial(i).*factorial(n-i));
           b = t1.*x.^i.*(1-x).^(n-i);
       end
   end
   
end