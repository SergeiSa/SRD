classdef SRDHandler_StateConverter_GenCoord2StateSpace < SRDHandler
    properties
        x;
        dx;
        
        dof_robot_StateSpace;
        
        Update;
    end
    methods
        function squized = get_x_dx(obj, ~)            
            squized = [obj.x, obj.dx];
        end
    end
end
   