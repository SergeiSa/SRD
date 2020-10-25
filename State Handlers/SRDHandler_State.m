classdef SRDHandler_State < SRDHandler
    properties
        q;
        v;
        a;
        Other;
        
        dof_robot;
    end
    methods
        function squized = get_position_velocity_acceleration(obj, ~)            
            squized = [obj.q, obj.v, obj.a];
        end
    end
end
   