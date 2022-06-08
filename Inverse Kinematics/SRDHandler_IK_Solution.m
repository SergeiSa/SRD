classdef SRDHandler_IK_Solution < SRDHandler
    properties
        get_position;
        get_position_velocity_acceleration;
        
        dof_configuration_space_robot;
    end
    methods
        function handle = get_position_handle(obj)
            handle = @(t) obj.get_position(t);
        end
        function handle = get_position_velocity_acceleration_handle(obj)
            handle = @get_position_velocity_acceleration_fnc;
            
            function [q, v, a] = get_position_velocity_acceleration_fnc(t)
                [q, v, a] = obj.get_position_velocity_acceleration(t);
            end
        end
    end
end
   