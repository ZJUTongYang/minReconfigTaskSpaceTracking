classdef WaypointStates<handle
    properties
        gIK_ % n*7 array, [color, IK]
        ee_pose_ % 4*$ array, 
    end
    methods
        function obj = WaypointStates(IK, ee_pose)
            obj.ee_pose_ = ee_pose;
            obj.gIK_ = zeros(size(IK, 1), 7);
            obj.gIK_(:, 2:end) = IK;
        end
    end
end