classdef Utilities < handle
    properties
        
    end
    
    methods%% Class for Utilities
        function self = Utilities()
           
        end
        % Determine which arm is closer to the part provided (via a
        % transform)
        function [closer] = CloserArm(self, tr,robot1, robot2)
            dist1 = pdist2(transl(tr)',transl(robot1.base)');
            dist2 = pdist2(transl(tr)',transl(robot2.base)');
            if dist1 <= dist2
                closer = 1;
            else
                closer = 2;
            end
            
        end
    end
end
        