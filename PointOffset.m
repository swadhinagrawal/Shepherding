classdef PointOffset < handle
    properties
        pose;
        p_dot;
        plot_vault;
    end
    methods
        function self = PointOffset(P,sheep_mean)
            self.pose = sheep_mean.pose + [P.offset*cos(sheep_mean.heading),P.offset*sin(sheep_mean.heading)];
            self.p_dot =  - P.kp*self.pose;
        end
        function self = Update(self,P,sheep_mean)
            self.pose = sheep_mean.pose + [P.offset*cos(sheep_mean.heading),P.offset*sin(sheep_mean.heading)];
            self.p_dot =  - P.kp*self.pose;
        end
    end
end   