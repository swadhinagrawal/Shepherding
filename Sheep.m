classdef Sheep
    properties
        pose;
        heading;
        plot_vault;
        s_dot;
    end
    methods
        function self = Sheep(P)
            self.pose = rand(1,2)*P.herd_radius+P.herd_center;
            self.heading = rand(1,1)*2*pi;
        end
        function self = Update(self,P,dogs)
            self.s_dot = [0,0];
            for k = 1:length(dogs)
                self.s_dot = self.s_dot + (- dogs(k).pose + self.pose)/(norm(dogs(k).pose - self.pose)^3);
            end
            self.pose = self.pose + P.dt*self.s_dot;
        end
    end
end  