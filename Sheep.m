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
%             self.pose = self.pose + P.dt*self.s_dot;
            angle_arithmatic = AngleWrap(atan2(self.s_dot(2),self.s_dot(1))) - self.heading;
            
            self.heading = self.heading + P.dt*angle_arithmatic;
            self.pose = self.pose + norm(P.dt*self.s_dot)*[cos(self.heading),sin(self.heading)];
        end
    end
end  