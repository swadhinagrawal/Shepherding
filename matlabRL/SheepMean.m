classdef SheepMean
    properties
        pose;
        heading;
        plot_vault;
        s_dot;
    end
    methods
        function self = SheepMean(sheeps)
            self.pose = [0,0];
            self.heading = 0;
            self.s_dot = [0,0];
            for i = 1:length(sheeps)
                self.pose = self.pose + sheeps(i).pose;
                self.heading = self.heading + sheeps(i).heading;
                self.s_dot = self.s_dot + sheeps(i).s_dot;
            end
            self.pose = self.pose/length(sheeps);
            self.heading = self.heading/length(sheeps);
            self.s_dot = self.s_dot/length(sheeps);
        end
        function self = Update(self,sheeps)
            self.pose = [0,0];
            self.heading = 0;
            for i = 1:length(sheeps)
                self.pose = self.pose + sheeps(i).pose;
                self.heading = self.heading + sheeps(i).heading;
                self.s_dot = self.s_dot + sheeps(i).s_dot;
            end
            self.pose = self.pose/length(sheeps);
            self.heading = self.heading/length(sheeps);
            self.s_dot = self.s_dot/length(sheeps);
        end
    end
end  