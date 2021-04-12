classdef SheepMean
    properties
        pose;
        heading;
        plot_vault;
    end
    methods
        function self = SheepMean(sheeps)
            self.pose = [0,0];
            self.heading = 0;
            for i = 1:length(sheeps)
                self.pose = self.pose + sheeps(i).pose;
                self.heading = self.heading + sheeps(i).heading;
            end
            self.pose = self.pose/length(sheeps);
            self.heading = self.heading/length(sheeps);
        end
        function self = Update(self,sheeps)
            self.pose = [0,0];
            self.heading = 0;
            for i = 1:length(sheeps)
                self.pose = self.pose + sheeps(i).pose;
                self.heading = self.heading + sheeps(i).heading;
            end
            self.pose = self.pose/length(sheeps);
            self.heading = self.heading/length(sheeps);
        end
    end
end  