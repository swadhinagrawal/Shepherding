classdef Sheep
    properties
        Id;
        pose;
        heading;
        plot_vault;
        s_dot;
        zoa;
        zoo;
        zor;
        angular_pose;
        Rr;
        Ro;
        Ra;
        visibility;
    end
    methods
        function self = Sheep(P,id)
            self.pose = rand(1,2)*P.herd_radius+P.herd_center;
            self.heading = rand(1,1)*2*pi;
            self.Rr = P.Rr;
            self.Ro = P.Ro;
            self.Ra = P.Ra;
            self.visibility = P.visibility;
            self.Id = id;
            self.s_dot = [cos(self.heading),sin(self.heading)];
        end
        function self = Update(self,P,dogs,sheeps)
            self = self.Neighbours(sheeps);
            net_steering = self.NetSteering(P);
            self.s_dot = 0.5*net_steering*P.dt;
            for k = 1:length(dogs)
                self.s_dot = self.s_dot + (- dogs(k).pose + self.pose)/(norm(dogs(k).pose - self.pose)^3);
            end
            self.pose = self.pose + P.dt*self.s_dot;
        end
        function self = Neighbours(self,sheeps)
            self.zor = [];
            self.zoo = [];
            self.zoa = [];
            for i=1:length(sheeps)
                if ne(sheeps(i).Id,self.Id)
                    dij = norm(sheeps(i).pose-self.pose);
                    self.angular_pose = self.BlindRegion(sheeps(i));
                    if (dij<=self.Rr) & (self.angular_pose < self.visibility)
                        self.zor = [self.zor;[sheeps(i)]];
                    elseif (dij > self.Rr) & (dij <= self.Ro) & (self.angular_pose < self.visibility)
                        self.zoo = [self.zoo;[sheeps(i)]];
                    elseif (dij > self.Ro) & (dij <= self.Ra) & (self.angular_pose < self.visibility)
                        self.zoa = [self.zoa;[sheeps(i)]];
                    else
                        continue
                    end
                end
            end
        end
        function angular_pose = BlindRegion(self,other)
            pointing_vec = other.pose - self.pose;
            pointing_vec = pointing_vec/norm(pointing_vec);
            angular_pose = acos(pointing_vec .* self.s_dot/norm(self.s_dot));
        end
        function force_r = Repulsion(self)
            repel = [0.0,0.0];
            for i = 1:length(self.zor)
                vec = self.zor(i,1).pose - self.pose;
                repel = repel + (vec/norm(vec));
            end
            force_r = -repel/length(self.zor);
        end
        function force_o = Alignment(self)
            align = [0.0,0.0];
            for i = 1:length(self.zoo)
                align = align + (self.zoo(i,1).s_dot/norm(self.zoo(i,1).s_dot));
            end
            force_o = align/length(self.zoo);
        end
        function force_a = Cohesion(self)
            attract = [0.0,0.0];
            for i = 1:length(self.zoa)
                vec = self.zoa(i,1).pose - self.pose;
                attract = attract + (vec/norm(vec));
            end
            force_a = attract/length(self.zoa);
        end
        function net_steer = NetSteering(self,P)
            if isempty(self.zor) == 0
                net_steer = self.Repulsion();
            else
                if isempty(self.zoo)==0 && isempty(self.zoa)==1
                    net_steer = self.Alignment();
                elseif isempty(self.zoo)==1 && isempty(self.zoa)==0
                    net_steer = self.Cohesion();
                elseif isempty(self.zoo)==0 && isempty(self.zoa)==0
                    net_steer = 0.5*(self.Cohesion()+self.Alignment());
                else
                    net_steer = self.s_dot/P.dt;
                end
            end
        end
    end
end  