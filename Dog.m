classdef Dog
    properties
        pose;
        delta_j;
        plot_vault;
        desired_pose;
        d_dot;
        Id;
    end
    methods
        function self = Dog(P,id)
            self.pose = rand(1,2)*(P.max_bound-P.min_bound)+P.min_bound;
            dist = [];
            for i =1:length(P.Obstacles(:,1))
                dist = [dist,norm(self.pose-P.Obstacles(i,1:2))];
            end
            while any(dist(:)<max(P.Obstacles(:,3)))    
                self.pose = rand(1,2)*(P.max_bound-P.min_bound)+P.min_bound;
            end
            self.Id = id;
        end
        function self = Update(self,P,sheep_mean,goal)
            away_goal = pi+ self.delta_j + goal.goal_direction;
            away_goal = AngleWrap(away_goal);
            self.desired_pose = sheep_mean.pose + P.d_s_closeness*[cos(away_goal),sin(away_goal)];
            self.d_dot = P.kd*(self.desired_pose - self.pose);
            x = norm(self.d_dot);
            x = self.Saturate(P.Dog_limit,x);
            self.d_dot = x*self.d_dot/norm(self.d_dot);
            self.d_dot = self.d_dot + self.Force(P);
            self.pose = self.pose + self.d_dot*P.dt ;
        end
        function value = Saturate(self,limit,value)
            if value>limit
                value = limit;
            end
        end
        function obstacle_force = Force(self,P)
            obstacle_force = [0,0];
            for i=1:length(P.Obstacles(:,1))
                dij = norm(P.Obstacles(i,1:2)-self.pose);
                if dij<=1.3*P.Obstacles(i,3)
                    obstacle_force = obstacle_force - ((P.Obstacles(i,1:2)-self.pose)/dij*(dij-P.Obstacles(i,3))^2);  
                end
            end
            obstacle_force = self.Saturate(P.Obs_limit,obstacle_force);
        end
    end
end   