classdef Dog
    properties
        pose;
        delta_j;
        plot_vault;
        desired_pose;
        d_dot;
    end
    methods
        function self = Dog(P)
            self.pose = rand(1,2)*(P.max_bound-P.min_bound)+P.min_bound;
        end
        function self = Update(self,P,sheep_mean,goal)
            away_goal = pi+ self.delta_j + goal.goal_direction;
            away_goal = AngleWrap(away_goal);
            self.desired_pose = sheep_mean.pose + P.d_s_closeness*[cos(away_goal),sin(away_goal)];
            self.d_dot = P.kd*(self.desired_pose - self.pose);
            self.pose = self.pose + self.d_dot*P.dt;
        end
    end
end   