classdef Goal
    properties
        pose;
        goal_direction;
        desired_velocity;
        plot_vault;
    end
    methods
        function self = Goal(P,sheep_mean,offset_point)
            self.pose = rand(1,2)*(P.max_bound-P.min_bound)+P.min_bound;
            self.goal_direction = self.pose - sheep_mean.pose;
            self.desired_velocity = self.goal_direction.*offset_point.p_dot;
            self.goal_direction = atan2(self.goal_direction(2),self.goal_direction(1));
            
        end
        function self = Update(self,sheep_mean,offset_point)
            self.goal_direction = self.pose - sheep_mean.pose;
            self.desired_velocity = self.goal_direction.*offset_point.p_dot;
            self.goal_direction = atan2(self.goal_direction(2),self.goal_direction(1));
            
        end
    end
end   