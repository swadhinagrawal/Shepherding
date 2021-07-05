classdef Animation
    properties
       sheep_body_size;
       figure;
       encirclement;
       myVideo;
       frames;
       dog_history;
       sheep_history;
       obstacles;
       sheep_inter_f;
       sheep_obs_f;
       sheep_dog_f;
       time;
    end
    methods
        function self = Animation(P,dogs,sheeps,goal,sheep_mean,offset_point)
            self.dog_history = [];
            self.sheep_history = [];
            self.sheep_inter_f = [];
            self.sheep_obs_f = [];
            self.sheep_dog_f = [];
            self.time = [];
            self.sheep_body_size = P.sheep_body_size;
            self.myVideo = VideoWriter('Shepherding.avi');
            self.myVideo.FrameRate = 20;
            open(self.myVideo)
            self.figure = figure();
            self.figure.Position = [10 10 1000 1000]; 
            subplot(2,3,1);
            
            hold on
            for i = 1:length(dogs)
                dogs(i).plot_vault = plot(dogs(i).pose(1),dogs(i).pose(2),'.r','MarkerSize',20);
            end
            for i = 1:length(sheeps)
                sheeps(i).plot_vault = quiver(sheeps(i).pose(1),sheeps(i).pose(2),self.sheep_body_size*cos(sheeps(i).heading),self.sheep_body_size*sin(sheeps(i).heading),'-ob');
            end
            goal.plot_vault = plot(goal.pose(1),goal.pose(2),'ok','MarkerSize',50);
            sheep_mean.plot_vault = plot(sheep_mean.pose(1),sheep_mean.pose(2),'.g','MarkerSize',20);
            offset_point.plot_vault = plot(offset_point.pose(1),offset_point.pose(2),'.k','MarkerSize',20);
            self.encirclement = viscircles(sheep_mean.pose,P.d_s_closeness,'color',[0.4940 0.1840 0.5560],'LineWidth',0.5,'LineStyle','-.');
            self.obstacles = viscircles(P.Obstacles(:,1:2),P.Obstacles(:,3),'color',[0.6350 0.0780 0.1840],'LineWidth',5);
            hold off
            axis equal
            xlim([-20 100])
            ylim([-20 100])
            
            self.frames = getframe(self.figure);
            writeVideo(self.myVideo,self.frames);
            self.dog_history = [self.dog_history ; [dogs(:).pose]];
            self.sheep_history = [self.sheep_history ; [sheeps(:).pose]];

        end
        function self = Update(self,P,dogs,sheeps,goal,sheep_mean,offset_point,t)
            clf;
            subplot(2,3,1);
            hold on
            self.encirclement = viscircles(sheep_mean.pose,P.d_s_closeness,'color',[0.4940 0.1840 0.5560],'LineWidth',0.5,'LineStyle','-.');
            self.obstacles = viscircles(P.Obstacles(:,1:2),P.Obstacles(:,3),'color',[0.6350 0.0780 0.1840],'LineWidth',5);
            for i = 1:length(dogs)
                dogs(i).plot_vault = plot(dogs(i).pose(1),dogs(i).pose(2),'.r','MarkerSize',20);
            end
            for i = 1:length(sheeps)
                sheeps(i).plot_vault = quiver(sheeps(i).pose(1),sheeps(i).pose(2),self.sheep_body_size*cos(sheeps(i).heading),self.sheep_body_size*sin(sheeps(i).heading),'-ob');
            end
            goal.plot_vault = plot(goal.pose(1),goal.pose(2),'ok','MarkerSize',50);
            sheep_mean.plot_vault = plot(sheep_mean.pose(1),sheep_mean.pose(2),'.g','MarkerSize',20);
            offset_point.plot_vault = plot(offset_point.pose(1),offset_point.pose(2),'.k','MarkerSize',20);
            
            hold off
            axis equal
            xlim([-20 100])
            ylim([-20 100])
            
            
            subplot(2,3,2);
            hold on
            self.obstacles = viscircles(P.Obstacles(:,1:2),P.Obstacles(:,3),'color',[0.6350 0.0780 0.1840],'LineWidth',0.5);
            goal.plot_vault = plot(goal.pose(1),goal.pose(2),'ok','MarkerSize',50);
            for i = 1:length(dogs)
                for j=1:2:2*length(dogs)
                    plot(self.dog_history(:,j),self.dog_history(:,j+1),'.r','MarkerSize',0.1);
                end
            end
            for i = 1:length(sheeps)
                for j=1:2:2*length(sheeps)
                    plot(self.sheep_history(:,j),self.sheep_history(:,j+1),'.b','MarkerSize',0.1);
                end
            end
            hold off
            axis equal
            xlim([-20 100])
            ylim([-20 100])
            
            
            self.dog_history = [self.dog_history ; [dogs(:).pose]];
            self.sheep_history = [self.sheep_history ; [sheeps(:).pose]];
            
            
            self.time = [self.time;t];
            interaction_force = [0,0];
            obstacle_f = [0,0];
            dog_force = [0,0];
            for i=1:length(sheeps)
                interaction_force = interaction_force + sheeps(i).interaction_force;
                obstacle_f = obstacle_f + sheeps(i).obstacle_f;
                dog_force = dog_force + sheeps(i).dog_force;
            end
            self.sheep_inter_f = [self.sheep_inter_f;norm(interaction_force)];
            self.sheep_obs_f = [self.sheep_obs_f;norm(obstacle_f)];
            self.sheep_dog_f = [self.sheep_dog_f;norm(dog_force)];
            subplot(2,3,4);
            hold on
            title('Inter sheep interaction')
            plot(self.time,self.sheep_inter_f,'-r');
            hold off

            subplot(2,3,5);
            hold on
            title('Obstacle force')
            plot(self.time,self.sheep_obs_f,'-m');
            hold off

            subplot(2,3,6);
            hold on
            title('Dog sheep interaction')
            plot(self.time,self.sheep_dog_f,'-b');
            hold off

            self.frames = getframe(self.figure);
            writeVideo(self.myVideo,self.frames);
        end
    end
end
