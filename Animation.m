classdef Animation
    properties
       sheep_body_size;
       figure;
       encirclement;
    end
    methods
        function self = Animation(P,dogs,sheeps,goal,sheep_mean,offset_point)
            self.sheep_body_size = P.sheep_body_size;
            self.figure = figure();
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
            hold off
            axis equal
            xlim([-5 40])
            ylim([-5 40])
        end
        function self = Update(self,P,dogs,sheeps,goal,sheep_mean,offset_point)
            clf;
            hold on
            self.encirclement = viscircles(sheep_mean.pose,P.d_s_closeness,'color',[0.4940 0.1840 0.5560],'LineWidth',0.5,'LineStyle','-.');
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
            xlim([-5 40])
            ylim([-5 40])
        end
    end
end
