params;

dogs = Generator(P.num_herders,@Dog,P);
sheeps = Generator(P.num_walkers,@Sheep,P);

sheep_mean = SheepMean(sheeps);
offset_point = PointOffset(P,sheep_mean);
goal = Goal(P,sheep_mean,offset_point);
animator = Animation(P,dogs,sheeps,goal,sheep_mean,offset_point);
count = 0;
t = 0;
while norm(sheep_mean.pose - goal.pose)>=0.8
    P.d_s_closeness = abs(RadialController(P,sheeps,sheep_mean,P.d_s_closeness));
    delta = AngleWrap(Delta(P,norm(goal.desired_velocity)));
    for j =1:length(dogs)
        dogs(j).delta_j = delta*(2*j-P.num_herders-1)/(2*P.num_herders - 2);
        dogs(j) = dogs(j).Update(P,sheep_mean,goal);
    end
    for i =1:length(sheeps)
        sheeps(i) = sheeps(i).Update(P,dogs,sheeps);
    end
    sheep_mean = sheep_mean.Update(sheeps);
    offset_point = offset_point.Update(P,sheep_mean);
    goal = goal.Update(sheep_mean,offset_point);
    animator = animator.Update(P,dogs,sheeps,goal,sheep_mean,offset_point,t);
    pause(0.01)
    count = count + 1;
    t = t + P.dt;
end
close(animator.myVideo)