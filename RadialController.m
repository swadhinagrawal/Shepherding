function r = RadialController(P,sheeps,sheep_mean,r)
    rdot = (P.r0 - r);
    for i=1:length(sheeps)
        rdot = rdot + 2*(sheeps(i).pose-sheep_mean.pose)*(sheeps(i).s_dot-sheep_mean.s_dot)'/length(sheeps);
    end
    r = r + rdot*P.dt;
end