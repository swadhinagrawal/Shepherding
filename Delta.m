function root = Delta(P,desired_velocity)
    func = @(d) sin(P.num_herders*d/(2-2*P.num_herders))/(P.d_s_closeness^2 * sin(d/(2-2*P.num_herders))) - desired_velocity ;
    reach = 0.001;
    vault = [inf,inf];
    max = 2*pi;
    min = 0;
    while min<max
        mid = (min+max)/2;
        if or(mid==min,mid==max)
            break
        end
        curr = func(mid); 
        if curr<vault(1)
           vault(1) = curr;
           vault(2) = mid;
        end
        if curr<reach
            max = mid;
        else
            min = mid;
        end
    end
    root = vault(2);
end