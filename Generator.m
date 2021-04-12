function agents = Generator(num_agent,obj,P)
    agents = [];
    for i = 1:num_agent
        agents = [agents,obj(P)];
    end
end