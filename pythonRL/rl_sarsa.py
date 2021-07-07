
import numpy as np
import matplotlib.pyplot as plt

plt.ion()


class params:
    def __init__(self):
        #   RL params
        self.steps_per_Epi = 2000
        #   Simulation params
        self.dt = 0.1
        #   Environment params
        self.boundary_min = 0
        self.boundary_max = 80
        self.num_D = 1
        self.num_S = 15
        self.herd_radius = 5
        self.herd_center = np.random.rand(1,2)[0]*(self.boundary_max-self.boundary_min-self.herd_radius)# + self.boundary_min + 1
        self.Rr = 1
        self.Ro = 3
        self.Ra = 12
        self.visibility = 2*np.pi
        self.sheep_body = 0.2
        self.goal_radius = 5
        self.Dog_sheep_interaction_limit = 2
        self.goal_threshold = self.goal_radius/self.num_S

class Dog:
    def __init__(self,id,P):
        self.id = id
        self.params = P
        self.pose = np.random.rand(1,2)[0]*(self.params.boundary_max-self.params.boundary_min) + self.params.boundary_min
        self.d_dot = None
    
    def doAction(self,a,s,sheepMean):
        force_mag = 1.0
        force = force_mag*np.array(a)
        self.d_dot = force*self.params.dt
        self.pose += force*(self.params.dt**2)

        # s = np.array([self.pose[0],self.pose[1],self.d_dot[0],self.d_dot[1],sheepMean.remaining_path,sheepMean.compass])
        s = np.array([0,0,0,0,sheepMean.remaining_path,sheepMean.compass])
        return s

class Sheep:
    def __init__(self,id,P):
        self.id = id
        self.params = P
        self.pose = np.random.uniform(-1,1,2)*self.params.herd_radius + self.params.herd_center
        self.heading = float(np.random.rand(1,1))*2*np.pi
        self.s_dot = np.array([np.cos(self.heading),np.sin(self.heading)])
        self.zor = []
        self.zoo = []
        self.zoa = []
        self.net_steer = None
        self.dog_force = None
        
    def update(self,sheeps,dogs,E):
        self = E.neighbours(self,sheeps)
        self = E.steer(self)
        self.s_dot = self.net_steer*self.params.dt
        self.dog_force = np.zeros(2)
        for k in range(len(dogs)):
            self.dog_force = self.dog_force + (- dogs[k].pose + self.pose)/(np.linalg.norm(dogs[k].pose - self.pose))**3
        self.dog_force = self.dog_force * E.saturate(self.params.Dog_sheep_interaction_limit,np.linalg.norm(self.dog_force))/np.linalg.norm(self.dog_force)
        self.s_dot = self.s_dot + self.dog_force
        angle_arithmatic = E.angleWrap(np.arctan2(self.s_dot[1],self.s_dot[0])) - self.heading
        self.heading += self.params.dt*angle_arithmatic
        self.pose = self.pose + np.linalg.norm(self.params.dt*self.s_dot)*np.array([np.cos(self.heading),np.sin(self.heading)])
        return self

class SheepMean:
    def __init__(self,sheeps,E,init):
        self.pose = np.zeros(2)
        self.compass = 0
        for agents in sheeps:
            self.pose = self.pose + agents.pose
            self.compass += agents.heading
        self.pose = self.pose/len(sheeps)
        self.compass = self.compass/len(sheeps)
        if init == 0:
            goal_d = E.angleWrap(np.arctan2(E.goal_direction[1],E.goal_direction[0]))
            self.compass = abs(self.compass - goal_d)
            self.remaining_path = np.linalg.norm(E.goal_direction)

class Environment:
    def __init__(self,P):
        self.params = P
        self.goal_pose = np.random.rand(1,2)[0]*(self.params.boundary_max-self.params.boundary_min) + self.params.boundary_min
        self.goal_direction = None

    def agentGenerator(self,num,obj):
        agents = []
        for i in range(num):
            agents.append(obj(i,self.params))
        return agents
    
    def saturate(self,val,lim):
        if abs(val)>lim:
            val = lim*(val/abs(val))
        return val
    
    def neighbours(self,thisAgent,restAgents):
        for agent in restAgents:
            if agent.id != thisAgent.id:
                d = np.linalg.norm(agent.pose - thisAgent.pose)
                angular_pose = self.blindRegion(thisAgent,agent)
                if d<=self.params.Rr and angular_pose<self.params.visibility:
                    thisAgent.zor.append(agent)
                elif d>self.params.Rr and d<=self.params.Ro and angular_pose<self.params.visibility:
                    thisAgent.zoo.append(agent)
                elif d>self.params.Ro and d<=self.params.Ra and angular_pose<self.params.visibility:
                    thisAgent.zoa.append(agent)
                else:
                    pass
        return thisAgent

    def blindRegion(self,thisAgent,otherAgent):
        pointing_vec = otherAgent.pose - thisAgent.pose
        pointing_vec = pointing_vec/np.linalg.norm(pointing_vec)
        ang_pose = np.arccos(np.dot(pointing_vec,thisAgent.s_dot)/np.linalg.norm(thisAgent.s_dot))
        return ang_pose

    def repulsion(self,thisAgent):
        repel = np.zeros(2)
        for i in thisAgent.zor:
            vec = i.pose - thisAgent.pose
            repel = repel + vec/np.linalg.norm(vec)
        repulsion = -repel/len(thisAgent.zor)
        return repulsion
    
    def alignment(self,thisAgent):
        align = np.zeros(2)
        for i in thisAgent.zoo:
            align = align + i.s_dot/np.linalg.norm(i.s_dot)
        alignment = align/len(thisAgent.zoo)
        return alignment
    
    def cohesion(self,thisAgent):
        attract = np.zeros(2)
        for i in thisAgent.zoa:
            vec = i.pose - thisAgent.pose
            attract = attract + vec/np.linalg.norm(vec)
        attraction = attract/len(thisAgent.zoa)
        return attraction

    def steer(self,thisAgent):
        if len(thisAgent.zor) != 0:
            thisAgent.net_steer = self.repulsion(thisAgent)
        else:
            if len(thisAgent.zoo) != 0 and len(thisAgent.zoa) == 0:
                thisAgent.net_steer = self.alignment(thisAgent)
            elif len(thisAgent.zoo) == 0 and len(thisAgent.zoa) != 0:
                thisAgent.net_steer = self.cohesion(thisAgent)
            elif len(thisAgent.zoo) != 0 and len(thisAgent.zoa) != 0:
                thisAgent.net_steer = 0.5*(self.alignment(thisAgent)+self.cohesion(thisAgent))
            else:
                thisAgent.net_steer = thisAgent.s_dot/self.params.dt
        return thisAgent

    def angleWrap(self,angle):
        while angle<0:
            angle += 2*np.pi
        while angle > 2*np.pi:
            angle-=2*np.pi
        return angle
    
    def goal(self,sheep_mean):
        self.goal_direction = self.goal_pose - sheep_mean.pose

class RLBase:
    def __init__(self,acts,alpha,gamma,epsilon,maxSteps,P):
        self.actions = acts
        self.states_list = []
        self.QT = None
        self.current_state = None
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.maxSteps = maxSteps
        self.n_acts = len(self.actions)
        self.n_states = len(self.states_list)
        self.params = P
    
    def states(self,x,y,vx,vy,remaining_path_min,remaining_path_max,compass_min,compass_max):
        x_min = x[0]
        x_max = x[1]
        y_min = y[0]
        y_max = y[1]
        vx_min = vx[0]
        vx_max = vx[1]
        vy_min = vy[0]
        vy_max = vy[1]

        # for i in np.arange(x_min,x_max,0.1):
        #     for j in np.arange(y_min,y_max,0.1):
        #         for k in np.arange(vx_min,vx_max,0.1):
        #             for l in np.arange(vy_min,vy_max,0.1):
        for m in np.arange(remaining_path_min,remaining_path_max,1):
            for n in np.arange(compass_min,compass_max,0.1):
                self.states_list.append(np.array([0,0,0,0,m,n]))
        self.states_list = np.array(self.states_list)
        # print(self.states_list.shape)
                # np.append(self.states_list,np.array([i,j,k,l,m,n]))
                
    
    def reward(self,s): # Need to redesign when base code works (pass current state)
        destination = False
        r = 100 - abs(s[5])*10 - abs(s[4])*20
        if s[5] < self.params.goal_threshold:
            r = -10000
            destination = True
        return [r,destination]

    def QTable(self):
        self.QT = np.random.rand(len(self.states_list),len(self.actions))

    def discretize(self,x):#current state
        self.current_state = np.where(np.array(abs(self.states_list-x)) == np.amin(abs(self.states_list-x),axis=0))

    def best_action(self,s):#current state index
        act = np.where(self.QT[s,:] == np.amax(self.QT[s,:]))
        choosen_action  = np.random.randint(0,len(act))
        return choosen_action

    def epsilonGreedy(self,s):#current state index
        if np.random.rand()>self.epsilon:
            a = self.best_action(s)
        else:
            a = np.random.randint(0,self.n_acts)
        return a

    def updateSARSA(self,s,a,r,sp,ap):# index of current and previous state and action
        # print(self.QT[s,a])
        # print("yes")
        self.QT[s,a] += self.alpha *(r + self.gamma*self.QT[sp,ap] - self.QT[s,a]) 

    def episode(self):
        x = [0,0,0,0,40,0.4]
        steps = 0
        totalReward = 0
        self.discretize(x)
        s = self.current_state
        a = self.epsilonGreedy(s)
        # print(s)
        E = Environment(self.params)
        dogs = E.agentGenerator(self.params.num_D,Dog)
        sheeps = E.agentGenerator(self.params.num_S,Sheep)
        init = 1
        sheep_mean = SheepMean(sheeps,E,init)
        init = 0
        anim = Animation(dogs,sheeps,sheep_mean,self.params)

        for i in range(self.maxSteps):
            # print(a)
            # print(s)
            for sh in range(len(sheeps)):
                sheeps[sh] = sheeps[sh].update(sheeps,dogs,E)
            E.goal(sheep_mean)
            sheep_mean = SheepMean(sheeps,E,init)
            anim.update(dogs,sheeps,sheep_mean)
            plt.show()
            plt.pause(0.0001)
            action = self.actions[a]
            ns = dogs[0].doAction(action,x,sheep_mean)
            [r,flag] = self.reward(ns)
            totalReward += r
            self.discretize(ns)
            index_ns = self.current_state

            na = self.epsilonGreedy(index_ns)
            self.updateSARSA(s,a,r,index_ns,na)
            s = index_ns
            a = na
            x = ns
            steps += 1
            if flag == True:
                break

        return [totalReward,steps]

class Animation:
    def __init__(self,dogs,sheeps,sheepMean,P):
        self.dog = np.array([])
        self.sheep = np.array([])
        self.sheepmean = None
        self.inter_sheep_f = np.array([])
        self.sheep_dog_f = np.array([])
        self.time = np.array([])
        self.params = P
        self.fig,self.ax = plt.subplots(ncols=2,nrows=2)
        self.fig.tight_layout()
        self.ax[0,0].set_aspect('equal')
        self.ax[0,0].set_xlim([self.params.boundary_min,self.params.boundary_max])
        self.ax[0,0].set_ylim([self.params.boundary_min,self.params.boundary_max])
        self.ax[0,1].set_aspect('equal')
        self.ax[1,1].set_aspect('equal')
        self.ax[1,0].set_aspect('equal')
        self.E = Environment(self.params)
        circle = plt.Circle((self.E.goal_pose[0],self.E.goal_pose[1]),self.params.goal_radius,color='black',fill=False)
        self.ax[0,0].add_patch(circle)
        self.ax[0,0].scatter(sheepMean.pose[0],sheepMean.pose[1],color='green',s=5)
        for dg in range(len(dogs)):
            self.ax[0,0].scatter(dogs[dg].pose[0],dogs[dg].pose[1],color='red',s = 10,marker='o')
        for sh in range(len(sheeps)):
            self.ax[0,0].quiver(sheeps[sh].pose[0],sheeps[sh].pose[1],self.params.sheep_body*np.cos(sheeps[sh].heading),self.params.sheep_body*np.sin(sheeps[sh].heading),color='blue',linewidths = 0.3)
        plt.show()
    
    def update(self,dogs,sheeps,sheepMean):
        self.ax[0,0].clear()
        self.ax[0,0].set_aspect('equal')
        self.ax[0,0].set_xlim([self.params.boundary_min,self.params.boundary_max])
        self.ax[0,0].set_ylim([self.params.boundary_min,self.params.boundary_max])
        circle = plt.Circle((self.E.goal_pose[0],self.E.goal_pose[1]),self.params.goal_radius,color='black',fill=False)
        self.ax[0,0].add_patch(circle)
        self.ax[0,0].scatter(sheepMean.pose[0],sheepMean.pose[1],color='green',s=5)
        for dg in range(len(dogs)):
            self.ax[0,0].scatter(dogs[dg].pose[0],dogs[dg].pose[1],color='red',s = 10,marker='o')
        for sh in range(len(sheeps)):
            self.ax[0,0].quiver(sheeps[sh].pose[0],sheeps[sh].pose[1],self.params.sheep_body*np.cos(sheeps[sh].heading),self.params.sheep_body*np.sin(sheeps[sh].heading),color='blue',linewidths = 0.3)
        plt.show()
    
P = params()
num_episodes = 1000
actions = [[0,0],[0,2],[0,-2],[-2,0],[2,0]]
alpha = 0.3
gamma = 1
epsilon = 0.001
maxSteps = 2

RL = RLBase(actions,alpha,gamma,epsilon,maxSteps,P)
RL.states([0,80],[0,80],[0,2],[0,2],0,80,0,2*np.pi/10)

RL.QTable()
# print(RL.QT.shape)

x = []
y = []
for eps in range(num_episodes):
    [totalReward,steps] = RL.episode()
    print("Episode",eps,"steps",steps,"Reward",totalReward,"Epsilon",RL.epsilon)
    RL.epsilon *= 0.99
    print(RL.epsilon)
    x.append(eps-1)
    y.append(steps)
    plt.plot(x,y)
    plt.xlabel("Episodes")
    plt.ylabel("Steps")
    plt.show()
    plt.pause(0.0001)
    plt.close()