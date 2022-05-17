import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.animation as ani
from SwarmClass import Swarm
from ForcingsClass import Forcings 
import ast


paramdict = {'Example 1':[10,(100,100),(5,5),0,(),{'position scale': 100, 'velocity scale': 50, 'vmax': 3,'collision scale': 1.5, 'boundary scale': 1,'social scale':1,'hunger scale':1,'fear scale':1}],
'Example 2':[15,(100,100),(5,5),0,((0,0),(100,100)),{'position scale': 50, 'velocity scale': 50, 'vmax': 3,'collision scale': 5, 'boundary scale': 1,'social scale':.7,'hunger scale':1,'fear scale':1}],
'Example 3':[50,(100,100),(7,7),0,((0,0),(100,100)),{'position scale': 50, 'velocity scale': 20, 'vmax': 3,'collision scale': 3, 'boundary scale': .5,'social scale':1,'hunger scale':1,'fear scale':1}],
'Example 4':[10,(100,100),(7,7),1,((0,0),(100,100)),{'position scale':300, 'velocity scale': 40, 'vmax': 3,'collision scale': 1, 'boundary scale': .5,'social scale':1,'hunger scale':1,'fear scale':1}],
'Example 5':[15,(100,100),(7,7),0,((0,0),(100,100)),{'position scale':50, 'velocity scale': 40, 'vmax': 3,'collision scale': 5, 'boundary scale': .5,'social scale':.7,'hunger scale':1,'fear scale':1}],
'Example 6':[7,(100,100),(7,7),1,((0,0),(100,100)),{'position scale':300, 'velocity scale': 40, 'vmax': 3,'collision scale': 1, 'boundary scale': .5,'social scale':1,'hunger scale':1,'fear scale':1}],
'Pred1':[3,(10,10),(15,15),1,(),{'position scale':400, 'velocity scale': 40, 'vmax': 3,'collision scale': .7, 'boundary scale': .3,'social scale':.7,'hunger scale':1,'fear scale':1}],
'Prey0':[20,(50,50),(7,7),0,(),{'position scale':150, 'velocity scale': 40, 'vmax': 3,'collision scale': 5, 'boundary scale': .5,'social scale':.7,'hunger scale':1,'fear scale':1}],
'Small PS':[20,(100,100),(5,5),0,((0,0),(100,100)),{'position scale':.1, 'velocity scale': 7, 'vmax': 2.5,'collision scale': 1, 'boundary scale': .5,'social scale':.7,'hunger scale':1,'fear scale':1}],
'Small CS':[20,(100,100),(5,5),0,((0,0),(100,100)),{'position scale':100, 'velocity scale': 7, 'vmax': 2.5,'collision scale': .001, 'boundary scale': .5,'social scale':.7,'hunger scale':1,'fear scale':1}],
'Pattern 1':[50,(100,100),(20,20),0,((0,0),(100,100)),{'position scale':200, 'velocity scale': 1, 'vmax': 2.5,'collision scale': .05, 'boundary scale': .01,'social scale':.7,'hunger scale':1,'fear scale':1}],
'Boid Bomb':[50,(1,1),(10,10),0,(),{'position scale':200, 'velocity scale': 1, 'vmax': 5,'collision scale': .05, 'boundary scale': .01,'social scale':.7,'hunger scale':1,'fear scale':1}],
'Prey Buffet':[100,(75,75),(5,5),0,((0,0),(75,75)),{'position scale':10, 'velocity scale': 1, 'vmax': 1,'collision scale': .05, 'boundary scale': .01,'social scale':.7,'hunger scale':1,'fear scale':1}],
'Pred2':[3,(75,75),(3,3),0,((0,0),(75,75)),{'position scale':40, 'velocity scale': 1, 'vmax': 1,'collision scale': .1, 'boundary scale': .01,'social scale':.7,'hunger scale':1,'fear scale':1}]}


class System(Swarm):

    def __init__(self,params=()):
        
        # #print('Input System bounds in form ((xmin,ymin),(xmax,ymax))')
        # self.extfield = Forcings(((0,0),(100,100)))#ast.literal_eval(input()))

        # print('Would you like to add an external velocity field? type y or n')
        # forcingcheck = input()

        # if forcingcheck == 'y':
        #     self.extfield.addfieldcomp()
   
        self.swarms  = self.initialize(params)
        self.plotbounds = self.plotinfo()
        
    def buildswarms(self,Nswarms,params):
        swarm_array = np.zeros(Nswarms,dtype=object)

        if len(params) ==0:

            for i in range(Nswarms):
                swarm_array[i] = Swarm()

        else:

            for i in range(Nswarms):
                swarm_array[i] = Swarm(paramdict[params[i]])


        return swarm_array

    
    def initialize(self,params):

        if  params ==():
            print('Input N swarms as int')
            Nswarms = int(input())

            # print('Would you like to impose plot bounds? y/n')
            # check = input()

            # if check == 'y':

            #     print('input plot bounds as tuple ex: (xmin,xmax),(ymin,ymax))')
            #     self.plotbounds = ast.literal_eval(input())

            return self.buildswarms(Nswarms,params)
            # swarm_array = np.zeros(Nswarms,dtype=object)
            # for i in range(Nswarms):
                
            #     swarm_array[i] = Swarm(params)


        else:
            Nswarms = len(params)

            return self.buildswarms(Nswarms,params)


    
    def resetsystem(self):

        self.swarms = self.initialconditions 

    def swarmneighborhood(self,swarm_idx):

        """
        This function inputs a swarm class and finds 
        its boid neighbors in self.swarms
        """

        mag = lambda Vector : np.sqrt(Vector[0]**2 + Vector[1]**2)

        N = len(self.swarms)

        thresh = mag(self.swarms[swarm_idx].neighborthresh)

        swarm_idxs = np.linspace(0,N-1,N,dtype=int)
        neighbors = np.zeros((N-1,len(self.swarms[swarm_idx].boids)),dtype=object)
        trophiclevels = np.zeros(N-1)

        neigh_idx = 0
        for idx in swarm_idxs[swarm_idxs != swarm_idx]:

            trophiclevels[neigh_idx] = self.swarms[idx].trophiclevel
                
            boid_idx = 0
            for boid_ in self.swarms[swarm_idx].boids:

                sep = np.array(list(map(lambda neigh : boid_.X-neigh.X,self.swarms[idx].boids)))
                neighbors[neigh_idx,boid_idx] = self.swarms[idx].boids[np.array(list(map(lambda s : mag(s)<thresh,sep)))]
                boid_idx += 1
            neigh_idx += 1

        return neighbors,trophiclevels


    def getswarmneighbors(self):
        
        N = len(self.swarms)
        allneighbors = np.zeros(N,dtype=object)
        swarm_idxs = np.linspace(0,N-1,N,dtype=int)

        for i in swarm_idxs:
            
            allneighbors[i] = self.swarmneighborhood(i)

        return allneighbors



    def systemupdate(self):

        outsideneighs = self.getswarmneighbors()

        N = len(self.swarms)

        centerxs = np.zeros(N,dtype=object)
        centerys = np.zeros(N,dtype=object)

        xs = np.zeros(N,dtype=object)
        ys = np.zeros(N,dtype=object)

        for i in range(len(self.swarms)):
            xs[i],ys[i],centerxs[i],centerys[i] = self.swarms[i].updateboids(outsideneighs[i])
       

        return xs,ys,centerxs,centerys

    def plotinfo(self):

        checklist = []
        for s in self.swarms:

            checklist.append(s.bounds) if s.bounds != () else 0
        checklist = list(filter(lambda x : x !=0,checklist))
        if len(checklist) == 0:

            return  0
        else:
            xmins = np.array(list(map(lambda x : x[0][0],checklist)))
            ymins = np.array(list(map(lambda y : y[0][1],checklist)))

            xmaxs = np.array(list(map(lambda x : x[1][0],checklist)))
            ymaxs = np.array(list(map(lambda y : y[1][1],checklist)))

            print(xmins,ymins)
            print(xmaxs,ymaxs)



            return ((min(xmins),min(ymins)),(max(xmaxs),max(ymaxs)))





    
    def animateswarms(self,timesteps,boidsize=(),anispd = 1):
        
        fig = plt.figure()

        
    
        def animate(t):

            plt.clf()

            xs,ys,centerxs,centerys = self.systemupdate()

            for i in range(len(xs)):

                plt.plot(xs[i],ys[i],'o',ms=2)
            if self.plotbounds ==0:
                plt.xticks(np.linspace(min(centerxs)-100,max(centerxs)+100,10))
                plt.yticks(np.linspace(min(centerys)-100,max(centerys)+100,10))
            
            else:
                plt.xticks(np.linspace(self.plotbounds[0][0],self.plotbounds[1][0],10))
                plt.yticks(np.linspace(self.plotbounds[0][1],self.plotbounds[1][1],10))              
      
            plt.show()
           
        an = ani.FuncAnimation(fig,animate,range(timesteps),interval=anispd)
        plt.show()

    def swarminfo(self,timesteps):
        
        t = 0
        centers = np.zeros((2,timesteps))
        while t < timesteps:

            x,y,centx,centy = self.systemupdate()

            centers[0,t] = centx
            centers[1,t] = centy

            t += 1
        return centers 
            



