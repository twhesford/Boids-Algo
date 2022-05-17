import numpy as np
from BoidClass import Boid
import matplotlib.animation as ani
import matplotlib.pyplot as plt
import ast
from ForcingsClass import Forcings
boid_param_info = "***********************************BOID INFO******************************************** \n \n All constants are inversely proportional to the strength of their corresponding rule \n \n position scale: determines how much the boids want to flock. Each boids distance from thier relative \n swarm center is divided by 'position scale and that value is added to the velocity so larger \n values make the boids more independent.\n \n velocity scale: same as position scale but just for velocity matching. Scales the boids average \n velocity. Smaller values tend to make the swarm move faster \n \n vmax: max magnitude of boid velocity \n \n collision scale: scales how fast the bounce off each other to avoid collisions. Low values \n make the boids bounce off each other while higher values make the turn around more gradual. \n If collision scale values are low relative to velocity values collisions can occur \n \n boundary scale: pretty much the same as collisionconst but for the boundaries if applicable. \n Boids are set to begin to turn away from the boundaries 10 units away and this constant \n determines how fast they turn around where small values accelerate them from the boundaries \n faster. This value can be tuned to allow for fast moving boids to briefly leave the boundary \n simulating jumping fish, or birds diving under water ect. \n \n *******************************************************************************************"




class Swarm(Boid):
    
    """
    Swarm holds an array of boids 
    """
    def __init__(self,params=(),initialvmax=()):
        
        # none values will be updated in self.initialize()
        self.neighborthresh = 'none'
        self.pltstyl = 'none'
        self.initbounds = 'none'
        self.bounds = 'none'
        self.trophiclevel = 'none'
        #self.extfield = Forcings(((0,0),(100,100)))
        self.boids = self.initialize(params,initialvmax) # create an initial swarm with random positions
        print(self.bounds)




    def swarmcenter(self,x,y):
        
        """
        This function inputs two arrays of the x and y position 
        for each boid in self.boids and returns an array of the 
        relative swarm center components for each boid
        """

        # finds the relative swarm center for each boid 
        xcenters = np.array(tuple(map(lambda i : (np.sum(x)-x[i])/(len(x)-1),range(len(x))))) 
        ycenters = np.array(tuple(map(lambda i : (np.sum(y)-y[i])/(len(y)-1),range(len(y))))) 
        return np.array(list(zip(xcenters,ycenters))) # returns an array of position vectors in tuples



    def swarmvelocity(self,vx,vy):
        
        """
        This function returns the average flock speed
        relative to each boid in self.boids
        """
        
        # take the average velocity of all boids in self.boids()
        meanvx = np.array(list(map(lambda i : (np.sum(vx)-vx[i])/(len(vx)-1),range(len(vx))))) # mean Vx
        meanvy = np.array(list(map(lambda i : (np.sum(vy)-vy[i])/(len(vy)-1),range(len(vy))))) # mean Vy
        
        return np.array(list(zip(meanvx,meanvy))) # returns an array of velocity vectors in tuples

    def externalforcings(self):

        xforcings = np.zeros(len(self.boids))
        yforcings = np.zeros(len(self.boids))
        for b in range(len(self.boids)):

            boidpos = self.boids[b].X

            x = int(boidpos[0])
            y = int(boidpos[1])

            if x > self.bounds[1][0]:
                x = self.bounds[1][0]

            if x < self.bounds[0][0]:
                x = self.bounds[0][0]

            if y > self.bounds[1][1]:
                y = self.bounds[1][1]

            if  y< self.bounds[0][1]:
                y = self.bounds[0][1]
                        
            xforcings[b] = self.extfield.field[x-1,y-1]

        return np.array(list(map(lambda x,y: np.array([x,y]),xforcings,yforcings)))



    def getneighbors(self):
        
        """          
        neigh is neighbor of boid_ if:
            the length of the boid_/2 + length of neigh/2 + x seperation < thresh[0]
            and 
            width of boid_/2 + width of neigh/2 + y seperation < thresh[1]
        end if 
            
        to simulate boid size and seperation if desired. If not the magnitudes are fine
        """
        
        mag = lambda Vector : np.sqrt(Vector[0]**2 + Vector[1]**2)
        
        thresh = self.neighborthresh
        t  = mag(thresh)
        neighbors = np.zeros(len(self.boids),dtype=object)
        
        boid_idx = 0
        for boid_ in self.boids:

            # finds distance from boid to all boids in Swarm
            sep = np.array(list(map(lambda neigh : boid_.X-neigh.X,self.boids)))
            
            # finds all boids within the neighborhood of boid and filters boid_- boid_ = 0
            neighbors[boid_idx] = self.boids[np.array(list(map(lambda s : mag(s)<t,sep)))]
            
            boid_idx += 1
            
        return neighbors




    def build_swarm(self,initvmax):
    
        # Swarm Parameters 
        print('enter trophic level as int')
        trophiclevel = int(input())

        print('enter top boundaries where Swarm is initialized: ex (x_max,y_max)')
        initbounds = ast.literal_eval(input()) # boundaries where the swarm is initialized. They are not enforced in boid movement
        
        # hard boundaries

        print('Would you like to impose boundaries on the Boids runtime position? type y or n')
        boundarycheck = input()
        bounds = ()
        #bounds = ((0,0),(100,100))
        plotbounds = False # determines how plot axis are displayed based on boundaries
        if boundarycheck == 'y':

            prompts = ('enter hard mins as tuple ex: (x min,y min)','enter hard maxs as tuple ex: (x max,y max)')
            bounds = np.zeros(2,dtype=object)

            lazylist = [10,-10]
            for i in range(2):
                print(prompts[i])
                bounds[i] =np.array(ast.literal_eval(input()))+lazylist[i]

            plotbounds = True

        # neighbor thresh
        print('enter neighbor thresh as  ex: (7,7)')
        neighborthresh = ast.literal_eval(input())

        # number of boids
        print('enter N boids as an integer')
        Nboids = int(input()) # number of boids 

        # Boid Parameters
        print('enter boid constants as dictionary in form',str({'position scale': 50, 'velocity scale': 50, 'vmax': 5, 'collision scale': .75, 'boundary scale': 1,'social scale':1,'hunger scale':1,'fear scale':1}))
        print('for descriptions of each parameter type: info')
        boidconsts = input()
        if boidconsts == 'info':
            print(boid_param_info,'\n \n')
            print('enter boid constants as dictionary in form',str({'position scale': 150, 'velocity scale': 50, 'vmax': 1.5, 'collision scale': 5, 'boundary scale': 1}))
            boidconsts = ast.literal_eval(input())

        else:
            boidconsts = ast.literal_eval(boidconsts)
        
        if len(initvmax) == 0:
            scale = 1/np.sqrt(2)
            init_v = boidconsts['vmax']*scale
            initvmax = np.full(2,init_v)

        self.initbounds = initbounds
        self.pltstyl = plotbounds
        self.bounds = bounds
        self.trophiclevel = trophiclevel
        self.neighborthresh = neighborthresh

        return Nboids,boidconsts,initvmax
        

    def unpackparams(self,param):
        print('Nboids,','initial bounds,','neighbor thresh,','trophic level,','bounds,','boid consts')
        print(param)
        Nswarms = param[0]
        self.initbounds = param[1]
        self.neighborthresh = param[2]
        self.trophiclevel = param[3]
        self.bounds = param[4]
        boidconsts = param[5]

        scale = 1/np.sqrt(2)
        initv = boidconsts['vmax']*scale # initial max velocity
        initvmax = (initv,initv)

        return Nswarms,boidconsts,initvmax



    def initialize(self,params,initialvmax):
        
        """
        This function initializes an array of Boids 
        with random positions in (0,0) <= x < (x_max,y_max)
        and velocity magnitudes in 0 <= v < vmax with  
        trajectories pointing to the relative center of the
        swarm from the perspective of each boid.
        """
        # pred prey system??
# {'position scale': 20, 'velocity scale': 10, 'vmax': 4, 'collision scale':1.1, 'boundary scale': .3,'neautswarm scale':1}
        
        if params == ():
            N,boidconsts,initvmax = self.build_swarm(initialvmax)
        
        else:

            N,boidconsts,initvmax = self.unpackparams(params)      
            


        # N = 10
        # boidconsts = {'position scale': 50, 'velocity scale': 50, 'vmax': 5, 
        # 'collision scale': .75, 'boundary scale': 1,'social scale':1,'hunger scale':1,'fear scale':1,'trophic level':0}
        # initvmax = np.array([np.sin(np.pi/4),np.sin(np.pi/4)])*boidconsts['vmax']
        # self.initbounds = (100,100)
        # self.neighborthresh = (10,10)
        # self.pltstyl = False
        # self.bounds = ((-1000,-1000),(1000,1000))




        mag = lambda Vector : np.sqrt(Vector[0]**2 + Vector[1]**2)

        # generate random initial non-repeating boid positions
        x0 = np.random.uniform(low=0,high=self.initbounds[0],size=N)# x0
        y0 = np.random.uniform(low=0,high=self.initbounds[1],size=N)# y0
        vx0 = np.random.uniform(low=-initvmax[0]/2,high=initvmax[0]/2,size=N) # v0
        vy0 = np.random.uniform(low=-initvmax[1]/2,high=initvmax[1]/2,size=N) # v0
  
        # create initial velocity vectors pointing to each relative cenbter with random speeds 
        swarmcenters = self.swarmcenter(x0,y0)# theta0
        swarmvelocities = self.swarmvelocity(vx0,vy0)

        # condense the x and y components of the initial position and velocity into an array
        X0 = np.array(list(map(lambda x,y : np.array([x,y]),x0,y0))) # X0 vector
        V0 = np.array(list(map(lambda x,y : np.array([x,y]),vx0,vy0))) # V0 vector
        
        # find boundaries
       
        initial_func = lambda X0,V0,center,velocity : Boid(X0,V0,center,velocity,boidconsts,self.trophiclevel)

        return np.array(list(map(initial_func,X0,V0,swarmcenters,swarmvelocities)))


    def updateboids(self,swarmneighbors=()):
        
        """
        This function updates each boid in 
        the swarm by calling Boid.update()
        """
                
        mag = lambda Vector : np.sqrt(Vector[0]**2 + Vector[1]**2)
        
        # Collect Boid info from self.boids
        X = np.array(list(map(lambda boid : boid.X,self.boids)),dtype=object) # positon vectors of self.boids
        V = np.array(list(map(lambda boid : boid.V,self.boids)),dtype=object) # velocity vectors of self.boids
        
        vx = np.array(list(map(lambda vector : vector[0],V))) # self.boids x velocities
        vy = np.array(list(map(lambda vector : vector[1],V))) # self.boids y velocities

        x = np.array(list(map(lambda vector : vector[0],X))) # self.boids x positions
        y = np.array(list(map(lambda vector : vector[1],X))) # self.boids y positions
        
        # Find swarm info from Boid info
        centers = self.swarmcenter(x,y) # relative swarm centers
        velocities = self.swarmvelocity(vx,vy) # relative swarm velocities
        
        # Find the neighbors of each boid
        neighbors = self.getneighbors()

        # find external forcings for each boid
        #extforcings =  self.externalforcings() #if type(self.extfield) != tuple else self.extfield.field
        #extforcings = np.array([(0,0)]*len(self.boids),dtype=object)
        
        if len(swarmneighbors) == 0:

            swarmneighbors = np.array(len(self.boids)*[(0,0)],dtype=object)



        idx = 0 
        for boid in self.boids:
  
            # update boids
            boid.update(centers[idx],velocities[idx],neighbors[idx],self.bounds,(swarmneighbors[0][:,idx],swarmneighbors[1]))
      
            self.boids = np.array(list(filter(lambda boid : np.isnan(mag(boid.X))==False,self.boids)))
            
            
            idx += 1
     
        # returns for plotting 
        centerx = np.mean(list(map(lambda x : x[0],centers)))
        centery = np.mean(list(map(lambda x : x[1],centers)))
        
        return x,y,centerx,centery




        
    def animateboids(self,timesteps,boidsize=2,anispd = .1):
        
        fig = plt.figure()
        
        def animate(t):
            
            plt.cla()
            x,y,centerx,centery = self.updateboids()
           
            plt.plot(x,y,'o',ms=boidsize)

            
            if self.pltstyl == False:
                plt.title('Unbounded Boids')
                plt.xticks(np.linspace(round(centerx - self.initbounds[0]/2,0),round(centerx + self.initbounds[0]/2,0),10))
                plt.yticks(np.linspace(round(centery - self.initbounds[1]/2,0),round(centery + self.initbounds[1]/2,0),10))
            else: 
                plt.title('Bounded Boids')
                plt.yticks(np.linspace(self.bounds[0][0],self.bounds[1][0],10))
                plt.xticks(np.linspace(self.bounds[0][1],self.bounds[1][1],10))
            plt.show()

        an = ani.FuncAnimation(fig,animate,range(timesteps),interval=anispd)
        plt.show()

    def getboidinfo(self,timesteps):

        t = 0
        initial_positions = np.zeros(len(self.boids),dtype=object)
        swarmcenters = np.zeros((2,timesteps))
        while t< timesteps:

            if t == 0:

                initial_positions[0:] = self.boids

            x,y,centerx,centery = self.updateboids() 

            swarmcenters[0,t] = centerx
            swarmcenters[1,t] = centery

        return initial_positions,swarmcenters