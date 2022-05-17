import numpy as np


class Boid():
    
    """
    Essentially a vector class:
    
    Boid stores the position and velocity of each 
    Boid in a school,flock,herd ect. All info 
    about the Boids surroundings is stored in 
    Swarm class.
    
    Boids are stored in the Swarm class
    """
    
    def __init__(self,X0,V0,swarmcenter0,swarmvelocity0,consts,trophiclevel):
        
        self.consts = consts
        self.trophiclevel = trophiclevel
        self.X,self.V = self.initialize(X0,V0,swarmcenter0,swarmvelocity0)
        
        
    def move2center(self,swarmcenter=()):
        
        """This function applies Rule 1 and 
           returns the velocity adjustment based
           on the Swarm class center """
        
        unitsfromcenter = np.array(swarmcenter) - np.array(self.X)
        
        return unitsfromcenter/self.consts['position scale']
    
    
    def matchvelocity(self,swarmvelocity=()):
        
        """This function applies Rule 2 and 
           returns the velocity adjustment based
           on the Swarm class velocity """
        
        velocitydifference = np.array(swarmvelocity) - np.array(self.V) 
        
        return velocitydifference/self.consts['velocity scale']

    
    def avoidcollisions(self,nearneighbors=()):
        
        """
        This function applies Rule 3 and returns 
        the velocity adjustment based on the Boids
        neighbors"""
        
        neighbor_seps = np.array([0,0],dtype=float)
        nearneighbors = np.array(list(filter(lambda x : len(x.X) != 0,nearneighbors)))
        
        if len(nearneighbors) == 0:
           
            return neighbor_seps
        
        for neighbor in nearneighbors:

            neighbor_seps -= neighbor.X - self.X
            
        return neighbor_seps/self.consts['collision scale']
    
    
    def keepinbounds(self,softbounds=()):
        
        """
        This function calculates the distance 
        of each boid from the boundaries."""

        mins = softbounds[0]
        maxs = softbounds[1]
        
        boundarycheck = np.zeros(2)
        
        # check if vector X in bounds
        dim = 0
        while dim < 2:
            
            if self.X[dim] < mins[dim]:

                boundarycheck[dim] = maxs[dim] - self.X[dim] 

            if self.X[dim] > maxs[dim]:

                boundarycheck[dim] = maxs[dim] - self.X[dim] 
             
            dim += 1
            
        return boundarycheck/self.consts['boundary scale']



    def deleteself(self):

        self.X,self.V = np.array([np.nan,np.nan]),np.array([np.nan,np.nan])
    
    
    def neutraloutsiders(self,neutralneighs):

        neutralneighbor_seps = np.array([0,0],dtype=float)
        
        for swarm in neutralneighs:
            if len(swarm) == 0:
                pass
            else:
                for neighbor in swarm:

                    neutralneighbor_seps -= neighbor.X - self.X
            
        return neutralneighbor_seps/self.consts['social scale']
        

    def agressiveoutsiders(self,agroneighs):

        mag = lambda Vector : np.sqrt(Vector[0]**2 + Vector[1]**2)

        agroneighbor_seps = np.array([0,0],dtype=float)
        for swarm in agroneighs:
            if len(swarm) == 0:
                pass
            else:
                for neighbor in swarm:
                    agroneighbor_seps -= neighbor.X - self.X

            # check to see if eaten
                a = mag(agroneighbor_seps)
                if abs(a)< 1:
                    agroneighbor_seps = np.array([np.nan,np.nan])

        return agroneighbor_seps/self.consts['fear scale']


    def preyswarms(self,deliciousneighs):

        deliciousneighbor_seps = np.array([0,0],dtype=float)
        mag = lambda Vector : np.sqrt(Vector[0]**2 + Vector[1]**2)

        for swarm in deliciousneighs:
            if len(swarm) == 0:
                pass
            else:
                for neighbor in swarm:

                    if np.isnan(mag(neighbor.X)) == True:
                        pass
                    else:

                        deliciousneighbor_seps += neighbor.X - self.X
            
        return deliciousneighbor_seps/self.consts['hunger scale']



    def adjust4outsiders(self,outsideneighbors):
   

        trophiclevels = outsideneighbors[1]
        neighbors = outsideneighbors[0]
        boidtlevel = self.trophiclevel

        neutralneighbors = neighbors[trophiclevels==boidtlevel]
        agroneighbors = neighbors[trophiclevels>boidtlevel]
        deliciousneighbors = neighbors[trophiclevels<boidtlevel]
        
        neutralseps = np.zeros(2)
        agroseps = np.zeros(2)
        preyseps = np.zeros(2)
    
        if len(neutralneighbors) != 0:
            neutralseps = self.neutraloutsiders(neutralneighbors)

        if len(agroneighbors) != 0:
            agroseps = self.agressiveoutsiders(agroneighbors)

        if len(deliciousneighbors) != 0:
            preyseps = self.preyswarms(deliciousneighbors)

        return neutralseps + agroseps + preyseps
      
        

    def initialize(self,X0=[],V0=[],swarmcenter0=[],swarmvelocity0=[]):
        
        mag = lambda Vector : np.sqrt(Vector[0]**2 + Vector[1]**2)
        
        unitvector = lambda Vector : Vector/mag(Vector)
        
        X0 = np.array(X0)
        V0 = np.array(V0)
 
        swarmcenter0 = np.array(swarmcenter0)
        swarmvelocity0 = np.array(swarmvelocity0)
        
        # Rule 1
        unitsfromcenter = (swarmcenter0 - X0)/ self.consts['position scale']
        
        # Rule 2
        velocitydifference = (swarmvelocity0 - V0) / self.consts['velocity scale']

        # Update Velocity
        initialV = V0 + velocitydifference + unitsfromcenter
        
        initialV = initialV if mag(initialV) < self.consts['vmax'] else unitvector(initialV)*self.consts['vmax'] # keeps velocity in check
       
        initialX = X0 + initialV
      
        return initialX,initialV
        
    
    
    
    def update(self,swarmcenter,swarmvelocity,neighbors,softbounds=(),outsideneighbors=()):
                
        """
        This function updates the position, and velocity
        of boid based on its surroundings.
        
        This can be updated with conditionals creating 
        small random fluctuations in velocity ad responses
        to fear, fooddrive, fatigue, currents/winds ect.
        """
        
        mag = lambda Vector : np.sqrt(Vector[0]**2 + Vector[1]**2)
        unitvector = lambda Vector : Vector/mag(Vector)
        
        swarmcenter0 = np.array(swarmcenter)
        swarmvelocity0 = np.array(swarmvelocity)
        
        # Rule 1
        deltaV0 = self.move2center(swarmcenter)
        
        # Rule 2
        deltaV1 = self.matchvelocity(swarmvelocity)

        # Rule 3
        deltaV2 = self.avoidcollisions(neighbors)
        
        # Boundary Ckeck
        deltaV3 = np.zeros(2)
        if len(softbounds) != 0:
            deltaV3 = self.keepinbounds(softbounds)

        # Neighbors from other Swarms
        deltaV4 = np.zeros(2)
        if len(outsideneighbors) != 0:
            deltaV4 = self.adjust4outsiders(outsideneighbors)



        #deltaV5 = extforcing
        #print(deltaV5)
        
        # Update Velocity
        updatedV = np.array(self.V) + np.array(deltaV0) + np.array(deltaV1) + np.array(deltaV2) + deltaV3 + deltaV4 
   
        updatedV = updatedV if mag(updatedV) < self.consts['vmax'] else unitvector(updatedV)*self.consts['vmax'] # keeps velocity in check
        #updatedV += np.array(deltaV5)
        self.V = updatedV
        self.X += self.V