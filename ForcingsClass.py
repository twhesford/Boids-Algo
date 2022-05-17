import numpy as np
import ast

class Forcings:
    
    """Just 1d forcings in the x direction as of 4/17"""

    def __init__(self,bounds):
        
        self.field = np.zeros((self.Ngridpoints(bounds)))
        
        

    def Ngridpoints(self,bounds):
        
        Nx = int(abs(bounds[1][0])-bounds[0][0])
        
        Ny = int(abs(bounds[1][1])-bounds[0][1])
        
        return Ny,Nx # to account for i,j indexing
    
    
    
    def addfieldcomp(self):
        
        print('enter new forcing value range in form ((x_lt,x_rt),(y_lt,y_rt))')
        
        newcomp = ast.literal_eval(input())
        
        x = np.linspace(newcomp[0][0],newcomp[1][0],self.field.shape[1])
        y = np.linspace(newcomp[0][1],newcomp[1][1],self.field.shape[0])
        
        newfield = np.zeros_like(self.field)

        for col in range(self.field.shape[1]):
            newfield[:,col] = y
        newfield[:,:] = newfield[:,:]*x
        
        self.field += newfield   