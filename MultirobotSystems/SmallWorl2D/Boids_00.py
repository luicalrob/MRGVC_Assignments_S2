## A solution to the Boids exercise
## Version: 20240215
## Author: Luis Calderon, Andres Martinez
## License: CC-BY-SA

from time import time, localtime, strftime
from random import uniform
import numpy as np
from point2d import Point2D 
# YOU MIGHT FIND USEFUL FUNCTIONS IN shapely, Point2D, AND gadgETs
from SmallWorl2D import Space, KPIdataset, Obstacle, MoBody, Mobot, Soul

class Boid(Soul):

    # YOUR AUXILIARY FUNCTIONS
    def __init__(self,body,T,dd):  # ADD YOUR ARGUMENTS
        self.control="MDMC"
        self.dd=dd
        self.alpha=2
        self.epsilon=1.5
        self.noise=0.1
        self.K=[0.5, 0.06, 0.25]
        # YOUR BOID INIT CODE
        super().__init__(body,T)

    def magnitude(self, d):
        p = -4*self.alpha*self.epsilon / d
        p = p * (2*pow(self.noise/d, 2*self.alpha) - pow(self.noise/d, self.alpha))
        return p

    def proximal_control(self, b):
        p = 0
        nearby = b.space.RnB(b.index(),(type(b),Mobot), b.space.R)
        for measurement in b.space.RnB(b.index(),(type(b),Mobot), b.space.R):
            p += self.magnitude(measurement[0])*np.exp(1j * measurement[1])
        return p
    
    def allignment_control(self, b):
        a = 0
        return a
    
    def mdmc(self, b, f):
        u=self.K[0]*f[0] + b.v
        w=self.K[1]*f[1]
        return u,w
        
    def update(self):
        if super().update():
            b=self.body
            i=b.index()
            s=b.space

            p=self.proximal_control(b)
            a=self.allignment_control(b)
            g=0
            f=p+a+g
            
            if self.control == "MDMC":
                u, w = self.mdmc(b, f)
            # else:
            #     o = 
            #     orientation_error = o * modulo(f)
            #     if orientation_error > 0:
            #         u = orientation_error * b.v
            #     else:
            #         u = 0
            #     w=b.K[2]*(dif_angulos)
            # YOUR BOID UPDATE CODE

    # actualizar todo lo que haga falta, distancias y todo para ver el comportamiento que tiene que tener cada uno

def set_mobot_formation(i, s, center, large, th=np.pi/2, fc=(0.2, 0.2, 0), v=0, v_max=None, w_max=None):
    if v_max is None:
        v_max = s.vN / 2
    if w_max is None:
        w_max = s.wN

    pos = (uniform(-large/2,large/2) + center[0], uniform(-large/2,large/2) + center[1])
    return Mobot(s, 'm' + str(i), pos=pos, th=th, fc=fc, v=v, v_max=v_max, w_max=w_max)

def init():

    ## Create Data Structures
    name='Boids_'+strftime("%Y%m%d%H%M", localtime())
    global s, N, R
    dd=0.75 # or whatever
    R=1.5
    s=Space(name,R=R,limits='hv',visual=True,showconn=False)
    KPIdataset(name,s,[1,1,0],[(0,'.y'),(1,'.k'),(2,'.g')])
        # 0 simulation time scale -- recommended "default" KPI
        # 1 Fraction remaining
        # 2 Fraction in largest group
        # ...

    ## Populate the world

    # N Mobots
    #N=25
    N=25
    i=0

    # limits in X = +-16, limits in Y = +-9
    posX = 6
    posY = -3
    large = 3.5

    while i<N:
        new=set_mobot_formation(i, s, center=(posX, posY), large=large)
        # en vez de posicionar aleatoriamente, poner todos juntitos y con velocidad nula, ya pondremos command vel en el boid
        if s.fits(new,s.room,safe=0.3):
            s.bodies.append(new)
            # YOUR BOID PARAMETRIZATION, DIFFERENT KINDS?
            Boid(new, 0, dd)
            # creamos un comportamiento "alma" para cada robot
            i += 1

    # and several Obstacles, or Killers, or whatever

    # init distances matrix and connections graph
    s.dist=np.zeros((len(s.bodies),len(s.bodies))) # distances between centroids in a np.matrix
    s.update_dist()
    s.update_conn()

    # WHATEVER ELSE YOU WANT TO DO INITIALLY, E.G., REPORT ON LOG

init()
end=False
while not end: # THE loop 
    s.step()

    # COLLISION MANAGEMENT
    ko=[]
    # YOUR COLLISION DETECTION CODE 
    s.remobodies(ko,'collision')
    # si se chocan quitarlos

    # KPIs
    KPI=[s.time/(time()-s.t0),0,0]
    for b in s.bodies:
        if b.on:
            if isinstance(b,Mobot):
                KPI[1] += 1
    KPI[1]/=N
    KPI[2]/=N
    # la idea es actualizar el mapa y toda la informacion, los comportamientos y todo a parte de la visualizacion
    # YOUR KPI COMPUTATIONS
    s.KPIds.update(KPI)

    end=s.has_been_closed() or s.KPIds.KPI[1]==0 or s.time>60 # or whatever
s.close()